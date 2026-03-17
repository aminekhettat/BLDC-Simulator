import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# ruff: noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.control.adaptive_tuning import AdaptiveFOCTuner
from src.control.foc_controller import FOCController
from src.control.svm_generator import SVMGenerator
from src.control.transforms import clarke_transform, park_transform
from src.core.load_model import LoadProfile
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.simulation_engine import SimulationEngine

PROFILE_PATH = PROJECT_ROOT / "data" / "motor_profiles" / "motenergy_me1718_48v.json"
SESSION_PATH = (
    PROJECT_ROOT
    / "data"
    / "tuning_sessions"
    / "until_converged"
    / "motenergy_me1718_48v_until_converged.json"
)
OUT_PATH = PROJECT_ROOT / "data" / "logs" / "calibration_me1718_no_fw_loaded_point.json"


class SmoothRampHoldLoad(LoadProfile):
    def __init__(self, start_s: float, end_s: float, final_torque: float):
        self.start_s = float(start_s)
        self.end_s = float(end_s)
        self.final_torque = float(final_torque)

    def get_torque(self, t: float) -> float:
        if t <= self.start_s:
            return 0.0
        if t >= self.end_s:
            return self.final_torque
        x = (t - self.start_s) / max(self.end_s - self.start_s, 1e-9)
        s = x * x * (3.0 - 2.0 * x)
        return self.final_torque * s


@dataclass
class Candidate:
    speed_kp: float
    speed_ki: float
    current_kp: float
    current_ki: float
    iq_limit_a: float
    use_d_priority: bool
    coupled_aw_gain: float


def to_motor_params(profile: dict) -> MotorParameters:
    mp = profile["motor_params"]
    return MotorParameters(
        nominal_voltage=float(mp["nominal_voltage"]),
        phase_resistance=float(mp["phase_resistance"]),
        phase_inductance=float(mp["phase_inductance"]),
        back_emf_constant=float(mp["back_emf_constant"]),
        torque_constant=float(mp["torque_constant"]),
        rotor_inertia=float(mp["rotor_inertia"]),
        friction_coefficient=float(mp["friction_coefficient"]),
        num_poles=int(mp["num_poles"]),
        poles_pairs=int(mp.get("poles_pairs", int(mp["num_poles"]) // 2)),
        ld=float(mp.get("ld", mp["phase_inductance"])),
        lq=float(mp.get("lq", mp["phase_inductance"])),
        model_type=str(mp.get("model_type", "dq")),
        emf_shape=str(mp.get("emf_shape", "sinusoidal")),
    )


def estimate_max_no_fw_speed_rpm(
    params: MotorParameters, utilization_margin: float = 0.95
) -> float:
    ke = max(float(params.back_emf_constant), 1e-12)
    v_max = (float(params.nominal_voltage) / math.sqrt(3.0)) * float(
        np.clip(utilization_margin, 0.1, 1.0)
    )
    omega_max = v_max / ke
    return float(omega_max * 60.0 / (2.0 * math.pi))


def margins_for(params: MotorParameters, cand: Candidate) -> dict:
    tuner = AdaptiveFOCTuner(params)
    s = tuner.analyze_speed_loop(cand.speed_kp, cand.speed_ki)["margin"]
    c = tuner.analyze_current_loop(cand.current_kp, cand.current_ki)["margin"]
    return {
        "speed_gain_margin_db": float(s.gain_margin_db),
        "speed_phase_margin_deg": float(s.phase_margin_deg),
        "current_gain_margin_db": float(c.gain_margin_db),
        "current_phase_margin_deg": float(c.phase_margin_deg),
    }


def evaluate_case(
    params: MotorParameters,
    cand: Candidate,
    target_speed_rpm: float,
    final_load_nm: float,
    dt: float = 5e-4,
    sim_end_s: float = 4.5,
    compute_margins: bool = False,
    min_efficiency_mech_power_w: float = 200.0,
    min_efficiency_load_nm: float = 0.5,
) -> dict:
    ramp_start_s = 0.9
    ramp_end_s = 2.8

    motor = BLDCMotor(params, dt=dt)
    load = SmoothRampHoldLoad(ramp_start_s, ramp_end_s, final_load_nm)
    engine = SimulationEngine(
        motor,
        load,
        dt=dt,
        compute_backend="cpu",
        max_history=max(6000, int(sim_end_s / dt) + 500),
    )
    controller = FOCController(motor=motor, enable_speed_loop=True)

    controller.set_cascaded_speed_loop(True, iq_limit_a=float(cand.iq_limit_a))
    controller.set_speed_pi_gains(
        kp=float(cand.speed_kp), ki=float(cand.speed_ki), kaw=0.05
    )
    controller.set_current_pi_gains(
        d_kp=float(cand.current_kp),
        d_ki=float(cand.current_ki),
        q_kp=float(cand.current_kp),
        q_ki=float(cand.current_ki),
        kaw=0.2,
    )
    if cand.use_d_priority:
        controller.set_voltage_saturation(
            mode="d_priority",
            coupled_antiwindup_enabled=True,
            coupled_antiwindup_gain=float(cand.coupled_aw_gain),
        )

    controller.set_current_references(id_ref=0.0, iq_ref=0.0)
    controller.set_angle_observer("Measured")
    controller.set_startup_transition(enabled=False, initial_mode="Measured")
    controller.set_startup_sequence(enabled=False)
    controller._enter_startup_phase("closed_loop")
    controller.startup_transition_done = True
    controller.startup_ready_to_switch = True
    controller.set_field_weakening(
        enabled=False,
        start_speed_rpm=1e9,
        gain=0.0,
        max_negative_id_a=0.0,
    )
    controller.set_decoupling(enable_d=True, enable_q=True)
    controller.set_speed_reference(float(target_speed_rpm))

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(dt)

    n = int(sim_end_s / dt)
    speeds = np.empty(n, dtype=np.float64)
    ids = np.empty(n, dtype=np.float64)
    iqs = np.empty(n, dtype=np.float64)
    p_in = np.empty(n, dtype=np.float64)
    p_mech = np.empty(n, dtype=np.float64)

    stable = True
    for k in range(n):
        t = (k + 1) * dt
        svm.set_phase_currents(motor.currents)
        mag, ang = controller.update(dt)

        if not np.isfinite(mag) or not np.isfinite(ang):
            stable = False
            break

        vabc = np.array(svm.modulate(mag, ang), dtype=np.float64)
        ia, ib, ic = [float(x) for x in motor.currents]
        engine.step(vabc, log_data=False)

        if (not np.isfinite(motor.omega)) or abs(motor.speed_rpm) > 1e5:
            stable = False
            break

        theta_e = float((motor.theta * motor.params.poles_pairs) % (2.0 * np.pi))
        i_alpha, i_beta = clarke_transform(ia, ib, ic)
        i_d, i_q = park_transform(i_alpha, i_beta, theta_e)
        tau_load = float(load.get_torque(t))

        speeds[k] = float(motor.speed_rpm)
        ids[k] = float(i_d)
        iqs[k] = float(i_q)
        p_in[k] = float(abs(np.dot(vabc, np.array([ia, ib, ic], dtype=np.float64))))
        p_mech[k] = float(abs(tau_load * motor.omega))

    if not stable:
        return {"stable": False, "all_criteria_passed": False, "score": 1e9}

    tail = max(int(1.0 / dt), 1)
    sp_tail = speeds[-tail:]
    id_tail = ids[-tail:]
    iq_tail = iqs[-tail:]
    pin_tail = p_in[-tail:]
    pmech_tail = p_mech[-tail:]

    mean_speed = float(np.mean(sp_tail))
    mean_speed_err = mean_speed - target_speed_rpm

    id_dc = float(np.mean(id_tail))
    iq_dc = float(np.mean(iq_tail))
    flux_angle_deg = float(np.degrees(np.arctan2(abs(iq_dc), abs(id_dc) + 1e-12)))
    orth_err = float(abs(90.0 - flux_angle_deg))

    mean_pin = float(np.mean(pin_tail))
    mean_pmech = float(np.mean(pmech_tail))
    eff_pct = float(100.0 * mean_pmech / mean_pin) if mean_pin > 1e-9 else 0.0

    margins = (
        margins_for(params, cand)
        if compute_margins
        else {
            "speed_gain_margin_db": 0.0,
            "speed_phase_margin_deg": 0.0,
            "current_gain_margin_db": 0.0,
            "current_phase_margin_deg": 0.0,
        }
    )

    criteria = {
        "speed_tracking_pm_2pct": bool(
            abs(mean_speed_err) <= 0.02 * abs(target_speed_rpm)
        ),
        "orthogonality_90_pm_5deg": bool(orth_err <= 5.0),
        "speed_phase_margin_ge_45deg": bool(
            (not compute_margins) or margins["speed_phase_margin_deg"] >= 45.0
        ),
        "current_phase_margin_ge_45deg": bool(
            (not compute_margins) or margins["current_phase_margin_deg"] >= 45.0
        ),
        "speed_gain_margin_ge_6db": bool(
            (not compute_margins) or margins["speed_gain_margin_db"] >= 6.0
        ),
        "current_gain_margin_ge_6db": bool(
            (not compute_margins) or margins["current_gain_margin_db"] >= 6.0
        ),
        "efficiency_ge_85pct": bool(eff_pct >= 85.0),
    }

    efficiency_gate_active = bool(
        mean_pmech >= float(min_efficiency_mech_power_w)
        and final_load_nm >= float(min_efficiency_load_nm)
    )
    speed_tracking_passed = bool(criteria["speed_tracking_pm_2pct"])
    orthogonality_stage_passed = bool(
        criteria["speed_tracking_pm_2pct"]
        and criteria["orthogonality_90_pm_5deg"]
        and criteria["speed_phase_margin_ge_45deg"]
        and criteria["current_phase_margin_ge_45deg"]
        and criteria["speed_gain_margin_ge_6db"]
        and criteria["current_gain_margin_ge_6db"]
    )
    efficiency_conditioned_passed = bool(
        orthogonality_stage_passed
        and ((not efficiency_gate_active) or criteria["efficiency_ge_85pct"])
    )

    score = (
        2.0 * abs(mean_speed_err) / max(abs(target_speed_rpm), 1.0)
        + 1.5 * (orth_err / 5.0)
        + 2.0 * max(0.0, 85.0 - eff_pct) / 85.0
    )
    if not efficiency_conditioned_passed:
        score += 0.8

    return {
        "stable": True,
        "all_criteria_passed": efficiency_conditioned_passed,
        "speed_tracking_passed": speed_tracking_passed,
        "orthogonality_stage_passed": orthogonality_stage_passed,
        "efficiency_conditioned_passed": efficiency_conditioned_passed,
        "efficiency_gate_active": efficiency_gate_active,
        "score": float(score),
        "criteria": criteria,
        "metrics": {
            "mean_speed_rpm_last_1s": mean_speed,
            "mean_speed_error_rpm_last_1s": mean_speed_err,
            "flux_angle_deg_last_1s": flux_angle_deg,
            "orthogonality_error_deg_last_1s": orth_err,
            "efficiency_pct_last_1s": eff_pct,
            "mean_electrical_power_w_last_1s": mean_pin,
            "mean_mechanical_power_w_last_1s": mean_pmech,
            "iq_dc_a_last_1s": iq_dc,
            "id_dc_a_last_1s": id_dc,
        },
        "margins": margins,
    }


def generate_candidates(base: Candidate, span: float) -> list[Candidate]:
    speed_factors = [max(0.4, 1.0 - span), 1.0, min(2.2, 1.0 + span)]
    current_factors = [max(0.4, 1.0 - span), 1.0, min(2.2, 1.0 + span)]
    iq_factors = [1.0, min(2.0, 1.0 + 0.5 * span)]
    aw_gains = [0.15, 0.25]

    out: list[Candidate] = []
    for sf in speed_factors:
        for cf in current_factors:
            for iqf in iq_factors:
                out.append(
                    Candidate(
                        speed_kp=base.speed_kp * sf,
                        speed_ki=max(1e-6, base.speed_ki * sf),
                        current_kp=base.current_kp * cf,
                        current_ki=max(1e-6, base.current_ki * cf),
                        iq_limit_a=base.iq_limit_a * iqf,
                        use_d_priority=False,
                        coupled_aw_gain=0.0,
                    )
                )
                for aw in aw_gains:
                    out.append(
                        Candidate(
                            speed_kp=base.speed_kp * sf,
                            speed_ki=max(1e-6, base.speed_ki * sf),
                            current_kp=base.current_kp * cf,
                            current_ki=max(1e-6, base.current_ki * cf),
                            iq_limit_a=base.iq_limit_a * iqf,
                            use_d_priority=True,
                            coupled_aw_gain=aw,
                        )
                    )
    return out


def tune_point_until_success(
    params: MotorParameters,
    base: Candidate,
    target_speed_rpm: float,
    torque_nm: float,
    search_dt: float = 1.0e-3,
    search_sim_end_s: float = 2.6,
    acceptance_key: str = "all_criteria_passed",
) -> tuple[Candidate | None, dict | None, int]:
    span = 0.12
    rounds = 0
    best_cand = None
    best_eval = None
    best_score = float("inf")
    t0 = time.monotonic()

    while rounds < 16:
        rounds += 1
        cands = generate_candidates(base, span)
        found_success = False
        print(
            f"TUNE_ROUND_START round={rounds} span={span:.4f} candidates={len(cands)}",
            flush=True,
        )

        for idx, cand in enumerate(cands, start=1):
            result = evaluate_case(
                params=params,
                cand=cand,
                target_speed_rpm=target_speed_rpm,
                final_load_nm=torque_nm,
                dt=search_dt,
                sim_end_s=search_sim_end_s,
            )
            s = float(result.get("score", 1e9))
            if s < best_score:
                best_score = s
                best_cand = cand
                best_eval = result
            if idx == 1 or idx % 12 == 0 or idx == len(cands):
                elapsed_s = time.monotonic() - t0
                print(
                    (
                        "TUNE_PROGRESS "
                        f"round={rounds} cand={idx}/{len(cands)} "
                        f"best_score={best_score:.6f} elapsed_s={elapsed_s:.1f}"
                    ),
                    flush=True,
                )

            if result.get(acceptance_key, False):
                found_success = True
                best_cand = cand
                best_eval = result
                elapsed_s = time.monotonic() - t0
                print(
                    (
                        "TUNE_SUCCESS "
                        f"round={rounds} cand={idx}/{len(cands)} "
                        f"elapsed_s={elapsed_s:.1f}"
                    ),
                    flush=True,
                )
                break

        if found_success:
            return best_cand, best_eval, rounds

        span = min(0.95, span * 1.35)
        elapsed_s = time.monotonic() - t0
        print(
            f"TUNE_EXPAND span={span:.4f} elapsed_s={elapsed_s:.1f}",
            flush=True,
        )

    return best_cand, best_eval, rounds


def main() -> None:
    profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    params = to_motor_params(profile)

    session = json.loads(SESSION_PATH.read_text(encoding="utf-8"))
    best = session["best_candidate"]
    base = Candidate(
        speed_kp=float(best["speed_pi"]["kp"]),
        speed_ki=float(best["speed_pi"]["ki"]),
        current_kp=float(best["current_pi"]["d_kp"]),
        current_ki=float(best["current_pi"]["d_ki"]),
        iq_limit_a=float(best.get("iq_limit_a", 300.0)),
        use_d_priority=False,
        coupled_aw_gain=0.0,
    )

    max_speed_est_rpm = estimate_max_no_fw_speed_rpm(params, utilization_margin=0.95)
    session_target_rpm = float(
        session.get("search_setup", {}).get("effective_target_speed_rpm", 0.0)
    )
    if session_target_rpm > 0.0:
        max_speed_no_fw_rpm = min(max_speed_est_rpm, session_target_rpm)
        print(
            (
                "MAX_SPEED_TARGET "
                f"estimate={max_speed_est_rpm:.3f} "
                f"session_effective={session_target_rpm:.3f} "
                f"selected={max_speed_no_fw_rpm:.3f}"
            ),
            flush=True,
        )
    else:
        max_speed_no_fw_rpm = max_speed_est_rpm
        print(
            f"MAX_SPEED_TARGET estimate_only={max_speed_no_fw_rpm:.3f}",
            flush=True,
        )

    # Step 1: convergence/stability at maximum no-field-weakening speed.
    print("STEP1_START max-speed no-FW convergence", flush=True)
    cand_speed, eval_speed, rounds_speed = tune_point_until_success(
        params=params,
        base=base,
        target_speed_rpm=max_speed_no_fw_rpm,
        torque_nm=0.0,
        acceptance_key="speed_tracking_passed",
    )
    if cand_speed is None or eval_speed is None:
        raise RuntimeError("Unable to find stable candidate at max no-FW speed.")

    rated = profile.get("rated_info", {})
    rated_current = float(
        rated.get("rated_current_a", rated.get("rated_current_a_rms", 100.0))
    )
    rated_torque = float(
        rated.get("rated_torque_nm", params.torque_constant * rated_current)
    )
    plausible_upper = 0.70 * min(rated_torque, params.torque_constant * rated_current)

    # Step 2/3: smooth load increase and retune until success at highest plausible point.
    low = 0.0
    high = plausible_upper
    best_loaded_candidate = cand_speed
    torque_iterations = []

    print("STEP2_START smooth torque ramp and retune", flush=True)
    for _ in range(9):
        mid = 0.5 * (low + high)
        cand_mid, eval_mid, rounds_mid = tune_point_until_success(
            params=params,
            base=best_loaded_candidate,
            target_speed_rpm=max_speed_no_fw_rpm,
            torque_nm=mid,
            search_dt=1.0e-3,
            search_sim_end_s=2.4,
            acceptance_key="speed_tracking_passed",
        )
        success = bool(eval_mid and eval_mid.get("speed_tracking_passed", False))
        torque_iterations.append(
            {
                "trial_torque_nm": float(mid),
                "success": success,
                "rounds": int(rounds_mid),
                "best_score": None
                if eval_mid is None
                else float(eval_mid.get("score", 1e9)),
                "efficiency_pct": None
                if eval_mid is None
                else float(eval_mid["metrics"].get("efficiency_pct_last_1s", 0.0)),
            }
        )

        if success and cand_mid is not None and eval_mid is not None:
            low = mid
            best_loaded_candidate = cand_mid
            print(f"TORQUE_OK {mid:.4f} Nm", flush=True)
        else:
            high = mid
            print(f"TORQUE_FAIL {mid:.4f} Nm", flush=True)

    # Final tune at selected working point.
    final_torque = float(low)
    print("STEP3_START final working-point tuning", flush=True)
    final_cand, final_eval, final_rounds = tune_point_until_success(
        params=params,
        base=best_loaded_candidate,
        target_speed_rpm=max_speed_no_fw_rpm,
        torque_nm=final_torque,
        search_dt=8.0e-4,
        search_sim_end_s=3.0,
        acceptance_key="orthogonality_stage_passed",
    )
    if final_cand is None or final_eval is None:
        raise RuntimeError("Final working-point tuning failed.")

    final_eval_hifi = evaluate_case(
        params=params,
        cand=final_cand,
        target_speed_rpm=max_speed_no_fw_rpm,
        final_load_nm=final_torque,
        dt=5.0e-4,
        sim_end_s=4.5,
        compute_margins=True,
    )

    report = {
        "schema": "bldc.no_fw_loaded_point_calibration.v1",
        "motor_profile": profile.get("profile_name", PROFILE_PATH.name),
        "inputs": {
            "profile_file": str(PROFILE_PATH).replace("\\", "/"),
            "baseline_session_file": str(SESSION_PATH).replace("\\", "/"),
            "field_weakening_enabled": False,
            "target_speed_strategy": "max_no_fw_estimate",
            "acceptance_strategy": "torque_speed_first_then_orthogonality_then_conditioned_efficiency",
            "plausible_torque_upper_nm": float(plausible_upper),
        },
        "step1_max_speed_convergence": {
            "target_speed_rpm": float(max_speed_no_fw_rpm),
            "rounds": int(rounds_speed),
            "candidate": {
                "speed_kp": float(cand_speed.speed_kp),
                "speed_ki": float(cand_speed.speed_ki),
                "current_kp": float(cand_speed.current_kp),
                "current_ki": float(cand_speed.current_ki),
                "iq_limit_a": float(cand_speed.iq_limit_a),
                "use_d_priority": bool(cand_speed.use_d_priority),
                "coupled_aw_gain": float(cand_speed.coupled_aw_gain),
            },
            "result": eval_speed,
        },
        "step2_torque_ramp_search": {
            "acceptance_key": "speed_tracking_passed",
            "iterations": torque_iterations,
            "selected_plausible_achievable_torque_nm": float(final_torque),
        },
        "step3_final_working_point_tuning": {
            "acceptance_key": "orthogonality_stage_passed",
            "target_speed_rpm": float(max_speed_no_fw_rpm),
            "target_load_nm": float(final_torque),
            "rounds": int(final_rounds),
            "candidate": {
                "speed_kp": float(final_cand.speed_kp),
                "speed_ki": float(final_cand.speed_ki),
                "current_kp": float(final_cand.current_kp),
                "current_ki": float(final_cand.current_ki),
                "iq_limit_a": float(final_cand.iq_limit_a),
                "use_d_priority": bool(final_cand.use_d_priority),
                "coupled_aw_gain": float(final_cand.coupled_aw_gain),
            },
            "result_fast_search": final_eval,
            "result_high_fidelity": final_eval_hifi,
            "success": bool(final_eval_hifi.get("all_criteria_passed", False)),
            "orthogonality_success": bool(
                final_eval_hifi.get("orthogonality_stage_passed", False)
            ),
            "efficiency_gate_active": bool(
                final_eval_hifi.get("efficiency_gate_active", False)
            ),
        },
    }

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print("REPORT_SAVED", str(OUT_PATH))


if __name__ == "__main__":
    main()
