import argparse
import itertools
import json
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.control.adaptive_tuning import AdaptiveFOCTuner
from src.control.foc_controller import FOCController
from src.control.svm_generator import SVMGenerator
from src.control.transforms import clarke_transform, park_transform
from src.core.load_model import LoadProfile
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.simulation_engine import SimulationEngine


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


def build_motor_params(profile: dict) -> MotorParameters:
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


def margins_for(params, speed_kp, speed_ki, current_kp, current_ki):
    tuner = AdaptiveFOCTuner(params)
    s = tuner.analyze_speed_loop(speed_kp, speed_ki)["margin"]
    c = tuner.analyze_current_loop(current_kp, current_ki)["margin"]
    return {
        "speed_gain_margin_db": float(s.gain_margin_db),
        "speed_phase_margin_deg": float(s.phase_margin_deg),
        "current_gain_margin_db": float(c.gain_margin_db),
        "current_phase_margin_deg": float(c.phase_margin_deg),
    }


def evaluate_case(
    params: MotorParameters,
    speed_kp: float,
    speed_ki: float,
    current_kp: float,
    current_ki: float,
    iq_limit_a: float,
    target_speed_rpm: float,
    final_load_nm: float,
    use_d_priority: bool,
    coupled_aw_gain: float,
    dt: float,
    sim_end_s: float,
):
    ramp_start_s = 0.7
    ramp_end_s = 2.4

    motor = BLDCMotor(params, dt=dt)
    load = SmoothRampHoldLoad(ramp_start_s, ramp_end_s, final_load_nm)
    engine = SimulationEngine(
        motor,
        load,
        dt=dt,
        compute_backend="cpu",
        max_history=max(4000, int(sim_end_s / dt) + 200),
    )
    controller = FOCController(motor=motor, enable_speed_loop=True)

    controller.set_cascaded_speed_loop(True, iq_limit_a=float(iq_limit_a))
    controller.set_speed_pi_gains(kp=float(speed_kp), ki=float(speed_ki), kaw=0.05)
    controller.set_current_pi_gains(
        d_kp=float(current_kp),
        d_ki=float(current_ki),
        q_kp=float(current_kp),
        q_ki=float(current_ki),
        kaw=0.2,
    )
    if use_d_priority:
        controller.set_voltage_saturation(
            mode="d_priority",
            coupled_antiwindup_enabled=True,
            coupled_antiwindup_gain=float(coupled_aw_gain),
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
        return {"stable": False, "score": 1e9}

    tail = max(int(1.0 / dt), 1)
    sp_tail = speeds[-tail:]
    id_tail = ids[-tail:]
    iq_tail = iqs[-tail:]
    pin_tail = p_in[-tail:]
    pmech_tail = p_mech[-tail:]

    mean_speed = float(np.mean(sp_tail))
    mean_speed_err = mean_speed - target_speed_rpm
    speed_ratio = mean_speed / max(abs(target_speed_rpm), 1e-9)

    id_dc = float(np.mean(id_tail))
    iq_dc = float(np.mean(iq_tail))
    flux_angle_deg = float(np.degrees(np.arctan2(abs(iq_dc), abs(id_dc) + 1e-12)))
    orth_err = float(abs(90.0 - flux_angle_deg))

    mean_pin = float(np.mean(pin_tail))
    mean_pmech = float(np.mean(pmech_tail))
    eff_pct = float(100.0 * mean_pmech / mean_pin) if mean_pin > 1e-9 else 0.0

    margins = margins_for(params, speed_kp, speed_ki, current_kp, current_ki)

    criteria = {
        "orthogonality_90_pm_5deg": bool(orth_err <= 5.0),
        "speed_tracking_pm_2pct": bool(
            abs(mean_speed_err) <= 0.02 * abs(target_speed_rpm)
        ),
        "speed_phase_margin_ge_45deg": bool(margins["speed_phase_margin_deg"] >= 45.0),
        "current_phase_margin_ge_45deg": bool(
            margins["current_phase_margin_deg"] >= 45.0
        ),
        "speed_gain_margin_ge_6db": bool(margins["speed_gain_margin_db"] >= 6.0),
        "current_gain_margin_ge_6db": bool(margins["current_gain_margin_db"] >= 6.0),
        "efficiency_ge_85pct": bool(eff_pct >= 85.0),
    }

    score = (
        2.5 * abs(mean_speed_err) / max(abs(target_speed_rpm), 1.0)
        + 1.5 * (orth_err / 5.0)
        + 1.5 * max(0.0, 85.0 - eff_pct) / 85.0
    )
    if not all(criteria.values()):
        score += 0.8

    return {
        "stable": True,
        "score": float(score),
        "criteria": criteria,
        "all_criteria_passed": bool(all(criteria.values())),
        "metrics": {
            "mean_speed_rpm_last_1s": mean_speed,
            "mean_speed_error_rpm_last_1s": mean_speed_err,
            "speed_ratio_last_1s": speed_ratio,
            "flux_angle_deg_last_1s": flux_angle_deg,
            "orthogonality_error_deg_last_1s": orth_err,
            "efficiency_pct_last_1s": eff_pct,
            "mean_electrical_power_w_last_1s": mean_pin,
            "mean_mechanical_power_w_last_1s": mean_pmech,
        },
        "margins": margins,
    }


def load_gains(session_path: Path):
    session = json.loads(session_path.read_text(encoding="utf-8"))
    best = session["best_candidate"]
    return {
        "speed_kp": float(best["speed_pi"]["kp"]),
        "speed_ki": float(best["speed_pi"]["ki"]),
        "current_kp": float(best["current_pi"]["d_kp"]),
        "current_ki": float(best["current_pi"]["d_ki"]),
        "iq_limit_a": float(best.get("iq_limit_a", 300.0)),
    }


def plausible_torque_levels(profile: dict):
    rated = profile["rated_info"]
    params = build_motor_params(profile)
    rated_torque_nm = float(
        rated.get(
            "rated_torque_nm",
            params.torque_constant * float(rated.get("rated_current_a", 100.0)),
        )
    )
    rated_current_a = float(rated.get("rated_current_a", 100.0))
    torque_from_kt = params.torque_constant * rated_current_a
    base = 0.70 * min(rated_torque_nm, torque_from_kt)
    return base, [0.35 * base, 0.70 * base, 1.0 * base]


def evaluate_candidate_over_grid(
    params,
    candidate,
    speed_grid,
    load_grid,
    use_d_priority,
    coupled_aw_gain,
    dt,
    sim_end_s,
):
    case_reports = []
    all_ok = True
    agg_score = 0.0
    worst_orth = 0.0

    for speed_rpm, load_nm in itertools.product(speed_grid, load_grid):
        result = evaluate_case(
            params=params,
            speed_kp=candidate["speed_kp"],
            speed_ki=candidate["speed_ki"],
            current_kp=candidate["current_kp"],
            current_ki=candidate["current_ki"],
            iq_limit_a=candidate["iq_limit_a"],
            target_speed_rpm=float(speed_rpm),
            final_load_nm=float(load_nm),
            use_d_priority=use_d_priority,
            coupled_aw_gain=coupled_aw_gain,
            dt=dt,
            sim_end_s=sim_end_s,
        )
        case_reports.append(
            {
                "target_speed_rpm": float(speed_rpm),
                "load_nm": float(load_nm),
                "result": result,
            }
        )
        if not result.get("stable", False):
            all_ok = False
            agg_score += 10.0
            worst_orth = 90.0
            continue

        agg_score += float(result["score"])
        worst_orth = max(
            worst_orth,
            float(result["metrics"]["orthogonality_error_deg_last_1s"]),
        )
        all_ok = all_ok and bool(result.get("all_criteria_passed", False))

    return {
        "all_criteria_passed": bool(all_ok),
        "aggregate_score": float(agg_score),
        "worst_orthogonality_error_deg": float(worst_orth),
        "cases": case_reports,
    }


def run_motor(profile_path: Path, session_path: Path, args):
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    params = build_motor_params(profile)
    baseline = load_gains(session_path)

    base_torque, load_grid = plausible_torque_levels(profile)
    speed_grid = [
        float(args.base_speed_rpm * 0.5),
        float(args.base_speed_rpm),
        float(args.base_speed_rpm * 1.5),
    ]

    speed_factors = [0.9, 1.0, 1.1]
    current_factors = [0.9, 1.0, 1.1]
    iq_factors = [1.0, 1.15]
    aw_gains = [0.10, 0.20]

    candidates = []
    for sf, cf, iqf, aw in itertools.product(
        speed_factors,
        current_factors,
        iq_factors,
        aw_gains,
    ):
        candidates.append(
            {
                "speed_kp": baseline["speed_kp"] * sf,
                "speed_ki": baseline["speed_ki"] * sf,
                "current_kp": baseline["current_kp"] * cf,
                "current_ki": baseline["current_ki"] * cf,
                "iq_limit_a": baseline["iq_limit_a"] * iqf,
                "coupled_aw_gain": aw,
            }
        )

    # Always include baseline with and without robust saturation.
    candidates.insert(
        0,
        {
            "speed_kp": baseline["speed_kp"],
            "speed_ki": baseline["speed_ki"],
            "current_kp": baseline["current_kp"],
            "current_ki": baseline["current_ki"],
            "iq_limit_a": baseline["iq_limit_a"],
            "coupled_aw_gain": 0.15,
        },
    )

    best = None
    best_eval = None
    trial_summaries = []

    for idx, candidate in enumerate(candidates, start=1):
        for use_d_priority in [False, True]:
            eval_result = evaluate_candidate_over_grid(
                params=params,
                candidate=candidate,
                speed_grid=speed_grid,
                load_grid=load_grid,
                use_d_priority=use_d_priority,
                coupled_aw_gain=float(candidate["coupled_aw_gain"]),
                dt=float(args.dt),
                sim_end_s=float(args.sim_end_s),
            )

            trial = {
                "trial_index": idx,
                "use_d_priority": bool(use_d_priority),
                "candidate": candidate,
                "all_criteria_passed": eval_result["all_criteria_passed"],
                "aggregate_score": eval_result["aggregate_score"],
                "worst_orthogonality_error_deg": eval_result[
                    "worst_orthogonality_error_deg"
                ],
            }
            trial_summaries.append(trial)

            if best_eval is None:
                best = {
                    "candidate": candidate,
                    "use_d_priority": bool(use_d_priority),
                }
                best_eval = eval_result
                continue

            # Prefer full pass, then lower aggregate score.
            prefer = False
            if (
                eval_result["all_criteria_passed"]
                and not best_eval["all_criteria_passed"]
            ):
                prefer = True
            elif eval_result["all_criteria_passed"] == best_eval["all_criteria_passed"]:
                if eval_result["aggregate_score"] < best_eval["aggregate_score"]:
                    prefer = True

            if prefer:
                best = {
                    "candidate": candidate,
                    "use_d_priority": bool(use_d_priority),
                }
                best_eval = eval_result

    return {
        "profile": profile["profile_name"],
        "profile_file": str(profile_path).replace("\\", "/"),
        "session_file": str(session_path).replace("\\", "/"),
        "base_plausible_torque_nm": float(base_torque),
        "speed_grid_rpm": speed_grid,
        "load_grid_nm": load_grid,
        "baseline_candidate": baseline,
        "search_trials": trial_summaries,
        "best": best,
        "best_eval": best_eval,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-speed-rpm", type=float, default=1500.0)
    parser.add_argument("--dt", type=float, default=5e-4)
    parser.add_argument("--sim-end-s", type=float, default=3.8)
    parser.add_argument(
        "--out",
        type=str,
        default="data/logs/multi_motor_loaded_orthogonality_report.json",
    )
    args = parser.parse_args()

    motors = [
        (
            Path("data/motor_profiles/motenergy_me1718_48v.json"),
            Path(
                "data/tuning_sessions/until_converged/motenergy_me1718_48v_until_converged.json"
            ),
        ),
        (
            Path("data/motor_profiles/motenergy_me1719_48v.json"),
            Path(
                "data/tuning_sessions/until_converged/motenergy_me1719_48v_until_converged.json"
            ),
        ),
        (
            Path("data/motor_profiles/innotec_255_ezs48_160.json"),
            Path(
                "data/tuning_sessions/until_converged/innotec_255_ezs48_160_until_converged.json"
            ),
        ),
    ]

    results = []
    for profile_path, session_path in motors:
        if not profile_path.exists() or not session_path.exists():
            results.append(
                {
                    "profile_file": str(profile_path),
                    "session_file": str(session_path),
                    "error": "missing_profile_or_session",
                }
            )
            continue
        results.append(run_motor(profile_path, session_path, args))

    overall_all_pass = all(
        r.get("best_eval", {}).get("all_criteria_passed", False)
        for r in results
        if "best_eval" in r
    )

    report = {
        "constraints": {
            "orthogonality": "90 +/- 5 deg",
            "speed_tracking": "+/- 2%",
            "gain_margin_db": ">= 6",
            "phase_margin_deg": ">= 45",
            "efficiency_pct": ">= 85",
            "field_weakening": "disabled",
        },
        "global_settings": {
            "base_speed_rpm": args.base_speed_rpm,
            "speed_grid_relative": [0.5, 1.0, 1.5],
            "load_grid_relative": [0.35, 0.70, 1.0],
            "dt": args.dt,
            "sim_end_s": args.sim_end_s,
        },
        "overall_all_pass": bool(overall_all_pass),
        "motors": results,
    }

    out_path = Path(args.out)
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print("REPORT_SAVED", str(out_path))


if __name__ == "__main__":
    main()
