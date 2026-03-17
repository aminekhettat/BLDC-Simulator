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
    def __init__(self, start_s, end_s, final_torque):
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


def simulate(params, gains, iq_limit_a, target_speed_rpm, plausible_torque_nm):
    dt = 3e-4
    sim_end_s = 6.0
    ramp_start_s = 1.2
    ramp_end_s = 3.0

    motor = BLDCMotor(params, dt=dt)
    load = SmoothRampHoldLoad(ramp_start_s, ramp_end_s, plausible_torque_nm)
    engine = SimulationEngine(
        motor,
        load,
        dt=dt,
        compute_backend="cpu",
        max_history=8000,
    )
    controller = FOCController(motor=motor, enable_speed_loop=True)

    controller.set_cascaded_speed_loop(True, iq_limit_a=float(iq_limit_a))
    controller.set_speed_pi_gains(
        kp=float(gains["speed_kp"]), ki=float(gains["speed_ki"]), kaw=0.05
    )
    controller.set_current_pi_gains(
        d_kp=float(gains["current_kp"]),
        d_ki=float(gains["current_ki"]),
        q_kp=float(gains["current_kp"]),
        q_ki=float(gains["current_ki"]),
        kaw=0.2,
    )
    controller.set_current_references(id_ref=0.0, iq_ref=0.0)
    controller.set_angle_observer("Measured")
    controller.set_startup_transition(enabled=False, initial_mode="Measured")
    controller.set_startup_sequence(enabled=False)
    controller._enter_startup_phase("closed_loop")
    controller.startup_transition_done = True
    controller.startup_ready_to_switch = True
    controller.set_field_weakening(
        enabled=False, start_speed_rpm=1e9, gain=0.0, max_negative_id_a=0.0
    )
    controller.set_decoupling(enable_d=True, enable_q=True)
    controller.set_speed_reference(float(target_speed_rpm))

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(dt)

    n = int(sim_end_s / dt)
    speeds = np.empty(n, dtype=np.float64)
    ids = np.empty(n, dtype=np.float64)
    iqs = np.empty(n, dtype=np.float64)
    loads = np.empty(n, dtype=np.float64)
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
        loads[k] = tau_load
        p_in[k] = float(abs(np.dot(vabc, np.array([ia, ib, ic], dtype=np.float64))))
        p_mech[k] = float(abs(tau_load * motor.omega))

    if not stable:
        return {
            "stable": False,
            "score": 1e9,
        }

    st = int((sim_end_s - 1.0) / dt)
    sp_tail = speeds[st:]
    id_tail = ids[st:]
    iq_tail = iqs[st:]
    load_tail = loads[st:]
    pin_tail = p_in[st:]
    pmech_tail = p_mech[st:]

    mean_speed = float(np.mean(sp_tail))
    mean_speed_err = float(mean_speed - target_speed_rpm)
    speed_ratio = float(mean_speed / target_speed_rpm)

    id_dc = float(np.mean(id_tail))
    iq_dc = float(np.mean(iq_tail))
    flux_angle_deg = float(np.degrees(np.arctan2(abs(iq_dc), abs(id_dc) + 1e-12)))
    orth_err = float(abs(90.0 - flux_angle_deg))

    mean_pin = float(np.mean(pin_tail))
    mean_pmech = float(np.mean(pmech_tail))
    eff_pct = float(100.0 * mean_pmech / mean_pin) if mean_pin > 1e-9 else 0.0

    margins = margins_for(
        params,
        gains["speed_kp"],
        gains["speed_ki"],
        gains["current_kp"],
        gains["current_ki"],
    )

    criteria = {
        "angle_90_plus_minus_2deg": bool(orth_err <= 2.0),
        "speed_tracking_plus_minus_2pct": bool(
            abs(mean_speed_err) <= 0.02 * target_speed_rpm
        ),
        "speed_phase_margin_ge_45deg": bool(margins["speed_phase_margin_deg"] >= 45.0),
        "current_phase_margin_ge_45deg": bool(
            margins["current_phase_margin_deg"] >= 45.0
        ),
        "speed_gain_margin_ge_6db": bool(margins["speed_gain_margin_db"] >= 6.0),
        "current_gain_margin_ge_6db": bool(margins["current_gain_margin_db"] >= 6.0),
        "efficiency_ge_85pct": bool(eff_pct >= 85.0),
    }

    # lower is better
    score = (
        2.5 * abs(mean_speed_err) / max(target_speed_rpm, 1.0)
        + 0.8 * (orth_err / 10.0)
        + 0.7 * max(0.0, 85.0 - eff_pct) / 85.0
    )
    if speed_ratio < 0.0:
        score += 5.0
    if not all(criteria.values()):
        score += 0.5

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
            "mean_tail_load_nm": float(np.mean(load_tail)),
        },
        "margins": margins,
    }


def main():
    profile = json.loads(
        Path("data/motor_profiles/motenergy_me1718_48v.json").read_text(
            encoding="utf-8"
        )
    )
    session = json.loads(
        Path(
            "data/tuning_sessions/until_converged/motenergy_me1718_48v_until_converged.json"
        ).read_text(encoding="utf-8")
    )

    mp = profile["motor_params"]
    rated = profile["rated_info"]

    params = MotorParameters(
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

    best = session["best_candidate"]
    baseline_gains = {
        "speed_kp": float(best["speed_pi"]["kp"]),
        "speed_ki": float(best["speed_pi"]["ki"]),
        "current_kp": float(best["current_pi"]["d_kp"]),
        "current_ki": float(best["current_pi"]["d_ki"]),
    }
    iq_limit_a = float(best.get("iq_limit_a", 300.0))

    target_speed_rpm = 1500.0
    rated_torque_nm = float(
        rated.get(
            "rated_torque_nm",
            params.torque_constant * float(rated.get("rated_current_a", 100.0)),
        )
    )
    rated_current_a = float(rated.get("rated_current_a", 100.0))
    torque_from_kt = params.torque_constant * rated_current_a
    plausible_torque_nm = 0.70 * min(rated_torque_nm, torque_from_kt)

    baseline = simulate(
        params,
        baseline_gains,
        iq_limit_a,
        target_speed_rpm,
        plausible_torque_nm,
    )

    speed_factors = [0.8, 1.0, 1.2]
    current_factors = [0.85, 1.0, 1.15]

    best_speed_gains = dict(baseline_gains)
    best_speed_eval = dict(baseline)
    speed_trials = []

    for skp_f, ski_f in itertools.product(speed_factors, speed_factors):
        candidate = dict(baseline_gains)
        candidate["speed_kp"] = baseline_gains["speed_kp"] * skp_f
        candidate["speed_ki"] = baseline_gains["speed_ki"] * ski_f
        ev = simulate(
            params, candidate, iq_limit_a, target_speed_rpm, plausible_torque_nm
        )
        speed_trials.append(
            {
                "speed_kp_factor": skp_f,
                "speed_ki_factor": ski_f,
                "score": ev["score"],
                "all_criteria_passed": ev.get("all_criteria_passed", False),
            }
        )
        if ev["score"] < best_speed_eval["score"]:
            best_speed_eval = ev
            best_speed_gains = candidate

    final_best_gains = dict(best_speed_gains)
    final_best_eval = dict(best_speed_eval)
    current_trials = []

    for ckp_f, cki_f in itertools.product(current_factors, current_factors):
        candidate = dict(best_speed_gains)
        candidate["current_kp"] = best_speed_gains["current_kp"] * ckp_f
        candidate["current_ki"] = best_speed_gains["current_ki"] * cki_f
        ev = simulate(
            params, candidate, iq_limit_a, target_speed_rpm, plausible_torque_nm
        )
        current_trials.append(
            {
                "current_kp_factor": ckp_f,
                "current_ki_factor": cki_f,
                "score": ev["score"],
                "all_criteria_passed": ev.get("all_criteria_passed", False),
            }
        )
        if ev["score"] < final_best_eval["score"]:
            final_best_eval = ev
            final_best_gains = candidate

    report = {
        "motor": profile["profile_name"],
        "target_speed_rpm": target_speed_rpm,
        "load_profile": {
            "type": "smoothstep_ramp_then_hold",
            "calculated_plausible_load_nm": plausible_torque_nm,
            "calc_basis": {
                "rated_torque_nm": rated_torque_nm,
                "kt_times_rated_current_nm": torque_from_kt,
                "factor": 0.70,
            },
        },
        "baseline_gains": baseline_gains,
        "baseline_eval": baseline,
        "search_strategy": {
            "stage_1": "speed_pi_grid_search_under_load",
            "stage_2": "current_pi_grid_search_under_load",
            "speed_factors": speed_factors,
            "current_factors": current_factors,
            "speed_trials_count": len(speed_trials),
            "current_trials_count": len(current_trials),
        },
        "best_gains_under_load": final_best_gains,
        "best_eval_under_load": final_best_eval,
        "all_criteria_passed": final_best_eval.get("all_criteria_passed", False),
    }

    out_path = Path("data/logs/me1718_loaded_autotune_search_report.json")
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print("REPORT_SAVED", str(out_path))


if __name__ == "__main__":
    main()
