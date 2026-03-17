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
    init_speed_kp = float(best["speed_pi"]["kp"])
    init_speed_ki = float(best["speed_pi"]["ki"])
    init_curr_kp = float(best["current_pi"]["d_kp"])
    init_curr_ki = float(best["current_pi"]["d_ki"])
    init_iq_limit = float(best.get("iq_limit_a", 200.0))

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

    no_load_time_s = 2.0
    ramp_duration_s = 3.0
    ramp_start_s = no_load_time_s
    ramp_end_s = ramp_start_s + ramp_duration_s
    sim_end_s = 10.0
    retune_check_s = 6.5

    dt = 1e-4
    motor = BLDCMotor(params, dt=dt)
    load = SmoothRampHoldLoad(ramp_start_s, ramp_end_s, plausible_torque_nm)
    engine = SimulationEngine(
        motor, load, dt=dt, compute_backend="cpu", max_history=12000
    )
    controller = FOCController(motor=motor, enable_speed_loop=True)

    controller.set_cascaded_speed_loop(True, iq_limit_a=init_iq_limit)
    controller.set_speed_pi_gains(kp=init_speed_kp, ki=init_speed_ki, kaw=0.05)
    controller.set_current_pi_gains(
        d_kp=init_curr_kp,
        d_ki=init_curr_ki,
        q_kp=init_curr_kp,
        q_ki=init_curr_ki,
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
    controller.set_speed_reference(target_speed_rpm)

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(dt)

    initial_margins = margins_for(
        params, init_speed_kp, init_speed_ki, init_curr_kp, init_curr_ki
    )

    retuned = False
    retuned_at_s = None
    new_gains = None

    n = int(sim_end_s / dt)
    speeds = np.empty(n, dtype=np.float64)
    ids = np.empty(n, dtype=np.float64)
    iqs = np.empty(n, dtype=np.float64)
    loads = np.empty(n, dtype=np.float64)
    p_in = np.empty(n, dtype=np.float64)
    p_mech = np.empty(n, dtype=np.float64)

    for k in range(n):
        t = (k + 1) * dt

        if (not retuned) and t >= retune_check_s:
            st = max(0, k - int(1.0 / dt))
            speed_tail = speeds[st:k] if k > st else np.array([0.0])
            id_tail = ids[st:k] if k > st else np.array([0.0])
            iq_tail = iqs[st:k] if k > st else np.array([1.0])

            speed_ok = (
                abs(float(np.mean(speed_tail)) - target_speed_rpm)
                <= 0.02 * target_speed_rpm
            )
            flux_angle_deg = float(
                np.degrees(
                    np.arctan2(
                        abs(float(np.mean(iq_tail))),
                        abs(float(np.mean(id_tail))) + 1e-12,
                    )
                )
            )
            ortho_ok = abs(90.0 - flux_angle_deg) <= 2.0
            margins_ok = (
                initial_margins["speed_phase_margin_deg"] >= 45.0
                and initial_margins["current_phase_margin_deg"] >= 45.0
                and initial_margins["speed_gain_margin_db"] >= 6.0
                and initial_margins["current_gain_margin_db"] >= 6.0
            )

            if not (speed_ok and ortho_ok and margins_ok):
                tuned = AdaptiveFOCTuner(params).tune(grid_size=9)
                controller.set_speed_pi_gains(
                    kp=tuned.speed_kp, ki=tuned.speed_ki, kaw=0.05
                )
                controller.set_current_pi_gains(
                    d_kp=tuned.current_kp,
                    d_ki=tuned.current_ki,
                    q_kp=tuned.current_kp,
                    q_ki=tuned.current_ki,
                    kaw=0.2,
                )
                new_gains = {
                    "speed_kp": float(tuned.speed_kp),
                    "speed_ki": float(tuned.speed_ki),
                    "current_kp": float(tuned.current_kp),
                    "current_ki": float(tuned.current_ki),
                }
                retuned = True
                retuned_at_s = float(t)

        svm.set_phase_currents(motor.currents)
        mag, ang = controller.update(dt)
        vabc = np.array(svm.modulate(mag, ang), dtype=np.float64)
        ia, ib, ic = [float(x) for x in motor.currents]

        engine.step(vabc, log_data=False)

        theta_e = float((motor.theta * motor.params.poles_pairs) % (2.0 * np.pi))
        i_alpha, i_beta = clarke_transform(ia, ib, ic)
        i_d, i_q = park_transform(i_alpha, i_beta, theta_e)
        tau_load = float(load.get_torque(t))

        speeds[k] = float(motor.speed_rpm)
        ids[k] = float(i_d)
        iqs[k] = float(i_q)
        loads[k] = tau_load
        p_in[k] = float(np.dot(vabc, np.array([ia, ib, ic], dtype=np.float64)))
        p_mech[k] = float(tau_load * motor.omega)

    st = int((sim_end_s - 1.5) / dt)
    sp_tail = speeds[st:]
    id_tail = ids[st:]
    iq_tail = iqs[st:]
    load_tail = loads[st:]
    pin_tail = np.clip(p_in[st:], 0.0, None)
    pmech_tail = np.clip(p_mech[st:], 0.0, None)

    mean_speed = float(np.mean(sp_tail))
    mean_speed_err = mean_speed - target_speed_rpm
    final_ratio = mean_speed / target_speed_rpm
    id_dc = float(np.mean(id_tail))
    iq_dc = float(np.mean(iq_tail))
    flux_angle_deg = float(np.degrees(np.arctan2(abs(iq_dc), abs(id_dc) + 1e-12)))
    orth_err = float(abs(90.0 - flux_angle_deg))
    mean_pin = float(np.mean(pin_tail))
    mean_pmech = float(np.mean(pmech_tail))
    eff_pct = float(100.0 * mean_pmech / mean_pin) if mean_pin > 1e-6 else 0.0

    if new_gains:
        final_margins = margins_for(
            params,
            new_gains["speed_kp"],
            new_gains["speed_ki"],
            new_gains["current_kp"],
            new_gains["current_ki"],
        )
    else:
        final_margins = initial_margins

    criteria = {
        "angle_90_plus_minus_2deg": bool(orth_err <= 2.0),
        "speed_tracking_plus_minus_2pct": bool(
            abs(mean_speed_err) <= 0.02 * target_speed_rpm
        ),
        "speed_phase_margin_ge_45deg": bool(
            final_margins["speed_phase_margin_deg"] >= 45.0
        ),
        "current_phase_margin_ge_45deg": bool(
            final_margins["current_phase_margin_deg"] >= 45.0
        ),
        "speed_gain_margin_ge_6db": bool(final_margins["speed_gain_margin_db"] >= 6.0),
        "current_gain_margin_ge_6db": bool(
            final_margins["current_gain_margin_db"] >= 6.0
        ),
        "efficiency_ge_85pct": bool(eff_pct >= 85.0),
    }

    report = {
        "motor": profile["profile_name"],
        "target_speed_rpm": target_speed_rpm,
        "load_profile": {
            "type": "smoothstep_ramp_then_hold",
            "ramp_start_s": ramp_start_s,
            "ramp_end_s": ramp_end_s,
            "calculated_plausible_load_nm": plausible_torque_nm,
            "calc_basis": {
                "rated_torque_nm": rated_torque_nm,
                "kt_times_rated_current_nm": torque_from_kt,
                "factor": 0.70,
            },
            "mean_tail_load_nm": float(np.mean(load_tail)),
        },
        "initial_gains": {
            "speed_kp": init_speed_kp,
            "speed_ki": init_speed_ki,
            "current_kp": init_curr_kp,
            "current_ki": init_curr_ki,
            "iq_limit_a": init_iq_limit,
        },
        "retune_applied_under_load": retuned,
        "retune_time_s": retuned_at_s,
        "retuned_gains": new_gains,
        "final_metrics": {
            "mean_speed_rpm_last_1p5s": mean_speed,
            "mean_speed_error_rpm_last_1p5s": mean_speed_err,
            "speed_ratio_last_1p5s": final_ratio,
            "flux_angle_deg_last_1p5s": flux_angle_deg,
            "orthogonality_error_deg_last_1p5s": orth_err,
            "efficiency_pct_last_1p5s": eff_pct,
            "mean_electrical_power_w_last_1p5s": mean_pin,
            "mean_mechanical_power_w_last_1p5s": mean_pmech,
        },
        "margins_initial": initial_margins,
        "margins_final": final_margins,
        "criteria": criteria,
        "all_criteria_passed": bool(all(criteria.values())),
    }

    out_path = Path("data/logs/me1718_loaded_autotune_experiment.json")
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print("REPORT_SAVED", str(out_path))


if __name__ == "__main__":
    main()
