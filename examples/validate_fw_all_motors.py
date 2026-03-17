"""
Multi-Motor Field Weakening Validation
Tests FW calibration on all available motor profiles in the project.
Results are aggregated into a comprehensive validation report.
"""

import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.control.foc_controller import FOCController
from src.control.svm_generator import SVMGenerator
from src.control.transforms import clarke_transform, park_transform
from src.core.load_model import LoadProfile
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.simulation_engine import SimulationEngine


class RampLoad(LoadProfile):
    def __init__(self, start: float, end: float, torque: float):
        self.start = float(start)
        self.end = float(end)
        self.torque = float(torque)

    def get_torque(self, t: float) -> float:
        if t <= self.start:
            return 0.0
        if t >= self.end:
            return self.torque
        x = (t - self.start) / max(self.end - self.start, 1e-9)
        s = x * x * (3.0 - 2.0 * x)
        return self.torque * s


@dataclass
class MotorTestConfig:
    profile_file: str
    session_file: str
    target_speed_rpm: float
    target_torque_nm: float


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
        poles_pairs=int(mp.get("poles_pairs", 3)),
        ld=float(mp.get("ld", mp["phase_inductance"])),
        lq=float(mp.get("lq", mp["phase_inductance"])),
    )


def evaluate_fw_operation(
    params,
    target_rpm,
    load_torque,
    speed_kp,
    speed_ki,
    fw_start_rpm,
    fw_gain,
    fw_id_max,
    fw_headroom,
    sim_duration=5.0,
    dt=1e-3,
):
    """Evaluate FW operation at target speed and load"""
    motor = BLDCMotor(params, dt=dt)
    load = RampLoad(0.5, 1.5, load_torque)
    engine = SimulationEngine(motor, load, dt=dt, compute_backend="cpu")
    ctrl = FOCController(motor=motor, enable_speed_loop=True)

    ctrl.set_cascaded_speed_loop(True, iq_limit_a=120.0)
    ctrl.set_speed_pi_gains(kp=speed_kp, ki=speed_ki, kaw=0.05)
    ctrl.set_current_pi_gains(d_kp=25.0, d_ki=25.0, q_kp=25.0, q_ki=25.0, kaw=0.2)
    ctrl.set_voltage_saturation(
        mode="d_priority",
        coupled_antiwindup_enabled=True,
        coupled_antiwindup_gain=0.3,
    )
    ctrl.set_current_references(id_ref=0.0, iq_ref=0.0)
    ctrl.set_angle_observer("Measured")
    ctrl.set_startup_transition(enabled=False, initial_mode="Measured")
    ctrl.set_startup_sequence(enabled=False)
    ctrl._enter_startup_phase("closed_loop")
    ctrl.startup_transition_done = True
    ctrl.startup_ready_to_switch = True

    ctrl.set_field_weakening(
        enabled=True,
        start_speed_rpm=fw_start_rpm,
        gain=fw_gain,
        max_negative_id_a=fw_id_max,
        headroom_target_v=fw_headroom,
    )
    ctrl.set_decoupling(enable_d=True, enable_q=True)
    ctrl.set_speed_reference(target_rpm)

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(dt)

    n = int(sim_duration / dt)
    speeds = np.zeros(n)
    s_err = np.zeros(n)
    ids = np.zeros(n)
    iqs = np.zeros(n)
    fw_inj = np.zeros(n)
    p_in = np.zeros(n)
    p_out = np.zeros(n)

    for k in range(n):
        t = (k + 1) * dt
        svm.set_phase_currents(motor.currents)
        mag, ang = ctrl.update(dt)

        if not np.isfinite(mag) or not np.isfinite(ang):
            return None

        vabc = np.array(svm.modulate(mag, ang), dtype=np.float64)
        ia, ib, ic = [float(x) for x in motor.currents]
        engine.step(vabc, log_data=False)

        if abs(motor.speed_rpm) > 1e5 or not np.isfinite(motor.omega):
            return None

        theta_e = float((motor.theta * motor.params.poles_pairs) % (2 * np.pi))
        i_a, i_b = clarke_transform(ia, ib, ic)
        i_d, i_q = park_transform(i_a, i_b, theta_e)
        tau_load = load.get_torque(t)

        speeds[k] = float(motor.speed_rpm)
        s_err[k] = float(motor.speed_rpm - target_rpm)
        ids[k] = float(i_d)
        iqs[k] = float(i_q)
        fw_inj[k] = float(ctrl.field_weakening_id_injection_a)
        p_in[k] = float(abs(np.dot(vabc, np.array([ia, ib, ic], dtype=np.float64))))
        p_out[k] = float(abs(tau_load * motor.omega))

    tail_idx = max(int(1.0 / dt), 1)
    tail = slice(-tail_idx, None)

    speed_mean = float(np.mean(speeds[tail]))
    speed_err_mean = float(np.mean(s_err[tail]))
    speed_err_pct = 100.0 * speed_err_mean / max(target_rpm, 1)
    id_mean = float(np.mean(ids[tail]))
    iq_mean = float(np.mean(iqs[tail]))
    fw_mean = float(np.mean(fw_inj[tail]))
    p_in_mean = float(np.mean(p_in[tail]))
    p_out_mean = float(np.mean(p_out[tail]))
    eff_pct = 100.0 * p_out_mean / max(p_in_mean, 0.1)

    fw_active = fw_mean < -0.1
    speed_ok = abs(speed_err_pct) < 5.0
    eff_ok = eff_pct > 75.0

    status = "PASS" if (speed_ok and fw_active and eff_ok) else "FAIL"

    return {
        "status": status,
        "speed_rpm": speed_mean,
        "speed_error_rpm": speed_err_mean,
        "speed_error_pct": speed_err_pct,
        "id_a": id_mean,
        "iq_a": iq_mean,
        "fw_injection_a": fw_mean,
        "fw_active": fw_active,
        "efficiency_pct": eff_pct,
        "p_in_w": p_in_mean,
        "p_out_w": p_out_mean,
    }


def validate_motor(motor_name: str, profile_path: Path, session_path: Path):
    """Validate FW calibration for a single motor"""
    print(f"\n{'=' * 70}")
    print(f"Testing: {motor_name}")
    print(f"{'=' * 70}")

    profile = json.loads(profile_path.read_text())
    session = json.loads(session_path.read_text())

    params = to_motor_params(profile)
    params.flux_weakening_id_coefficient = 0.016
    params.flux_weakening_min_ratio = 0.08

    best = session["best_candidate"]
    rated_info = profile.get("rated_info", {})
    target_speed = float(rated_info.get("rated_speed_rpm", 4000.0))
    target_torque = float(
        rated_info.get(
            "rated_torque_nm",
            params.torque_constant * float(rated_info.get("rated_current_a", 60.0)),
        )
    )

    print(f"Profile: {profile.get('profile_name', motor_name)}")
    print(f"Target: {target_speed:.0f} RPM, {target_torque:.2f} Nm load")

    result = evaluate_fw_operation(
        params=params,
        target_rpm=target_speed,
        load_torque=target_torque * 0.7,  # 70% rated
        speed_kp=float(best["speed_pi"]["kp"]) * 0.10,
        speed_ki=float(best["speed_pi"]["ki"]) * 0.10,
        fw_start_rpm=1100.0,
        fw_gain=1.1,
        fw_id_max=14.0,
        fw_headroom=1.0,
        sim_duration=5.0,
        dt=1e-3,
    )

    if result is None:
        print("❌ FAILED: Simulation unstable")
        return None

    print(f"\nResults:")
    print(
        f"  Speed: {result['speed_rpm']:.1f} RPM (error: {result['speed_error_rpm']:+.1f} RPM)"
    )
    print(f"  Speed Error: {result['speed_error_pct']:+.2f}%")
    print(f"  d-axis (flux): {result['id_a']:.2f} A")
    print(f"  q-axis (torque): {result['iq_a']:.2f} A")
    print(f"  FW Injection: {result['fw_injection_a']:.2f} A")
    print(f"  Efficiency: {result['efficiency_pct']:.1f}%")
    print(f"  Input Power: {result['p_in_w']:.0f} W")
    print(f"  Output Power: {result['p_out_w']:.0f} W")
    print(f"\nStatus: {result['status']}")

    return result


def main():
    motor_configs = [
        MotorTestConfig(
            profile_file="motenergy_me1718_48v.json",
            session_file="motenergy_me1718_48v_until_converged.json",
            target_speed_rpm=4000.0,
            target_torque_nm=10.0,
        ),
        MotorTestConfig(
            profile_file="motenergy_me1719_48v.json",
            session_file="motenergy_me1719_48v_until_converged.json",
            target_speed_rpm=4000.0,
            target_torque_nm=10.5,
        ),
        MotorTestConfig(
            profile_file="innotec_255_ezs48_160.json",
            session_file="innotec_255_ezs48_160_until_converged.json",
            target_speed_rpm=4000.0,
            target_torque_nm=9.8,
        ),
    ]

    profiles_dir = PROJECT_ROOT / "data" / "motor_profiles"
    sessions_dir = PROJECT_ROOT / "data" / "tuning_sessions" / "until_converged"
    logs_dir = PROJECT_ROOT / "data" / "logs"

    results = {}
    t0 = time.monotonic()

    for config in motor_configs:
        profile_path = profiles_dir / config.profile_file
        session_path = sessions_dir / config.session_file

        if not profile_path.exists() or not session_path.exists():
            print(f"⚠️  Skipping {config.profile_file}: files not found")
            continue

        motor_name = config.profile_file.replace(".json", "").replace("_", " ").title()
        result = validate_motor(motor_name, profile_path, session_path)

        if result:
            results[config.profile_file] = result

    elapsed = time.monotonic() - t0

    # Summary report
    print(f"\n{'=' * 70}")
    print(f"VALIDATION SUMMARY (elapsed: {elapsed:.1f}s)")
    print(f"{'=' * 70}")

    passed = sum(1 for r in results.values() if r["status"] == "PASS")
    total = len(results)

    for motor_file, result in results.items():
        status_icon = "✅" if result["status"] == "PASS" else "❌"
        print(f"{status_icon} {motor_file}: {result['status']}")
        print(
            f"    Speed: {result['speed_rpm']:.1f} RPM (error: {result['speed_error_pct']:+.2f}%)"
        )
        print(
            f"    FW: {result['fw_injection_a']:.2f} A, Eff: {result['efficiency_pct']:.1f}%"
        )

    print(f"\nOverall: {passed}/{total} motors passed FW validation")

    report = {
        "schema": "bldc.fw_multi_motor_validation.v1",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "elapsed_seconds": elapsed,
        "motors_tested": len(results),
        "motors_passed": passed,
        "results": results,
        "status": "PASS" if passed == total else "FAIL",
    }

    report_path = logs_dir / "fw_validation_all_motors.json"
    report_path.write_text(json.dumps(report, indent=2))
    print(f"\nReport: {report_path.name}")

    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
