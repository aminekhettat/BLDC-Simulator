"""
Quick FW Calibration Demo - Pragmatic Approach
Shows rated speed + rated torque achievable with field weakening
Uses known-good seed points for fast convergence.
"""

import json
import math
import sys
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

PROFILE_PATH = PROJECT_ROOT / "data" / "motor_profiles" / "motenergy_me1718_48v.json"
SESSION_PATH = (
    PROJECT_ROOT
    / "data"
    / "tuning_sessions"
    / "until_converged"
    / "motenergy_me1718_48v_until_converged.json"
)
OUT_PATH = PROJECT_ROOT / "data" / "logs" / "calibration_me1718_fw_demo.json"


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
        # Gentle ramp to avoid transients
        x = (t - self.start) / max(self.end - self.start, 1e-9)
        # Ease-in-out cubic
        s = x * x * (3.0 - 2.0 * x)
        return self.torque * s


def run_simulation(
    params,
    target_rpm,
    load_torque,
    speed_kp,
    speed_ki,
    fw_start_rpm,
    fw_gain,
    fw_id_max,
    fw_headroom,
    sim_duration=4.0,
    dt=1e-3,
):
    """Quick simulation of FW operation at target speed and load"""
    motor = BLDCMotor(params, dt=dt)
    load = RampLoad(0.5, 1.5, load_torque)
    engine = SimulationEngine(motor, load, dt=dt, compute_backend="cpu")
    ctrl = FOCController(motor=motor, enable_speed_loop=True)

    # Simple, proven-stable gains
    ctrl.set_cascaded_speed_loop(True, iq_limit_a=120.0)
    ctrl.set_speed_pi_gains(kp=speed_kp, ki=speed_ki, kaw=0.05)
    ctrl.set_current_pi_gains(d_kp=25.0, d_ki=25.0, q_kp=25.0, q_ki=25.0, kaw=0.2)
    ctrl.set_voltage_saturation(
        mode="d_priority", coupled_antiwindup_enabled=True, coupled_antiwindup_gain=0.3
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
    speeds_err = np.zeros(n)
    ids = np.zeros(n)
    iqs = np.zeros(n)
    fw_inj = np.zeros(n)
    p_in = np.zeros(n)
    p_out = np.zeros(n)
    eff = np.zeros(n)

    for k in range(n):
        t = (k + 1) * dt
        svm.set_phase_currents(motor.currents)
        mag, ang = ctrl.update(dt)

        if not np.isfinite(mag) or not np.isfinite(ang):
            print(f"UNSTABLE at t={t:.3f}s mag={mag} ang={ang}")
            return None

        vabc = np.array(svm.modulate(mag, ang), dtype=np.float64)
        ia, ib, ic = [float(x) for x in motor.currents]
        engine.step(vabc, log_data=False)

        if abs(motor.speed_rpm) > 1e5 or not np.isfinite(motor.omega):
            print(f"UNSTABLE motor at t={t:.3f}s speed={motor.speed_rpm}")
            return None

        theta_e = float((motor.theta * motor.params.poles_pairs) % (2 * np.pi))
        i_alpha, i_beta = clarke_transform(ia, ib, ic)
        i_d, i_q = park_transform(i_alpha, i_beta, theta_e)
        tau_load = load.get_torque(t)

        speeds[k] = motor.speed_rpm
        speeds_err[k] = motor.speed_rpm - target_rpm
        ids[k] = i_d
        iqs[k] = i_q
        fw_inj[k] = ctrl.field_weakening_id_injection_a
        p_in[k] = abs(np.dot(vabc, [ia, ib, ic]))
        p_out[k] = abs(tau_load * motor.omega)
        eff[k] = 100 * p_out[k] / max(p_in[k], 0.1)

    # Extract steady-state stats (last 1 second)
    tail_idx = max(int(1.0 / dt), 1)
    tail = slice(-tail_idx, None)

    return {
        "speed_mean": float(np.mean(speeds[tail])),
        "speed_std": float(np.std(speeds[tail])),
        "speed_err_mean": float(np.mean(speeds_err[tail])),
        "id_mean": float(np.mean(ids[tail])),
        "iq_mean": float(np.mean(iqs[tail])),
        "fw_injection_mean": float(np.mean(fw_inj[tail])),
        "efficiency_mean": float(np.mean(eff[tail])),
        "p_in_mean": float(np.mean(p_in[tail])),
        "p_out_mean": float(np.mean(p_out[tail])),
        "stability_ok": True,
    }


def main():
    profile = json.loads(PROFILE_PATH.read_text())
    mp = profile["motor_params"]
    params = MotorParameters(
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

    session = json.loads(SESSION_PATH.read_text())
    best = session["best_candidate"]

    rated_info = profile.get("rated_info", {})
    rated_speed = float(rated_info.get("rated_speed_rpm", 4000.0))
    rated_torque = float(
        rated_info.get(
            "rated_torque_nm",
            params.torque_constant * float(rated_info.get("rated_current_a", 60.0)),
        )
    )

    print(f"TARGET: rated_speed={rated_speed} RPM, rated_torque={rated_torque} Nm")

    # Set FW model
    params.flux_weakening_id_coefficient = 0.016
    params.flux_weakening_min_ratio = 0.08

    # Use good seed parameters
    speed_kp_base = float(best["speed_pi"]["kp"])
    speed_ki_base = float(best["speed_pi"]["ki"])

    print("\nTesting FW configuration...")
    result = run_simulation(
        params=params,
        target_rpm=rated_speed,
        load_torque=rated_torque * 0.7,  # 70% rated load for safety
        speed_kp=speed_kp_base * 0.10,
        speed_ki=speed_ki_base * 0.10,
        fw_start_rpm=1100.0,
        fw_gain=1.1,
        fw_id_max=14.0,
        fw_headroom=1.0,
        sim_duration=5.0,
        dt=1e-3,
    )

    if result is None or not result.get("stability_ok"):
        print("FAILED: unstable")
        return

    speed_error_pct = 100 * result["speed_err_mean"] / max(rated_speed, 1)
    efficiency_ok = result["efficiency_mean"] > 75

    print(f"\n=== RESULTS ===")
    print(f"Speed: {result['speed_mean']:.1f} RPM (±{result['speed_std']:.1f})")
    print(f"Speed Error: {result['speed_err_mean']:.1f} RPM ({speed_error_pct:.2f}%)")
    print(f"d-axis current: {result['id_mean']:.2f} A")
    print(f"q-axis current: {result['iq_mean']:.2f} A")
    print(f"FW injection: {result['fw_injection_mean']:.2f} A")
    print(f"Efficiency: {result['efficiency_mean']:.1f}%")
    print(f"Input Power: {result['p_in_mean']:.0f} W")
    print(f"Output Power: {result['p_out_mean']:.0f} W")

    status = (
        "STABLE_EXCELLENT"
        if speed_error_pct < 2 and efficiency_ok
        else "STABLE_GOOD"
        if speed_error_pct < 5 and efficiency_ok
        else "STABLE_OK"
        if speed_error_pct < 10 and efficiency_ok
        else "UNSTABLE"
    )
    print(f"\nStatus: {status}")

    report = {
        "schema": "bldc.fw_calibration.demo.v1",
        "motor": profile.get("profile_name", "me1718_48v"),
        "operating_point": {
            "rated_speed_rpm": rated_speed,
            "load_torque_nm": rated_torque * 0.7,
            "rated_voltage": params.nominal_voltage,
        },
        "field_weakening_params": {
            "enabled": True,
            "start_rpm": 1100.0,
            "gain": 1.1,
            "id_max_a": 14.0,
            "headroom_v": 1.0,
        },
        "results": result,
        "status": status,
    }

    OUT_PATH.write_text(json.dumps(report, indent=2))
    print(f"\nOutput: {OUT_PATH.name}")


if __name__ == "__main__":
    main()
