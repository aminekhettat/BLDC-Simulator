"""Convergence check for built-in 48V motor profiles without field weakening.

Runs each target motor profile with FOC speed loop and verifies convergence to
rated speed using a 5% band criterion.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.control import AdaptiveFOCTuner, FOCController, SVMGenerator
from src.core.load_model import ConstantLoad
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.simulation_engine import SimulationEngine
from src.utils.motor_profiles import load_motor_profile


def _to_motor_params(m: dict) -> MotorParameters:
    num_poles = int(m["num_poles"])
    return MotorParameters(
        model_type=m.get("model_type", "dq"),
        emf_shape=m.get("emf_shape", "sinusoidal"),
        nominal_voltage=float(m["nominal_voltage"]),
        phase_resistance=float(m["phase_resistance"]),
        phase_inductance=float(m["phase_inductance"]),
        back_emf_constant=float(m["back_emf_constant"]),
        torque_constant=float(m["torque_constant"]),
        rotor_inertia=float(m["rotor_inertia"]),
        friction_coefficient=float(m["friction_coefficient"]),
        num_poles=num_poles,
        poles_pairs=int(num_poles / 2),
        ld=float(m.get("ld", m["phase_inductance"])),
        lq=float(m.get("lq", m["phase_inductance"])),
    )


def run_case(profile_path: Path, sim_time_s: float = 2.5) -> dict:
    profile = load_motor_profile(profile_path)
    rated = profile.get("rated_info", {})
    rated_speed = float(rated.get("rated_speed_rpm", 0.0))
    if rated_speed <= 0.0:
        raise ValueError(f"Missing rated_speed_rpm in {profile_path.name}")

    rated_current = float(rated.get("rated_current_a", rated.get("rated_current_a_rms", 20.0)))

    params = _to_motor_params(profile["motor_params"])
    dt = 1.0 / 20000.0
    steps = int(sim_time_s / dt)

    motor = BLDCMotor(params, dt=dt)
    engine = SimulationEngine(
        motor,
        ConstantLoad(0.0),
        dt=dt,
        compute_backend="cpu",
        max_history=6000,
    )

    tuner = AdaptiveFOCTuner(params=params)
    tuning = tuner.tune(grid_size=8)

    controller = FOCController(motor=motor, enable_speed_loop=True)
    controller.set_cascaded_speed_loop(True, iq_limit_a=max(5.0, 0.9 * rated_current))
    controller.set_speed_pi_gains(
        kp=tuning.speed_kp,
        ki=tuning.speed_ki,
        kaw=0.05,
    )
    controller.set_current_pi_gains(
        d_kp=tuning.current_kp,
        d_ki=tuning.current_ki,
        q_kp=tuning.current_kp,
        q_ki=tuning.current_ki,
        kaw=0.2,
    )
    controller.set_current_references(id_ref=0.0, iq_ref=0.0)
    controller.set_speed_reference(rated_speed)
    controller.set_angle_observer("Measured")
    controller.set_field_weakening(
        enabled=False,
        start_speed_rpm=1.0e9,
        gain=0.0,
        max_negative_id_a=0.0,
    )

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(dt)

    stride = max(1, steps // 2500)
    speed_samples = []
    time_samples = []
    stable = True

    for k in range(steps):
        svm.set_phase_currents(motor.currents)
        magnitude, angle = controller.update(dt)
        phase_voltages = svm.modulate(magnitude, angle)
        engine.step(phase_voltages, log_data=False)

        if (k % stride) == 0 or k == steps - 1:
            speed_samples.append(float(motor.speed_rpm))
            time_samples.append(float((k + 1) * dt))

        if not np.isfinite(motor.omega) or abs(motor.speed_rpm) > 1.0e6:
            stable = False
            break

    speed = np.array(speed_samples, dtype=np.float64)
    t = np.array(time_samples, dtype=np.float64)
    err = rated_speed - speed

    if speed.size < 20:
        return {
            "profile": profile["profile_name"],
            "file": profile_path.name,
            "stable": False,
            "converged": False,
            "reason": "insufficient_history",
        }

    band = 0.05 * rated_speed
    within = np.abs(err) <= band
    settle_idx = None
    for i in range(len(within)):
        if np.all(within[i:]):
            settle_idx = i
            break

    tail = max(100, len(err) // 10)
    final_speed = float(speed[-1])
    final_err = float(err[-1])
    tail_mean = float(np.mean(np.abs(err[-tail:])))
    tail_max = float(np.max(np.abs(err[-tail:])))

    converged = bool(
        stable
        and final_speed > 0.0
        and abs(final_err) <= band
        and tail_mean <= band
        and tail_max <= 2.0 * band
        and settle_idx is not None
    )

    return {
        "profile": profile["profile_name"],
        "file": profile_path.name,
        "stable": stable,
        "converged": converged,
        "field_weakening_enabled": False,
        "rated_speed_rpm": rated_speed,
        "final_speed_rpm": final_speed,
        "final_error_rpm": final_err,
        "tail_abs_mean_error_rpm": tail_mean,
        "tail_abs_max_error_rpm": tail_max,
        "settling_time_5pct_s": None if settle_idx is None else float(t[settle_idx]),
        "sim_time_s": float(t[-1]),
    }


def main() -> None:
    profile_paths = [
        PROJECT_ROOT / "data" / "motor_profiles" / "motenergy_me1718_48v.json",
        PROJECT_ROOT / "data" / "motor_profiles" / "motenergy_me1719_48v.json",
        PROJECT_ROOT / "data" / "motor_profiles" / "innotec_255_ezs48_160.json",
    ]

    results = []
    for p in profile_paths:
        results.append(run_case(p))

    output = {
        "scenario": "rated_speed_convergence_no_field_weakening",
        "results": results,
    }

    logs_dir = PROJECT_ROOT / "data" / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    report_path = logs_dir / "motor_profile_convergence_no_fw.json"
    report_path.write_text(json.dumps(output, indent=2), encoding="utf-8")

    print(json.dumps(output, indent=2))
    print(f"REPORT_SAVED {report_path}")


if __name__ == "__main__":
    main()
