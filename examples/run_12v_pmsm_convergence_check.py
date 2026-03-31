"""Run a 12V PMSM/BLDC convergence check using published motor parameters.

Source motor data (web):
- FAULHABER 1218E012B: https://eshop.faulhaber.com/en/1218E012B/1218E012B

Values used from the product page:
- Nominal voltage: 12 V
- Terminal resistance: 12 Ohm
- Rotor inductance: 132 uH
- Torque constant: 3.51 mNm/A
- Rotor inertia: 0.08 gcm^2
- No-load current: 0.047 A
- No-load speed: 31500 rpm
- 2 Pole Technology
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.control import AdaptiveFOCTuner, FOCController, SVMGenerator
from src.core.load_model import ConstantLoad
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.simulation_engine import SimulationEngine


def run_case(
    params: MotorParameters,
    dt: float,
    steps: int,
    speed_ref_rpm: float,
    load_torque_nm: float,
    iq_limit_a: float,
    speed_kp: float,
    speed_ki: float,
    current_kp: float,
    current_ki: float,
) -> dict:
    motor = BLDCMotor(params, dt=dt)
    engine = SimulationEngine(
        motor,
        ConstantLoad(load_torque_nm),
        dt=dt,
        max_history=max(steps + 100, 10000),
    )

    controller = FOCController(motor=motor, enable_speed_loop=True)
    controller.set_cascaded_speed_loop(True, iq_limit_a=iq_limit_a)
    controller.set_speed_pi_gains(kp=speed_kp, ki=speed_ki, kaw=0.05)
    controller.set_current_pi_gains(
        d_kp=current_kp,
        d_ki=current_ki,
        q_kp=current_kp,
        q_ki=current_ki,
        kaw=0.2,
    )
    controller.set_current_references(id_ref=0.0, iq_ref=0.0)
    controller.set_speed_reference(speed_ref_rpm)
    controller.set_angle_observer("Measured")

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(engine.dt)

    stable = True
    for _ in range(steps):
        svm.set_phase_currents(motor.currents)
        magnitude, angle = controller.update(engine.dt)
        phase_voltages = svm.modulate(magnitude, angle)
        engine.step(phase_voltages, log_data=True)
        if not np.isfinite(motor.omega) or abs(motor.speed_rpm) > 1.0e6:
            stable = False
            break

    history = engine.get_history()
    t = history["time"]
    speed = history["speed"]
    if speed.size < 20:
        return {"stable": False, "reason": "insufficient_history"}

    err = speed_ref_rpm - speed
    half = len(err) // 2
    tail = max(200, len(err) // 10)

    first_half_abs_mean = float(np.mean(np.abs(err[:half])))
    second_half_abs_mean = float(np.mean(np.abs(err[half:])))
    tail_abs_mean = float(np.mean(np.abs(err[-tail:])))
    tail_abs_max = float(np.max(np.abs(err[-tail:])))

    band = 0.05 * speed_ref_rpm
    within = np.abs(err) <= band
    settle_idx = None
    false_indices = np.where(~within)[0]
    if false_indices.size == 0:
        settle_idx = 0
    else:
        candidate = int(false_indices[-1] + 1)
        if candidate < len(within):
            settle_idx = candidate

    settle_time = float(t[settle_idx]) if settle_idx is not None else None
    asymptotic = bool(second_half_abs_mean < first_half_abs_mean)
    bounded_tail = bool(tail_abs_max < 0.5 * speed_ref_rpm)

    return {
        "stable": bool(stable),
        "sim_time_s": float(t[-1]),
        "final_speed_rpm": float(speed[-1]),
        "tail_abs_mean_err_rpm": tail_abs_mean,
        "tail_abs_max_err_rpm": tail_abs_max,
        "first_half_abs_mean_err_rpm": first_half_abs_mean,
        "second_half_abs_mean_err_rpm": second_half_abs_mean,
        "asymptotic_convergence_check": asymptotic,
        "bounded_tail_check": bounded_tail,
        "settle_time_5pct_s": settle_time,
    }


def main() -> None:
    # SI conversions from datasheet units.
    kt = 3.51e-3  # 3.51 mNm/A -> Nm/A
    rotor_inertia = 0.08e-7  # 0.08 gcm^2 -> kg*m^2
    omega_no_load = 31500.0 * 2.0 * np.pi / 60.0  # rpm -> rad/s
    # Approximate viscous friction from no-load current torque.
    friction_coeff = (kt * 0.047) / max(omega_no_load, 1e-12)

    params = MotorParameters(
        model_type="dq",
        emf_shape="sinusoidal",
        nominal_voltage=12.0,
        phase_resistance=12.0,
        phase_inductance=132e-6,
        back_emf_constant=kt,
        torque_constant=kt,
        rotor_inertia=rotor_inertia,
        friction_coefficient=friction_coeff,
        num_poles=2,
        poles_pairs=1,
    )

    speed_ref = 6000.0
    dt = 1.0 / 20000.0
    steps = 16000
    load_torque_nm = 2.0e-4

    tuner = AdaptiveFOCTuner(params=params)
    tuning = tuner.tune(grid_size=10)

    metrics = run_case(
        params=params,
        dt=dt,
        steps=steps,
        speed_ref_rpm=speed_ref,
        load_torque_nm=load_torque_nm,
        iq_limit_a=0.08,
        speed_kp=tuning.speed_kp,
        speed_ki=tuning.speed_ki,
        current_kp=tuning.current_kp,
        current_ki=tuning.current_ki,
    )

    print("Motor dataset: FAULHABER 1218E012B (12V BLDC/PMSM)")
    print(
        {
            "phase_resistance_ohm": 12.0,
            "phase_inductance_h": 132e-6,
            "torque_constant_nm_per_a": kt,
            "back_emf_constant_vs_per_rad": kt,
            "rotor_inertia_kg_m2": rotor_inertia,
            "friction_coeff_nms_per_rad": friction_coeff,
            "num_poles": 2,
            "poles_pairs": 1,
        }
    )

    print("Adaptive tuning diagnostics:")
    print(
        {
            "current_loop_gain_margin_db": tuning.current_margin.gain_margin_db,
            "current_loop_phase_margin_deg": tuning.current_margin.phase_margin_deg,
            "speed_loop_gain_margin_db": tuning.speed_margin.gain_margin_db,
            "speed_loop_phase_margin_deg": tuning.speed_margin.phase_margin_deg,
            "current_loop_controllable": tuning.current_controllable,
            "current_loop_observable": tuning.current_observable,
            "speed_loop_controllable": tuning.speed_controllable,
            "speed_loop_observable": tuning.speed_observable,
        }
    )

    print("Selected controller candidate:")
    print(
        {
            "dt_s": dt,
            "steps": steps,
            "speed_ref_rpm": speed_ref,
            "load_torque_nm": load_torque_nm,
            "iq_limit_a": 0.08,
            "speed_kp": tuning.speed_kp,
            "speed_ki": tuning.speed_ki,
            "current_kp": tuning.current_kp,
            "current_ki": tuning.current_ki,
        }
    )
    print("Convergence metrics:")
    print(metrics)


if __name__ == "__main__":
    main()
