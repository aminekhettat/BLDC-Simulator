"""Non-functional smoke tests for runtime stability and baseline throughput."""

import time

import numpy as np
import pytest

from src.core import BLDCMotor, ConstantLoad, MotorParameters, SimulationEngine


def _build_engine(dt: float = 0.0005) -> SimulationEngine:
    motor = BLDCMotor(MotorParameters(), dt=dt)
    load = ConstantLoad(0.05)
    return SimulationEngine(motor, load, dt=dt)


def _build_engine_with_params(params: MotorParameters, dt: float = 0.0005) -> SimulationEngine:
    motor = BLDCMotor(params, dt=dt)
    load = ConstantLoad(0.05)
    return SimulationEngine(motor, load, dt=dt)


def test_simulation_loop_throughput_smoke() -> None:
    """Ensure core stepping throughput remains acceptable on commodity CI runners."""
    engine = _build_engine(dt=0.0005)
    start = time.perf_counter()

    for _ in range(2000):
        engine.step(np.array([4.0, -2.0, -2.0]), log_data=False)

    elapsed_s = time.perf_counter() - start
    assert elapsed_s < 12.0


def test_history_integrity_after_long_run() -> None:
    """Ensure long-run history logging keeps synchronized, finite telemetry arrays."""
    engine = _build_engine(dt=0.0005)

    for _ in range(1500):
        engine.step(np.array([5.0, -2.5, -2.5]), log_data=True)

    history = engine.get_history()
    time_len = history["time"].size

    assert time_len == 1500
    assert history["speed"].size == time_len
    assert history["torque"].size == time_len
    assert history["efficiency"].size == time_len
    assert history["supply_voltage"].size == time_len
    assert np.isfinite(history["speed"]).all()
    assert np.isfinite(history["torque"]).all()


# ---------------------------------------------------------------------------
# Motor parameter edge cases
# ---------------------------------------------------------------------------


def test_high_resistance_motor_remains_stable() -> None:
    """High-resistance motor (R=10 Ω, L scaled to preserve L/R=0.002 s) must stay finite.

    R and L are both scaled 4× from the default so the RK4 stability criterion
    |λ_e·dt| = |(R/L)·dt| remains well within the stable region (< 2.785).
    """
    params = MotorParameters(
        nominal_voltage=48.0,
        phase_resistance=10.0,  # 4× default R
        phase_inductance=0.020,  # 4× default L → L/R = 0.002 s (same as default)
        back_emf_constant=0.1,
        torque_constant=0.1,
        rotor_inertia=0.0005,
        friction_coefficient=0.001,
        num_poles=8,
        poles_pairs=4,
    )
    engine = _build_engine_with_params(params)
    for _ in range(500):
        engine.step(np.array([24.0, -12.0, -12.0]), log_data=True)
    history = engine.get_history()
    assert np.isfinite(history["speed"]).all()
    assert np.isfinite(history["torque"]).all()


def test_low_inertia_motor_remains_stable() -> None:
    """Low rotor inertia (1e-6 kg·m², 500× below default) must not overflow in RK4.

    1e-9 kg·m² would violate |λ_m·dt| < 2.785 at dt=0.0005 s; 1e-6 kg·m² gives
    λ_m·dt = (b/J)·dt = (0.001/1e-6)·0.0005 = 0.5 — safely within the stable region.
    """
    params = MotorParameters(
        nominal_voltage=48.0,
        phase_resistance=2.5,
        phase_inductance=0.005,
        back_emf_constant=0.1,
        torque_constant=0.1,
        rotor_inertia=1e-6,  # 500× below default 0.0005 kg·m²
        friction_coefficient=0.001,
        num_poles=8,
        poles_pairs=4,
    )
    engine = _build_engine_with_params(params)
    for _ in range(500):
        engine.step(np.array([5.0, -2.5, -2.5]), log_data=True)
    history = engine.get_history()
    assert np.isfinite(history["speed"]).all()
    assert np.isfinite(history["torque"]).all()


def test_high_pole_count_motor_remains_stable() -> None:
    """24-pole motor should track electrical dynamics without diverging over 500 steps."""
    params = MotorParameters(
        nominal_voltage=48.0,
        phase_resistance=2.5,
        phase_inductance=0.005,
        back_emf_constant=0.1,
        torque_constant=0.1,
        rotor_inertia=0.0005,
        friction_coefficient=0.001,
        num_poles=24,
        poles_pairs=12,
    )
    engine = _build_engine_with_params(params)
    for _ in range(500):
        engine.step(np.array([5.0, -2.5, -2.5]), log_data=True)
    history = engine.get_history()
    assert np.isfinite(history["speed"]).all()
    assert np.isfinite(history["torque"]).all()


def test_salient_motor_ld_ne_lq_remains_stable() -> None:
    """Salient dq motor (Ld=3 mH, Lq=8 mH) must remain numerically stable over 500 steps."""
    params = MotorParameters(
        model_type="dq",
        emf_shape="sinusoidal",
        nominal_voltage=48.0,
        phase_resistance=2.5,
        phase_inductance=0.005,
        back_emf_constant=0.1,
        torque_constant=0.1,
        rotor_inertia=0.0005,
        friction_coefficient=0.001,
        num_poles=8,
        poles_pairs=4,
        ld=0.003,
        lq=0.008,
    )
    engine = _build_engine_with_params(params)
    for _ in range(500):
        engine.step(np.array([5.0, -2.5, -2.5]), log_data=True)
    history = engine.get_history()
    assert np.isfinite(history["speed"]).all()
    assert np.isfinite(history["torque"]).all()


# ---------------------------------------------------------------------------
# Load and supply discontinuity tests
# ---------------------------------------------------------------------------


def test_step_load_increase_remains_stable() -> None:
    """Sudden load step (0 Nm → 2 Nm) mid-simulation must not cause NaN/inf."""
    load = ConstantLoad(0.0)
    motor = BLDCMotor(MotorParameters(), dt=0.0005)
    engine = SimulationEngine(motor, load, dt=0.0005)

    for _ in range(500):
        engine.step(np.array([5.0, -2.5, -2.5]), log_data=True)

    load.torque = 2.0  # step change
    for _ in range(500):
        engine.step(np.array([5.0, -2.5, -2.5]), log_data=True)

    history = engine.get_history()
    assert np.isfinite(history["speed"]).all()
    assert np.isfinite(history["torque"]).all()


def test_voltage_cutoff_free_deceleration_stable() -> None:
    """Zero-voltage free-deceleration after spin-up must not produce NaN over 300 steps."""
    engine = _build_engine(dt=0.0005)

    for _ in range(200):
        engine.step(np.array([5.0, -2.5, -2.5]), log_data=False)

    for _ in range(300):
        engine.step(np.array([0.0, 0.0, 0.0]), log_data=True)

    history = engine.get_history()
    assert np.isfinite(history["speed"]).all()
    assert np.isfinite(history["torque"]).all()


def test_oscillating_load_remains_stable() -> None:
    """Alternating load sign every 50 steps must not cause divergence over 600 steps."""
    load = ConstantLoad(0.1)
    motor = BLDCMotor(MotorParameters(), dt=0.0005)
    engine = SimulationEngine(motor, load, dt=0.0005)

    for i in range(600):
        if i % 100 == 0:
            load.torque = 0.5 if (i // 100) % 2 == 0 else 0.0
        engine.step(np.array([5.0, -2.5, -2.5]), log_data=True)

    history = engine.get_history()
    assert np.isfinite(history["speed"]).all()
    assert np.isfinite(history["torque"]).all()


# ---------------------------------------------------------------------------
# Simulation time-step sensitivity
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("dt", [0.0001, 0.0005, 0.001, 0.002])
def test_various_dt_produce_finite_history(dt: float) -> None:
    """Multiple dt values must all yield finite speed/torque over 300 steps."""
    engine = _build_engine(dt=dt)
    for _ in range(300):
        engine.step(np.array([5.0, -2.5, -2.5]), log_data=True)
    history = engine.get_history()
    assert np.isfinite(history["speed"]).all(), f"NaN in speed at dt={dt}"
    assert np.isfinite(history["torque"]).all(), f"NaN in torque at dt={dt}"
