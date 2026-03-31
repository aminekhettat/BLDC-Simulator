"""
Atomic features tested in this module:
- margin estimators return finite values
- state space checks are true for nominal model
- tune returns finite positive gains
"""

from __future__ import annotations

import os
import sys

import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.control.adaptive_tuning import AdaptiveFOCTuner  # noqa: E402
from src.core.motor_model import MotorParameters  # noqa: E402


def _test_params() -> MotorParameters:
    return MotorParameters(
        nominal_voltage=12.0,
        phase_resistance=7.2,
        phase_inductance=0.00055,
        back_emf_constant=0.0056,
        torque_constant=0.0056,
        rotor_inertia=6.0e-8,
        friction_coefficient=6.0e-7,
        num_poles=2,
        poles_pairs=1,
    )


def test_margin_estimators_return_finite_values() -> None:
    tuner = AdaptiveFOCTuner(_test_params())

    current = tuner.analyze_current_loop(kp=0.05, ki=40.0)
    speed = tuner.analyze_speed_loop(kp=0.03, ki=3.0)

    current_margin = current["margin"]
    speed_margin = speed["margin"]
    assert np.isfinite(current_margin.phase_margin_deg)
    assert current_margin.gain_crossover_hz is None or np.isfinite(current_margin.gain_crossover_hz)
    assert np.isfinite(speed_margin.phase_margin_deg)
    assert speed_margin.gain_crossover_hz is None or np.isfinite(speed_margin.gain_crossover_hz)


def test_state_space_checks_are_true_for_nominal_model() -> None:
    tuner = AdaptiveFOCTuner(_test_params())

    current = tuner.analyze_current_loop(kp=0.05, ki=40.0)
    speed = tuner.analyze_speed_loop(kp=0.03, ki=3.0)

    assert current["controllable"] and current["observable"]
    assert speed["controllable"] and speed["observable"]


def test_tune_returns_finite_positive_gains() -> None:
    tuner = AdaptiveFOCTuner(_test_params())
    result = tuner.tune(grid_size=6)

    assert result.current_kp > 0.0
    assert result.current_ki > 0.0
    assert result.speed_kp > 0.0
    assert result.speed_ki > 0.0
    assert np.isfinite(result.current_margin.phase_margin_deg)
    assert np.isfinite(result.speed_margin.phase_margin_deg)
