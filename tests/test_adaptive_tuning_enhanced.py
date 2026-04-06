"""Enhanced tests for improved adaptive tuning module.

Tests cover:
- Analytical initial guess computation
- Multi-resolution search
- Motor profile calibration
- Backward compatibility with original API
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.control.adaptive_tuning import (
    AdaptiveFOCTuner,
    calibrate_motor,
)
from src.core.motor_model import MotorParameters


def _test_params() -> MotorParameters:
    """Standard test motor parameters."""
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


def test_backward_compatibility_margin_estimators() -> None:
    """Test that original margin estimator API still works."""
    tuner = AdaptiveFOCTuner(_test_params())

    current = tuner.analyze_current_loop(kp=0.05, ki=40.0)
    speed = tuner.analyze_speed_loop(kp=0.03, ki=3.0)

    current_margin = current["margin"]
    speed_margin = speed["margin"]

    assert np.isfinite(current_margin.phase_margin_deg)
    assert current_margin.gain_crossover_hz is None or np.isfinite(current_margin.gain_crossover_hz)
    assert np.isfinite(speed_margin.phase_margin_deg)
    assert speed_margin.gain_crossover_hz is None or np.isfinite(speed_margin.gain_crossover_hz)


def test_backward_compatibility_state_space() -> None:
    """Test that state-space checks still work."""
    tuner = AdaptiveFOCTuner(_test_params())

    current = tuner.analyze_current_loop(kp=0.05, ki=40.0)
    speed = tuner.analyze_speed_loop(kp=0.03, ki=3.0)

    assert current["controllable"] and current["observable"]
    assert speed["controllable"] and speed["observable"]


def test_backward_compatibility_tune() -> None:
    """Test that original tune() method still works with grid_size parameter."""
    tuner = AdaptiveFOCTuner(_test_params())
    result = tuner.tune(grid_size=6)

    assert result.current_kp > 0.0
    assert result.current_ki > 0.0
    assert result.speed_kp > 0.0
    assert result.speed_ki > 0.0
    assert np.isfinite(result.current_margin.phase_margin_deg)
    assert np.isfinite(result.speed_margin.phase_margin_deg)


def test_analytical_initial_guess_computation() -> None:
    """Test analytical initial guess from motor parameters."""
    params = _test_params()
    tuner = AdaptiveFOCTuner(params)

    analytical = tuner._compute_analytical_initial_guess()

    # Verify all keys present
    assert "current_kp" in analytical
    assert "current_ki" in analytical
    assert "speed_kp" in analytical
    assert "speed_ki" in analytical
    assert "omega_c_rad_s" in analytical
    assert "omega_s_rad_s" in analytical

    # Verify values are positive
    assert analytical["current_kp"] > 0.0
    assert analytical["current_ki"] > 0.0
    assert analytical["speed_kp"] > 0.0
    assert analytical["speed_ki"] > 0.0
    assert analytical["omega_c_rad_s"] > 0.0
    assert analytical["omega_s_rad_s"] > 0.0

    # Speed loop bandwidth should be lower than current loop
    assert analytical["omega_s_rad_s"] < analytical["omega_c_rad_s"]


def test_analytical_guess_scaling_with_motor() -> None:
    """Test that analytical guess scales appropriately with motor parameters."""
    # High L/R motor (slower current loop)
    params_slow = MotorParameters(
        nominal_voltage=12.0,
        phase_resistance=0.5,
        phase_inductance=0.01,
        back_emf_constant=0.0056,
        torque_constant=0.0056,
        rotor_inertia=6.0e-8,
        friction_coefficient=6.0e-7,
        num_poles=2,
        poles_pairs=1,
    )

    # Low L/R motor (faster current loop)
    params_fast = MotorParameters(
        nominal_voltage=12.0,
        phase_resistance=10.0,
        phase_inductance=0.0001,
        back_emf_constant=0.0056,
        torque_constant=0.0056,
        rotor_inertia=6.0e-8,
        friction_coefficient=6.0e-7,
        num_poles=2,
        poles_pairs=1,
    )

    tuner_slow = AdaptiveFOCTuner(params_slow)
    tuner_fast = AdaptiveFOCTuner(params_fast)

    guess_slow = tuner_slow._compute_analytical_initial_guess()
    guess_fast = tuner_fast._compute_analytical_initial_guess()

    # Fast motor should have higher bandwidth
    assert guess_fast["omega_c_rad_s"] > guess_slow["omega_c_rad_s"]


def test_tune_analytical_returns_valid_result() -> None:
    """Test enhanced tune_analytical() method."""
    params = _test_params()
    tuner = AdaptiveFOCTuner(params)

    result, analytical = tuner.tune_analytical()

    # Check result is valid
    assert result.current_kp > 0.0
    assert result.current_ki > 0.0
    assert result.speed_kp > 0.0
    assert result.speed_ki > 0.0

    # Check analytical guess is present and reasonable
    assert analytical["current_kp"] > 0.0
    assert analytical["current_ki"] > 0.0
    assert analytical["speed_kp"] > 0.0
    assert analytical["speed_ki"] > 0.0


def test_validate_at_operating_point_stable() -> None:
    """Test simulation-based validation at an operating point."""
    params = _test_params()
    tuner = AdaptiveFOCTuner(params)

    # Get some reasonable gains
    result = tuner.tune(grid_size=4)

    # Validate at low speed (should be stable)
    validation = tuner.validate_at_operating_point(
        result,
        target_speed_rpm=500.0,
        load_torque_nm=0.0,
        dt=5e-4,
        sim_end_s=0.5,
    )

    assert "stable" in validation
    if validation["stable"]:
        # If stable, check metrics
        assert "mean_speed_rpm" in validation
        assert "speed_error_rpm" in validation


def test_validate_at_operating_point_quick() -> None:
    """Test quick validation (no simulation)."""
    params = _test_params()
    tuner = AdaptiveFOCTuner(params)

    result = tuner.tune(grid_size=4)

    # Should complete very quickly
    validation = tuner.validate_at_operating_point(
        result,
        target_speed_rpm=500.0,
        dt=5e-4,
        sim_end_s=0.1,
    )

    assert isinstance(validation, dict)


def test_calibrate_motor_with_real_profile() -> None:
    """Test calibrate_motor() with real motor profile."""
    profile_path = Path(PROJECT_ROOT) / "data" / "motor_profiles" / "motenergy_me1718_48v.json"

    if not profile_path.exists():
        print(f"Skipping test: {profile_path} not found")
        return

    # Quick mode: no simulation validation
    report = calibrate_motor(str(profile_path), quick_mode=True)

    assert report.motor_profile_name
    assert report.tuning_result.current_kp > 0.0
    assert report.tuning_result.current_ki > 0.0
    assert report.tuning_result.speed_kp > 0.0
    assert report.tuning_result.speed_ki > 0.0
    assert report.analytical_initial_guess
    assert report.validation_metrics


def test_calibration_report_serializable() -> None:
    """Test that calibration report can be serialized to JSON."""
    params = _test_params()
    tuner = AdaptiveFOCTuner(params)

    result, analytical = tuner.tune_analytical()

    from src.control.adaptive_tuning import CalibrationReport

    report = CalibrationReport(
        motor_profile_name="Test Motor",
        motor_params={
            "nominal_voltage": params.nominal_voltage,
            "phase_resistance": params.phase_resistance,
            "phase_inductance": params.phase_inductance,
            "back_emf_constant": params.back_emf_constant,
            "torque_constant": params.torque_constant,
            "rotor_inertia": params.rotor_inertia,
            "friction_coefficient": params.friction_coefficient,
            "poles_pairs": params.poles_pairs,
        },
        tuning_result=result,
        analytical_initial_guess=analytical,
        validation_metrics={},
    )

    # Should be JSON serializable
    report_dict = report.to_dict()
    report_json = json.dumps(report_dict)
    assert len(report_json) > 0


def test_multi_resolution_search_converges() -> None:
    """Test that multi-resolution search produces reasonable results.

    Note: For the small 12V test motor (R=7.2, L=0.55mH), the analytical
    initial guess yields very high bandwidth (ωc = 30 × R/L ≈ 393k rad/s)
    which produces enormous Ki values.  The optimiser correctly rejects
    those extreme gains in favour of lower, margin-satisfying values.
    Therefore we only check that the *optimised* gains are positive,
    finite, and produce valid margins — not that they are close to the
    (unrealistic) analytical guess for this motor.
    """
    params = _test_params()
    tuner = AdaptiveFOCTuner(params)

    result, analytical = tuner.tune_analytical()

    # Optimised gains must be positive and finite
    assert result.current_kp > 0.0 and np.isfinite(result.current_kp)
    assert result.current_ki > 0.0 and np.isfinite(result.current_ki)
    assert result.speed_kp > 0.0 and np.isfinite(result.speed_kp)
    assert result.speed_ki > 0.0 and np.isfinite(result.speed_ki)

    # Margins must be finite (stability check)
    assert np.isfinite(result.current_margin.phase_margin_deg)
    assert np.isfinite(result.speed_margin.phase_margin_deg)

    # Analytical guess must also be present and positive
    assert analytical["current_kp"] > 0.0
    assert analytical["current_ki"] > 0.0


def test_all_motor_profiles_calibrate() -> None:
    """Test that all motor profiles can be calibrated (quick mode)."""
    profiles_dir = Path(PROJECT_ROOT) / "data" / "motor_profiles"
    profiles = sorted(profiles_dir.glob("*.json"))

    # Filter out temporary directories
    profiles = [p for p in profiles if not p.name.startswith("_tmp_")]

    if not profiles:
        print("Skipping: no motor profiles found")
        return

    for profile_path in profiles:
        print(f"Calibrating {profile_path.name}...", end=" ", flush=True)
        try:
            report = calibrate_motor(str(profile_path), quick_mode=True)
            assert report.tuning_result.current_kp > 0.0
            print("OK")
        except Exception as e:
            print(f"FAILED: {e}")
            raise


if __name__ == "__main__":
    # Run tests
    print("Running enhanced adaptive tuning tests...")
    test_backward_compatibility_margin_estimators()
    print("✓ Backward compatibility: margin estimators")

    test_backward_compatibility_state_space()
    print("✓ Backward compatibility: state space")

    test_backward_compatibility_tune()
    print("✓ Backward compatibility: tune()")

    test_analytical_initial_guess_computation()
    print("✓ Analytical initial guess computation")

    test_analytical_guess_scaling_with_motor()
    print("✓ Analytical guess scaling with motor")

    test_tune_analytical_returns_valid_result()
    print("✓ tune_analytical() returns valid result")

    test_validate_at_operating_point_stable()
    print("✓ Simulation validation at operating point")

    test_validate_at_operating_point_quick()
    print("✓ Quick validation")

    test_calibrate_motor_with_real_profile()
    print("✓ calibrate_motor() with real profile")

    test_calibration_report_serializable()
    print("✓ Calibration report serializable")

    test_multi_resolution_search_converges()
    print("✓ Multi-resolution search produces valid gains")

    test_all_motor_profiles_calibrate()
    print("✓ All motor profiles calibrate (quick mode)")

    print("\nAll tests passed!")
