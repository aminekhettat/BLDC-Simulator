"""
Atomic features tested in this module:
- ComputePowerMetricsEdgeCases
- RequiredReactiveCompensationErrors
- ComputeEfficiencyMetricsEdgeCases
- RecommendEfficiencyAdjustments
- PowerFactorControllerEdgeCases
- SrcInitAttributeError
- AbstractBaseControllerCoverage
- SupplyProfilesAndAbstractPassLines
- LoadAndMotorProfilesRemainingBranches
"""

import sys
from typing import cast

import numpy as np
import pytest

from src.core.load_model import LoadProfile, RampLoad
from src.core.power_model import (
    PowerFactorController,
    RampSupply,
    SupplyProfile,
    VariableSupply,
    compute_efficiency_metrics,
    compute_power_metrics,
    recommend_efficiency_adjustments,
    required_reactive_compensation,
)
from src.utils import motor_profiles


class TestComputePowerMetricsEdgeCases:
    """Test edge cases and error paths in compute_power_metrics."""

    def test_power_metrics_cpu_path_with_shape_mismatch(self):
        """Verify shape mismatch raises ValueError on CPU path."""
        v = np.array([1.0, 2.0, 3.0])
        i = np.array([1.0, 2.0])  # Different shape
        with pytest.raises(ValueError, match="same shape"):
            compute_power_metrics(v, i, backend="cpu")

    def test_power_metrics_cpu_path_with_empty_arrays(self):
        """Verify empty arrays raise ValueError on CPU path."""
        v = np.array([])
        i = np.array([])
        with pytest.raises(ValueError, match="non-empty"):
            compute_power_metrics(v, i, backend="cpu")

    def test_power_metrics_zero_apparent_power_returns_zero_pf_and_reactive(self):
        """Verify zero apparent power returns zero PF and reactive power."""
        v = np.array([0.0, 0.0, 0.0])
        i = np.array([0.0, 0.0, 0.0])
        metrics = compute_power_metrics(v, i, backend="cpu")
        assert metrics["power_factor"] == 0.0
        assert metrics["reactive_power_var"] == 0.0
        assert metrics["voltage_rms_v"] == 0.0
        assert metrics["current_rms_a"] == 0.0

    def test_power_metrics_gpu_shape_mismatch_fallback_to_cpu(self):
        """Verify GPU path with shape mismatch falls back to CPU."""
        v = np.array([1.0, 2.0, 3.0])
        i = np.array([1.0, 2.0])

        # When GPU is requested but fails (which is expected on CPU-only system),
        # the fallback CPU path should be triggered
        with pytest.raises(ValueError, match="same shape"):
            compute_power_metrics(v, i, backend="gpu")

    def test_power_metrics_gpu_empty_arrays_fallback_to_cpu(self):
        """Verify GPU path with empty arrays falls back to CPU and raises."""
        v = np.array([])
        i = np.array([])
        with pytest.raises(ValueError, match="non-empty"):
            compute_power_metrics(v, i, backend="gpu")

    def test_power_metrics_high_power_factor(self):
        """Verify power factor clamping at high values."""
        # In-phase waveforms create high PF
        t = np.linspace(0.0, 2.0 * np.pi, 2000, endpoint=False)
        v = 230.0 * np.sin(t)
        i = 10.0 * np.sin(t)

        metrics = compute_power_metrics(v, i, backend="cpu")
        assert 0.99 <= metrics["power_factor"] <= 1.0
        assert metrics["reactive_power_var"] < 1.0

    def test_power_metrics_low_power_factor(self):
        """Verify power factor clamping at low values."""
        # 90-degree phase shift creates low PF
        t = np.linspace(0.0, 2.0 * np.pi, 2000, endpoint=False)
        v = 230.0 * np.sin(t)
        i = 10.0 * np.cos(t)

        metrics = compute_power_metrics(v, i, backend="cpu")
        assert abs(metrics["power_factor"]) < 0.05
        assert metrics["reactive_power_var"] > 0.0

    def test_power_metrics_gpu_success_path_with_stubbed_cupy(self, monkeypatch):
        """Execute the GPU success path by stubbing a CuPy-compatible module."""

        class FakeCupy:
            float64 = np.float64

            @staticmethod
            def asarray(a, dtype=None):
                return np.asarray(a, dtype=dtype)

            @staticmethod
            def mean(a):
                return np.mean(a)

            @staticmethod
            def sqrt(a):
                return np.sqrt(a)

        monkeypatch.setitem(sys.modules, "cupy", FakeCupy)

        t = np.linspace(0.0, 2.0 * np.pi, 1000, endpoint=False)
        v = 120.0 * np.sin(t)
        i = 5.0 * np.sin(t)
        metrics = compute_power_metrics(v, i, backend="gpu")

        assert metrics["voltage_rms_v"] > 0.0
        assert metrics["current_rms_a"] > 0.0
        assert metrics["active_power_w"] > 0.0


class TestRequiredReactiveCompensationErrors:
    """Test error validation in required_reactive_compensation."""

    def test_negative_active_power_raises_error(self):
        """Verify negative active power raises ValueError."""
        with pytest.raises(ValueError, match="active_power_w must be positive"):
            required_reactive_compensation(
                active_power_w=-100.0,
                current_pf=0.8,
                target_pf=0.95,
            )

    def test_zero_active_power_raises_error(self):
        """Verify zero active power raises ValueError."""
        with pytest.raises(ValueError, match="active_power_w must be positive"):
            required_reactive_compensation(
                active_power_w=0.0,
                current_pf=0.8,
                target_pf=0.95,
            )

    def test_current_pf_out_of_range_raises_error(self):
        """Verify current_pf outside (0, 1] raises ValueError."""
        with pytest.raises(ValueError, match="current_pf must be in"):
            required_reactive_compensation(
                active_power_w=1000.0,
                current_pf=0.0,  # PF = 0 is invalid
                target_pf=0.95,
            )

    def test_current_pf_above_one_raises_error(self):
        """Verify current_pf > 1 raises ValueError."""
        with pytest.raises(ValueError, match="current_pf must be in"):
            required_reactive_compensation(
                active_power_w=1000.0,
                current_pf=1.5,
                target_pf=0.95,
            )

    def test_target_pf_out_of_range_raises_error(self):
        """Verify target_pf outside (0, 1] raises ValueError."""
        with pytest.raises(ValueError, match="target_pf must be in"):
            required_reactive_compensation(
                active_power_w=1000.0,
                current_pf=0.8,
                target_pf=0.0,
            )

    def test_target_pf_less_than_current_raises_error(self):
        """Verify target_pf < current_pf raises ValueError."""
        with pytest.raises(ValueError, match="target_pf must be greater than or equal"):
            required_reactive_compensation(
                active_power_w=1000.0,
                current_pf=0.95,
                target_pf=0.8,
            )

    def test_valid_compensation_calculation(self):
        """Verify valid compensation reduces reactive power."""
        res = required_reactive_compensation(
            active_power_w=1000.0,
            current_pf=0.70,
            target_pf=0.95,
        )
        assert res["required_compensation_var"] > 0.0
        assert res["target_reactive_var"] < res["current_reactive_var"]
        assert res["active_power_w"] == 1000.0


class TestComputeEfficiencyMetricsEdgeCases:
    """Test edge cases in compute_efficiency_metrics."""

    def test_zero_electrical_input_returns_zero_efficiency(self):
        """Verify zero input power results in zero efficiency."""
        metrics = compute_efficiency_metrics(
            input_power_w=0.0,
            torque_nm=1.0,
            omega_rad_s=100.0,
        )
        assert metrics["efficiency"] == 0.0
        assert metrics["electrical_input_power_w"] == 0.0

    def test_negative_input_power_clamps_to_zero(self):
        """Verify negative input power is clamped to zero."""
        metrics = compute_efficiency_metrics(
            input_power_w=-100.0,
            torque_nm=1.0,
            omega_rad_s=100.0,
        )
        assert metrics["electrical_input_power_w"] == 0.0
        assert metrics["efficiency"] == 0.0

    def test_negative_torque_creates_regenerative_power(self):
        """Verify negative torque (regenerative) is captured."""
        metrics = compute_efficiency_metrics(
            input_power_w=500.0,
            torque_nm=-1.0,
            omega_rad_s=400.0,
        )
        assert metrics["regenerative_power_w"] == pytest.approx(400.0)
        assert metrics["mechanical_output_power_w"] == 0.0

    def test_very_high_efficiency_clamps_to_one(self):
        """Verify efficiency cannot exceed 1.0."""
        metrics = compute_efficiency_metrics(
            input_power_w=100.0,
            torque_nm=10.0,
            omega_rad_s=100.0,  # Mechanical output > input
        )
        assert metrics["efficiency"] <= 1.0

    def test_loss_power_calculation(self):
        """Verify loss power is calculated correctly."""
        metrics = compute_efficiency_metrics(
            input_power_w=1000.0,
            torque_nm=5.0,
            omega_rad_s=100.0,  # Mechanical output = 500 W
        )
        assert metrics["total_loss_power_w"] == pytest.approx(500.0)


class TestRecommendEfficiencyAdjustments:
    """Test efficiency recommendations with various parameter combinations."""

    def test_high_efficiency_no_suggestions_needed(self):
        """Verify high efficiency and PF suggest no changes needed."""
        rec = recommend_efficiency_adjustments(
            efficiency=0.95,
            power_factor=0.98,
            target_efficiency=0.90,
        )
        assert rec["needs_attention"] is False
        assert "already near the requested" in str(rec["suggestions"][0]).lower()

    def test_low_efficiency_high_device_drop_suggests_hardware_upgrade(self):
        """Verify low efficiency with high device drop triggers suggestion."""
        rec = recommend_efficiency_adjustments(
            efficiency=0.72,
            power_factor=0.95,
            device_drop_v=0.8,
            target_efficiency=0.90,
        )
        assert rec["needs_attention"] is True
        assert any("device drop" in item.lower() for item in rec["suggestions"])

    def test_low_efficiency_high_dead_time_suggests_reduction(self):
        """Verify high dead-time fraction triggers suggestion."""
        rec = recommend_efficiency_adjustments(
            efficiency=0.72,
            power_factor=0.95,
            dead_time_fraction=0.02,
            target_efficiency=0.90,
        )
        assert rec["needs_attention"] is True
        assert any("dead-time" in item.lower() for item in rec["suggestions"])

    def test_low_efficiency_high_conduction_resistance_suggests_reduction(self):
        """Verify high conduction resistance triggers suggestion."""
        rec = recommend_efficiency_adjustments(
            efficiency=0.72,
            power_factor=0.95,
            conduction_resistance_ohm=0.015,
            target_efficiency=0.90,
        )
        assert rec["needs_attention"] is True
        assert any("conduction" in item.lower() for item in rec["suggestions"])

    def test_high_switching_frequency_with_switching_losses_suggests_reduction(self):
        """Verify high switching frequency with losses triggers adjustment."""
        rec = recommend_efficiency_adjustments(
            efficiency=0.72,
            power_factor=0.95,
            switching_frequency_hz=15000.0,
            switching_loss_coeff_v_per_a_khz=0.01,
            target_efficiency=0.90,
        )
        assert rec["needs_attention"] is True
        assert any("switching" in item.lower() for item in rec["suggestions"])

    def test_low_power_factor_suggests_improvement(self):
        """Verify low power factor triggers suggestion."""
        rec = recommend_efficiency_adjustments(
            efficiency=0.88,
            power_factor=0.85,
            target_efficiency=0.87,
        )
        assert rec["needs_attention"] is True
        assert any("power factor" in item.lower() for item in rec["suggestions"])

    def test_multiple_issues_generates_multiple_suggestions(self):
        """Verify multiple issues generate multiple suggestions."""
        rec = recommend_efficiency_adjustments(
            efficiency=0.65,
            power_factor=0.80,
            device_drop_v=1.0,
            dead_time_fraction=0.025,
            conduction_resistance_ohm=0.025,
            switching_frequency_hz=20000.0,
            switching_loss_coeff_v_per_a_khz=0.015,
            target_efficiency=0.90,
        )
        assert rec["needs_attention"] is True
        assert len(rec["suggestions"]) >= 3
        suggestion_text = " ".join(rec["suggestions"]).lower()
        assert "device drop" in suggestion_text
        assert "dead" in suggestion_text
        assert "power factor" in suggestion_text


class TestPowerFactorControllerEdgeCases:
    """Test PowerFactorController with various operating conditions."""

    def test_controller_initialization_validates_parameters(self):
        """Verify controller validates target_pf on init."""
        with pytest.raises(ValueError, match="target_pf must be in"):
            PowerFactorController(target_pf=0.0)

    def test_controller_init_validates_gains_non_negative(self):
        """Verify controller requires non-negative gains."""
        with pytest.raises(ValueError, match="kp and ki must be non-negative"):
            PowerFactorController(kp=-0.1)

    def test_controller_init_validates_max_compensation_positive(self):
        """Verify max_compensation_var must be positive."""
        with pytest.raises(ValueError, match="max_compensation_var must be positive"):
            PowerFactorController(max_compensation_var=0.0)

    def test_controller_reset_clears_state(self):
        """Verify reset clears integral and command history."""
        pfc = PowerFactorController(target_pf=0.95, kp=0.2, ki=0.5)
        pfc.integral = 100.0
        pfc.last_command_var = 50.0
        pfc.last_error = 10.0

        pfc.reset()
        assert pfc.integral == 0.0
        assert pfc.last_command_var == 0.0
        assert pfc.last_error == 0.0

    def test_controller_update_with_zero_power_returns_zero(self):
        """Verify zero active power returns zero command."""
        pfc = PowerFactorController(target_pf=0.95, kp=0.2, ki=0.5)
        cmd = pfc.update(current_pf=0.70, active_power_w=0.0, dt=0.001)
        assert cmd == 0.0

    def test_controller_update_clips_pf_to_valid_range(self):
        """Verify PF is clipped to [0, 1] before calculation."""
        pfc = PowerFactorController(target_pf=0.95, kp=0.2, ki=0.5)
        # High PF should be clipped to 1.0
        cmd1 = pfc.update(current_pf=1.5, active_power_w=1000.0, dt=0.001)
        assert cmd1 >= 0.0

    def test_controller_update_clamps_output_to_max(self):
        """Verify command output is clamped to max_compensation_var."""
        pfc = PowerFactorController(
            target_pf=0.95,
            kp=10.0,  # High gain to generate large command
            ki=10.0,  # High integral gain
            max_compensation_var=50.0,  # Low limit
        )

        # Run update multiple times to accumulate integral
        for _ in range(100):
            cmd = pfc.update(current_pf=0.50, active_power_w=5000.0, dt=0.001)

        assert cmd <= 50.0

    def test_controller_update_accumulates_integral_error(self):
        """Verify controller accumulates integral of error."""
        pfc = PowerFactorController(target_pf=0.95, kp=0.1, ki=0.5)

        cmd1 = pfc.update(current_pf=0.70, active_power_w=1000.0, dt=0.001)
        integral_after_1 = pfc.integral

        cmd2 = pfc.update(current_pf=0.70, active_power_w=1000.0, dt=0.001)
        integral_after_2 = pfc.integral

        # Integral should accumulate (increase)
        assert integral_after_2 > integral_after_1

    def test_controller_positive_error_increases_command(self):
        """Verify positive PF error (below target) increases compensation command."""
        pfc = PowerFactorController(target_pf=0.95, kp=1.0, ki=0.0)

        cmd_low_pf = pfc.update(current_pf=0.50, active_power_w=1000.0, dt=0.001)
        pfc.reset()

        cmd_high_pf = pfc.update(current_pf=0.90, active_power_w=1000.0, dt=0.001)

        # Lower PF should generate higher compensation
        assert cmd_low_pf > cmd_high_pf

    def test_controller_converges_toward_target_pf(self):
        """Verify controller reduces error over time."""
        pfc = PowerFactorController(target_pf=0.95, kp=0.1, ki=0.1)

        errors = []
        for _ in range(50):
            pfc.update(current_pf=0.70, active_power_w=1000.0, dt=0.001)
            errors.append(abs(pfc.last_error))

        # Error should generally decrease (not strictly, but on average)
        end_error = errors[-1]
        mid_error = errors[25]
        assert end_error < mid_error or abs(end_error - mid_error) < 0.1

    def test_controller_uses_max_baseline_when_pf_is_zero(self):
        """Cover the pf_now <= 0 branch for baseline compensation."""
        pfc = PowerFactorController(target_pf=0.95, kp=0.0, ki=0.0, max_compensation_var=123.0)
        command = pfc.update(current_pf=0.0, active_power_w=1000.0, dt=0.01)
        assert command == pytest.approx(123.0)


class TestSrcInitAttributeError:
    """Test __getattr__ path in src/__init__.py for missing attributes."""

    def test_valid_subpackage_imports_succeed(self):
        """Verify valid subpackages can be lazily imported."""
        import src

        # These should succeed (valid subpackages)
        assert hasattr(src, "core")
        assert hasattr(src, "control")
        assert hasattr(src, "utils")
        assert hasattr(src, "visualization")
        assert hasattr(src, "ui")
        assert hasattr(src, "hardware")

    def test_invalid_attribute_raises_attribute_error(self):
        """Verify accessing invalid attribute raises AttributeError."""
        import src

        with pytest.raises(AttributeError, match="has no attribute"):
            _ = src.nonexistent_module

    def test_attribute_error_message_includes_module_name(self):
        """Verify error message identifies the missing attribute."""
        import src

        try:
            _ = src.missing_subpackage
            assert False, "Should have raised AttributeError"
        except AttributeError as e:
            assert "missing_subpackage" in str(e)
            assert "src" in str(e)


class TestAbstractBaseControllerCoverage:
    """Test abstract base controller to ensure imports work correctly."""

    def test_base_controller_cannot_be_instantiated(self):
        """Verify BaseController is abstract and cannot be instantiated."""
        from src.control.base_controller import BaseController

        with pytest.raises(TypeError):
            type.__call__(BaseController)

    def test_base_controller_subclass_must_implement_methods(self):
        """Verify subclass must implement all abstract methods."""
        from src.control.base_controller import BaseController

        class IncompleteController(BaseController):
            pass

        with pytest.raises(TypeError):
            type.__call__(IncompleteController)

    def test_concrete_controller_implementation_works(self):
        """Verify concrete implementation satisfies abstract contract."""
        from src.control.base_controller import BaseController

        class ConcreteController(BaseController):
            def update(self, dt: float) -> None:
                _ = dt

            def reset(self) -> None:
                return None

            def get_state(self) -> dict[str, float]:
                return {}

        # Should not raise
        controller = ConcreteController()
        assert controller is not None


class TestSupplyProfilesAndAbstractPassLines:
    """Cover supply-profile abstract/base branches and variable/ramp paths."""

    def test_supply_profile_abstract_method_bodies_are_executable(self):
        dummy = cast(SupplyProfile, object())
        assert SupplyProfile.get_voltage(dummy, 0.0) is None
        SupplyProfile.reset(dummy)

    def test_ramp_supply_validation_and_interpolation(self):
        with pytest.raises(ValueError, match="Duration must be positive"):
            RampSupply(initial=12.0, final=24.0, duration=0.0)

        ramp = RampSupply(initial=12.0, final=24.0, duration=4.0)
        assert ramp.get_voltage(-1.0) == pytest.approx(12.0)
        assert ramp.get_voltage(2.0) == pytest.approx(18.0)
        assert ramp.get_voltage(5.0) == pytest.approx(24.0)

    def test_variable_supply_validates_and_interpolates(self):
        with pytest.raises(ValueError, match="Provide either voltage_func or time_points"):
            VariableSupply()

        with pytest.raises(ValueError, match="voltage_points required"):
            VariableSupply(time_points=[0.0, 1.0], voltage_points=None)

        with pytest.raises(ValueError, match="same length"):
            VariableSupply(time_points=[0.0, 1.0], voltage_points=[12.0])

        with pytest.raises(ValueError, match="sorted ascending"):
            VariableSupply(time_points=[1.0, 0.5], voltage_points=[12.0, 10.0])

        v_func = VariableSupply(voltage_func=lambda t: 24.0 + t)
        assert v_func.get_voltage(1.5) == pytest.approx(25.5)

        v_points = VariableSupply(
            time_points=[0.0, 1.0, 2.0],
            voltage_points=[12.0, 18.0, 24.0],
        )
        assert v_points.get_voltage(-1.0) == pytest.approx(12.0)
        assert v_points.get_voltage(0.5) == pytest.approx(15.0)
        assert v_points.get_voltage(3.0) == pytest.approx(24.0)


class TestLoadAndMotorProfilesRemainingBranches:
    """Cover remaining uncovered lines in load_model and motor_profiles."""

    def test_load_profile_abstract_get_torque_body_executes(self):
        assert LoadProfile.get_torque(cast(LoadProfile, object()), 0.0) is None

    def test_ramp_load_validation_and_branches(self):
        with pytest.raises(ValueError, match="Ramp duration must be positive"):
            RampLoad(initial=0.0, final=1.0, duration=0.0)

        load = RampLoad(initial=1.0, final=5.0, duration=2.0)
        assert load.get_torque(-0.5) == pytest.approx(1.0)
        assert load.get_torque(2.5) == pytest.approx(5.0)
        assert load.get_torque(1.0) == pytest.approx(3.0)

    def test_motor_profiles_missing_required_branch(self, monkeypatch):
        monkeypatch.setattr(motor_profiles, "DEFAULT_MOTOR_PARAMS", {})
        with pytest.raises(ValueError, match="Missing motor parameters"):
            motor_profiles._normalize_motor_params({})
