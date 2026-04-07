"""
Comprehensive core-layer tests
================================

Covers every public path, branch, and edge case in:
  - ``src.core.motor_model``   (MotorParameters, BLDCMotor)
  - ``src.core.load_model``    (ConstantLoad, RampLoad, VariableLoad, CyclicLoad)
  - ``src.core.power_model``   (compute_power_metrics, required_reactive_compensation,
                                 compute_efficiency_metrics, recommend_efficiency_adjustments,
                                 PowerFactorController, ConstantSupply, RampSupply,
                                 VariableSupply)
  - ``src.core.simulation_engine`` (SimulationEngine)
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from src.core.load_model import (
    ConstantLoad,
    CyclicLoad,
    LoadProfile,
    RampLoad,
    VariableLoad,
)
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.power_model import (
    ConstantSupply,
    PowerFactorController,
    RampSupply,
    VariableSupply,
    compute_efficiency_metrics,
    compute_power_metrics,
    recommend_efficiency_adjustments,
    required_reactive_compensation,
)
from src.core.simulation_engine import SimulationEngine

# ============================================================================
# MotorParameters
# ============================================================================

class TestMotorParameters:

    def test_defaults(self):
        p = MotorParameters()
        assert p.nominal_voltage == 48.0
        assert p.num_poles == 8
        assert p.poles_pairs == 4

    def test_ld_lq_default_to_phase_inductance(self):
        p = MotorParameters(phase_inductance=0.003)
        assert p.ld == pytest.approx(0.003)
        assert p.lq == pytest.approx(0.003)

    def test_ld_lq_explicit_override(self):
        p = MotorParameters(phase_inductance=0.003, ld=0.002, lq=0.005)
        assert p.ld == pytest.approx(0.002)
        assert p.lq == pytest.approx(0.005)

    def test_negative_fw_coefficient_raises(self):
        with pytest.raises(ValueError, match="non-negative"):
            MotorParameters(flux_weakening_id_coefficient=-0.1)

    def test_fw_min_ratio_zero_raises(self):
        with pytest.raises(ValueError, match="flux_weakening_min_ratio"):
            MotorParameters(flux_weakening_min_ratio=0.0)

    def test_fw_min_ratio_above_one_raises(self):
        with pytest.raises(ValueError, match="flux_weakening_min_ratio"):
            MotorParameters(flux_weakening_min_ratio=1.1)

    def test_valid_fw_min_ratio_one(self):
        p = MotorParameters(flux_weakening_min_ratio=1.0)
        assert p.flux_weakening_min_ratio == 1.0


# ============================================================================
# BLDCMotor — construction validation
# ============================================================================

class TestBLDCMotorConstruction:

    def test_zero_dt_raises(self):
        with pytest.raises(ValueError, match="dt must be positive"):
            BLDCMotor(MotorParameters(), dt=0.0)

    def test_negative_dt_raises(self):
        with pytest.raises(ValueError, match="dt must be positive"):
            BLDCMotor(MotorParameters(), dt=-1e-4)

    def test_zero_inductance_raises(self):
        with pytest.raises(ValueError, match="inductance must be positive"):
            BLDCMotor(MotorParameters(phase_inductance=-1e-9), dt=1e-4)

    def test_dq_model_zero_ld_raises(self):
        with pytest.raises(ValueError, match="Ld.*positive"):
            BLDCMotor(
                MotorParameters(model_type="dq", phase_inductance=0.001, ld=-0.001),
                dt=1e-4,
            )

    def test_dq_model_zero_lq_raises(self):
        with pytest.raises(ValueError, match="Lq.*positive"):
            BLDCMotor(
                MotorParameters(model_type="dq", phase_inductance=0.001, lq=-0.001),
                dt=1e-4,
            )

    def test_initial_state_all_zeros(self):
        m = BLDCMotor(MotorParameters())
        assert m.omega == pytest.approx(0.0)
        assert m.theta == pytest.approx(0.0)
        assert np.allclose(m.currents, 0.0)


# ============================================================================
# BLDCMotor — properties
# ============================================================================

class TestBLDCMotorProperties:

    @pytest.fixture
    def motor(self):
        return BLDCMotor(MotorParameters(back_emf_constant=0.1), dt=5e-5)

    def test_speed_rpm_zero_at_init(self, motor):
        assert motor.speed_rpm == pytest.approx(0.0)

    def test_speed_rpm_positive_when_spinning(self, motor):
        motor.state[3] = 100.0
        assert motor.speed_rpm > 0.0

    def test_back_emf_shape(self, motor):
        assert motor.back_emf.shape == (3,)

    def test_electromagnetic_torque_type(self, motor):
        assert isinstance(motor.electromagnetic_torque, float)

    def test_theta_wraps_to_2pi(self, motor):
        motor.state[4] = 7.0  # > 2π
        assert motor.theta < 2 * math.pi


# ============================================================================
# BLDCMotor — EMF models
# ============================================================================

class TestBLDCMotorEMF:

    def test_trapezoidal_emf_nonzero_at_speed(self):
        m = BLDCMotor(MotorParameters(emf_shape="trapezoidal"), dt=1e-4)
        m.state[3] = 200.0
        emf = m._calculate_back_emf(m.theta)
        assert np.any(np.abs(emf) > 0)

    def test_sinusoidal_emf_at_theta_zero(self):
        p = MotorParameters(emf_shape="sinusoidal", back_emf_constant=0.1)
        m = BLDCMotor(p, dt=1e-4)
        m.state[3] = 100.0
        m.state[4] = 0.0
        emf = m._calculate_back_emf(0.0)
        # At θ=0 electrical, sin(0)=0 for phase A
        assert np.isclose(emf[0], 0.0, atol=1e-9)

    def test_sinusoidal_emf_peak_at_theta_pi_over_2(self):
        p = MotorParameters(emf_shape="sinusoidal", back_emf_constant=0.1, poles_pairs=1)
        m = BLDCMotor(p, dt=1e-4)
        m.state[3] = 100.0
        theta_mech = math.pi / 2.0
        m.state[4] = theta_mech
        emf = m._calculate_back_emf(theta_mech)
        expected_peak = 0.1 * 100.0
        assert np.isclose(emf[0], expected_peak, rtol=1e-6)

    def test_emf_scales_with_speed(self):
        m = BLDCMotor(MotorParameters(), dt=1e-4)
        m.state[4] = 0.5
        m.state[3] = 50.0
        emf_low = np.linalg.norm(m._calculate_back_emf(0.5))
        m.state[3] = 500.0
        emf_high = np.linalg.norm(m._calculate_back_emf(0.5))
        assert emf_high > emf_low

    def test_trapezoidal_static_method_range(self):
        for angle in np.linspace(0, 2 * math.pi, 50):
            v = BLDCMotor._trapezoidal(angle)
            assert -1.0 <= v <= 1.0


# ============================================================================
# BLDCMotor — step and dynamics
# ============================================================================

class TestBLDCMotorDynamics:

    @pytest.fixture
    def motor(self):
        return BLDCMotor(MotorParameters(
            phase_resistance=1.0,
            phase_inductance=0.005,
            back_emf_constant=0.05,
            torque_constant=0.05,
            rotor_inertia=0.001,
            friction_coefficient=0.0001,
            num_poles=8,
            poles_pairs=4,
        ), dt=5e-5)

    def test_step_changes_state(self, motor):
        v = np.array([10.0, -5.0, -5.0])
        motor.step(v, load_torque=0.0)
        assert motor.omega != 0.0 or np.any(motor.currents != 0.0)

    def test_motor_accelerates_under_voltage(self, motor):
        v = np.array([20.0, -10.0, -10.0])
        for _ in range(500):
            motor.step(v, 0.0)
        assert motor.omega > 0.0

    def test_load_torque_slows_motor(self, motor):
        v = np.array([20.0, -10.0, -10.0])
        for _ in range(500):
            motor.step(v, 0.0)
        omega_no_load = motor.omega

        motor.reset()
        for _ in range(500):
            motor.step(v, load_torque=2.0)
        omega_loaded = motor.omega

        assert omega_loaded < omega_no_load

    def test_reset_zeros_state(self, motor):
        motor.step(np.array([10, -5, -5]), 0)
        motor.reset()
        assert motor.omega == pytest.approx(0.0)
        assert np.allclose(motor.currents, 0.0)

    def test_reset_with_initial_speed(self, motor):
        motor.reset(initial_speed=50.0)
        assert motor.omega == pytest.approx(50.0)

    def test_dq_model_accelerates(self):
        p = MotorParameters(model_type="dq", emf_shape="sinusoidal",
                            phase_resistance=1.0, phase_inductance=0.005,
                            back_emf_constant=0.05, torque_constant=0.05,
                            rotor_inertia=0.001, friction_coefficient=0.0001,
                            num_poles=8, poles_pairs=4)
        m = BLDCMotor(p, dt=5e-5)
        m.state[4] = math.pi / 2.0
        for _ in range(200):
            m.step(np.array([10.0, -5.0, -5.0]), 0.0)
        assert m.omega > 0.0

    def test_torque_calculation_returns_float(self):
        m = BLDCMotor(MotorParameters(), dt=1e-4)
        t = m._calculate_torque(np.array([1.0, -0.5, -0.5]), theta=0.3)
        assert isinstance(t, float)


# ============================================================================
# LoadProfile — ConstantLoad
# ============================================================================

class TestConstantLoad:

    def test_returns_constant_always(self):
        load = ConstantLoad(torque=3.5)
        for t in [0.0, 1.0, 100.0, -1.0]:
            assert load.get_torque(t) == pytest.approx(3.5)

    def test_default_torque_zero(self):
        assert ConstantLoad().get_torque(0.0) == pytest.approx(0.0)

    def test_reset_is_noop(self):
        load = ConstantLoad(torque=2.0)
        load.reset()
        assert load.get_torque(0.0) == pytest.approx(2.0)

    def test_negative_torque(self):
        load = ConstantLoad(torque=-1.5)
        assert load.get_torque(5.0) == pytest.approx(-1.5)


# ============================================================================
# LoadProfile — RampLoad
# ============================================================================

class TestRampLoad:

    def test_zero_duration_raises(self):
        with pytest.raises(ValueError, match="positive"):
            RampLoad(initial=0.0, final=1.0, duration=0.0)

    def test_negative_duration_raises(self):
        with pytest.raises(ValueError):
            RampLoad(initial=0.0, final=1.0, duration=-1.0)

    def test_at_t_zero(self):
        load = RampLoad(initial=1.0, final=5.0, duration=4.0)
        assert load.get_torque(0.0) == pytest.approx(1.0)

    def test_at_t_negative(self):
        load = RampLoad(initial=2.0, final=4.0, duration=2.0)
        assert load.get_torque(-1.0) == pytest.approx(2.0)

    def test_at_t_midpoint(self):
        load = RampLoad(initial=0.0, final=4.0, duration=4.0)
        assert load.get_torque(2.0) == pytest.approx(2.0)

    def test_at_t_end(self):
        load = RampLoad(initial=0.0, final=3.0, duration=2.0)
        assert load.get_torque(2.0) == pytest.approx(3.0)

    def test_after_ramp_clamps_to_final(self):
        load = RampLoad(initial=0.0, final=3.0, duration=2.0)
        assert load.get_torque(10.0) == pytest.approx(3.0)


# ============================================================================
# LoadProfile — VariableLoad
# ============================================================================

class TestVariableLoad:

    def test_no_args_raises(self):
        with pytest.raises(ValueError):
            VariableLoad()

    def test_time_without_torque_raises(self):
        with pytest.raises(ValueError):
            VariableLoad(time_points=[0.0, 1.0])

    def test_mismatched_lengths_raises(self):
        with pytest.raises(ValueError, match="same length"):
            VariableLoad(time_points=[0.0, 1.0], torque_points=[1.0])

    def test_non_ascending_time_raises(self):
        with pytest.raises(ValueError, match="ascending"):
            VariableLoad(time_points=[1.0, 0.0], torque_points=[0.0, 1.0])

    def test_function_mode(self):
        load = VariableLoad(torque_func=lambda t: t * 2.0)
        assert load.get_torque(3.0) == pytest.approx(6.0)

    def test_data_points_interpolation(self):
        load = VariableLoad(
            time_points=[0.0, 2.0, 4.0],
            torque_points=[0.0, 10.0, 0.0],
        )
        assert load.get_torque(1.0) == pytest.approx(5.0)
        assert load.get_torque(3.0) == pytest.approx(5.0)

    def test_data_points_before_range(self):
        load = VariableLoad(time_points=[1.0, 2.0], torque_points=[5.0, 10.0])
        assert load.get_torque(0.0) == pytest.approx(5.0)

    def test_data_points_after_range(self):
        load = VariableLoad(time_points=[0.0, 1.0], torque_points=[2.0, 8.0])
        assert load.get_torque(99.0) == pytest.approx(8.0)

    def test_single_data_point(self):
        load = VariableLoad(time_points=[0.5], torque_points=[7.0])
        assert load.get_torque(0.5) == pytest.approx(7.0)
        assert load.get_torque(10.0) == pytest.approx(7.0)


# ============================================================================
# LoadProfile — CyclicLoad
# ============================================================================

class TestCyclicLoad:

    def test_negative_frequency_raises(self):
        with pytest.raises(ValueError, match="Frequency"):
            CyclicLoad(frequency=-1.0)

    def test_zero_frequency_dc(self):
        load = CyclicLoad(offset=2.0, amplitude=1.0, frequency=0.0)
        assert load.get_torque(0.0) == pytest.approx(2.0)
        assert load.get_torque(100.0) == pytest.approx(2.0)

    def test_peak_at_quarter_period(self):
        """At t = T/4 = 1/(4f), sin = 1 → torque = offset + amplitude."""
        load = CyclicLoad(offset=1.0, amplitude=2.0, frequency=1.0, phase=0.0)
        assert load.get_torque(0.25) == pytest.approx(3.0, rel=1e-5)

    def test_offset_only(self):
        load = CyclicLoad(offset=5.0, amplitude=0.0, frequency=1.0)
        for t in [0, 0.5, 1.0]:
            assert load.get_torque(t) == pytest.approx(5.0)

    def test_is_load_profile_subclass(self):
        assert isinstance(CyclicLoad(), LoadProfile)


# ============================================================================
# compute_power_metrics
# ============================================================================

class TestComputePowerMetrics:

    def test_unity_pf_in_phase(self):
        t = np.linspace(0, 2 * math.pi, 2000, endpoint=False)
        v = 230.0 * np.sin(t)
        i = 10.0 * np.sin(t)
        m = compute_power_metrics(v, i)
        assert m["power_factor"] == pytest.approx(1.0, abs=1e-3)
        assert m["reactive_power_var"] == pytest.approx(0.0, abs=2.0)

    def test_zero_pf_quadrature(self):
        t = np.linspace(0, 2 * math.pi, 2000, endpoint=False)
        v = np.sin(t)
        i = np.cos(t)   # 90° phase shift
        m = compute_power_metrics(v, i)
        assert abs(m["power_factor"]) < 0.05
        assert m["active_power_w"] == pytest.approx(0.0, abs=0.05)

    def test_mismatched_shapes_raises(self):
        with pytest.raises(ValueError, match="same shape"):
            compute_power_metrics(np.ones(10), np.ones(5))

    def test_empty_arrays_raises(self):
        with pytest.raises(ValueError, match="non-empty"):
            compute_power_metrics(np.array([]), np.array([]))

    def test_all_zero_returns_zero_pf(self):
        m = compute_power_metrics(np.zeros(100), np.zeros(100))
        assert m["power_factor"] == pytest.approx(0.0)
        assert m["apparent_power_va"] == pytest.approx(0.0)

    def test_output_keys(self):
        m = compute_power_metrics(np.ones(10), np.ones(10))
        for key in ("voltage_rms_v", "current_rms_a", "active_power_w",
                    "apparent_power_va", "reactive_power_var", "power_factor"):
            assert key in m

    def test_pf_clipped_to_minus_one_to_one(self):
        # Contrived case: force raw ratio > 1
        m = compute_power_metrics(np.array([1.0, 1.0]), np.array([1.0, 1.0]))
        assert -1.0 <= m["power_factor"] <= 1.0


# ============================================================================
# required_reactive_compensation
# ============================================================================

class TestRequiredReactiveCompensation:

    def test_basic_compensation(self):
        r = required_reactive_compensation(
            active_power_w=1000.0,
            current_pf=0.70,
            target_pf=0.95,
        )
        assert r["required_compensation_var"] > 0.0
        assert r["target_pf"] == pytest.approx(0.95)

    def test_no_compensation_needed_when_pf_meets_target(self):
        r = required_reactive_compensation(
            active_power_w=500.0,
            current_pf=0.95,
            target_pf=0.95,
        )
        assert r["required_compensation_var"] == pytest.approx(0.0, abs=1e-6)

    def test_nonpositive_power_raises(self):
        with pytest.raises(ValueError):
            required_reactive_compensation(0.0, 0.8, 0.95)

    def test_pf_out_of_range_raises(self):
        with pytest.raises(ValueError):
            required_reactive_compensation(100.0, 1.1, 0.95)

    def test_target_less_than_current_raises(self):
        with pytest.raises(ValueError):
            required_reactive_compensation(100.0, 0.9, 0.8)

    def test_output_keys(self):
        r = required_reactive_compensation(100.0, 0.7, 0.9)
        for key in ("active_power_w", "current_pf", "target_pf",
                    "current_reactive_var", "target_reactive_var",
                    "required_compensation_var"):
            assert key in r


# ============================================================================
# compute_efficiency_metrics
# ============================================================================

class TestComputeEfficiencyMetrics:

    def test_positive_efficiency(self):
        m = compute_efficiency_metrics(1000.0, 5.0, 150.0)
        assert 0.0 < m["efficiency"] <= 1.0

    def test_zero_input_power_gives_zero_efficiency(self):
        m = compute_efficiency_metrics(0.0, 1.0, 10.0)
        assert m["efficiency"] == pytest.approx(0.0)

    def test_regenerative_power_detected(self):
        # negative shaft power = regeneration
        m = compute_efficiency_metrics(0.0, -2.0, 100.0)
        assert m["regenerative_power_w"] > 0.0

    def test_efficiency_capped_at_one(self):
        m = compute_efficiency_metrics(100.0, 1000.0, 1000.0)
        assert m["efficiency"] <= 1.0

    def test_output_keys(self):
        m = compute_efficiency_metrics(100.0, 1.0, 50.0)
        for key in ("electrical_input_power_w", "mechanical_output_power_w",
                    "regenerative_power_w", "total_loss_power_w", "efficiency"):
            assert key in m


# ============================================================================
# recommend_efficiency_adjustments
# ============================================================================

class TestRecommendEfficiencyAdjustments:

    def test_good_operating_point_no_action_needed(self):
        r = recommend_efficiency_adjustments(efficiency=0.95, power_factor=0.97)
        assert not r["needs_attention"]
        assert r["suggestions"]

    def test_high_device_drop_triggers_suggestion(self):
        r = recommend_efficiency_adjustments(
            efficiency=0.75, power_factor=0.95, device_drop_v=0.8
        )
        assert any("device drop" in s.lower() for s in r["suggestions"])

    def test_high_dead_time_triggers_suggestion(self):
        r = recommend_efficiency_adjustments(
            efficiency=0.75, power_factor=0.95, dead_time_fraction=0.02
        )
        assert any("dead" in s.lower() for s in r["suggestions"])

    def test_high_conduction_resistance_triggers_suggestion(self):
        r = recommend_efficiency_adjustments(
            efficiency=0.75, power_factor=0.95, conduction_resistance_ohm=0.05
        )
        assert any("conduction" in s.lower() for s in r["suggestions"])

    def test_high_switching_frequency_triggers_suggestion(self):
        r = recommend_efficiency_adjustments(
            efficiency=0.75, power_factor=0.95,
            switching_frequency_hz=15000.0,
            switching_loss_coeff_v_per_a_khz=0.5,
        )
        assert any("switching" in s.lower() for s in r["suggestions"])

    def test_low_pf_triggers_pfc_suggestion(self):
        r = recommend_efficiency_adjustments(efficiency=0.95, power_factor=0.80)
        assert any("power factor" in s.lower() for s in r["suggestions"])

    def test_needs_attention_when_efficiency_below_target(self):
        r = recommend_efficiency_adjustments(
            efficiency=0.80, power_factor=0.95, target_efficiency=0.90
        )
        assert r["needs_attention"]


# ============================================================================
# PowerFactorController
# ============================================================================

class TestPowerFactorController:

    def test_invalid_target_pf_zero_raises(self):
        with pytest.raises(ValueError):
            PowerFactorController(target_pf=0.0)

    def test_invalid_target_pf_above_one_raises(self):
        with pytest.raises(ValueError):
            PowerFactorController(target_pf=1.1)

    def test_negative_kp_raises(self):
        with pytest.raises(ValueError):
            PowerFactorController(kp=-1.0)

    def test_negative_ki_raises(self):
        with pytest.raises(ValueError):
            PowerFactorController(ki=-1.0)

    def test_nonpositive_max_var_raises(self):
        with pytest.raises(ValueError):
            PowerFactorController(max_compensation_var=0.0)

    def test_update_returns_nonnegative(self):
        pfc = PowerFactorController(target_pf=0.95, max_compensation_var=5000.0)
        cmd = pfc.update(current_pf=0.70, active_power_w=1000.0, dt=1e-4)
        assert cmd >= 0.0

    def test_update_returns_zero_when_no_power(self):
        pfc = PowerFactorController(target_pf=0.95)
        cmd = pfc.update(current_pf=0.80, active_power_w=0.0, dt=1e-4)
        assert cmd == 0.0

    def test_update_returns_zero_when_dt_zero(self):
        pfc = PowerFactorController(target_pf=0.95)
        cmd = pfc.update(current_pf=0.80, active_power_w=500.0, dt=0.0)
        assert cmd == 0.0

    def test_reset_clears_integral(self):
        pfc = PowerFactorController(target_pf=0.95)
        for _ in range(100):
            pfc.update(current_pf=0.70, active_power_w=500.0, dt=1e-4)
        pfc.reset()
        assert pfc.integral == pytest.approx(0.0)
        assert pfc.last_command_var == pytest.approx(0.0)

    def test_command_capped_at_max_var(self):
        pfc = PowerFactorController(target_pf=0.95, max_compensation_var=100.0)
        cmd = pfc.update(current_pf=0.01, active_power_w=1e6, dt=1.0)
        assert cmd <= 100.0

    def test_command_zero_when_pf_already_meets_target(self):
        pfc = PowerFactorController(target_pf=0.90)
        cmd = pfc.update(current_pf=0.95, active_power_w=1000.0, dt=1e-4)
        # baseline = 0 since pf >= target; trim might be negative → clipped to 0
        assert cmd >= 0.0


# ============================================================================
# SupplyProfile subclasses
# ============================================================================

class TestConstantSupply:

    def test_always_constant(self):
        s = ConstantSupply(48.0)
        for t in [-1.0, 0.0, 1.0, 1000.0]:
            assert s.get_voltage(t) == pytest.approx(48.0)

    def test_reset_noop(self):
        s = ConstantSupply(24.0)
        s.reset()
        assert s.get_voltage(0.0) == pytest.approx(24.0)


class TestRampSupply:

    def test_zero_duration_raises(self):
        with pytest.raises(ValueError):
            RampSupply(initial=48.0, final=24.0, duration=0.0)

    def test_at_t_zero(self):
        s = RampSupply(48.0, 24.0, 2.0)
        assert s.get_voltage(0.0) == pytest.approx(48.0)

    def test_at_midpoint(self):
        s = RampSupply(48.0, 24.0, 2.0)
        assert s.get_voltage(1.0) == pytest.approx(36.0)

    def test_after_ramp(self):
        s = RampSupply(48.0, 24.0, 2.0)
        assert s.get_voltage(10.0) == pytest.approx(24.0)

    def test_before_ramp(self):
        s = RampSupply(48.0, 24.0, 2.0)
        assert s.get_voltage(-1.0) == pytest.approx(48.0)


class TestVariableSupply:

    def test_no_args_raises(self):
        with pytest.raises(ValueError):
            VariableSupply()

    def test_time_without_voltage_raises(self):
        with pytest.raises(ValueError):
            VariableSupply(time_points=[0.0, 1.0])

    def test_mismatched_lengths_raises(self):
        with pytest.raises(ValueError):
            VariableSupply(time_points=[0.0, 1.0], voltage_points=[1.0])

    def test_non_ascending_raises(self):
        with pytest.raises(ValueError):
            VariableSupply(time_points=[1.0, 0.0], voltage_points=[1.0, 2.0])

    def test_function_mode(self):
        s = VariableSupply(voltage_func=lambda t: 48.0 - t)
        assert s.get_voltage(5.0) == pytest.approx(43.0)

    def test_data_interpolation(self):
        s = VariableSupply(time_points=[0.0, 4.0], voltage_points=[48.0, 24.0])
        assert s.get_voltage(2.0) == pytest.approx(36.0)

    def test_clamp_before_start(self):
        s = VariableSupply(time_points=[1.0, 2.0], voltage_points=[10.0, 20.0])
        assert s.get_voltage(0.0) == pytest.approx(10.0)

    def test_clamp_after_end(self):
        s = VariableSupply(time_points=[0.0, 1.0], voltage_points=[10.0, 20.0])
        assert s.get_voltage(99.0) == pytest.approx(20.0)


# ============================================================================
# SimulationEngine
# ============================================================================

def _make_engine(dt: float = 5e-5) -> SimulationEngine:
    p = MotorParameters(
        nominal_voltage=48.0,
        phase_resistance=2.5,
        phase_inductance=0.005,
        back_emf_constant=0.1,
        torque_constant=0.1,
        rotor_inertia=0.0005,
        friction_coefficient=0.001,
        num_poles=8,
        poles_pairs=4,
    )
    return SimulationEngine(BLDCMotor(p, dt=dt), ConstantLoad(0.5), dt=dt)


class TestSimulationEngineBasics:

    def test_initial_time_zero(self):
        e = _make_engine()
        assert e.time == pytest.approx(0.0)
        assert e.step_count == 0

    def test_step_advances_time(self):
        e = _make_engine()
        e.step(np.zeros(3), log_data=True)
        assert e.time > 0.0
        assert e.step_count == 1

    def test_history_is_empty_before_steps(self):
        e = _make_engine()
        h = e.get_history()
        assert len(h["time"]) == 0

    def test_history_grows_with_steps(self):
        e = _make_engine()
        for _ in range(50):
            e.step(np.ones(3) * 5.0, log_data=True)
        h = e.get_history()
        assert len(h["time"]) == 50

    def test_history_keys_present(self):
        e = _make_engine()
        e.step(np.zeros(3), log_data=True)
        h = e.get_history()
        for key in ("time", "currents_a", "omega", "theta", "torque", "voltages_a"):
            assert key in h

    def test_reset_clears_history(self):
        e = _make_engine()
        e.step(np.zeros(3), log_data=True)
        e.reset()
        assert e.time == pytest.approx(0.0)
        assert len(e.get_history()["time"]) == 0

    def test_step_without_logging_does_not_grow_history(self):
        e = _make_engine()
        e.step(np.zeros(3), log_data=False)
        assert len(e.get_history()["time"]) == 0


class TestSimulationEngineSupplyProfile:

    def test_ramp_supply_logged(self):
        p = MotorParameters()
        m = BLDCMotor(p, dt=1e-4)
        e = SimulationEngine(m, ConstantLoad(0.0), dt=1e-4,
                             supply_profile=RampSupply(48.0, 24.0, 0.01))
        e.step(np.zeros(3), log_data=True)
        h = e.get_history()
        assert "supply_voltage" in h
        assert h["supply_voltage"][0] == pytest.approx(48.0)


class TestSimulationEngineStabilityAdvisory:

    def test_stable_dt(self):
        e = _make_engine(dt=5e-5)
        adv = e.get_numerical_stability_advisory()
        assert adv["severity"] == "stable"
        assert adv["is_stable"] is True

    def test_unstable_dt(self):
        # R=2.5, L=0.005 → τ=0.002 s, dt_limit=0.00557 s
        e = _make_engine(dt=7e-3)
        adv = e.get_numerical_stability_advisory()
        assert adv["severity"] == "unstable"
        assert adv["is_stable"] is False

    def test_marginal_dt(self):
        e = _make_engine(dt=4e-3)
        adv = e.get_numerical_stability_advisory()
        assert adv["severity"] == "marginal"

    def test_advisory_contains_required_keys(self):
        adv = _make_engine().get_numerical_stability_advisory()
        for key in ("severity", "is_stable", "dt_limit_s", "tau_e_s"):
            assert key in adv
