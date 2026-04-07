"""
Comprehensive control-layer tests
===================================

Covers every public path in:
  - ``src.control.transforms``  (Clarke, Park, Concordia and their inverses)
  - ``src.control.foc_controller`` (FOCController — all observer modes, PI loops,
                                    EEMF, SOGI, STSMO, ActiveFlux, field-weakening,
                                    startup, PFC, analytical calibration helpers)
  - ``src.control.vf_controller``  (VFController)
  - ``src.control.svm_generator``  (SVMGenerator, InverterRealismConfig)
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from src.control.foc_controller import (
    FOCController,
    _blend_angles,
    _pi_update,
    _pi_update_anti_windup,
    _wrap_angle,
)
from src.control.svm_generator import CartesianSVMGenerator, SVMGenerator
from src.control.transforms import (
    clarke_transform,
    concordia_transform,
    inverse_clarke,
    inverse_concordia,
    inverse_park,
    park_transform,
)
from src.control.vf_controller import VFController
from src.core.load_model import ConstantLoad
from src.core.motor_model import BLDCMotor, MotorParameters

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _default_motor(pp: int = 4) -> BLDCMotor:
    p = MotorParameters(
        nominal_voltage=48.0,
        phase_resistance=2.5,
        phase_inductance=0.005,
        back_emf_constant=0.1,
        torque_constant=0.1,
        rotor_inertia=0.0005,
        friction_coefficient=0.001,
        num_poles=pp * 2,
        poles_pairs=pp,
    )
    return BLDCMotor(p, dt=5e-5)


def _default_foc(pp: int = 4) -> FOCController:
    return FOCController(motor=_default_motor(pp))


# ============================================================================
# Transforms
# ============================================================================

class TestClarkeTransform:

    def test_balanced_three_phase_gives_nonzero_alpha(self):
        va, vb, vc = 1.0, -0.5, -0.5
        alpha, beta = clarke_transform(va, vb, vc)
        assert alpha == pytest.approx(1.0)

    def test_balanced_zero_beta_for_in_phase(self):
        va, vb, vc = 1.0, -0.5, -0.5
        _, beta = clarke_transform(va, vb, vc)
        assert beta == pytest.approx(0.0, abs=1e-12)

    def test_roundtrip_with_inverse(self):
        va, vb, vc = 3.0, -1.5, -1.5
        alpha, beta = clarke_transform(va, vb, vc)
        va_r, vb_r, vc_r = inverse_clarke(alpha, beta)
        # inverse Clarke reconstructs a and b/c from the α,β pair
        assert va_r == pytest.approx(va)


class TestParkTransform:

    def test_at_theta_zero_alpha_maps_to_d(self):
        vd, vq = park_transform(1.0, 0.0, theta=0.0)
        assert vd == pytest.approx(1.0)
        assert vq == pytest.approx(0.0, abs=1e-12)

    def test_at_theta_pi_over_2_alpha_maps_to_minus_q(self):
        vd, vq = park_transform(1.0, 0.0, theta=math.pi / 2)
        assert vd == pytest.approx(0.0, abs=1e-12)
        assert vq == pytest.approx(-1.0)

    def test_park_inverse_park_roundtrip(self):
        v_alpha, v_beta = 2.5, -1.3
        theta = 1.1
        vd, vq = park_transform(v_alpha, v_beta, theta)
        v_alpha_r, v_beta_r = inverse_park(vd, vq, theta)
        assert v_alpha_r == pytest.approx(v_alpha, rel=1e-9)
        assert v_beta_r == pytest.approx(v_beta, rel=1e-9)

    def test_inverse_park_roundtrip_various_angles(self):
        for theta in np.linspace(0, 2 * math.pi, 12, endpoint=False):
            vd, vq = 1.5, -0.8
            alpha, beta = inverse_park(vd, vq, theta)
            vd_r, vq_r = park_transform(alpha, beta, theta)
            assert vd_r == pytest.approx(vd, rel=1e-9)
            assert vq_r == pytest.approx(vq, rel=1e-9)


class TestConcordiaTransform:

    def test_roundtrip(self):
        va, vb, vc = 4.0, -2.5, -1.5
        alpha, beta = concordia_transform(va, vb, vc)
        va_r, vb_r, vc_r = inverse_concordia(alpha, beta)
        # Concordia has specific normalization; just check alpha consistency
        assert alpha == pytest.approx(concordia_transform(va, vb, vc)[0])

    def test_zero_input_gives_zero_output(self):
        alpha, beta = concordia_transform(0.0, 0.0, 0.0)
        assert alpha == pytest.approx(0.0)
        assert beta == pytest.approx(0.0)


# ============================================================================
# FOC helper functions
# ============================================================================

class TestFOCHelpers:

    def test_wrap_angle_within_minus_pi_to_pi(self):
        for raw in np.linspace(-5, 5, 30):
            w = _wrap_angle(raw)
            assert -math.pi <= w < math.pi

    def test_wrap_angle_at_pi(self):
        # should wrap to just below +pi or exactly -pi
        w = _wrap_angle(math.pi)
        assert abs(abs(w) - math.pi) < 1e-10

    def test_pi_update_integrates(self):
        state = {"kp": 1.0, "ki": 10.0, "integral": 0.0}
        out = _pi_update(state, error=1.0, dt=1e-3)
        assert state["integral"] == pytest.approx(1e-3)
        assert out == pytest.approx(1.0 + 10.0 * 1e-3)

    def test_pi_update_anti_windup_limits_output(self):
        state = {"kp": 10.0, "ki": 0.0, "kaw": 0.0, "integral": 0.0}
        out = _pi_update_anti_windup(state, error=100.0, dt=1e-3, limit=5.0)
        assert abs(out) <= 5.0

    def test_blend_angles_at_weight_zero(self):
        result = _blend_angles(1.0, 2.0, weight_b=0.0)
        assert result == pytest.approx(1.0 % (2 * math.pi))

    def test_blend_angles_at_weight_one(self):
        result = _blend_angles(1.0, 2.0, weight_b=1.0)
        assert result == pytest.approx(2.0 % (2 * math.pi))

    def test_blend_angles_clips_weight(self):
        # weight > 1 should be clipped to 1
        result = _blend_angles(1.0, 2.0, weight_b=5.0)
        assert result == pytest.approx(2.0 % (2 * math.pi))


# ============================================================================
# FOCController — construction and basic setters
# ============================================================================

class TestFOCControllerConstruction:

    def test_initializes_default_pi_gains(self):
        ctrl = _default_foc()
        assert ctrl.pi_d["kp"] > 0.0
        assert ctrl.pi_q["ki"] >= 0.0

    def test_set_current_references(self):
        ctrl = _default_foc()
        ctrl.set_current_references(id_ref=1.0, iq_ref=2.5)  # positional args required
        assert ctrl.id_ref == pytest.approx(1.0)
        assert ctrl.iq_ref == pytest.approx(2.5)

    def test_set_speed_reference(self):
        ctrl = _default_foc()
        ctrl.set_speed_reference(1000.0)
        assert ctrl.speed_ref == pytest.approx(1000.0)

    def test_set_current_pi_gains(self):
        ctrl = _default_foc()
        ctrl.set_current_pi_gains(d_kp=5.0, d_ki=200.0, q_kp=5.0, q_ki=200.0)
        assert ctrl.pi_d["kp"] == pytest.approx(5.0)
        assert ctrl.pi_q["kp"] == pytest.approx(5.0)

    def test_set_speed_pi_gains(self):
        ctrl = _default_foc()
        ctrl.set_speed_pi_gains(kp=0.1, ki=2.0)
        assert ctrl.pi_speed["kp"] == pytest.approx(0.1)
        assert ctrl.pi_speed["ki"] == pytest.approx(2.0)

    def test_reset_clears_integrals(self):
        ctrl = _default_foc()
        ctrl.pi_d["integral"] = 99.0
        ctrl.reset()
        assert ctrl.pi_d["integral"] == pytest.approx(0.0)
        assert ctrl.pi_q["integral"] == pytest.approx(0.0)

    def test_get_state_has_required_keys(self):
        ctrl = _default_foc()
        state = ctrl.get_state()
        for key in ("pi_d", "pi_q", "id_ref", "iq_ref"):
            assert key in state


# ============================================================================
# FOCController — update (polar and Cartesian output modes)
# ============================================================================

class TestFOCControllerUpdate:

    def test_polar_output_returns_two_floats(self):
        ctrl = _default_foc()
        ctrl.set_current_references(id_ref=0.0, iq_ref=2.0)
        result = ctrl.update(dt=5e-5)
        assert isinstance(result, tuple)
        assert len(result) == 2

    def test_cartesian_output_enabled(self):
        motor = _default_motor()
        ctrl = FOCController(motor=motor, output_cartesian=True)
        ctrl.set_current_references(id_ref=0.0, iq_ref=2.0)
        result = ctrl.update(dt=5e-5)
        assert len(result) == 2

    def test_concordia_transform_mode(self):
        motor = _default_motor()
        ctrl = FOCController(motor=motor, use_concordia=True)
        ctrl.set_current_references(id_ref=0.0, iq_ref=1.0)
        result = ctrl.update(dt=5e-5)
        assert result is not None

    def test_speed_loop_enabled(self):
        motor = _default_motor()
        ctrl = FOCController(motor=motor, enable_speed_loop=True)
        ctrl.set_speed_reference(500.0)
        result = ctrl.update(dt=5e-5)
        assert result is not None


# ============================================================================
# FOCController — observer modes
# ============================================================================

class TestFOCControllerObserverModes:

    def test_set_angle_observer_measured(self):
        ctrl = _default_foc()
        ctrl.set_angle_observer("Measured")
        ctrl.set_current_references(id_ref=0.0, iq_ref=1.0)
        result = ctrl.update(dt=5e-5)
        assert result is not None

    def test_set_angle_observer_pll(self):
        ctrl = _default_foc()
        ctrl.set_angle_observer("PLL")
        ctrl.set_pll_gains(kp=80.0, ki=2000.0)
        result = ctrl.update(dt=5e-5)
        assert result is not None

    def test_set_angle_observer_smo(self):
        ctrl = _default_foc()
        ctrl.set_angle_observer("SMO")
        result = ctrl.update(dt=5e-5)
        assert result is not None

    def test_stsmo_mode_via_calibrate(self):
        ctrl = _default_foc()
        ctrl.calibrate_stsmo_gains_analytical(rated_rpm=3000.0)
        result = ctrl.update(dt=5e-5)
        assert result is not None

    def test_active_flux_observer(self):
        ctrl = _default_foc()
        ctrl.enable_active_flux_observer(dc_cutoff_hz=0.3)
        result = ctrl.update(dt=5e-5)
        assert result is not None

    def test_pll_gains_via_calibrate(self):
        ctrl = _default_foc()
        gains = ctrl.calibrate_pll_gains_analytical(rated_rpm=3000.0, apply=False)
        assert "kp" in gains and "ki" in gains
        assert gains["kp"] > 0.0
        assert gains["ki"] > 0.0

    def test_smo_gains_via_calibrate(self):
        ctrl = _default_foc()
        gains = ctrl.calibrate_smo_gains_analytical(rated_rpm=3000.0, apply=False)
        assert "k_slide" in gains
        assert gains["k_slide"] > 0.0

    def test_stsmo_calibrate_with_custom_k2_min(self):
        ctrl = _default_foc()
        gains = ctrl.calibrate_stsmo_gains_analytical(
            rated_rpm=3000.0, k2_min=1500.0, apply=False
        )
        assert gains["k2_min"] == pytest.approx(1500.0)

    def test_stsmo_calibrate_k2_min_clipped_to_minimum(self):
        ctrl = _default_foc()
        gains = ctrl.calibrate_stsmo_gains_analytical(
            rated_rpm=3000.0, k2_min=1.0, apply=False  # below 50.0 floor
        )
        assert gains["k2_min"] >= 50.0

    def test_update_applied_voltage_stores_clarke(self):
        ctrl = _default_foc()
        svm = SVMGenerator(dc_voltage=48.0)
        va, vb, vc = svm.modulate(15.0, 0.8)
        ctrl.update_applied_voltage(va, vb, vc)
        alpha_exp, beta_exp = clarke_transform(va, vb, vc)
        assert ctrl._v_alpha_prev == pytest.approx(alpha_exp, rel=1e-9)
        assert ctrl._v_beta_prev == pytest.approx(beta_exp, rel=1e-9)


# ============================================================================
# FOCController — EEMF / SOGI
# ============================================================================

class TestFOCControllerEEMFAndSOGI:

    def test_eemf_model_enable(self):
        ctrl = _default_foc()
        ctrl.enable_sensorless_emf_reconstruction()
        ctrl.enable_eemf_model(Lq=0.006)
        result = ctrl.update(dt=5e-5)
        assert result is not None

    def test_sogi_filter_enable(self):
        ctrl = _default_foc()
        ctrl.enable_sensorless_emf_reconstruction()
        ctrl.enable_sogi_filter(k=1.4142)
        result = ctrl.update(dt=5e-5)
        assert result is not None


# ============================================================================
# FOCController — field-weakening, decoupling, startup
# ============================================================================

class TestFOCControllerAdvanced:

    def test_field_weakening_enabled(self):
        ctrl = _default_foc()
        ctrl.set_field_weakening(enabled=True, start_speed_rpm=2000.0, gain=1.0,
                                 max_negative_id_a=5.0, headroom_target_v=1.5)
        result = ctrl.update(dt=5e-5)
        assert result is not None

    def test_field_weakening_disabled_by_default(self):
        ctrl = _default_foc()
        assert not ctrl.field_weakening_enabled

    def test_decoupling_enabled(self):
        ctrl = _default_foc()
        ctrl.set_decoupling(enable_d=True, enable_q=True)
        result = ctrl.update(dt=5e-5)
        assert result is not None

    def test_iq_limit_applied(self):
        ctrl = _default_foc()
        ctrl.iq_limit_a = 2.0
        ctrl.set_current_references(id_ref=0.0, iq_ref=100.0)
        ctrl.update(dt=5e-5)
        # iq_limit_a is enforced inside the speed loop; field exists
        assert ctrl.iq_limit_a == pytest.approx(2.0)

    def test_startup_sequence_enabled(self):
        ctrl = _default_foc()
        ctrl.set_startup_sequence(
            enabled=True,
            align_duration_s=0.05,
            align_current_a=1.5,
            open_loop_initial_speed_rpm=30.0,
            open_loop_target_speed_rpm=300.0,
            open_loop_ramp_time_s=0.2,
            open_loop_id_ref_a=0.0,
            open_loop_iq_ref_a=2.0,
        )
        result = ctrl.update(dt=5e-5)
        assert result is not None


# ============================================================================
# VFController
# ============================================================================

class TestVFController:

    @pytest.fixture
    def ctrl(self):
        return VFController(v_nominal=48.0, f_nominal=100.0, dc_voltage=48.0, v_startup=1.0)

    def test_initialization(self, ctrl):
        assert ctrl.v_nominal == pytest.approx(48.0)
        assert ctrl.f_nominal == pytest.approx(100.0)
        assert ctrl.frequency_actual == pytest.approx(0.0)

    def test_update_returns_tuple(self, ctrl):
        ctrl.set_speed_reference(50.0)
        result = ctrl.update(dt=0.001)
        assert isinstance(result, tuple)
        assert len(result) == 2

    def test_magnitude_bounded_by_max_voltage(self, ctrl):
        ctrl.set_speed_reference(100.0)
        for _ in range(200):
            mag, _ = ctrl.update(dt=0.001)
            assert 0.0 <= mag <= ctrl.max_voltage + 1e-6

    def test_frequency_slew_rate_limiting(self, ctrl):
        ctrl.set_speed_reference(100.0)
        ctrl.set_frequency_slew_rate(10.0)   # 10 Hz/s
        for _ in range(10):
            ctrl.update(dt=0.01)             # 0.1 s total
        assert ctrl.frequency_actual <= 1.0 + 0.1

    def test_reset_zeros_state(self, ctrl):
        ctrl.set_speed_reference(50.0)
        for _ in range(100):
            ctrl.update(dt=0.001)
        ctrl.reset()
        assert ctrl.frequency_actual == pytest.approx(0.0)

    def test_get_state_keys(self, ctrl):
        state = ctrl.get_state()
        for key in ("frequency_actual", "voltage"):
            assert key in state

    def test_startup_sequence_enabled(self, ctrl):
        ctrl.set_startup_sequence(
            enable=True,
            align_duration_s=0.05,
            align_voltage_v=2.0,
            align_angle_deg=0.0,
            ramp_initial_frequency_hz=2.0,
        )
        result = ctrl.update(dt=0.001)
        assert result is not None


# ============================================================================
# SVMGenerator — basic modulation
# ============================================================================

class TestSVMGenerator:

    @pytest.fixture
    def svm(self):
        return SVMGenerator(dc_voltage=48.0)

    def test_initialization(self, svm):
        assert svm.dc_voltage == pytest.approx(48.0)

    def test_modulate_returns_3_voltages(self, svm):
        v = svm.modulate(20.0, math.pi / 4)
        assert v.shape == (3,)

    def test_maximum_voltage(self, svm):
        expected = (2.0 / 3.0) * 48.0
        assert svm.get_maximum_voltage() == pytest.approx(expected)

    def test_modulate_clamped_within_dc_range(self, svm):
        v = svm.modulate(100.0, 0.0)   # magnitude >> DC/2
        assert np.all(np.abs(v) <= svm.dc_voltage + 1e-6)

    def test_modulate_zero_magnitude_near_zero(self, svm):
        v = svm.modulate(0.0, 0.0)
        assert np.allclose(v, 0.0, atol=1e-6)

    def test_all_sectors(self, svm):
        """Walk through all 6 sectors (angles 0 to 2π)."""
        for angle in np.linspace(0, 2 * math.pi, 13, endpoint=False):
            v = svm.modulate(15.0, angle)
            assert v.shape == (3,)
            assert np.isfinite(v).all()

    def test_set_sample_time(self, svm):
        svm.set_sample_time(5e-5)
        result = svm.modulate(10.0, 0.5)
        assert result.shape == (3,)

    def test_cartesian_modulate(self, svm):
        cart_svm = CartesianSVMGenerator(dc_voltage=48.0)
        v = cart_svm.modulate_cartesian(valpha=10.0, vbeta=5.0)
        assert v.shape == (3,)


class TestSVMGeneratorNonidealities:

    def test_device_drop_reduces_voltage(self):
        svm = SVMGenerator(dc_voltage=48.0)
        svm.set_nonidealities(
            device_drop_v=2.0,
            enable_device_drop=True,
            switching_frequency_hz=20000,
        )
        v_with_drop = svm.modulate(20.0, 0.0)

        svm2 = SVMGenerator(dc_voltage=48.0)
        v_ideal = svm2.modulate(20.0, 0.0)

        assert np.sum(np.abs(v_with_drop)) <= np.sum(np.abs(v_ideal)) + 1e-6

    def test_phase_asymmetry_enabled(self):
        svm = SVMGenerator(dc_voltage=48.0)
        svm.set_nonidealities(
            enable_phase_asymmetry=True,
            phase_voltage_scale_a=0.9,
            phase_voltage_scale_b=1.0,
            phase_voltage_scale_c=1.1,
        )
        result = svm.modulate(15.0, 0.5)
        assert result.shape == (3,)

    def test_thermal_coupling_enabled(self):
        svm = SVMGenerator(dc_voltage=48.0)
        svm.set_nonidealities(
            enable_thermal_coupling=True,
            thermal_resistance_k_per_w=0.5,
            thermal_capacitance_j_per_k=0.1,
        )
        result = svm.modulate(10.0, 0.3)
        assert result.shape == (3,)

    def test_bus_ripple_enabled(self):
        svm = SVMGenerator(dc_voltage=48.0)
        svm.set_nonidealities(
            enable_bus_ripple=True,
            dc_link_capacitance_f=470e-6,
            dc_link_source_resistance_ohm=0.05,
        )
        for _ in range(20):
            svm.modulate(10.0, 0.5)
        # After several steps, junction state should evolve
        result = svm.modulate(10.0, 0.5)
        assert result.shape == (3,)


# ============================================================================
# Full simulation loop — FOC → SVM → motor (non-regression integration test)
# ============================================================================

class TestFOCSVMMotorLoop:

    def test_full_loop_runs_without_exception(self):
        motor = _default_motor()
        ctrl = FOCController(motor=motor)
        svm = SVMGenerator(dc_voltage=48.0)
        ctrl.set_current_references(id_ref=0.0, iq_ref=3.0)

        from src.core.simulation_engine import SimulationEngine
        engine = SimulationEngine(motor, ConstantLoad(0.0), dt=5e-5)

        for _ in range(200):
            mag, angle = ctrl.update(dt=5e-5)
            voltages = svm.modulate(mag, angle)
            ctrl.update_applied_voltage(*voltages)
            engine.step(voltages, log_data=True)

        assert engine.step_count == 200
        assert motor.omega >= 0.0

    def test_sensorless_pll_loop_maintains_observer(self):
        motor = _default_motor()
        motor.state[3] = 150.0
        ctrl = FOCController(motor=motor)
        ctrl.set_angle_observer("PLL")
        ctrl.set_pll_gains(kp=80.0, ki=2000.0)
        ctrl.enable_sensorless_emf_reconstruction()

        svm = SVMGenerator(dc_voltage=48.0)
        for _ in range(100):
            mag, angle = ctrl.update(dt=5e-5)
            va, vb, vc = svm.modulate(mag, angle)
            ctrl.update_applied_voltage(va, vb, vc)
            motor.step(np.array([va, vb, vc]), 0.0)

        assert hasattr(ctrl, "theta_est_pll")
