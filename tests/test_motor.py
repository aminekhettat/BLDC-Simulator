"""
Atomic features tested in this module:
- MotorModel
- SVMGenerator
- VFController
- LoadProfiles
- SimulationEngine
- MotorDynamics
"""

import numpy as np
import pytest

from src.control import SVMGenerator, VFController
from src.core import (
    BLDCMotor,
    ConstantLoad,
    MotorParameters,
    RampLoad,
    SimulationEngine,
)


class TestMotorModel:
    """Test BLDC motor model."""

    @pytest.fixture
    def motor_params(self):
        """Standard motor parameters for testing."""
        return MotorParameters(
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

    @pytest.fixture
    def motor(self, motor_params):
        """Create motor instance."""
        return BLDCMotor(motor_params, dt=0.0001)

    def test_motor_initialization(self, motor):
        """Test motor initializes correctly."""
        assert motor.omega == 0.0
        assert motor.theta == 0.0
        assert np.allclose(motor.currents, [0, 0, 0])

    def test_motor_step(self, motor):
        """Test motor step execution."""
        voltages = np.array([10.0, -5.0, -5.0])
        motor.step(voltages, load_torque=0.0)

        # After one step, state should change
        assert motor.omega != 0.0 or np.any(motor.currents != 0)

    def test_sinusoidal_emf_shape(self, motor_params):
        """Test that sinusoidal back-EMF is calculated correctly."""
        motor_params.emf_shape = "sinusoidal"
        motor = BLDCMotor(motor_params, dt=0.0001)
        motor.state[3] = 100.0  # omega = 100 rad/s

        # at theta_e = 0, emf_a should be 0
        motor.state[4] = 0.0  # theta = 0
        emf = motor._calculate_back_emf(motor.theta)
        assert np.isclose(emf[0], 0.0)

        # at theta_e = pi/2, emf_a should be at its peak (Ke * omega)
        theta_mech_90_deg_elec = (np.pi / 2) / motor.params.poles_pairs
        motor.state[4] = theta_mech_90_deg_elec
        emf = motor._calculate_back_emf(motor.theta)
        expected_peak_emf = motor.params.back_emf_constant * motor.omega
        assert np.isclose(emf[0], expected_peak_emf)

    def test_dq_model_runs(self, motor_params):
        """Test that the d-q motor model runs without error."""
        motor_params.model_type = "dq"
        motor_params.emf_shape = "sinusoidal"  # d-q model usually used with sinusoidal
        motor = BLDCMotor(motor_params, dt=0.0001)

        voltages = np.array([10.0, -5.0, -5.0])

        # Align rotor electrical angle so the applied voltages produce a non-zero
        # q-axis component (torque-producing in the dq model).
        motor.state[4] = np.pi / 2
        initial_omega = motor.omega

        # Run a few steps to see if it accelerates
        for _ in range(100):
            motor.step(voltages, load_torque=0.0)

        assert motor.omega > initial_omega

    def test_dq_torque_calculation(self, motor_params):
        """Test torque calculation for d-q model."""
        motor_params.model_type = "dq"
        motor = BLDCMotor(motor_params, dt=0.0001)
        torque = motor._calculate_torque(currents=np.array([1, -0.5, -0.5]), theta=0.1)
        assert isinstance(torque, float)

    def test_dq_flux_weakening_reduces_acceleration(self, motor_params):
        """Negative Id should reduce effective PM flux in dq model when enabled."""
        motor_params.model_type = "dq"
        motor_params.emf_shape = "sinusoidal"
        motor_params.flux_weakening_id_coefficient = 0.02
        motor_params.flux_weakening_min_ratio = 0.2

        motor_no_fw = BLDCMotor(motor_params, dt=0.0001)
        motor_fw = BLDCMotor(motor_params, dt=0.0001)

        # Same electrical angle and q-axis voltage command.
        motor_no_fw.state[4] = np.pi / 2
        motor_fw.state[4] = np.pi / 2

        # Inject negative Id via initial abc currents for motor_fw only.
        # At theta_e=pi/2, alpha maps to d-axis; use ia=1, ib=ic=-0.5 to get i_alpha>0,
        # then invert sign for negative d-axis current.
        motor_fw.state[0:3] = np.array([-1.0, 0.5, 0.5], dtype=float)

        voltages = np.array([10.0, -5.0, -5.0], dtype=float)
        for _ in range(80):
            motor_no_fw.step(voltages, load_torque=0.0)
            motor_fw.step(voltages, load_torque=0.0)

        assert motor_fw.omega < motor_no_fw.omega

    def test_motor_properties(self, motor):
        """Test motor property accessors."""
        assert isinstance(motor.speed_rpm, float)
        assert isinstance(motor.back_emf, np.ndarray)
        assert isinstance(motor.electromagnetic_torque, float)
        assert motor.back_emf.shape == (3,)

    def test_motor_reset(self, motor):
        """Test motor reset."""
        motor.step(np.array([10, -5, -5]), 0)
        motor.reset(initial_speed=10.0)

        assert motor.omega == 10.0
        assert np.allclose(motor.currents, [0, 0, 0])


class TestSVMGenerator:
    """Test SVM modulation."""

    @pytest.fixture
    def svm(self):
        """Create SVM generator."""
        return SVMGenerator(dc_voltage=48.0)

    def test_svm_initialization(self, svm):
        """Test SVM initializes correctly."""
        assert svm.dc_voltage == 48.0
        assert len(svm.sector_angles) == 7

    def test_svm_modulation(self, svm):
        """Test SVM voltage generation."""
        voltages = svm.modulate(magnitude=20.0, angle=np.pi / 6)

        assert voltages.shape == (3,)
        assert all(isinstance(v, (float, np.floating)) for v in voltages)

    def test_svm_maximum_voltage(self, svm):
        """Test SVM maximum voltage calculation."""
        max_v = svm.get_maximum_voltage()
        expected_max = (2.0 / 3.0) * 48.0
        assert np.isclose(max_v, expected_max)

    def test_svm_voltage_clamping(self, svm):
        """Test that voltages don't exceed maximum linear magnitude limit."""
        voltages = svm.modulate(magnitude=100.0, angle=0)

        # according to SVM algorithm, maximum magnitude should be 2/3 of Vdc
        max_voltages = np.abs(voltages)
        assert np.all(max_voltages <= (2.0 / 3.0) * svm.dc_voltage + 0.1)


class TestVFController:
    """Test V/f controller."""

    @pytest.fixture
    def controller(self):
        """Create V/f controller."""
        return VFController(v_nominal=48.0, f_nominal=100.0, dc_voltage=48.0, v_startup=1.0)

    def test_vf_initialization(self, controller):
        """Test V/f controller initializes correctly."""
        assert controller.v_nominal == 48.0
        assert controller.f_nominal == 100.0
        assert controller.frequency_actual == 0.0

    def test_vf_update(self, controller):
        """Test V/f controller update."""
        controller.set_speed_reference(50.0)

        mag, angle = controller.update(dt=0.001)

        assert isinstance(mag, (float, np.floating))
        assert isinstance(angle, (float, np.floating))
        assert 0 <= mag <= controller.max_voltage

    def test_vf_frequency_slew_limiting(self, controller):
        """Test frequency slew rate limiting."""
        controller.set_speed_reference(100.0)
        controller.set_frequency_slew_rate(10.0)  # 10 Hz/s

        # After 0.1s at 10 Hz/s slew rate, frequency should be ~1 Hz
        for _ in range(10):
            controller.update(dt=0.01)

        assert controller.frequency_actual <= 1.0 + 0.1  # Small tolerance


class TestLoadProfiles:
    """Test load profile models."""

    def test_constant_load(self):
        """Test constant load."""
        load = ConstantLoad(torque=2.0)

        assert load.get_torque(0.0) == 2.0
        assert load.get_torque(1.0) == 2.0
        assert load.get_torque(10.0) == 2.0

    def test_ramp_load(self):
        """Test ramp load."""
        load = RampLoad(initial=0.0, final=2.0, duration=2.0)

        assert load.get_torque(0.0) == 0.0
        assert np.isclose(load.get_torque(1.0), 1.0)
        assert load.get_torque(2.0) == 2.0
        assert load.get_torque(3.0) == 2.0

    def test_ramp_load_validation(self):
        """Test ramp load parameter validation."""
        with pytest.raises(ValueError):
            RampLoad(initial=0, final=2, duration=0)


class TestSimulationEngine:
    """Test simulation engine."""

    @pytest.fixture
    def engine(self):
        """Create simulation engine."""
        params = MotorParameters(
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
        motor = BLDCMotor(params, dt=0.0001)
        load = ConstantLoad(torque=0.5)
        return SimulationEngine(motor, load, dt=0.0001)

    def test_engine_initialization(self, engine):
        """Test engine initializes correctly."""
        assert engine.time == 0.0
        assert engine.step_count == 0

    def test_engine_with_custom_supply(self):
        """Engine should honor supplied supply profile and log voltage."""
        params = MotorParameters(
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
        motor = BLDCMotor(params, dt=0.0001)
        load = ConstantLoad(torque=0.5)
        from src.core.power_model import RampSupply

        supply = RampSupply(initial=48.0, final=30.0, duration=0.01)
        engine2 = SimulationEngine(motor, load, dt=0.0001, supply_profile=supply)
        engine2.step(np.zeros(3), log_data=True)
        hist = engine2.get_history()
        assert "supply_voltage" in hist
        assert hist["supply_voltage"][0] == pytest.approx(48.0)

    def test_engine_step(self, engine):
        """Test engine simulation step."""
        voltages = np.array([10.0, -5.0, -5.0])
        engine.step(voltages, log_data=True)

        assert engine.time > 0
        assert engine.step_count == 1

    def test_engine_history(self, engine):
        """Test history tracking."""
        voltages = np.array([10.0, -5.0, -5.0])

        for _ in range(100):
            engine.step(voltages, log_data=True)

        history = engine.get_history()

        assert len(history["time"]) == 100
        assert all(
            key in history for key in ["currents_a", "omega", "theta", "torque", "voltages_a"]
        )

    def test_engine_reset(self, engine):
        """Test engine reset."""
        engine.step(np.array([10, -5, -5]), log_data=True)
        engine.reset()

        assert engine.time == 0.0
        assert engine.step_count == 0
        assert len(engine.get_history()["time"]) == 0


class TestMotorDynamics:
    """Test motor dynamics behavior."""

    def test_acceleration_with_constant_voltage(self):
        """Test motor accelerates with constant voltage."""
        params = MotorParameters(
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
        motor = BLDCMotor(params, dt=0.0001)

        # Apply constant voltage
        voltages = np.array([20.0, -10.0, -10.0])

        initial_omega = motor.omega

        # Run multiple steps
        for _ in range(1000):
            motor.step(voltages, load_torque=0)

        # Motor should accelerate
        assert motor.omega > initial_omega

    def test_back_emf_increases_with_speed(self):
        """Test that back-EMF increases with motor speed."""
        params = MotorParameters(
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
        motor = BLDCMotor(params, dt=0.0001)

        # Low speed
        motor.state[3] = 10.0  # omega = 10 rad/s
        low_vals = motor._calculate_back_emf(motor.theta)
        low_speed_emf = np.linalg.norm(low_vals)

        # High speed
        motor.state[3] = 100.0  # omega = 100 rad/s
        high_vals = motor._calculate_back_emf(motor.theta)
        high_speed_emf = np.linalg.norm(high_vals)

        # EMF should scale with speed
        assert high_speed_emf > low_speed_emf


class TestDTArchitectureAndStability:
    """Cross-cutting regression tests for DT architecture, SVM/AF voltage loop,
    and numerical stability advisory.

    These tests freeze decisions that cut across multiple subsystems:
    - The controller time step must equal one PWM period (MCU equivalence rule).
    - The SVM modulator's Clarke output differs from the inverse-Park command
      (root cause of the Active Flux observer failure above 500 RPM).
    - FOCController.update_applied_voltage must store Clarke(SVM phases) so the
      AF integrator receives the true terminal voltage.
    - SimulationEngine._build_numerical_stability_advisory must classify dt
      correctly against the RK4 stability limit.
    """

    # ── DT architecture constants ─────────────────────────────────────────────

    def test_dt_ctrl_equals_one_pwm_period(self):
        """DT_CTRL must equal 1/PWM_HZ — one controller step per PWM cycle."""
        from examples.validate_all_observers import DT_CTRL, PWM_HZ

        assert DT_CTRL == pytest.approx(1.0 / PWM_HZ, rel=1e-9), (
            f"DT_CTRL={DT_CTRL} != 1/PWM_HZ={1.0/PWM_HZ}. "
            "The controller must run once per PWM period."
        )

    def test_dt_motor_equals_dt_ctrl_divided_by_n_sub(self):
        """DT_MOTOR must equal DT_CTRL / N_SUB — motor sub-step within control period."""
        from examples.validate_all_observers import DT_CTRL, DT_MOTOR, N_SUB

        assert DT_MOTOR == pytest.approx(DT_CTRL / N_SUB, rel=1e-9), (
            f"DT_MOTOR={DT_MOTOR} != DT_CTRL/N_SUB={DT_CTRL/N_SUB}. "
            "Motor integration step must be DT_CTRL divided by the sub-step count."
        )

    # ── SVM voltage mismatch — Active Flux root cause ─────────────────────────

    def test_svm_clarke_output_differs_from_inverse_park_command(self):
        """Clarke(SVM phases) must differ from inverse-Park command by > 5 %.

        Root cause of the ActiveFlux failure above 500 RPM: the SVM
        space-vector modulator produces phase voltages whose Clarke transform
        is not equal to the inverse-Park of the FOC voltage commands (~26 %
        larger).  The AF open-loop integrator must use the SVM voltage via
        update_applied_voltage, not the smaller inverse-Park value.
        """
        import math
        from src.control.transforms import clarke_transform, inverse_park

        svm = SVMGenerator(dc_voltage=48.0)
        vd, vq, theta = 0.0, 17.2, math.pi / 4.0
        v_alpha_cmd, v_beta_cmd = inverse_park(vd, vq, theta)
        mag_cmd = math.hypot(v_alpha_cmd, v_beta_cmd)

        va, vb, vc = svm.modulate(mag_cmd, theta)
        v_alpha_svm, v_beta_svm = clarke_transform(va, vb, vc)
        mag_svm = math.hypot(v_alpha_svm, v_beta_svm)

        ratio = mag_svm / mag_cmd if mag_cmd > 1e-9 else 0.0
        assert abs(ratio - 1.0) > 0.05, (
            f"Expected |Clarke(SVM)| to differ from |inv-Park(cmd)| by > 5 %, "
            f"got ratio={ratio:.4f}. "
            "If this fails the SVM behaviour has changed — verify AF observer still works."
        )

    def test_update_applied_voltage_stores_clarke_of_svm_phases(self):
        """update_applied_voltage must store Clarke(va,vb,vc) in _v_alpha_prev.

        This is the fix for the AF open-loop integrator: after calling
        update_applied_voltage the internal voltage registers hold the actual
        terminal voltage, not the smaller inverse-Park command.
        """
        import math
        from src.control import FOCController
        from src.control.transforms import clarke_transform

        motor = BLDCMotor(MotorParameters())
        ctrl = FOCController(motor=motor)
        svm = SVMGenerator(dc_voltage=48.0)

        va, vb, vc = svm.modulate(15.0, 0.8)
        ctrl.update_applied_voltage(va, vb, vc)

        v_alpha_exp, v_beta_exp = clarke_transform(va, vb, vc)
        assert ctrl._v_alpha_prev == pytest.approx(v_alpha_exp, rel=1e-9)
        assert ctrl._v_beta_prev  == pytest.approx(v_beta_exp,  rel=1e-9)

    # ── Active Flux integrator ────────────────────────────────────────────────

    def test_active_flux_integrator_advances_psi_s_using_v_alpha_prev(self):
        """The AF stator-flux integrator must advance psi_s in the direction of
        _v_alpha_prev, confirming the correct voltage source is wired in.
        """
        import math
        from src.control import FOCController
        from src.control.transforms import clarke_transform

        motor = BLDCMotor(MotorParameters())
        motor.state[0:3] = np.array([0.3, -0.15, -0.15])
        motor.state[3] = 150.0
        motor.state[4] = 0.6
        motor._last_emf = motor._calculate_back_emf(motor.theta)
        ctrl = FOCController(motor=motor)

        v_inject = 20.0
        ctrl._v_alpha_prev = v_inject
        ctrl._v_beta_prev  = 0.0
        ctrl._psi_s_alpha  = 0.0
        ctrl._psi_s_beta   = 0.0

        i_alpha, i_beta = clarke_transform(*motor.currents)
        dt = 1.0 / 20_000
        ctrl._update_active_flux(dt, i_alpha, i_beta)

        R = float(motor.params.phase_resistance)
        expected_sign = math.copysign(1.0, v_inject - R * i_alpha)
        assert ctrl._psi_s_alpha != pytest.approx(0.0, abs=1e-10)
        assert math.copysign(1.0, ctrl._psi_s_alpha) == expected_sign

    def test_active_flux_omega_est_remains_valid_with_applied_voltage_fix(self):
        """With update_applied_voltage called after each svm.modulate(),
        the AF observer must maintain a non-trivial flux and omega estimate
        over 500 integration steps — regression guard for the ~26 % SVM/
        inverse-Park voltage mismatch fix.
        """
        import math
        from src.control import FOCController
        from src.control.transforms import clarke_transform

        dt = 1.0 / 20_000
        motor = BLDCMotor(MotorParameters())
        motor.state[3] = 150.0
        motor.state[4] = 0.5
        motor._last_emf = motor._calculate_back_emf(motor.theta)

        ctrl = FOCController(motor=motor)
        svm  = SVMGenerator(dc_voltage=float(motor.params.nominal_voltage))
        ctrl.enable_active_flux_observer(dc_cutoff_hz=0.3)

        params = motor.params
        pp    = float(params.poles_pairs)
        ke_v  = float(params.back_emf_constant)
        psi_m = ke_v / pp
        theta_e = float(motor.state[4] * pp) % (2.0 * math.pi)
        ia, ib, ic = motor.currents
        i_alpha0, i_beta0 = clarke_transform(ia, ib, ic)

        ctrl._psi_s_alpha    = psi_m * math.cos(theta_e) + float(params.phase_inductance) * i_alpha0
        ctrl._psi_s_beta     = psi_m * math.sin(theta_e) + float(params.phase_inductance) * i_beta0
        ctrl._psi_af_prev_a  = psi_m * math.cos(theta_e)
        ctrl._psi_af_prev_b  = psi_m * math.sin(theta_e)
        ctrl._theta_est_af   = theta_e
        ctrl._omega_est_af   = float(motor.state[3] * pp)
        ctrl._omega_elec_est = ctrl._omega_est_af

        for _ in range(500):
            va, vb, vc = svm.modulate(10.0, float(ctrl._theta_est_af))
            ctrl.update_applied_voltage(va, vb, vc)   # ← critical fix
            ia, ib, ic = motor.currents
            i_alpha, i_beta = clarke_transform(ia, ib, ic)
            ctrl._update_active_flux(dt, i_alpha, i_beta)
            motor.step(np.array([va, vb, vc]), load_torque=0.0)

        omega_e_true = abs(motor.state[3] * pp)
        assert ctrl._psi_af_mag > 1e-4, (
            f"AF flux collapsed to {ctrl._psi_af_mag:.2e} Wb. "
            "Check that update_applied_voltage is called after svm.modulate()."
        )
        assert ctrl._omega_est_af > 0.05 * omega_e_true, (
            f"omega_est_af={ctrl._omega_est_af:.1f} is < 5 % of true "
            f"omega_e={omega_e_true:.1f}."
        )

    # ── Numerical stability advisory (engine level, no GUI) ───────────────────
    #
    # Default MotorParameters: R=2.5 Ω, L=0.005 H  → tau_e=0.002 s
    #   dt_limit = 2.785 × tau_e = 0.00557 s
    #   stable:   dt < 0.5 × dt_limit = 0.002785 s
    #   marginal: 0.002785 s ≤ dt < 0.00557 s
    #   unstable: dt ≥ 0.00557 s

    def _make_engine_at_dt(self, dt: float) -> SimulationEngine:
        motor = BLDCMotor(MotorParameters())
        return SimulationEngine(motor=motor, load_profile=ConstantLoad(torque=0.0), dt=dt)

    def test_stability_advisory_stable_when_dt_well_within_limit(self):
        """dt=5e-5 s is well below 50 % of dt_limit → severity must be 'stable'."""
        adv = self._make_engine_at_dt(5e-5).get_numerical_stability_advisory()
        assert adv["severity"] == "stable"
        assert adv["is_stable"] is True

    def test_stability_advisory_unstable_when_dt_exceeds_rk4_limit(self):
        """dt=7e-3 s exceeds dt_limit ≈ 5.57e-3 s → severity must be 'unstable'."""
        adv = self._make_engine_at_dt(7e-3).get_numerical_stability_advisory()
        assert adv["severity"] == "unstable"
        assert adv["is_stable"] is False

    def test_stability_advisory_marginal_between_50_and_100_pct_of_limit(self):
        """dt=4e-3 s is between 50 % and 100 % of dt_limit → severity 'marginal'."""
        adv = self._make_engine_at_dt(4e-3).get_numerical_stability_advisory()
        assert adv["severity"] == "marginal"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
