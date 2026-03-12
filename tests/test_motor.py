"""
Unit Tests for BLDC Motor Control System

Tests for core components:
- Motor model
- Control blocks
- Load profiles
- Simulation engine

Run with:
    pytest tests/test_motor.py -v
"""

import pytest
import numpy as np

from src.core import (
    BLDCMotor,
    MotorParameters,
    SimulationEngine,
    ConstantLoad,
    RampLoad,
)  # noqa: E402
from src.control import SVMGenerator, VFController  # noqa: E402


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
        return VFController(
            v_nominal=48.0, f_nominal=100.0, dc_voltage=48.0, v_startup=1.0
        )

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
            key in history
            for key in ["currents_a", "omega", "theta", "torque", "voltages_a"]
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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
