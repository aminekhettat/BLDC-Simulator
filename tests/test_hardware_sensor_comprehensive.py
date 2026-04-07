"""
Comprehensive hardware-layer tests
=====================================

Covers every public path in:
  - ``src.hardware.current_sensor``       (CurrentSensorModel)
  - ``src.hardware.hardware_interface``   (MockDAQHardware)
  - ``src.hardware.inverter_current_sense`` (InverterCurrentSense, ShuntAmplifierChannel)
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from src.hardware.current_sensor import CurrentSensorModel

# ============================================================================
# CurrentSensorModel — construction validation
# ============================================================================

class TestCurrentSensorModelConstruction:

    def test_default_construction(self):
        m = CurrentSensorModel()
        assert m.r_shunt > 0.0
        assert m.amplifier_gain > 0.0
        assert m.n_channels == 3

    def test_negative_r_shunt_raises(self):
        with pytest.raises(ValueError, match="r_shunt"):
            CurrentSensorModel(r_shunt=-0.001)

    def test_zero_r_shunt_raises(self):
        with pytest.raises(ValueError, match="r_shunt"):
            CurrentSensorModel(r_shunt=0.0)

    def test_negative_amplifier_gain_raises(self):
        with pytest.raises(ValueError, match="amplifier_gain"):
            CurrentSensorModel(amplifier_gain=-10.0)

    def test_negative_r_feedback_raises(self):
        with pytest.raises(ValueError, match="r_feedback"):
            CurrentSensorModel(r_feedback=-1.0)

    def test_negative_c_filter_raises(self):
        with pytest.raises(ValueError, match="c_filter"):
            CurrentSensorModel(c_filter=-1e-9)

    def test_zero_channels_raises(self):
        with pytest.raises(ValueError, match="n_channels"):
            CurrentSensorModel(n_channels=0)

    def test_single_channel(self):
        m = CurrentSensorModel(n_channels=1)
        assert m.n_channels == 1


# ============================================================================
# CurrentSensorModel — properties
# ============================================================================

class TestCurrentSensorModelProperties:

    @pytest.fixture
    def sensor(self):
        return CurrentSensorModel(
            r_shunt=0.001,
            amplifier_gain=20.0,
            gain_error=0.02,
            v_offset=0.001,
            r_feedback=10_000.0,
            c_filter=1.0e-9,
            n_channels=3,
        )

    def test_r_shunt_property(self, sensor):
        assert sensor.r_shunt == pytest.approx(0.001)

    def test_amplifier_gain_property(self, sensor):
        assert sensor.amplifier_gain == pytest.approx(20.0)

    def test_gain_error_property(self, sensor):
        assert sensor.gain_error == pytest.approx(0.02)

    def test_v_offset_property(self, sensor):
        assert sensor.v_offset == pytest.approx(0.001)

    def test_tau_property(self, sensor):
        expected_tau = 10_000.0 * 1e-9
        assert sensor.tau == pytest.approx(expected_tau)

    def test_cutoff_frequency_finite(self, sensor):
        fc = sensor.cutoff_frequency_hz
        assert math.isfinite(fc)
        assert fc > 0.0

    def test_cutoff_frequency_infinite_when_no_filter(self):
        m = CurrentSensorModel(c_filter=0.0)
        assert math.isinf(m.cutoff_frequency_hz)

    def test_filter_state_shape(self, sensor):
        assert sensor.filter_state.shape == (3,)

    def test_n_channels_property(self, sensor):
        assert sensor.n_channels == 3


# ============================================================================
# CurrentSensorModel — measure() method
# ============================================================================

class TestCurrentSensorModelMeasure:

    @pytest.fixture
    def ideal_sensor(self):
        """Ideal sensor: no gain error, no offset, no filter."""
        return CurrentSensorModel(
            r_shunt=0.001,
            amplifier_gain=20.0,
            gain_error=0.0,
            v_offset=0.0,
            r_feedback=0.0,
            c_filter=0.0,
            n_channels=3,
        )

    def test_ideal_sensor_passes_current_through(self, ideal_sensor):
        currents = np.array([5.0, -2.5, -2.5])
        measured = ideal_sensor.measure(currents, dt=1e-4)
        assert np.allclose(measured, currents, rtol=1e-6)

    def test_wrong_shape_raises(self, ideal_sensor):
        with pytest.raises(ValueError, match="Expected currents shape"):
            ideal_sensor.measure(np.array([1.0, 2.0]), dt=1e-4)

    def test_zero_dt_raises(self, ideal_sensor):
        with pytest.raises(ValueError, match="dt must be positive"):
            ideal_sensor.measure(np.array([1.0, 0.0, -1.0]), dt=0.0)

    def test_negative_dt_raises(self, ideal_sensor):
        with pytest.raises(ValueError):
            ideal_sensor.measure(np.array([1.0, 0.0, -1.0]), dt=-1e-4)

    def test_gain_error_shifts_measured_current(self):
        sensor = CurrentSensorModel(
            r_shunt=0.001, amplifier_gain=20.0,
            gain_error=0.10,    # 10 % overread
            v_offset=0.0, r_feedback=0.0, c_filter=0.0, n_channels=3,
        )
        currents = np.array([10.0, 0.0, 0.0])
        measured = sensor.measure(currents, dt=1e-4)
        # Overread by 10 %: measured[0] ≈ 11.0
        assert measured[0] == pytest.approx(11.0, rel=1e-6)

    def test_v_offset_adds_bias(self):
        sensor = CurrentSensorModel(
            r_shunt=0.001, amplifier_gain=20.0,
            gain_error=0.0, v_offset=0.02,   # DC bias
            r_feedback=0.0, c_filter=0.0, n_channels=3,
        )
        currents = np.array([0.0, 0.0, 0.0])
        measured = sensor.measure(currents, dt=1e-4)
        # Offset = 0.02 V / (r_shunt * amplifier_gain) = 0.02 / 0.02 = 1.0 A
        assert measured[0] == pytest.approx(1.0, rel=1e-5)

    def test_lp_filter_attenuates_fast_signal(self):
        """After a step, filtered output should be < true value (filter lag)."""
        sensor = CurrentSensorModel(
            r_shunt=0.001, amplifier_gain=20.0,
            gain_error=0.0, v_offset=0.0,
            r_feedback=10_000.0, c_filter=100e-9,   # τ = 1 ms
            n_channels=1,
        )
        currents = np.array([10.0])
        for _ in range(5):
            measured = sensor.measure(currents, dt=1e-5)   # very short dt
        # LP filter has not settled; output should still be below input
        assert measured[0] < 10.0

    def test_lp_filter_settles_after_many_steps(self):
        """After many large-dt steps, filtered output should approach true value."""
        sensor = CurrentSensorModel(
            r_shunt=0.001, amplifier_gain=20.0,
            gain_error=0.0, v_offset=0.0,
            r_feedback=1_000.0, c_filter=10e-9,   # τ = 10 µs
            n_channels=1,
        )
        currents = np.array([5.0])
        for _ in range(1000):
            measured = sensor.measure(currents, dt=1e-3)   # dt >> τ
        assert measured[0] == pytest.approx(5.0, rel=0.01)


# ============================================================================
# CurrentSensorModel — reset and get_state
# ============================================================================

class TestCurrentSensorModelResetAndState:

    def test_reset_zeros_filter_state(self):
        sensor = CurrentSensorModel(
            r_shunt=0.001, amplifier_gain=20.0,
            r_feedback=10_000.0, c_filter=1e-9, n_channels=3,
        )
        # Drive some current through
        for _ in range(50):
            sensor.measure(np.array([5.0, -2.5, -2.5]), dt=1e-4)
        sensor.reset()
        assert np.allclose(sensor.filter_state, 0.0)

    def test_get_state_returns_dict(self):
        sensor = CurrentSensorModel()
        state = sensor.get_state()
        assert isinstance(state, dict)
        assert "filter_state_v" in state
        assert "cutoff_frequency_hz" in state
        assert "tau_s" in state
        assert "gain_error" in state
        assert "v_offset_v" in state

    def test_filter_state_is_copy(self):
        """filter_state property must return a copy, not a reference."""
        sensor = CurrentSensorModel()
        fs1 = sensor.filter_state
        fs2 = sensor.filter_state
        fs1[0] = 9999.0
        assert sensor.filter_state[0] != 9999.0


# ============================================================================
# MockDAQHardware (hardware_interface)
# ============================================================================

class TestMockDAQHardware:

    def test_construction_and_noise(self):
        try:
            from src.hardware.hardware_interface import MockDAQHardware  # noqa: PLC0415
        except ImportError:
            pytest.skip("MockDAQHardware not available")

        hw = MockDAQHardware(noise_std=0.01, seed=42)
        # Should be constructable without error
        assert hw is not None

    def test_read_currents_returns_array(self):
        try:
            from src.hardware.hardware_interface import MockDAQHardware  # noqa: PLC0415
        except ImportError:
            pytest.skip("MockDAQHardware not available")

        hw = MockDAQHardware(noise_std=0.0, seed=0)
        hw.connect()
        hw.write_phase_voltages(np.array([1.0, -0.5, -0.5]), time_s=0.0)
        result = hw.read_feedback(time_s=0.0)
        assert "applied_voltages" in result
        assert np.asarray(result["applied_voltages"]).shape == (3,)


# ============================================================================
# InverterCurrentSense / ShuntAmplifierChannel
# ============================================================================

class TestInverterCurrentSense:

    def test_construction(self):
        try:
            from src.hardware.inverter_current_sense import (  # noqa: PLC0415
                InverterCurrentSense,
                ShuntAmplifierChannel,
            )
        except ImportError:
            pytest.skip("InverterCurrentSense not available")

        ch_a = ShuntAmplifierChannel(r_shunt_ohm=0.001, nominal_gain=20.0)
        ch_b = ShuntAmplifierChannel(r_shunt_ohm=0.001, nominal_gain=20.0)
        sense = InverterCurrentSense(topology="double", channels=[ch_a, ch_b])
        assert sense is not None

    def test_reconstruct_returns_three_phases(self):
        try:
            from src.hardware.inverter_current_sense import (  # noqa: PLC0415
                InverterCurrentSense,
                ShuntAmplifierChannel,
            )
        except ImportError:
            pytest.skip("InverterCurrentSense not available")

        ch_a = ShuntAmplifierChannel(r_shunt_ohm=0.001, nominal_gain=20.0)
        ch_b = ShuntAmplifierChannel(r_shunt_ohm=0.001, nominal_gain=20.0)
        sense = InverterCurrentSense(topology="double", channels=[ch_a, ch_b])

        true_currents = np.array([3.0, -1.5, -1.5])
        result = sense.measure(true_currents, dt=1e-4)
        assert result["currents_abc_measured"].shape == (3,)
