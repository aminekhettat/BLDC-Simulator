import numpy as np
import pytest

from src.hardware import InverterCurrentSense, ShuntAmplifierChannel


class TestInverterCurrentSenseTopology:
    def test_triple_topology_channel_count(self):
        sense = InverterCurrentSense(topology="triple")
        assert sense.n_shunts == 3

    def test_double_topology_channel_count(self):
        sense = InverterCurrentSense(topology="double")
        assert sense.n_shunts == 2

    def test_single_topology_channel_count(self):
        sense = InverterCurrentSense(topology="single")
        assert sense.n_shunts == 1

    def test_invalid_topology_raises(self):
        with pytest.raises(ValueError):
            InverterCurrentSense(topology="quad")  # type: ignore[arg-type]


class TestMeasurementAndClamp:
    def test_triple_measurement_uses_nominal_reconstruction(self):
        channels = [
            ShuntAmplifierChannel(
                r_shunt_ohm=0.001,
                nominal_gain=10.0,
                nominal_offset_v=1.5,
                actual_gain=11.0,
                actual_offset_v=1.6,
                cutoff_frequency_hz=1e9,
                vcc=5.0,
            )
            for _ in range(3)
        ]
        sense = InverterCurrentSense(topology="triple", channels=channels)
        out = sense.measure(np.array([2.0, -1.0, 0.5]), dt=1e-4)
        meas = out["currents_abc_measured"]
        assert meas.shape == (3,)
        assert (
            meas[0] > 2.0
        )  # positive gain/offset deviation should bias reconstruction up

    def test_adc_clamp_is_applied(self):
        ch = ShuntAmplifierChannel(
            r_shunt_ohm=0.01,
            nominal_gain=10.0,
            nominal_offset_v=0.0,
            actual_gain=1000.0,
            actual_offset_v=0.0,
            cutoff_frequency_hz=1e9,
            vcc=3.3,
        )
        sense = InverterCurrentSense(topology="single", channels=[ch])
        out = sense.measure(np.array([100.0, -50.0, -50.0]), dt=1e-4)
        assert float(out["amplifier_outputs_v"][0]) <= 3.3 + 1e-12
        assert bool(out["adc_saturated"][0]) is True

    def test_double_reconstructs_phase_c_from_kcl(self):
        sense = InverterCurrentSense(topology="double")
        out = sense.measure(np.array([2.0, -1.5, -0.5]), dt=1e-4)
        i = out["currents_abc_measured"]
        assert i[2] == pytest.approx(-(i[0] + i[1]), rel=1e-9)


class TestVoltageDrop:
    def test_triple_voltage_drop_applies_per_phase(self):
        sense = InverterCurrentSense(topology="triple")
        v_eff, drop = sense.apply_shunt_voltage_drop(
            np.array([10.0, -5.0, -5.0]), np.array([2.0, -1.0, -1.0])
        )
        assert drop.shape == (3,)
        assert v_eff.shape == (3,)

    def test_single_voltage_drop_is_common_signed(self):
        sense = InverterCurrentSense(topology="single")
        v_cmd = np.array([4.0, -2.0, -2.0])
        v_eff, drop = sense.apply_shunt_voltage_drop(v_cmd, np.array([2.0, -1.0, -1.0]))
        assert drop[0] > 0.0
        assert drop[1] < 0.0
        assert drop[2] < 0.0
        assert np.all(np.abs(v_eff) <= np.abs(v_cmd) + 1e-12)


class TestRuntimeActualTuning:
    def test_set_actual_channel_changes_output(self):
        sense = InverterCurrentSense(topology="triple")
        base = sense.measure(np.array([1.0, -0.5, -0.5]), dt=1e-4)[
            "currents_abc_measured"
        ][0]
        sense.set_actual_channel(0, gain=30.0, offset_v=2.0)
        changed = sense.measure(np.array([1.0, -0.5, -0.5]), dt=1e-4)[
            "currents_abc_measured"
        ][0]
        assert changed != pytest.approx(base)
