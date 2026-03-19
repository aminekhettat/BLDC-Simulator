"""
Atomic features tested in this module:
- TripleTopologyIdealReconstruction
- DoubleTopologyIdealReconstruction
- SingleShuntSectorAwareReconstruction
- SectorFromVoltages
- EngineHistoryTrueVsMeasured
"""

import math
import numpy as np
import pytest

from src.hardware import InverterCurrentSense, ShuntAmplifierChannel
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.load_model import ConstantLoad
from src.core.simulation_engine import SimulationEngine


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _ideal_channel(vcc: float = 3.3) -> ShuntAmplifierChannel:
    """Channel with matching nominal/actual parameters and transparent filter."""
    return ShuntAmplifierChannel(
        r_shunt_ohm=0.001,
        nominal_gain=20.0,
        nominal_offset_v=1.65,
        actual_gain=20.0,
        actual_offset_v=1.65,
        cutoff_frequency_hz=1e12,  # transparent filter (τ → 0)
        vcc=vcc,
    )


def _make_motor_and_engine(topology: str) -> tuple[BLDCMotor, SimulationEngine]:
    params = MotorParameters(
        resistance=0.5,
        inductance=1e-3,
        back_emf_constant=0.05,
        moment_of_inertia=1e-4,
        num_pole_pairs=4,
    )
    motor = BLDCMotor(params, dt=1e-4)
    load = ConstantLoad(torque=0.1)
    n_shunts = {"triple": 3, "double": 2, "single": 1}[topology]
    channels = [_ideal_channel() for _ in range(n_shunts)]
    sense = InverterCurrentSense(topology=topology, channels=channels)
    engine = SimulationEngine(motor, load, dt=1e-4, current_sense=sense)
    return motor, engine


# ---------------------------------------------------------------------------
# _channel_measure sanity: ideal channel passes current unchanged
# ---------------------------------------------------------------------------


class TestIdealChannelTransparency:
    def test_ideal_channel_returns_exact_current(self):
        ch = _ideal_channel()
        sense = InverterCurrentSense(
            topology="triple", channels=[_ideal_channel() for _ in range(3)]
        )
        # Measure a known current directly via _channel_measure
        i_in = 5.0
        i_reco, v_adc, saturated = sense._channel_measure(i_in, dt=1e-4, idx=0)
        assert i_reco == pytest.approx(i_in, abs=1e-10)
        assert not saturated

    def test_ideal_channel_negative_current_exact(self):
        sense = InverterCurrentSense(
            topology="triple", channels=[_ideal_channel() for _ in range(3)]
        )
        i_in = -3.5
        i_reco, _, _ = sense._channel_measure(i_in, dt=1e-4, idx=0)
        assert i_reco == pytest.approx(i_in, abs=1e-10)


# ---------------------------------------------------------------------------
# Triple topology: all three channels independent, perfect reconstruction
# ---------------------------------------------------------------------------


class TestTripleTopologyIdealReconstruction:
    @pytest.mark.parametrize(
        "currents",
        [
            [5.0, -2.0, -3.0],
            [0.0, 0.0, 0.0],
            [-1.5, 3.0, -1.5],
            [10.0, -5.0, -5.0],
        ],
    )
    def test_triple_ideal_perfect_reconstruction(self, currents):
        sense = InverterCurrentSense(
            topology="triple",
            channels=[_ideal_channel() for _ in range(3)],
        )
        out = sense.measure(np.array(currents, dtype=float), dt=1e-4)
        measured = out["currents_abc_measured"]
        assert measured == pytest.approx(currents, abs=1e-10)

    def test_triple_topology_three_adc_outputs(self):
        sense = InverterCurrentSense(
            topology="triple",
            channels=[_ideal_channel() for _ in range(3)],
        )
        out = sense.measure(np.array([2.0, -1.0, -1.0]), dt=1e-4)
        assert len(out["amplifier_outputs_v"]) == 3

    def test_triple_topology_kirchhoff_holds(self):
        sense = InverterCurrentSense(
            topology="triple",
            channels=[_ideal_channel() for _ in range(3)],
        )
        out = sense.measure(np.array([4.0, -1.5, -2.5]), dt=1e-4)
        m = out["currents_abc_measured"]
        assert m[0] + m[1] + m[2] == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Double topology: direct A+B channels, C from Kirchhoff
# ---------------------------------------------------------------------------


class TestDoubleTopologyIdealReconstruction:
    @pytest.mark.parametrize(
        "currents",
        [
            [5.0, -2.0, -3.0],
            [-1.5, 3.0, -1.5],
            [0.0, 0.0, 0.0],
        ],
    )
    def test_double_ideal_perfect_reconstruction(self, currents):
        sense = InverterCurrentSense(
            topology="double",
            channels=[_ideal_channel() for _ in range(2)],
        )
        out = sense.measure(np.array(currents, dtype=float), dt=1e-4)
        m = out["currents_abc_measured"]
        assert m == pytest.approx(currents, abs=1e-10)

    def test_double_phase_c_kirchhoff_exact(self):
        sense = InverterCurrentSense(
            topology="double",
            channels=[_ideal_channel() for _ in range(2)],
        )
        currents = [3.0, -1.0, -2.0]
        out = sense.measure(np.array(currents, dtype=float), dt=1e-4)
        m = out["currents_abc_measured"]
        # C must equal -(A+B) from Kirchhoff
        assert m[2] == pytest.approx(-(m[0] + m[1]), abs=1e-12)

    def test_double_ignores_sector_hint(self):
        """Double topology should not use svm_sector; result is independent."""
        sense = InverterCurrentSense(
            topology="double",
            channels=[_ideal_channel() for _ in range(2)],
        )
        currents = np.array([4.0, -1.5, -2.5])
        out_no_sector = sense.measure(currents, dt=1e-4)
        sense.reset()
        out_with_sector = sense.measure(currents, dt=1e-4, svm_sector=3)
        np.testing.assert_allclose(
            out_no_sector["currents_abc_measured"],
            out_with_sector["currents_abc_measured"],
            atol=1e-12,
        )


# ---------------------------------------------------------------------------
# sector_from_voltages: maps voltage angle to SVM sector
# ---------------------------------------------------------------------------


class TestSectorFromVoltages:
    @pytest.mark.parametrize(
        "angle_deg, expected_sector",
        [
            (5, 1),  # sector 1: 0-60°
            (30, 1),
            (65, 2),  # sector 2: 60-120°
            (100, 2),
            (125, 3),  # sector 3: 120-180°
            (170, 3),
            (185, 4),  # sector 4: 180-240°
            (220, 4),
            (245, 5),  # sector 5: 240-300°
            (280, 5),
            (308, 6),  # sector 6: 300-360°
            (350, 6),
        ],
    )
    def test_sector_from_pure_voltage_angle(self, angle_deg, expected_sector):
        angle = math.radians(angle_deg)
        mag = 20.0
        # generate Clarke-domain vector and invert to 3-phase
        v_alpha = mag * math.cos(angle)
        v_beta = mag * math.sin(angle)
        # Inverse Clarke: va = v_alpha, vb = (-v_alpha + sqrt(3)*v_beta)/2...
        v_a = v_alpha
        v_b = -0.5 * v_alpha + (math.sqrt(3) / 2.0) * v_beta
        v_c = -0.5 * v_alpha - (math.sqrt(3) / 2.0) * v_beta
        sector = InverterCurrentSense.sector_from_voltages(np.array([v_a, v_b, v_c]))
        assert sector == expected_sector

    def test_near_zero_voltage_returns_none(self):
        sector = InverterCurrentSense.sector_from_voltages(
            np.array([0.001, -0.001, 0.0])
        )
        assert sector is None

    def test_zero_voltages_returns_none(self):
        sector = InverterCurrentSense.sector_from_voltages(np.zeros(3))
        assert sector is None


# ---------------------------------------------------------------------------
# Single-shunt sector-aware reconstruction: exact for all 6 sectors
# ---------------------------------------------------------------------------


class TestSingleShuntSectorAwareReconstruction:
    """Each sector uses two active-vector samples to recover all 3 phase currents."""

    def _sense(self) -> InverterCurrentSense:
        return InverterCurrentSense(
            topology="single",
            channels=[_ideal_channel()],
        )

    @pytest.mark.parametrize(
        "sector, currents",
        [
            # Sector 1 (angle 0-60°): typical: i_a>0, i_c<0
            (1, [5.0, -2.0, -3.0]),
            # Sector 2 (angle 60-120°): i_b>0
            (2, [-3.0, 6.0, -3.0]),
            # Sector 3 (angle 120-180°): i_b>0, i_a<0
            (3, [-2.5, 5.0, -2.5]),
            # Sector 4 (angle 180-240°): i_a<0
            (4, [-5.0, 2.0, 3.0]),
            # Sector 5 (angle 240-300°): i_c>0
            (5, [3.0, -6.0, 3.0]),
            # Sector 6 (angle 300-360°): i_a>0, i_b<0
            (6, [2.5, -5.0, 2.5]),
        ],
    )
    def test_sector_aware_perfect_reconstruction(self, sector, currents):
        sense = self._sense()
        out = sense.measure(np.array(currents, dtype=float), dt=1e-4, svm_sector=sector)
        m = out["currents_abc_measured"]
        assert m == pytest.approx(currents, abs=1e-9), (
            f"Sector {sector}: expected {currents}, got {m.tolist()}"
        )

    def test_kirchhoff_holds_after_sector_aware_reconstruction(self):
        sense = self._sense()
        for sector, currents in [
            (1, [4.0, -1.5, -2.5]),
            (3, [-1.5, 4.0, -2.5]),
            (5, [2.0, -4.5, 2.5]),
        ]:
            sense.reset()
            out = sense.measure(
                np.array(currents, dtype=float), dt=1e-4, svm_sector=sector
            )
            m = out["currents_abc_measured"]
            assert m[0] + m[1] + m[2] == pytest.approx(0.0, abs=1e-9), (
                f"Kirchhoff violated in sector {sector}: sum={m.sum()}"
            )

    def test_zero_currents_reconstruct_to_zero(self):
        for sector in range(1, 7):
            sense = self._sense()
            out = sense.measure(np.zeros(3), dt=1e-4, svm_sector=sector)
            np.testing.assert_allclose(
                out["currents_abc_measured"], np.zeros(3), atol=1e-12
            )

    def test_sector_hint_none_falls_back_gracefully(self):
        """No sector → fallback approximation; result is scale-consistent."""
        sense = self._sense()
        currents = np.array([5.0, -2.0, -3.0])
        out = sense.measure(currents, dt=1e-4, svm_sector=None)
        m = out["currents_abc_measured"]
        assert m.shape == (3,)
        assert np.isfinite(m).all()

    def test_sector_aware_matches_triple_with_same_ideal_channels(self):
        """With identical ideal channels, single-shunt sector-aware and triple must agree."""
        triple = InverterCurrentSense(
            topology="triple",
            channels=[_ideal_channel() for _ in range(3)],
        )
        single = self._sense()

        for sector, currents in [
            (1, [5.0, -2.0, -3.0]),
            (2, [-3.0, 6.0, -3.0]),
            (4, [-5.0, 2.0, 3.0]),
        ]:
            triple.reset()
            single.reset()
            m_triple = triple.measure(np.array(currents, dtype=float), dt=1e-4)[
                "currents_abc_measured"
            ]
            m_single = single.measure(
                np.array(currents, dtype=float), dt=1e-4, svm_sector=sector
            )["currents_abc_measured"]
            np.testing.assert_allclose(m_triple, m_single, atol=1e-9)


# ---------------------------------------------------------------------------
# Engine integration: history["currents_*_true"] vs history["currents_*"]
# ---------------------------------------------------------------------------


class TestEngineHistoryTrueVsMeasured:
    def _run_engine(self, topology: str, n_steps: int = 200) -> dict:
        _, engine = _make_motor_and_engine(topology)
        voltages = np.array([10.0, -5.0, -5.0])
        for _ in range(n_steps):
            engine.step(voltages)
        return engine.get_history()

    def test_true_history_keys_present(self):
        history = self._run_engine("triple")
        for key in ("currents_a_true", "currents_b_true", "currents_c_true"):
            assert key in history, f"Missing key: {key}"

    def test_true_history_length_equals_measured(self):
        history = self._run_engine("double", n_steps=100)
        assert len(history["currents_a_true"]) == len(history["currents_a"])
        assert len(history["currents_b_true"]) == len(history["currents_b"])
        assert len(history["currents_c_true"]) == len(history["currents_c"])

    @pytest.mark.parametrize("topology", ["triple", "double"])
    def test_ideal_channels_measured_matches_true(self, topology):
        """With ideal channels, measured history must equal true history."""
        history = self._run_engine(topology, n_steps=300)
        for ph in ("a", "b", "c"):
            np.testing.assert_allclose(
                history[f"currents_{ph}"],
                history[f"currents_{ph}_true"],
                atol=1e-9,
                err_msg=f"topology={topology}, phase={ph}: measured ≠ true",
            )

    def test_single_shunt_sector_aware_measured_matches_true(self):
        """Single-shunt with sector-aware reconstruction must track true currents
        at all timesteps (ideal channels: no gain/offset error, transparent filter)."""
        _, engine = _make_motor_and_engine("single")
        # Drive with sinusoidal voltages so all sectors are visited
        dt = engine.dt
        mag = 15.0
        for k in range(600):
            theta = 2.0 * math.pi * 50.0 * k * dt  # 50 Hz reference
            v_a = mag * math.cos(theta)
            v_b = mag * math.cos(theta - 2.0 * math.pi / 3.0)
            v_c = mag * math.cos(theta + 2.0 * math.pi / 3.0)
            engine.step(np.array([v_a, v_b, v_c]))

        history = engine.get_history()
        for ph in ("a", "b", "c"):
            np.testing.assert_allclose(
                history[f"currents_{ph}"],
                history[f"currents_{ph}_true"],
                atol=1e-9,
                err_msg=f"single shunt phase {ph}: measured ≠ true",
            )

    def test_no_current_sense_true_and_measured_identical(self):
        """Without current_sense, history true and measured must be the same arrays."""
        params = MotorParameters(
            resistance=0.5,
            inductance=1e-3,
            back_emf_constant=0.05,
            moment_of_inertia=1e-4,
            num_pole_pairs=4,
        )
        motor = BLDCMotor(params, dt=1e-4)
        load = ConstantLoad(torque=0.1)
        engine = SimulationEngine(motor, load, dt=1e-4)  # no current_sense
        for _ in range(100):
            engine.step(np.array([10.0, -5.0, -5.0]))
        history = engine.get_history()
        for ph in ("a", "b", "c"):
            np.testing.assert_allclose(
                history[f"currents_{ph}"],
                history[f"currents_{ph}_true"],
                atol=1e-12,
            )

    def test_engine_reset_clears_true_history(self):
        _, engine = _make_motor_and_engine("triple")
        for _ in range(50):
            engine.step(np.array([5.0, -2.5, -2.5]))
        engine.reset()
        history = engine.get_history()
        assert len(history["currents_a_true"]) == 0
        assert len(history["currents_b_true"]) == 0
        assert len(history["currents_c_true"]) == 0

    def test_get_current_state_includes_true_values_with_sense(self):
        _, engine = _make_motor_and_engine("triple")
        for _ in range(10):
            engine.step(np.array([10.0, -5.0, -5.0]))
        state = engine.get_current_state()
        assert "currents_a_true" in state
        assert "currents_b_true" in state
        assert "currents_c_true" in state

    def test_get_controller_phase_currents_returns_measured_with_ideal_triple(self):
        _, engine = _make_motor_and_engine("triple")
        for _ in range(20):
            engine.step(np.array([10.0, -5.0, -5.0]))
        ctrl_currents = engine.get_controller_phase_currents()
        true_currents = engine.motor.currents
        # With ideal triple-shunt channels, controller view = true physics
        np.testing.assert_allclose(ctrl_currents, true_currents, atol=1e-9)
