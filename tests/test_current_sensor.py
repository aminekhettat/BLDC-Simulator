"""
Atomic features tested in this module:
- Construction
- Properties
- IdealBypassFilter
- GainError
- OffsetError
- LowPassFilter
- Reset
- InputValidation
- GetState
"""

import math

import numpy as np
import pytest

from src.hardware.current_sensor import CurrentSensorModel

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_ideal(f_c_hz: float = 10_000.0, n_ch: int = 3) -> CurrentSensorModel:
    """Return an ideal sensor (no gain/offset error) with given cutoff."""
    r_fb = 10_000.0
    c = 1.0 / (2.0 * math.pi * r_fb * f_c_hz)
    return CurrentSensorModel(
        r_shunt=0.001,
        amplifier_gain=20.0,
        gain_error=0.0,
        v_offset=0.0,
        r_feedback=r_fb,
        c_filter=c,
        n_channels=n_ch,
    )


# ---------------------------------------------------------------------------
# Construction / validation
# ---------------------------------------------------------------------------


class TestConstruction:
    def test_default_construction(self):
        s = CurrentSensorModel()
        assert s.n_channels == 3
        assert s.r_shunt == pytest.approx(0.001)
        assert s.amplifier_gain == pytest.approx(20.0)
        assert s.gain_error == pytest.approx(0.0)
        assert s.v_offset == pytest.approx(0.0)

    def test_invalid_r_shunt_raises(self):
        with pytest.raises(ValueError, match="r_shunt"):
            CurrentSensorModel(r_shunt=-0.001)

    def test_invalid_zero_r_shunt_raises(self):
        with pytest.raises(ValueError, match="r_shunt"):
            CurrentSensorModel(r_shunt=0.0)

    def test_invalid_amplifier_gain_raises(self):
        with pytest.raises(ValueError, match="amplifier_gain"):
            CurrentSensorModel(amplifier_gain=0.0)

    def test_invalid_r_feedback_raises(self):
        with pytest.raises(ValueError, match="r_feedback"):
            CurrentSensorModel(r_feedback=-1.0)

    def test_invalid_c_filter_raises(self):
        with pytest.raises(ValueError, match="c_filter"):
            CurrentSensorModel(c_filter=-1e-9)

    def test_invalid_n_channels_raises(self):
        with pytest.raises(ValueError, match="n_channels"):
            CurrentSensorModel(n_channels=0)


# ---------------------------------------------------------------------------
# Properties
# ---------------------------------------------------------------------------


class TestProperties:
    def test_cutoff_frequency_matches_rc(self):
        r_fb, c = 10_000.0, 1e-9
        s = CurrentSensorModel(r_feedback=r_fb, c_filter=c)
        expected = 1.0 / (2.0 * math.pi * r_fb * c)
        assert s.cutoff_frequency_hz == pytest.approx(expected, rel=1e-9)

    def test_cutoff_inf_when_c_filter_zero(self):
        s = CurrentSensorModel(c_filter=0.0, r_feedback=1_000.0)
        assert math.isinf(s.cutoff_frequency_hz)

    def test_tau_equals_r_times_c(self):
        r_fb, c = 5_000.0, 2e-9
        s = CurrentSensorModel(r_feedback=r_fb, c_filter=c)
        assert s.tau == pytest.approx(r_fb * c, rel=1e-12)

    def test_filter_state_is_copy(self):
        s = CurrentSensorModel()
        view = s.filter_state
        view[0] = 9999.0  # mutate the returned copy
        assert s.filter_state[0] != 9999.0  # internal state unchanged


# ---------------------------------------------------------------------------
# Ideal sensor (no error, no filter) â€” bypass path
# ---------------------------------------------------------------------------


class TestIdealBypassFilter:
    """c_filter=0 â†’ Ï„=0 â†’ instant response, no distortion."""

    def test_single_channel_ideal(self):
        s = CurrentSensorModel(r_shunt=0.001, amplifier_gain=20.0, c_filter=0.0, n_channels=1)
        out = s.measure(np.array([5.0]), dt=1e-4)
        assert out[0] == pytest.approx(5.0, rel=1e-9)

    def test_three_channel_ideal(self):
        s = CurrentSensorModel(r_shunt=0.001, amplifier_gain=20.0, c_filter=0.0, n_channels=3)
        inp = np.array([3.0, -2.0, 1.5])
        out = s.measure(inp, dt=1e-4)
        np.testing.assert_allclose(out, inp, rtol=1e-9)

    def test_output_independent_of_r_shunt_direction(self):
        """Changing R_shunt should not change reconstructed current (ideal)."""
        inp = np.array([10.0])
        for r in [0.0005, 0.001, 0.005, 0.01]:
            s = CurrentSensorModel(r_shunt=r, amplifier_gain=20.0, c_filter=0.0, n_channels=1)
            out = s.measure(inp.copy(), dt=1e-4)
            assert out[0] == pytest.approx(10.0, rel=1e-9), f"failed for r_shunt={r}"


# ---------------------------------------------------------------------------
# Gain error
# ---------------------------------------------------------------------------


class TestGainError:
    def test_positive_gain_error_scales_current_up(self):
        s = CurrentSensorModel(gain_error=0.10, c_filter=0.0, n_channels=1)
        i_true = np.array([5.0])
        i_meas = s.measure(i_true, dt=1e-4)
        # actual_gain = nominal Ã— 1.10, reconstruction uses nominal â†’ factor 1.10
        assert i_meas[0] == pytest.approx(5.0 * 1.10, rel=1e-9)

    def test_negative_gain_error_scales_current_down(self):
        s = CurrentSensorModel(gain_error=-0.05, c_filter=0.0, n_channels=1)
        i_true = np.array([4.0])
        i_meas = s.measure(i_true, dt=1e-4)
        assert i_meas[0] == pytest.approx(4.0 * 0.95, rel=1e-9)

    def test_gain_error_zero_is_transparent(self):
        s = CurrentSensorModel(gain_error=0.0, c_filter=0.0, n_channels=1)
        i_true = np.array([7.3])
        assert s.measure(i_true, dt=1e-4)[0] == pytest.approx(7.3, rel=1e-9)


# ---------------------------------------------------------------------------
# Offset error
# ---------------------------------------------------------------------------


class TestOffsetError:
    def test_positive_offset_shifts_measured_current(self):
        r_shunt = 0.001
        gain = 20.0
        v_off = 0.05  # 50 mV offset
        s = CurrentSensorModel(
            r_shunt=r_shunt,
            amplifier_gain=gain,
            v_offset=v_off,
            c_filter=0.0,
            n_channels=1,
        )
        i_true = np.array([10.0])
        i_meas = s.measure(i_true, dt=1e-4)
        expected_shift = v_off / (r_shunt * gain)
        assert i_meas[0] == pytest.approx(10.0 + expected_shift, rel=1e-6)

    def test_zero_current_with_offset_returns_offset_residual(self):
        r_shunt = 0.001
        gain = 20.0
        v_off = 0.02
        s = CurrentSensorModel(
            r_shunt=r_shunt,
            amplifier_gain=gain,
            v_offset=v_off,
            c_filter=0.0,
            n_channels=1,
        )
        i_true = np.array([0.0])
        i_meas = s.measure(i_true, dt=1e-4)
        assert i_meas[0] == pytest.approx(v_off / (r_shunt * gain), rel=1e-6)


# ---------------------------------------------------------------------------
# LP filter â€” step response and time-constant accuracy
# ---------------------------------------------------------------------------


class TestLowPassFilter:
    def test_step_response_settles_to_input(self):
        """After many time-constants the filter output must track the input."""
        f_c = 5_000.0  # 5 kHz
        s = _make_ideal(f_c_hz=f_c, n_ch=1)
        dt = 1e-5  # 100 kHz sample rate â†’ well above cutoff
        i_step = np.array([1.0])

        # Simulate ~10 time constants
        tau = s.tau
        n_steps = int(10.0 * tau / dt)
        out = None
        for _ in range(n_steps):
            out = s.measure(i_step, dt=dt)
        # After 10Ï„ the backward-Euler output must be within 0.05 % of the step
        assert out is not None
        assert out[0] == pytest.approx(1.0, abs=5e-4)

    def test_step_response_at_one_tau_below_63pct(self):
        """At t â‰ˆ Ï„ the backward-Euler output should be near 1-exp(-1) â‰ˆ 63 %."""
        f_c = 10_000.0
        s = _make_ideal(f_c_hz=f_c, n_ch=1)
        dt = 1e-7  # very small step â†’ accurate approximation
        i_step = np.array([1.0])

        tau = s.tau
        n_steps = int(tau / dt)
        out = None
        for _ in range(n_steps):
            out = s.measure(i_step, dt=dt)
        # Theoretical: 1 - exp(-1) â‰ˆ 0.6321; backward Euler converges to this
        assert out is not None
        assert 0.60 < out[0] < 0.66

    def test_filter_attenuates_signal_above_cutoff(self):
        """Signal far above f_c should be strongly attenuated."""
        f_c = 1_000.0  # 1 kHz cutoff
        s = _make_ideal(f_c_hz=f_c, n_ch=1)
        dt = 1e-6  # 1 MHz sample rate

        # Drive with sinewave at 100 Ã— f_c
        f_sig = 100.0 * f_c
        omega = 2.0 * math.pi * f_sig
        n_cycles = 5
        n_steps = int(n_cycles / (f_sig * dt))
        amplitudes = []
        for k in range(n_steps):
            t = k * dt
            i_in = np.array([math.sin(omega * t)])
            out = s.measure(i_in, dt=dt)
            if k > n_steps // 2:  # collect second half (filter settled)
                amplitudes.append(abs(out[0]))

        # At 100 Ã— f_c the expected gain is â‰ˆ 1/100; allow 3Ã— margin
        assert max(amplitudes) < 0.03

    def test_filter_passes_dc_unchanged(self):
        """DC signal should pass through the filter with unity gain (ideal sensor)."""
        s = _make_ideal(f_c_hz=5_000.0, n_ch=1)
        dt = 1e-5
        i_dc = np.array([3.0])
        tau = s.tau
        n_steps = int(20.0 * tau / dt)
        out = None
        for _ in range(n_steps):
            out = s.measure(i_dc, dt=dt)
        assert out is not None
        assert out[0] == pytest.approx(3.0, rel=1e-3)


# ---------------------------------------------------------------------------
# reset()
# ---------------------------------------------------------------------------


class TestReset:
    def test_reset_clears_filter_state(self):
        s = _make_ideal(f_c_hz=1_000.0, n_ch=1)
        dt = 1e-4
        for _ in range(100):
            s.measure(np.array([5.0]), dt=dt)
        assert s.filter_state[0] != pytest.approx(0.0, abs=1e-3)
        s.reset()
        assert s.filter_state[0] == pytest.approx(0.0, abs=1e-12)

    def test_measure_after_reset_starts_from_zero(self):
        s = _make_ideal(f_c_hz=500.0, n_ch=1)
        dt = 1e-4
        # Prime the filter
        for _ in range(500):
            s.measure(np.array([1.0]), dt=dt)
        s.reset()
        out_first = s.measure(np.array([1.0]), dt=dt)
        # After reset, first output = dt/(Ï„+dt) Ã— 1.0/R_shunt/gain (from zero)
        tau = s.tau
        expected = dt / (tau + dt)  # fraction of the step (before gain inversion, remains 1:1)
        assert out_first[0] == pytest.approx(expected, rel=1e-6)


# ---------------------------------------------------------------------------
# Input validation
# ---------------------------------------------------------------------------


class TestInputValidation:
    def test_wrong_channel_count_raises(self):
        s = CurrentSensorModel(n_channels=3)
        with pytest.raises(ValueError, match="shape"):
            s.measure(np.array([1.0, 2.0]), dt=1e-4)  # 2 channels, expected 3

    def test_negative_dt_raises(self):
        s = CurrentSensorModel(n_channels=1)
        with pytest.raises(ValueError, match="dt"):
            s.measure(np.array([1.0]), dt=-1e-4)

    def test_zero_dt_raises(self):
        s = CurrentSensorModel(n_channels=1)
        with pytest.raises(ValueError, match="dt"):
            s.measure(np.array([1.0]), dt=0.0)


# ---------------------------------------------------------------------------
# get_state()
# ---------------------------------------------------------------------------


class TestGetState:
    def test_get_state_contains_required_keys(self):
        s = CurrentSensorModel()
        state = s.get_state()
        for key in (
            "filter_state_v",
            "cutoff_frequency_hz",
            "tau_s",
            "gain_error",
            "v_offset_v",
        ):
            assert key in state, f"Missing key: {key}"

    def test_get_state_filter_state_length_matches_channels(self):
        s = CurrentSensorModel(n_channels=3)
        assert len(s.get_state()["filter_state_v"]) == 3

    def test_get_state_reflects_current_filter_values(self):
        s = CurrentSensorModel(c_filter=0.0, n_channels=1)
        s.measure(np.array([2.0]), dt=1e-4)
        state = s.get_state()
        # With bypass filter, V_filt = V_amp = gain Ã— R_shunt Ã— I
        v_amp = s.amplifier_gain * s.r_shunt * 2.0
        assert state["filter_state_v"][0] == pytest.approx(v_amp, rel=1e-9)
