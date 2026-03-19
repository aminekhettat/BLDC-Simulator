"""Phase-current measurement model based on shunt resistors and a differential
amplifier with a single-pole low-pass anti-aliasing filter.

Signal chain (per channel)
--------------------------
1. **Shunt resistor**:
   ``V_shunt = I_true × R_shunt``

2. **Differential amplifier** (with optional gain and DC offset errors):
   ``V_amp = amplifier_gain × (1 + gain_error) × V_shunt + v_offset``

3. **Single-pole low-pass filter** (backward-Euler IIR):
   Time constant ``τ = r_feedback × c_filter``  →  cutoff ``f_c = 1 / (2π τ)``
   Discrete update (backward Euler): ``y[k] = (τ·y[k-1] + dt·x[k]) / (τ + dt)``

4. **Reconstruction** (what the controller reads):
   ``I_measured = V_filt / (R_shunt × amplifier_gain)``

With ideal parameters (``gain_error=0``, ``v_offset=0``) the model reduces to a
low-pass filtered copy of the true current.  Non-zero ``gain_error`` or
``v_offset`` introduces systematic measurement errors that propagate through the
FOC controller, degrading regulation accuracy --- this is the effect modelled by
the second Near-Term roadmap item ("evaluate how gain/offset errors affect
controller behaviour").
"""

from __future__ import annotations

import math

import numpy as np


class CurrentSensorModel:
    """Phase-current sensor model: shunt + differential amplifier + LP filter.

    Parameters
    ----------
    r_shunt:
        Shunt resistance (Ω).  Common range 0.0001–0.01 Ω.
    amplifier_gain:
        Nominal differential-amplifier gain (V/V).  Converts V_shunt to a
        measurable voltage range.  Common range 10–200 V/V.
    gain_error:
        Fractional gain error applied on top of *amplifier_gain*.  A value of
        ``0.02`` models a 2 % positive gain error.  Default 0.0 (ideal).
    v_offset:
        DC offset voltage (V) added at the amplifier output.  Models input-
        referred offset in the sense amplifier.  Default 0.0 (ideal).
    r_feedback:
        Feedback resistor of the amplifier (Ω).  Together with *c_filter* it
        sets the LP filter time constant ``τ = r_feedback × c_filter``.
    c_filter:
        Filter capacitor across the feedback resistor (F).  Set to 0 to
        bypass the filter entirely.
    n_channels:
        Number of independent measurement channels.  Typically 3 (3-phase).
    """

    def __init__(
        self,
        r_shunt: float = 0.001,
        amplifier_gain: float = 20.0,
        gain_error: float = 0.0,
        v_offset: float = 0.0,
        r_feedback: float = 10_000.0,
        c_filter: float = 1.0e-9,
        n_channels: int = 3,
    ) -> None:
        if r_shunt <= 0:
            raise ValueError(f"r_shunt must be positive, got {r_shunt}")
        if amplifier_gain <= 0:
            raise ValueError(f"amplifier_gain must be positive, got {amplifier_gain}")
        if r_feedback < 0:
            raise ValueError(f"r_feedback must be non-negative, got {r_feedback}")
        if c_filter < 0:
            raise ValueError(f"c_filter must be non-negative, got {c_filter}")
        if n_channels < 1:
            raise ValueError(f"n_channels must be >= 1, got {n_channels}")

        self._r_shunt = float(r_shunt)
        self._amplifier_gain = float(amplifier_gain)
        self._gain_error = float(gain_error)
        self._v_offset = float(v_offset)
        self._tau = float(r_feedback) * float(c_filter)  # τ = R_fb × C
        self._n_channels = int(n_channels)

        # Per-channel LP filter state (holds the filtered voltage)
        self._filter_state = np.zeros(self._n_channels, dtype=np.float64)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def r_shunt(self) -> float:
        """Shunt resistance in Ω."""
        return self._r_shunt

    @property
    def amplifier_gain(self) -> float:
        """Nominal amplifier gain (V/V)."""
        return self._amplifier_gain

    @property
    def gain_error(self) -> float:
        """Fractional gain error (dimensionless)."""
        return self._gain_error

    @property
    def v_offset(self) -> float:
        """Amplifier output DC offset (V)."""
        return self._v_offset

    @property
    def tau(self) -> float:
        """LP filter time constant τ = r_feedback × c_filter (s)."""
        return self._tau

    @property
    def cutoff_frequency_hz(self) -> float:
        """LP filter −3 dB cutoff frequency (Hz).

        Returns ``float('inf')`` when the filter is bypassed (τ = 0).
        """
        if self._tau == 0.0:
            return float("inf")
        return 1.0 / (2.0 * math.pi * self._tau)

    @property
    def n_channels(self) -> int:
        """Number of measurement channels."""
        return self._n_channels

    @property
    def filter_state(self) -> np.ndarray:
        """Read-only view of the current filter state (filtered voltage, V)."""
        return self._filter_state.copy()

    # ------------------------------------------------------------------
    # Public methods
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Reset all LP filter states to zero (call before a new simulation run)."""
        self._filter_state[:] = 0.0

    def measure(self, currents: np.ndarray, dt: float) -> np.ndarray:
        """Convert true phase currents to sensor-output (measured) currents.

        Parameters
        ----------
        currents:
            Array of true phase currents (A), shape ``(n_channels,)``.
        dt:
            Simulation time step (s).  Must be positive.

        Returns
        -------
        numpy.ndarray
            Measured (reconstructed) phase currents (A), shape ``(n_channels,)``.
            These are what the controller reads; they incorporate gain error,
            offset error, and LP-filter phase/amplitude distortion.
        """
        currents = np.asarray(currents, dtype=np.float64)
        if currents.shape != (self._n_channels,):
            raise ValueError(
                f"Expected currents shape ({self._n_channels},), got {currents.shape}"
            )
        if dt <= 0:
            raise ValueError(f"dt must be positive, got {dt}")

        # 1. Shunt: convert current to voltage
        v_shunt = currents * self._r_shunt

        # 2. Amplifier: apply actual gain (nominal × (1 + error)) and DC offset
        actual_gain = self._amplifier_gain * (1.0 + self._gain_error)
        v_amp = actual_gain * v_shunt + self._v_offset

        # 3. Backward-Euler single-pole LP filter
        #    y[k] = (τ·y[k-1] + dt·x[k]) / (τ + dt)
        #    When τ = 0, the filter is bypassed (y[k] = x[k]).
        if self._tau > 0.0:
            self._filter_state = (self._tau * self._filter_state + dt * v_amp) / (
                self._tau + dt
            )
        else:
            self._filter_state = v_amp.copy()

        # 4. Reconstruct current using *nominal* gain (this is what the MCU computes)
        i_measured = self._filter_state / (self._r_shunt * self._amplifier_gain)
        return i_measured

    def get_state(self) -> dict:
        """Return a snapshot of the sensor model state for logging / display."""
        return {
            "filter_state_v": self._filter_state.tolist(),
            "cutoff_frequency_hz": self.cutoff_frequency_hz,
            "tau_s": self._tau,
            "gain_error": self._gain_error,
            "v_offset_v": self._v_offset,
        }
