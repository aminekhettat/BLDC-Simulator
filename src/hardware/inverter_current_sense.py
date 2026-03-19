"""Topology-aware inverter current sensing model.

This module models realistic low-side shunt sensing for single, double, and
triple shunt inverter configurations. Each shunt has an associated differential
amplifier chain with nominal/actual gain and offset plus a first-order
low-pass filter and ADC saturation.

The controller reconstructs currents using *nominal* amplifier information while
the analog front-end uses *actual* values, which allows runtime gain/offset
deviation studies.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal
import math

import numpy as np

Topology = Literal["single", "double", "triple"]


@dataclass
class ShuntAmplifierChannel:
    """One shunt + op-amp + ADC channel model."""

    r_shunt_ohm: float = 0.001
    nominal_gain: float = 20.0
    nominal_offset_v: float = 1.65
    actual_gain: float = 20.0
    actual_offset_v: float = 1.65
    cutoff_frequency_hz: float = 20_000.0
    vcc: float = 3.3

    def __post_init__(self) -> None:
        if self.r_shunt_ohm <= 0.0:
            raise ValueError("r_shunt_ohm must be positive")
        if self.nominal_gain <= 0.0:
            raise ValueError("nominal_gain must be positive")
        if self.actual_gain <= 0.0:
            raise ValueError("actual_gain must be positive")
        if self.cutoff_frequency_hz <= 0.0:
            raise ValueError("cutoff_frequency_hz must be positive")
        if self.vcc <= 0.0:
            raise ValueError("vcc must be positive")


class InverterCurrentSense:
    """Inverter current sensing model with shunt topology awareness.

    Notes
    -----
    - `triple`: one shunt/op-amp per phase (A, B, C).
    - `double`: shunts in legs A and B. C is reconstructed from Kirchhoff.
    - `single`: one collector shunt. For controller-facing phase-current output,
      this implementation uses an "ideal PWM observability" approximation where
      the measured bus-current envelope scales the internal phase-current shape.
      This is intentionally marked approximate and can be replaced by a strict
      PWM-window reconstruction model later.
    """

    def __init__(
        self,
        topology: Topology = "triple",
        channels: list[ShuntAmplifierChannel] | None = None,
    ) -> None:
        if topology not in ("single", "double", "triple"):
            raise ValueError("topology must be one of: single, double, triple")

        self.topology: Topology = topology
        expected = {"single": 1, "double": 2, "triple": 3}[topology]

        if channels is None:
            channels = [ShuntAmplifierChannel() for _ in range(expected)]
        if len(channels) != expected:
            raise ValueError(
                f"Topology '{topology}' requires {expected} channel(s), got {len(channels)}"
            )

        self.channels = channels
        self._filter_state_v = np.zeros(expected, dtype=np.float64)
        self._last_measured_currents_abc = np.zeros(3, dtype=np.float64)

    @property
    def n_shunts(self) -> int:
        return len(self.channels)

    def reset(self) -> None:
        self._filter_state_v[:] = 0.0
        self._last_measured_currents_abc[:] = 0.0

    def set_actual_channel(
        self, index: int, gain: float | None = None, offset_v: float | None = None
    ) -> None:
        """Update actual gain/offset online while control still uses nominal values."""
        ch = self.channels[index]
        if gain is not None:
            if gain <= 0.0:
                raise ValueError("actual gain must be positive")
            ch.actual_gain = float(gain)
        if offset_v is not None:
            ch.actual_offset_v = float(offset_v)

    def _channel_measure(
        self, i_shunt_a: float, dt: float, idx: int
    ) -> tuple[float, float, bool]:
        """Return (i_reconstructed, v_adc, saturated) for one channel."""
        ch = self.channels[idx]

        # True analog chain
        v_shunt = i_shunt_a * ch.r_shunt_ohm
        v_amp = ch.actual_gain * v_shunt + ch.actual_offset_v

        # LP filter: backward Euler
        tau = 1.0 / (2.0 * math.pi * ch.cutoff_frequency_hz)
        v_filt = (tau * self._filter_state_v[idx] + dt * v_amp) / (tau + dt)

        # ADC clamp
        v_adc = float(np.clip(v_filt, 0.0, ch.vcc))
        saturated = (v_adc <= 1e-12) or (v_adc >= ch.vcc - 1e-12)
        self._filter_state_v[idx] = v_adc

        # Controller reconstruction with nominal information
        i_reco = (v_adc - ch.nominal_offset_v) / (ch.nominal_gain * ch.r_shunt_ohm)
        return i_reco, v_adc, saturated

    def measure(self, currents_abc: np.ndarray, dt: float) -> dict:
        """Measure and reconstruct phase currents for the configured topology."""
        currents = np.asarray(currents_abc, dtype=np.float64)
        if currents.shape != (3,):
            raise ValueError(f"Expected currents_abc shape (3,), got {currents.shape}")
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        i_a, i_b, i_c = float(currents[0]), float(currents[1]), float(currents[2])
        v_adc = np.zeros(self.n_shunts, dtype=np.float64)
        sat = np.zeros(self.n_shunts, dtype=bool)

        if self.topology == "triple":
            i_ra, v_adc[0], sat[0] = self._channel_measure(i_a, dt, 0)
            i_rb, v_adc[1], sat[1] = self._channel_measure(i_b, dt, 1)
            i_rc, v_adc[2], sat[2] = self._channel_measure(i_c, dt, 2)
            measured = np.array([i_ra, i_rb, i_rc], dtype=np.float64)

        elif self.topology == "double":
            i_ra, v_adc[0], sat[0] = self._channel_measure(i_a, dt, 0)
            i_rb, v_adc[1], sat[1] = self._channel_measure(i_b, dt, 1)
            i_rc = -(i_ra + i_rb)
            measured = np.array([i_ra, i_rb, i_rc], dtype=np.float64)

        else:
            # Single-shunt approximation: collector return-current envelope drives
            # one analog channel; phase vector shape follows internal true-current
            # ratios (ideal PWM observability approximation for this first slice).
            i_bus = 0.5 * (abs(i_a) + abs(i_b) + abs(i_c))
            i_bus_reco, v_adc[0], sat[0] = self._channel_measure(i_bus, dt, 0)

            denom = max(0.5 * (abs(i_a) + abs(i_b) + abs(i_c)), 1e-9)
            scale = i_bus_reco / denom
            measured = np.array(
                [i_a * scale, i_b * scale, i_c * scale], dtype=np.float64
            )

        self._last_measured_currents_abc = measured
        return {
            "topology": self.topology,
            "currents_abc_measured": measured,
            "amplifier_outputs_v": v_adc,
            "adc_saturated": sat,
        }

    def apply_shunt_voltage_drop(
        self, commanded_voltages_abc: np.ndarray, currents_abc: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Apply voltage drop caused by shunt insertion in the inverter return paths.

        Returns `(effective_voltages_abc, phase_drop_abc)`.
        """
        v_cmd = np.asarray(commanded_voltages_abc, dtype=np.float64)
        i_abc = np.asarray(currents_abc, dtype=np.float64)
        if v_cmd.shape != (3,) or i_abc.shape != (3,):
            raise ValueError(
                "commanded_voltages_abc and currents_abc must have shape (3,)"
            )

        drop = np.zeros(3, dtype=np.float64)

        if self.topology == "triple":
            drop[0] = i_abc[0] * self.channels[0].r_shunt_ohm
            drop[1] = i_abc[1] * self.channels[1].r_shunt_ohm
            drop[2] = i_abc[2] * self.channels[2].r_shunt_ohm
            v_eff = v_cmd - drop

        elif self.topology == "double":
            drop[0] = i_abc[0] * self.channels[0].r_shunt_ohm
            drop[1] = i_abc[1] * self.channels[1].r_shunt_ohm
            v_eff = v_cmd - drop

        else:
            i_bus = 0.5 * (abs(i_abc[0]) + abs(i_abc[1]) + abs(i_abc[2]))
            v_drop_common = i_bus * self.channels[0].r_shunt_ohm
            sign = np.sign(v_cmd)
            drop = v_drop_common * sign
            v_eff = v_cmd - drop

        return v_eff, drop

    def get_state(self) -> dict:
        return {
            "topology": self.topology,
            "n_shunts": self.n_shunts,
            "last_measured_currents_abc": self._last_measured_currents_abc.tolist(),
            "filter_state_v": self._filter_state_v.tolist(),
        }
