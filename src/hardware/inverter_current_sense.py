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
    - `single`: one DC-bus shunt. When an SVM sector is provided, true
      sector-aware reconstruction is used: each active vector contributes one
      measurable bus-current sample that directly encodes one phase current
      (or its negative). Two samples + Kirchhoff reconstruct all three phases
      exactly with ideal channels. Without a sector hint, a fallback
      approximation is used instead.
    """

    # Per-sector SVM active-vector observable bus currents for single-shunt.
    # Each entry: ((sign_v1, phase_idx_v1), (sign_v2, phase_idx_v2), kirchhoff_idx)
    # Meaning: i_bus_sample = sign * i_phase[phase_idx]
    # Reconstruction: i_phase[phase_idx] = sign * i_bus_reconstructed
    # Kirchhoff:      i_phase[kirchhoff_idx] = -(i_ph_v1 + i_ph_v2)
    # Derivation: i_shunt = sum(i_x for phases where upper switch is ON).
    # SVM vectors per sector match sector_patterns in SVMGenerator.
    _SECTOR_OBSERVABLE: dict = {
        1: ((1, 0), (-1, 2), 1),  # V1=[1,0,0]: bus=+i_a  V2=[1,1,0]: bus=-i_c  KH:i_b
        2: ((-1, 2), (1, 1), 0),  # V1=[1,1,0]: bus=-i_c  V2=[0,1,0]: bus=+i_b  KH:i_a
        3: ((1, 1), (-1, 0), 2),  # V1=[0,1,0]: bus=+i_b  V2=[0,1,1]: bus=-i_a  KH:i_c
        4: ((-1, 0), (1, 2), 1),  # V1=[0,1,1]: bus=-i_a  V2=[0,0,1]: bus=+i_c  KH:i_b
        5: ((1, 2), (-1, 1), 0),  # V1=[0,0,1]: bus=+i_c  V2=[1,0,1]: bus=-i_b  KH:i_a
        6: ((-1, 1), (1, 0), 2),  # V1=[1,0,1]: bus=-i_b  V2=[1,0,0]: bus=+i_a  KH:i_c
    }

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

    @staticmethod
    def sector_from_voltages(
        voltages_abc: np.ndarray, min_magnitude: float = 0.1
    ) -> "int | None":
        """Derive the SVM sector (1-6) from averaged 3-phase voltages.

        Returns ``None`` when the voltage vector is too small to determine a
        reliable sector (near-zero modulation index), in which case the
        single-shunt measurement falls back to the envelope approximation.
        """
        v = np.asarray(voltages_abc, dtype=np.float64)
        v_alpha = (2.0 * v[0] - v[1] - v[2]) / 3.0
        v_beta = (v[1] - v[2]) / math.sqrt(3.0)
        magnitude = math.hypot(v_alpha, v_beta)
        if magnitude < min_magnitude:
            return None
        angle = math.atan2(v_beta, v_alpha) % (2.0 * math.pi)
        sector = int(angle / (math.pi / 3.0)) + 1
        return max(1, min(6, sector))

    def measure(
        self,
        currents_abc: np.ndarray,
        dt: float,
        svm_sector: "int | None" = None,
    ) -> dict:
        """Measure and reconstruct phase currents for the configured topology.

        Parameters
        ----------
        currents_abc:
            True motor phase currents [i_a, i_b, i_c] from the physics model.
        dt:
            Simulation time step [s].
        svm_sector:
            Optional SVM sector (1-6) for single-shunt sector-aware
            reconstruction. When provided, two bus-current samples (one per
            active vector) are computed from the sector table, passed through
            the analog channel, and used to recover all three phase currents
            exactly (ideal channels) or with realistic errors (non-ideal).
            When ``None``, the fallback envelope approximation is used.
        """
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
            if svm_sector is not None and svm_sector in self._SECTOR_OBSERVABLE:
                # Sector-aware reconstruction: two ADC samples per PWM cycle.
                # During active vector V1: i_bus_v1 = sign_v1 * i_phase[ph_v1]
                # During active vector V2: i_bus_v2 = sign_v2 * i_phase[ph_v2]
                # Both samples flow through the same analog path (single channel).
                (sign_v1, ph_v1), (sign_v2, ph_v2), ph_kh = self._SECTOR_OBSERVABLE[
                    svm_sector
                ]
                i_bus_v1 = sign_v1 * float(currents[ph_v1])
                i_bus_v2 = sign_v2 * float(currents[ph_v2])

                # Process both samples sequentially through the single channel.
                # (Both are taken within the same PWM cycle; filter advances once.)
                i_reco_v1, _, sat_v1 = self._channel_measure(i_bus_v1, dt, 0)
                i_reco_v2, v_adc[0], sat_v2 = self._channel_measure(i_bus_v2, dt, 0)
                sat[0] = sat_v1 or sat_v2

                # Recover individual phase currents from bus samples.
                i_ph_v1 = sign_v1 * i_reco_v1
                i_ph_v2 = sign_v2 * i_reco_v2
                i_ph_kh = -(i_ph_v1 + i_ph_v2)

                measured = np.zeros(3, dtype=np.float64)
                measured[ph_v1] = i_ph_v1
                measured[ph_v2] = i_ph_v2
                measured[ph_kh] = i_ph_kh
            else:
                # Fallback: bus-current envelope scales the true phase shape.
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
            "currents_abc_measured": measured.copy(),  # copy: prevent aliasing via reset()
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
