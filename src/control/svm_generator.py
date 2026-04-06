"""
SVM Generator Module
====================

This module implements Space Vector Modulation (SVM) for 3-phase voltage
generation and includes an optional stateful inverter-realism layer.

The realism layer is deliberately lightweight compared to a switching-device
transient simulator, but it captures the most important inverter effects used in
drive-system studies:

- Constant semiconductor conduction drop
- Current-dependent conduction loss
- Switching-frequency-dependent voltage loss
- Direction-aware dead-time distortion
- Freewheel diode path loss
- Minimum pulse suppression near zero voltage
- DC-link capacitor/ESR/source-resistance dynamics
- Junction-temperature-dependent parameter drift
- Per-phase asymmetry and mismatch

Each realism feature can be enabled or disabled independently so the simulator
can be tuned from ideal-average behavior up to a more realistic reduced-order
inverter model.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np


@dataclass
class InverterRealismConfig:
    """Configuration container for all switchable inverter-realism blocks."""

    enable_device_drop: bool = False
    device_drop_v: float = 0.0

    enable_dead_time: bool = False
    dead_time_fraction: float = 0.0

    enable_conduction_drop: bool = False
    conduction_resistance_ohm: float = 0.0

    enable_switching_loss: bool = False
    switching_frequency_hz: float = 0.0
    switching_loss_coeff_v_per_a_khz: float = 0.0

    enable_diode_freewheel: bool = False
    diode_drop_v: float = 0.0
    diode_resistance_ohm: float = 0.0

    enable_min_pulse: bool = False
    min_pulse_fraction: float = 0.0

    enable_bus_ripple: bool = False
    dc_link_capacitance_f: float = 0.0
    dc_link_source_resistance_ohm: float = 0.0
    dc_link_esr_ohm: float = 0.0

    enable_thermal_coupling: bool = False
    thermal_resistance_k_per_w: float = 0.0
    thermal_capacitance_j_per_k: float = 1.0
    ambient_temperature_c: float = 25.0
    temp_coeff_resistance_per_c: float = 0.0
    temp_coeff_drop_per_c: float = 0.0

    enable_phase_asymmetry: bool = False
    phase_voltage_scale_a: float = 1.0
    phase_voltage_scale_b: float = 1.0
    phase_voltage_scale_c: float = 1.0
    phase_drop_scale_a: float = 1.0
    phase_drop_scale_b: float = 1.0
    phase_drop_scale_c: float = 1.0


class SVMGenerator:
    """
    Space Vector Modulation Generator.

    The core SVM implementation remains an averaged inverter model. Realism is
    layered on top of that average model so users can independently enable or
    disable each effect.
    """

    def __init__(
        self,
        dc_voltage: float = 48.0,
        device_drop_v: float = 0.0,
        dead_time_fraction: float = 0.0,
        conduction_resistance_ohm: float = 0.0,
        switching_frequency_hz: float = 0.0,
        switching_loss_coeff_v_per_a_khz: float = 0.0,
    ):
        if dc_voltage <= 0.0:
            raise ValueError("DC voltage must be positive")

        self.dc_voltage = float(dc_voltage)
        self.source_dc_voltage = float(dc_voltage)
        self.phase_currents = np.zeros(3, dtype=np.float64)
        self.sample_time_s = 1e-4
        self.realism = InverterRealismConfig()

        # Stateful realism blocks keep their own bus and temperature states.
        self._cap_voltage = float(dc_voltage)
        self._junction_temperature_c = float(self.realism.ambient_temperature_c)
        self._last_telemetry: dict[str, float | int | bool] = {}

        # Preserve the legacy public attributes used by existing tests and any
        # downstream analysis code.
        self.sector_angles = np.array(
            [
                0.0,
                np.pi / 3.0,
                2.0 * np.pi / 3.0,
                np.pi,
                4.0 * np.pi / 3.0,
                5.0 * np.pi / 3.0,
                2.0 * np.pi,
            ],
            dtype=np.float64,
        )
        angles = np.array(
            [
                0.0,
                np.pi / 3.0,
                2.0 * np.pi / 3.0,
                np.pi,
                4.0 * np.pi / 3.0,
                5.0 * np.pi / 3.0,
            ],
            dtype=np.float64,
        )
        self.vectors = np.array(
            [[np.cos(angle), np.sin(angle)] for angle in angles], dtype=np.float64
        )

        self.set_nonidealities(
            device_drop_v=device_drop_v,
            dead_time_fraction=dead_time_fraction,
            conduction_resistance_ohm=conduction_resistance_ohm,
            switching_frequency_hz=switching_frequency_hz,
            switching_loss_coeff_v_per_a_khz=switching_loss_coeff_v_per_a_khz,
        )

    def set_sample_time(self, sample_time_s: float) -> None:
        """Set simulation step used by bus and thermal state updates."""
        if sample_time_s <= 0.0:
            raise ValueError("sample_time_s must be positive")
        self.sample_time_s = float(sample_time_s)

    def reset_realism_state(self) -> None:
        """Reset capacitor voltage, temperature, and cached telemetry."""
        self._cap_voltage = float(self.source_dc_voltage)
        self._junction_temperature_c = float(self.realism.ambient_temperature_c)
        self._last_telemetry = self._make_empty_telemetry(self._compute_available_bus_voltage())

    def modulate(self, magnitude: float, angle: float) -> np.ndarray:
        """Generate 3-phase SVM voltages and apply enabled realism blocks."""
        available_bus = self._compute_available_bus_voltage()
        max_magnitude = (2.0 / 3.0) * available_bus
        magnitude = float(np.clip(magnitude, 0.0, max_magnitude))
        angle = float(angle % (2.0 * np.pi))

        sector = int(np.floor(angle / (np.pi / 3.0))) + 1
        sector = int(np.clip(sector, 1, 6))

        angle_in_sector = angle - (sector - 1) * (np.pi / 3.0)
        if angle_in_sector < 0.0:
            angle_in_sector += np.pi / 3.0
        if angle_in_sector >= np.pi / 3.0:
            angle_in_sector -= np.pi / 3.0

        v_ref_norm = magnitude * np.sqrt(3.0) / max(available_bus, 1e-12)
        t1 = v_ref_norm * np.sin(np.pi / 3.0 - angle_in_sector) / np.sin(np.pi / 3.0)
        t2 = v_ref_norm * np.sin(angle_in_sector) / np.sin(np.pi / 3.0)
        t0 = 1.0 - t1 - t2

        t1 = float(np.clip(t1, 0.0, 1.0))
        t2 = float(np.clip(t2, 0.0, 1.0))
        t0 = float(np.clip(t0, 0.0, 1.0))

        ideal_voltages = self._generate_phase_voltages(
            sector=sector,
            t1=t1,
            t2=t2,
            t0=t0,
            dc_voltage=available_bus,
        )
        return self._apply_nonidealities(ideal_voltages)

    def _generate_phase_voltages(
        self,
        sector: int,
        t1: float,
        t2: float,
        t0: float,
        dc_voltage: float,
    ) -> np.ndarray:
        """Generate averaged phase voltages from SVM dwell times."""
        sector_patterns = {
            1: {"v1": [1, 0, 0], "v2": [1, 1, 0]},
            2: {"v1": [1, 1, 0], "v2": [0, 1, 0]},
            3: {"v1": [0, 1, 0], "v2": [0, 1, 1]},
            4: {"v1": [0, 1, 1], "v2": [0, 0, 1]},
            5: {"v1": [0, 0, 1], "v2": [1, 0, 1]},
            6: {"v1": [1, 0, 1], "v2": [1, 0, 0]},
        }
        pattern = sector_patterns.get(sector, sector_patterns[1])

        v1 = np.array(pattern["v1"], dtype=np.float64)
        v2 = np.array(pattern["v2"], dtype=np.float64)
        v_avg = t1 * v1 + t2 * v2 + (t0 / 2.0) * np.ones(3, dtype=np.float64)
        voltages = (v_avg * dc_voltage) - (dc_voltage / 2.0)
        return voltages.astype(np.float64)

    def get_maximum_voltage(self) -> float:
        """Return presently available linear-range SVM voltage magnitude."""
        return (2.0 / 3.0) * self._compute_available_bus_voltage()

    @property
    def device_drop_v(self) -> float:
        """Backward-compatible access to configured device drop."""
        return self.realism.device_drop_v

    @property
    def dead_time_fraction(self) -> float:
        """Backward-compatible access to configured dead-time fraction."""
        return self.realism.dead_time_fraction

    @property
    def conduction_resistance_ohm(self) -> float:
        """Backward-compatible access to conduction resistance."""
        return self.realism.conduction_resistance_ohm

    @property
    def switching_frequency_hz(self) -> float:
        """Backward-compatible access to switching frequency."""
        return self.realism.switching_frequency_hz

    @property
    def switching_loss_coeff_v_per_a_khz(self) -> float:
        """Backward-compatible access to switching-loss coefficient."""
        return self.realism.switching_loss_coeff_v_per_a_khz

    def set_nonidealities(  # noqa: C901  # TODO: split validation logic into sub-validators (14)
        self,
        device_drop_v: float = 0.0,
        dead_time_fraction: float = 0.0,
        conduction_resistance_ohm: float = 0.0,
        switching_frequency_hz: float = 0.0,
        switching_loss_coeff_v_per_a_khz: float = 0.0,
        enable_device_drop: bool | None = None,
        enable_dead_time: bool | None = None,
        enable_conduction_drop: bool | None = None,
        enable_switching_loss: bool | None = None,
        enable_diode_freewheel: bool = False,
        diode_drop_v: float = 0.0,
        diode_resistance_ohm: float = 0.0,
        enable_min_pulse: bool = False,
        min_pulse_fraction: float = 0.0,
        enable_bus_ripple: bool = False,
        dc_link_capacitance_f: float = 0.0,
        dc_link_source_resistance_ohm: float = 0.0,
        dc_link_esr_ohm: float = 0.0,
        enable_thermal_coupling: bool = False,
        thermal_resistance_k_per_w: float = 0.0,
        thermal_capacitance_j_per_k: float = 1.0,
        ambient_temperature_c: float = 25.0,
        temp_coeff_resistance_per_c: float = 0.0,
        temp_coeff_drop_per_c: float = 0.0,
        enable_phase_asymmetry: bool = False,
        phase_voltage_scale_a: float = 1.0,
        phase_voltage_scale_b: float = 1.0,
        phase_voltage_scale_c: float = 1.0,
        phase_drop_scale_a: float = 1.0,
        phase_drop_scale_b: float = 1.0,
        phase_drop_scale_c: float = 1.0,
    ) -> None:
        """Configure inverter-realism parameters and explicit feature toggles."""
        if device_drop_v < 0.0:
            raise ValueError("device_drop_v must be non-negative")
        if dead_time_fraction < 0.0 or dead_time_fraction > 0.2:
            raise ValueError("dead_time_fraction must be between 0 and 0.2")
        if conduction_resistance_ohm < 0.0:
            raise ValueError("conduction_resistance_ohm must be non-negative")
        if switching_frequency_hz < 0.0:
            raise ValueError("switching_frequency_hz must be non-negative")
        if switching_loss_coeff_v_per_a_khz < 0.0:
            raise ValueError("switching_loss_coeff_v_per_a_khz must be non-negative")
        if diode_drop_v < 0.0 or diode_resistance_ohm < 0.0:
            raise ValueError("diode drop and resistance must be non-negative")
        if min_pulse_fraction < 0.0 or min_pulse_fraction > 0.5:
            raise ValueError("min_pulse_fraction must be between 0 and 0.5")
        if dc_link_capacitance_f < 0.0:
            raise ValueError("dc_link_capacitance_f must be non-negative")
        if dc_link_source_resistance_ohm < 0.0 or dc_link_esr_ohm < 0.0:
            raise ValueError("dc-link resistances must be non-negative")
        if thermal_resistance_k_per_w < 0.0:
            raise ValueError("thermal_resistance_k_per_w must be non-negative")
        if thermal_capacitance_j_per_k <= 0.0:
            raise ValueError("thermal_capacitance_j_per_k must be positive")
        if temp_coeff_resistance_per_c < 0.0 or temp_coeff_drop_per_c < 0.0:
            raise ValueError("temperature coefficients must be non-negative")

        phase_voltage_scales = (
            phase_voltage_scale_a,
            phase_voltage_scale_b,
            phase_voltage_scale_c,
        )
        phase_drop_scales = (
            phase_drop_scale_a,
            phase_drop_scale_b,
            phase_drop_scale_c,
        )
        if any(scale <= 0.0 for scale in phase_voltage_scales + phase_drop_scales):
            raise ValueError("phase scale factors must be positive")

        self.realism.enable_device_drop = (
            bool(device_drop_v > 0.0) if enable_device_drop is None else bool(enable_device_drop)
        )
        self.realism.enable_dead_time = (
            bool(dead_time_fraction > 0.0) if enable_dead_time is None else bool(enable_dead_time)
        )
        self.realism.enable_conduction_drop = (
            bool(conduction_resistance_ohm > 0.0)
            if enable_conduction_drop is None
            else bool(enable_conduction_drop)
        )
        self.realism.enable_switching_loss = (
            bool(switching_frequency_hz > 0.0 and switching_loss_coeff_v_per_a_khz > 0.0)
            if enable_switching_loss is None
            else bool(enable_switching_loss)
        )

        self.realism.device_drop_v = float(device_drop_v)
        self.realism.dead_time_fraction = float(dead_time_fraction)
        self.realism.conduction_resistance_ohm = float(conduction_resistance_ohm)
        self.realism.switching_frequency_hz = float(switching_frequency_hz)
        self.realism.switching_loss_coeff_v_per_a_khz = float(switching_loss_coeff_v_per_a_khz)

        self.realism.enable_diode_freewheel = bool(enable_diode_freewheel)
        self.realism.diode_drop_v = float(diode_drop_v)
        self.realism.diode_resistance_ohm = float(diode_resistance_ohm)

        self.realism.enable_min_pulse = bool(enable_min_pulse)
        self.realism.min_pulse_fraction = float(min_pulse_fraction)

        self.realism.enable_bus_ripple = bool(enable_bus_ripple)
        self.realism.dc_link_capacitance_f = float(dc_link_capacitance_f)
        self.realism.dc_link_source_resistance_ohm = float(dc_link_source_resistance_ohm)
        self.realism.dc_link_esr_ohm = float(dc_link_esr_ohm)

        self.realism.enable_thermal_coupling = bool(enable_thermal_coupling)
        self.realism.thermal_resistance_k_per_w = float(thermal_resistance_k_per_w)
        self.realism.thermal_capacitance_j_per_k = float(thermal_capacitance_j_per_k)
        self.realism.ambient_temperature_c = float(ambient_temperature_c)
        self.realism.temp_coeff_resistance_per_c = float(temp_coeff_resistance_per_c)
        self.realism.temp_coeff_drop_per_c = float(temp_coeff_drop_per_c)

        self.realism.enable_phase_asymmetry = bool(enable_phase_asymmetry)
        self.realism.phase_voltage_scale_a = float(phase_voltage_scale_a)
        self.realism.phase_voltage_scale_b = float(phase_voltage_scale_b)
        self.realism.phase_voltage_scale_c = float(phase_voltage_scale_c)
        self.realism.phase_drop_scale_a = float(phase_drop_scale_a)
        self.realism.phase_drop_scale_b = float(phase_drop_scale_b)
        self.realism.phase_drop_scale_c = float(phase_drop_scale_c)

        self.reset_realism_state()

    def set_phase_currents(self, phase_currents: np.ndarray) -> None:
        """Provide phase currents used by current-dependent inverter effects."""
        arr = np.asarray(phase_currents, dtype=np.float64)
        if arr.shape != (3,):
            raise ValueError("phase_currents must be a 3-element array")
        self.phase_currents = arr

    def _apply_nonidealities(self, voltages: np.ndarray) -> np.ndarray:
        """Apply all enabled inverter-realism blocks and update telemetry."""
        out = np.array(voltages, dtype=np.float64, copy=True)
        current_abs = np.abs(self.phase_currents)
        current_sign = np.sign(self.phase_currents)

        available_bus = self._compute_available_bus_voltage()
        half_bus = max(available_bus / 2.0, 1e-12)
        voltage_scales = self._phase_voltage_scales()
        drop_scales = self._phase_drop_scales()

        device_loss = 0.0
        conduction_loss = 0.0
        switching_loss = 0.0
        dead_time_loss = 0.0
        diode_loss = 0.0
        min_pulse_events = 0

        # Phase mismatch modifies each leg independently before loss blocks.
        if self.realism.enable_phase_asymmetry:
            out *= voltage_scales

        # Minimum pulse suppression models pulse swallowing around zero output.
        if self.realism.enable_min_pulse and self.realism.min_pulse_fraction > 0.0:
            threshold = self.realism.min_pulse_fraction * half_bus
            suppressed = np.abs(out) < threshold
            min_pulse_events = int(np.count_nonzero(suppressed))
            out = np.where(suppressed, 0.0, out)

        # Dead-time distortion depends on the direction of phase current.
        if self.realism.enable_dead_time and self.realism.dead_time_fraction > 0.0:
            activity = np.clip(np.abs(out) / half_bus, 0.0, 1.0)
            dead_time_error = (
                current_sign * self.realism.dead_time_fraction * available_bus * activity
            )
            out = out - dead_time_error
            dead_time_loss = float(np.sum(np.abs(dead_time_error) * current_abs))

        temp_delta_c = max(
            self._junction_temperature_c - self.realism.ambient_temperature_c,
            0.0,
        )
        resistance_temp_scale = 1.0 + (
            self.realism.temp_coeff_resistance_per_c * temp_delta_c
            if self.realism.enable_thermal_coupling
            else 0.0
        )
        drop_temp_scale = 1.0 + (
            self.realism.temp_coeff_drop_per_c * temp_delta_c
            if self.realism.enable_thermal_coupling
            else 0.0
        )

        total_drop = np.zeros(3, dtype=np.float64)

        if self.realism.enable_device_drop and self.realism.device_drop_v > 0.0:
            device_drop = self.realism.device_drop_v * drop_temp_scale * drop_scales
            total_drop += device_drop
            device_loss = float(np.sum(device_drop * current_abs))

        if self.realism.enable_conduction_drop and self.realism.conduction_resistance_ohm > 0.0:
            conduction_res = (
                self.realism.conduction_resistance_ohm * resistance_temp_scale * drop_scales
            )
            conduction_drop = conduction_res * current_abs
            total_drop += conduction_drop
            conduction_loss = float(np.sum(conduction_res * current_abs**2))

        if (
            self.realism.enable_switching_loss
            and self.realism.switching_frequency_hz > 0.0
            and self.realism.switching_loss_coeff_v_per_a_khz > 0.0
        ):
            f_khz = self.realism.switching_frequency_hz / 1000.0
            switching_drop = self.realism.switching_loss_coeff_v_per_a_khz * current_abs * f_khz
            total_drop += switching_drop
            switching_loss = float(np.sum(switching_drop * current_abs))

        phase_sign = np.sign(out)
        freewheel_mask = np.logical_and(
            self.realism.enable_diode_freewheel,
            np.logical_and(current_abs > 1e-12, phase_sign * current_sign < 0.0),
        )
        if np.any(freewheel_mask):
            diode_drop = np.where(
                freewheel_mask,
                self.realism.diode_drop_v + self.realism.diode_resistance_ohm * current_abs,
                0.0,
            )
            total_drop += diode_drop
            diode_loss = float(np.sum(diode_drop * current_abs))

        out = np.where(
            np.abs(out) > total_drop,
            out - np.sign(out) * total_drop,
            0.0,
        )
        out = np.clip(out, -half_bus, half_bus)

        total_inverter_loss = float(
            device_loss + conduction_loss + switching_loss + dead_time_loss + diode_loss
        )
        motor_terminal_power = float(np.dot(out, self.phase_currents))
        source_power = motor_terminal_power + total_inverter_loss
        source_current = (
            source_power / max(self.source_dc_voltage, 1e-12)
            if abs(self.source_dc_voltage) > 1e-12
            else 0.0
        )

        self._update_bus_ripple_state(source_current)
        self._update_thermal_state(total_inverter_loss)

        effective_bus_next = self._compute_available_bus_voltage()
        self._last_telemetry = {
            "source_dc_voltage": float(self.source_dc_voltage),
            "effective_dc_voltage": float(available_bus),
            "dc_link_cap_voltage": float(self._cap_voltage),
            "dc_link_ripple_v": float(abs(self.source_dc_voltage - effective_bus_next)),
            "dc_link_bus_current_a": float(source_current),
            "motor_terminal_power_w": float(motor_terminal_power),
            "device_loss_power_w": float(device_loss),
            "conduction_loss_power_w": float(conduction_loss),
            "switching_loss_power_w": float(switching_loss),
            "dead_time_loss_power_w": float(dead_time_loss),
            "diode_loss_power_w": float(diode_loss),
            "total_inverter_loss_power_w": float(total_inverter_loss),
            "junction_temperature_c": float(self._junction_temperature_c),
            "common_mode_voltage": float(np.mean(out)),
            "min_pulse_event_count": int(min_pulse_events),
            "phase_voltage_scale_a": float(voltage_scales[0]),
            "phase_voltage_scale_b": float(voltage_scales[1]),
            "phase_voltage_scale_c": float(voltage_scales[2]),
            "phase_drop_scale_a": float(drop_scales[0]),
            "phase_drop_scale_b": float(drop_scales[1]),
            "phase_drop_scale_c": float(drop_scales[2]),
            "enable_device_drop": bool(self.realism.enable_device_drop),
            "enable_dead_time": bool(self.realism.enable_dead_time),
            "enable_conduction_drop": bool(self.realism.enable_conduction_drop),
            "enable_switching_loss": bool(self.realism.enable_switching_loss),
            "enable_diode_freewheel": bool(self.realism.enable_diode_freewheel),
            "enable_min_pulse": bool(self.realism.enable_min_pulse),
            "enable_bus_ripple": bool(self.realism.enable_bus_ripple),
            "enable_thermal_coupling": bool(self.realism.enable_thermal_coupling),
            "enable_phase_asymmetry": bool(self.realism.enable_phase_asymmetry),
        }
        return out

    def _compute_available_bus_voltage(self) -> float:
        """Return the bus voltage currently available to the bridge."""
        if not self.realism.enable_bus_ripple or self.realism.dc_link_capacitance_f <= 0.0:
            return max(self.source_dc_voltage, 1e-9)

        estimated_bus_current = float(self._last_telemetry.get("dc_link_bus_current_a", 0.0))
        effective_bus = self._cap_voltage - (estimated_bus_current * self.realism.dc_link_esr_ohm)
        return max(effective_bus, 1e-9)

    def _update_bus_ripple_state(self, source_current: float) -> None:
        """Update reduced-order DC-link capacitor state for the next step."""
        if not self.realism.enable_bus_ripple or self.realism.dc_link_capacitance_f <= 0.0:
            self._cap_voltage = float(self.source_dc_voltage)
            return

        c_dc = self.realism.dc_link_capacitance_f
        source_r = self.realism.dc_link_source_resistance_ohm
        dt = self.sample_time_s

        if source_r <= 1e-12:
            self._cap_voltage = float(self.source_dc_voltage)
            return

        recharge_current = (self.source_dc_voltage - self._cap_voltage) / source_r
        capacitor_current = recharge_current - source_current
        self._cap_voltage = max(
            self._cap_voltage + (capacitor_current / c_dc) * dt,
            0.0,
        )

    def _update_thermal_state(self, total_loss_power_w: float) -> None:
        """Update reduced-order junction temperature state for the next step."""
        if not self.realism.enable_thermal_coupling:
            self._junction_temperature_c = float(self.realism.ambient_temperature_c)
            return

        r_th = self.realism.thermal_resistance_k_per_w
        c_th = self.realism.thermal_capacitance_j_per_k
        if r_th <= 1e-12 or c_th <= 1e-12:
            self._junction_temperature_c = float(self.realism.ambient_temperature_c)
            return

        cooling_power = (self._junction_temperature_c - self.realism.ambient_temperature_c) / r_th
        d_temp = (total_loss_power_w - cooling_power) * self.sample_time_s / c_th
        self._junction_temperature_c += d_temp

    def _phase_voltage_scales(self) -> np.ndarray:
        """Return per-phase voltage mismatch multipliers."""
        if not self.realism.enable_phase_asymmetry:
            return np.ones(3, dtype=np.float64)
        return np.array(
            [
                self.realism.phase_voltage_scale_a,
                self.realism.phase_voltage_scale_b,
                self.realism.phase_voltage_scale_c,
            ],
            dtype=np.float64,
        )

    def _phase_drop_scales(self) -> np.ndarray:
        """Return per-phase loss mismatch multipliers."""
        if not self.realism.enable_phase_asymmetry:
            return np.ones(3, dtype=np.float64)
        return np.array(
            [
                self.realism.phase_drop_scale_a,
                self.realism.phase_drop_scale_b,
                self.realism.phase_drop_scale_c,
            ],
            dtype=np.float64,
        )

    def get_last_telemetry(self) -> dict[str, float | int | bool]:
        """Return telemetry from the last modulation step."""
        if not self._last_telemetry:
            self._last_telemetry = self._make_empty_telemetry(self._compute_available_bus_voltage())
        return dict(self._last_telemetry)

    def get_realism_state(self) -> dict[str, Any]:
        """Return the complete inverter configuration and runtime state."""
        return {
            **self.get_last_telemetry(),
            **asdict(self.realism),
            "sample_time_s": float(self.sample_time_s),
            "source_dc_voltage": float(self.source_dc_voltage),
            "cap_voltage": float(self._cap_voltage),
            "junction_temperature_c": float(self._junction_temperature_c),
        }

    def _make_empty_telemetry(
        self,
        effective_bus: float,
    ) -> dict[str, float | int | bool]:
        """Build a zeroed telemetry packet used before the first modulation step."""
        return {
            "source_dc_voltage": float(self.source_dc_voltage),
            "effective_dc_voltage": float(effective_bus),
            "dc_link_cap_voltage": float(self._cap_voltage),
            "dc_link_ripple_v": 0.0,
            "dc_link_bus_current_a": 0.0,
            "motor_terminal_power_w": 0.0,
            "device_loss_power_w": 0.0,
            "conduction_loss_power_w": 0.0,
            "switching_loss_power_w": 0.0,
            "dead_time_loss_power_w": 0.0,
            "diode_loss_power_w": 0.0,
            "total_inverter_loss_power_w": 0.0,
            "junction_temperature_c": float(self._junction_temperature_c),
            "common_mode_voltage": 0.0,
            "min_pulse_event_count": 0,
            "phase_voltage_scale_a": 1.0,
            "phase_voltage_scale_b": 1.0,
            "phase_voltage_scale_c": 1.0,
            "phase_drop_scale_a": 1.0,
            "phase_drop_scale_b": 1.0,
            "phase_drop_scale_c": 1.0,
            "enable_device_drop": bool(self.realism.enable_device_drop),
            "enable_dead_time": bool(self.realism.enable_dead_time),
            "enable_conduction_drop": bool(self.realism.enable_conduction_drop),
            "enable_switching_loss": bool(self.realism.enable_switching_loss),
            "enable_diode_freewheel": bool(self.realism.enable_diode_freewheel),
            "enable_min_pulse": bool(self.realism.enable_min_pulse),
            "enable_bus_ripple": bool(self.realism.enable_bus_ripple),
            "enable_thermal_coupling": bool(self.realism.enable_thermal_coupling),
            "enable_phase_asymmetry": bool(self.realism.enable_phase_asymmetry),
        }

    def set_dc_voltage(self, dc_voltage: float) -> None:
        """Update the source DC bus value seen by the inverter model."""
        if dc_voltage <= 0.0:
            raise ValueError("DC voltage must be positive")
        self.dc_voltage = float(dc_voltage)
        self.source_dc_voltage = float(dc_voltage)
        if not self.realism.enable_bus_ripple:
            self._cap_voltage = float(dc_voltage)


class CartesianSVMGenerator(SVMGenerator):
    """Extended SVM generator accepting Cartesian alpha-beta voltages."""

    def modulate_cartesian(self, valpha: float, vbeta: float) -> np.ndarray:
        """Generate SVM voltages from Cartesian alpha-beta coordinates."""
        magnitude = float(np.hypot(valpha, vbeta))
        angle = float(np.arctan2(vbeta, valpha))
        return self.modulate(magnitude, angle)

    def cartesian_to_threephase(self, valpha: float, vbeta: float) -> np.ndarray:
        """Convert Cartesian alpha-beta voltages to 3-phase quantities."""
        sqrt3_2 = np.sqrt(3.0) / 2.0
        v_a = valpha
        v_b = -0.5 * valpha + sqrt3_2 * vbeta
        v_c = -0.5 * valpha - sqrt3_2 * vbeta
        return np.array([v_a, v_b, v_c], dtype=np.float64)
