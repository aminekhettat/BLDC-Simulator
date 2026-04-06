"""
Power Supply Model
==================

This module provides models of the DC power supply feeding the inverter.
A time-dependent voltage profile can be defined to simulate battery sag,
AC ripple rectification, or other variations.

:author: BLDC Control Team
:version: 0.8.0

.. versionadded:: 0.8.0
    Initial implementation of power supply models
"""

from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import TypedDict

import numpy as np


class EfficiencyAdjustmentRecommendation(TypedDict):
    """Structured recommendation payload for efficiency heuristics."""

    efficiency: float
    power_factor: float
    target_efficiency: float
    needs_attention: bool
    suggestions: list[str]


def compute_power_metrics(
    voltage: np.ndarray,
    current: np.ndarray,
    backend: str = "cpu",
) -> dict[str, float]:
    """Compute basic power-quality metrics from voltage/current waveforms.

    Parameters
    ----------
    voltage : np.ndarray
        Instantaneous voltage samples.
    current : np.ndarray
        Instantaneous current samples, aligned with ``voltage``.

    Returns
    -------
    dict
        Active/apparent/reactive power, RMS values, and power factor.
    """
    use_gpu = str(backend).lower() == "gpu"

    if use_gpu:
        try:
            import cupy as cp  # type: ignore

            v = cp.asarray(voltage, dtype=cp.float64)
            i = cp.asarray(current, dtype=cp.float64)

            if v.shape != i.shape:
                raise ValueError("voltage and current arrays must have the same shape")
            if v.size == 0:
                raise ValueError("voltage and current arrays must be non-empty")

            v_rms = float(cp.sqrt(cp.mean(v * v)).item())
            i_rms = float(cp.sqrt(cp.mean(i * i)).item())
            active_power = float(cp.mean(v * i).item())
            apparent_power = float(v_rms * i_rms)
        except Exception:
            # Safe fallback to CPU path if GPU backend fails at runtime.
            use_gpu = False

    if not use_gpu:
        v = np.asarray(voltage, dtype=np.float64)
        i = np.asarray(current, dtype=np.float64)

        if v.shape != i.shape:
            raise ValueError("voltage and current arrays must have the same shape")
        if v.size == 0:
            raise ValueError("voltage and current arrays must be non-empty")

        v_rms = float(np.sqrt(np.mean(v * v)))
        i_rms = float(np.sqrt(np.mean(i * i)))
        active_power = float(np.mean(v * i))
        apparent_power = float(v_rms * i_rms)

    if apparent_power <= 1e-12:
        power_factor = 0.0
        reactive_power = 0.0
    else:
        raw_pf = active_power / apparent_power
        power_factor = float(np.clip(raw_pf, -1.0, 1.0))
        reactive_power = float(np.sqrt(max(apparent_power**2 - active_power**2, 0.0)))

    return {
        "voltage_rms_v": v_rms,
        "current_rms_a": i_rms,
        "active_power_w": active_power,
        "apparent_power_va": apparent_power,
        "reactive_power_var": reactive_power,
        "power_factor": power_factor,
    }


def required_reactive_compensation(
    active_power_w: float,
    current_pf: float,
    target_pf: float,
) -> dict[str, float]:
    """Estimate reactive compensation needed to improve power factor.

    Notes
    -----
    This helper assumes positive active power and lagging-load correction sizing.
    """
    p = float(active_power_w)
    pf_now = abs(float(current_pf))
    pf_target = abs(float(target_pf))

    if p <= 0.0:
        raise ValueError("active_power_w must be positive")
    if not (0.0 < pf_now <= 1.0):
        raise ValueError("current_pf must be in (0, 1]")
    if not (0.0 < pf_target <= 1.0):
        raise ValueError("target_pf must be in (0, 1]")
    if pf_target < pf_now:
        raise ValueError("target_pf must be greater than or equal to current_pf")

    q_now = p * np.tan(np.arccos(pf_now))
    q_target = p * np.tan(np.arccos(pf_target))
    compensation = max(q_now - q_target, 0.0)

    return {
        "active_power_w": p,
        "current_pf": pf_now,
        "target_pf": pf_target,
        "current_reactive_var": float(q_now),
        "target_reactive_var": float(q_target),
        "required_compensation_var": float(compensation),
    }


def compute_efficiency_metrics(
    input_power_w: float,
    torque_nm: float,
    omega_rad_s: float,
) -> dict[str, float]:
    """Compute simple drivetrain efficiency metrics from power flow values."""
    electrical_input = max(float(input_power_w), 0.0)
    shaft_power = float(torque_nm) * float(omega_rad_s)
    mechanical_output = max(shaft_power, 0.0)
    regenerative_power = max(-shaft_power, 0.0)
    total_loss = max(electrical_input - mechanical_output, 0.0)

    if electrical_input <= 1e-12:
        efficiency = 0.0
    else:
        efficiency = float(np.clip(mechanical_output / electrical_input, 0.0, 1.0))

    return {
        "electrical_input_power_w": electrical_input,
        "mechanical_output_power_w": mechanical_output,
        "regenerative_power_w": regenerative_power,
        "total_loss_power_w": total_loss,
        "efficiency": efficiency,
    }


def recommend_efficiency_adjustments(
    efficiency: float,
    power_factor: float,
    device_drop_v: float = 0.0,
    dead_time_fraction: float = 0.0,
    conduction_resistance_ohm: float = 0.0,
    switching_frequency_hz: float = 0.0,
    switching_loss_coeff_v_per_a_khz: float = 0.0,
    target_efficiency: float = 0.90,
) -> EfficiencyAdjustmentRecommendation:
    """Return simple heuristic suggestions for improving simulated efficiency."""
    eff = float(np.clip(efficiency, 0.0, 1.0))
    pf = float(np.clip(abs(power_factor), 0.0, 1.0))
    target = float(np.clip(target_efficiency, 0.0, 1.0))
    suggestions: list[str] = []

    if eff < target:
        if device_drop_v > 0.5:
            suggestions.append("Reduce inverter device drop or choose lower-Vf switching devices.")
        if dead_time_fraction > 0.01:
            suggestions.append("Reduce dead-time fraction if switching margins allow it.")
        if conduction_resistance_ohm > 0.01:
            suggestions.append(
                "Lower conduction resistance to reduce current-dependent voltage drop."
            )
        if switching_frequency_hz > 12000.0 and switching_loss_coeff_v_per_a_khz > 0.0:
            suggestions.append(
                "Lower PWM switching frequency or switching-loss coefficient to trade ripple for lower inverter losses."  # noqa: E501
            )

    if pf < 0.90:
        suggestions.append(
            "Improve power factor with PFC tuning or a less reactive operating point."
        )

    if not suggestions:
        suggestions.append(
            "Current inverter settings are already near the requested efficiency target."
        )

    return {
        "efficiency": eff,
        "power_factor": pf,
        "target_efficiency": target,
        "needs_attention": eff < target or pf < 0.90,
        "suggestions": suggestions,
    }


class PowerFactorController:
    """Simple closed-loop power factor correction command generator.

    The controller estimates a reactive compensation command from current PF,
    active power, and a target PF. It is designed as a non-invasive telemetry
    hook for simulation workflows.
    """

    def __init__(
        self,
        target_pf: float = 0.95,
        kp: float = 0.10,
        ki: float = 1.0,
        max_compensation_var: float = 10000.0,
    ):
        if not (0.0 < target_pf <= 1.0):
            raise ValueError("target_pf must be in (0, 1]")
        if kp < 0.0 or ki < 0.0:
            raise ValueError("kp and ki must be non-negative")
        if max_compensation_var <= 0.0:
            raise ValueError("max_compensation_var must be positive")

        self.target_pf = float(target_pf)
        self.kp = float(kp)
        self.ki = float(ki)
        self.max_compensation_var = float(max_compensation_var)
        self.integral = 0.0
        self.last_command_var = 0.0
        self.last_error = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.last_command_var = 0.0
        self.last_error = 0.0

    def update(self, current_pf: float, active_power_w: float, dt: float) -> float:
        """Update controller and return recommended compensation [VAR]."""
        pf_now = abs(float(current_pf))
        pf_now = float(np.clip(pf_now, 0.0, 1.0))
        p = max(float(active_power_w), 0.0)

        if dt <= 0.0 or p <= 1e-9:
            self.last_error = self.target_pf - pf_now
            self.last_command_var = 0.0
            return 0.0

        error = self.target_pf - pf_now
        self.integral = float(np.clip(self.integral + error * dt, -1.0, 1.0))

        if pf_now <= 0.0:
            baseline = self.max_compensation_var
        elif pf_now < self.target_pf:
            baseline = required_reactive_compensation(
                active_power_w=p,
                current_pf=max(pf_now, 1e-6),
                target_pf=self.target_pf,
            )["required_compensation_var"]
        else:
            baseline = 0.0

        trim = p * (self.kp * error + self.ki * self.integral)
        command = float(np.clip(baseline + trim, 0.0, self.max_compensation_var))

        self.last_error = error
        self.last_command_var = command
        return command


class SupplyProfile(ABC):
    """
    Abstract base class for supply voltage profiles.
    """

    @abstractmethod
    def get_voltage(self, time: float) -> float:
        """Return supply voltage at time t."""

    def reset(self) -> None:
        """Reset profile state if applicable."""


class ConstantSupply(SupplyProfile):
    """Constant DC voltage supply."""

    def __init__(self, voltage: float = 48.0):
        self.voltage = voltage

    def get_voltage(self, time: float) -> float:
        return self.voltage


class RampSupply(SupplyProfile):
    """Linear ramp of supply voltage.

    Allows simulating startup charge or sag.
    """

    def __init__(self, initial: float, final: float, duration: float):
        if duration <= 0:
            raise ValueError("Duration must be positive")
        self.initial = initial
        self.final = final
        self.duration = duration

    def get_voltage(self, time: float) -> float:
        if time <= 0:
            return self.initial
        if time >= self.duration:
            return self.final
        slope = (self.final - self.initial) / self.duration
        return self.initial + slope * time


class VariableSupply(SupplyProfile):
    """Custom time-dependent supply using function or data points."""

    def __init__(
        self,
        voltage_func: Callable[[float], float] | None = None,
        time_points: list[float] | None = None,
        voltage_points: list[float] | None = None,
    ):
        if voltage_func is None and time_points is None:
            raise ValueError("Provide either voltage_func or time_points")
        self.voltage_func = voltage_func
        self.time_array: np.ndarray | None
        self.voltage_array: np.ndarray | None
        if time_points is not None:
            if voltage_points is None:
                raise ValueError("voltage_points required with time_points")
            self.time_array = np.array(time_points, dtype=np.float64)
            self.voltage_array = np.array(voltage_points, dtype=np.float64)
            if len(self.time_array) != len(self.voltage_array):
                raise ValueError("time and voltage lists must have same length")
            if len(self.time_array) > 1 and not np.all(np.diff(self.time_array) > 0):
                raise ValueError("time_points must be sorted ascending")
        else:
            self.time_array = None
            self.voltage_array = None

    def get_voltage(self, time: float) -> float:
        if self.voltage_func is not None:
            return float(self.voltage_func(time))
        if self.time_array is None or self.voltage_array is None:
            raise RuntimeError("VariableSupply is not initialized with time/voltage arrays")
        if time <= self.time_array[0]:
            return float(self.voltage_array[0])
        if time >= self.time_array[-1]:
            return float(self.voltage_array[-1])
        idx = np.searchsorted(self.time_array, time)
        t0, t1 = self.time_array[idx - 1], self.time_array[idx]
        v0, v1 = self.voltage_array[idx - 1], self.voltage_array[idx]
        alpha = (time - t0) / (t1 - t0)
        return float(v0 + alpha * (v1 - v0))
