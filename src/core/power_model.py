"""
Power Supply Model
==================

This module provides models of the DC power supply feeding the inverter.
A time-dependent voltage profile can be defined to simulate battery sag,
AC ripple rectification, or other variations.

:author: BLDC Control Team
:version: 1.0.0

.. versionadded:: 1.0.0
    Initial implementation of power supply models
"""

import numpy as np
from abc import ABC, abstractmethod
from typing import Optional, Callable, List


class SupplyProfile(ABC):
    """
    Abstract base class for supply voltage profiles.
    """

    @abstractmethod
    def get_voltage(self, time: float) -> float:
        """Return supply voltage at time t."""
        pass

    def reset(self) -> None:
        """Reset profile state if applicable."""
        pass


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
        elif time >= self.duration:
            return self.final
        else:
            slope = (self.final - self.initial) / self.duration
            return self.initial + slope * time


class VariableSupply(SupplyProfile):
    """Custom time-dependent supply using function or data points."""

    def __init__(
        self,
        voltage_func: Optional[Callable[[float], float]] = None,
        time_points: Optional[List[float]] = None,
        voltage_points: Optional[List[float]] = None,
    ):
        if voltage_func is None and time_points is None:
            raise ValueError("Provide either voltage_func or time_points")
        self.voltage_func = voltage_func
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
        if time <= self.time_array[0]:
            return self.voltage_array[0]
        elif time >= self.time_array[-1]:
            return self.voltage_array[-1]
        else:
            idx = np.searchsorted(self.time_array, time)
            t0, t1 = self.time_array[idx - 1], self.time_array[idx]
            v0, v1 = self.voltage_array[idx - 1], self.voltage_array[idx]
            alpha = (time - t0) / (t1 - t0)
            return v0 + alpha * (v1 - v0)
