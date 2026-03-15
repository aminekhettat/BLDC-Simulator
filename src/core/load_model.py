"""
Load Model Module
=================

This module provides various load models for the motor simulation.
The load models generate torque profiles that can be constant, ramping,
or variable over time.

Supported load types:
- Constant load (fixed torque)
- Ramp load (linear change in torque over time)
- Variable load (custom time-dependent profile)

:author: Amine Khettat
:version: 1.0.0

.. versionadded:: 1.0.0
    Initial implementation of load models
"""

import numpy as np
from abc import ABC, abstractmethod
from typing import Optional, Callable, List


class LoadProfile(ABC):
    """
    Abstract base class for load profiles.

    All load models inherit from this class and implement the
    :meth:`get_torque` method to provide time-dependent load torque.
    """

    @abstractmethod
    def get_torque(self, time: float) -> float:
        """
        Get load torque at specified time.

        :param time: Simulation time [seconds]
        :type time: float
        :return: Load torque [N*m]
        :rtype: float
        """
        pass

    def reset(self) -> None:
        """
        Reset load profile to initial state.

        Not all profiles need resetting, but this method provides
        a consistent interface.
        """
        pass


class ConstantLoad(LoadProfile):
    """
    Constant load model.

    Applies a fixed torque throughout simulation.

    Example:
        >>> load = ConstantLoad(torque=2.5)
        >>> load.get_torque(0.0)
        2.5
        >>> load.get_torque(10.0)
        2.5
    """

    def __init__(self, torque: float = 0.0):
        """
        Initialize constant load.

        :param torque: Constant load torque [N*m], defaults to 0.0
        :type torque: float
        """
        self.torque = torque

    def get_torque(self, time: float) -> float:
        """
        Get constant load torque.

        :param time: Simulation time [seconds] (not used)
        :type time: float
        :return: Load torque [N*m]
        :rtype: float
        """
        return self.torque


class RampLoad(LoadProfile):
    """
    Ramping load model.

    The load torque ramps linearly from initial value to final value
    over a specified duration.

    **Load Profile:**

    .. math::
        \\tau(t) = \\tau_0 + \\frac{(\\tau_f - \\tau_0)}{t_ramp} * t \\quad (0 \\leq t \\leq t_{ramp})

    .. math::
        \\tau(t) = \\tau_f \\quad (t > t_{ramp})

    Example:
        >>> load = RampLoad(initial=0.0, final=5.0, duration=2.0)
        >>> load.get_torque(1.0)  # At t=1s
        2.5
        >>> load.get_torque(3.0)  # After ramp
        5.0
    """

    def __init__(self, initial: float = 0.0, final: float = 1.0, duration: float = 1.0):
        """
        Initialize ramp load.

        :param initial: Initial load torque [N*m], defaults to 0.0
        :type initial: float
        :param final: Final load torque [N*m], defaults to 1.0
        :type final: float
        :param duration: Ramp duration [seconds], defaults to 1.0
        :type duration: float

        :raises ValueError: If duration <= 0
        """
        if duration <= 0:
            raise ValueError("Ramp duration must be positive")

        self.initial = initial
        self.final = final
        self.duration = duration

    def get_torque(self, time: float) -> float:
        """
        Get ramping load torque.

        :param time: Simulation time [seconds]
        :type time: float
        :return: Load torque [N*m]
        :rtype: float
        """
        if time <= 0:
            return self.initial
        elif time >= self.duration:
            return self.final
        else:
            # Linear interpolation
            slope = (self.final - self.initial) / self.duration
            return self.initial + slope * time


class VariableLoad(LoadProfile):
    """
    Variable load model with custom time-dependent profile.

    Allows definition of arbitrary load torque profiles using:
    - Time-torque paired data points (interpolated)
    - Custom callback function

    Example (with function):
        >>> def load_func(t):
        ...     return 2.0 * np.sin(2*np.pi*t)
        >>> load = VariableLoad(torque_func=load_func)
        >>> load.get_torque(0.25)
        2.0

    Example (with time points):
        >>> times = [0.0, 1.0, 2.0]
        >>> torques = [0.0, 5.0, 2.0]
        >>> load = VariableLoad(time_points=times, torque_points=torques)
    """

    def __init__(
        self,
        torque_func: Optional[Callable[[float], float]] = None,
        time_points: Optional[List[float]] = None,
        torque_points: Optional[List[float]] = None,
    ):
        """
        Initialize variable load with either function or data points.

        :param torque_func: Custom torque function: f(time) -> torque, optional
        :type torque_func: callable, optional
        :param time_points: List of time points [seconds], optional
        :type time_points: list, optional
        :param torque_points: List of torque values [N*m], optional
        :type torque_points: list, optional

        :raises ValueError: If both torque_func and time_points are None
        :raises ValueError: If time_points and torque_points lengths don't match
        :raises ValueError: If time_points not sorted in ascending order
        """
        if torque_func is None and time_points is None:
            raise ValueError("Must provide either torque_func or time_points")

        self.torque_func = torque_func

        if time_points is not None:
            if torque_points is None:
                raise ValueError("Must provide torque_points with time_points")

            time_array = np.array(time_points, dtype=np.float64)
            torque_array = np.array(torque_points, dtype=np.float64)

            if len(time_array) != len(torque_array):
                raise ValueError("time_points and torque_points must have same length")

            if len(time_array) > 1:
                if not np.all(np.diff(time_array) > 0):
                    raise ValueError("time_points must be in ascending order")

            self.time_points = time_array
            self.torque_points = torque_array
        else:
            self.time_points = None
            self.torque_points = None

    def get_torque(self, time: float) -> float:
        """
        Get variable load torque.

        Uses linear interpolation if data points are defined,
        otherwise calls the torque function.

        :param time: Simulation time [seconds]
        :type time: float
        :return: Load torque [N*m]
        :rtype: float

        .. note::
            For time values beyond the defined range with data points,
            the value is clamped to the last defined point.
        """
        if self.torque_func is not None:
            return float(self.torque_func(time))

        # Use data points with linear interpolation
        if time <= self.time_points[0]:
            return self.torque_points[0]
        elif time >= self.time_points[-1]:
            return self.torque_points[-1]
        else:
            # Find interval and interpolate
            idx = np.searchsorted(self.time_points, time)
            t0, t1 = self.time_points[idx - 1], self.time_points[idx]
            tau0, tau1 = self.torque_points[idx - 1], self.torque_points[idx]

            # Linear interpolation
            alpha = (time - t0) / (t1 - t0)
            return tau0 + alpha * (tau1 - tau0)


class CyclicLoad(LoadProfile):
    """
    Cyclic load model with sinusoidal profile.

    Generates periodic load torque: τ(t) = τ_offset + τ_amplitude * sin(2π*f*t + φ)

    **Load Profile:**

    .. math::
        \\tau(t) = \\tau_{offset} + \\tau_{amplitude} \\cdot \\sin(2\\pi f t + \\varphi)

    Example:
        >>> load = CyclicLoad(offset=1.0, amplitude=2.0, frequency=1.0)
        >>> load.get_torque(0.0)
        1.0
        >>> load.get_torque(0.25)  # At peak
        3.0
    """

    def __init__(
        self,
        offset: float = 0.0,
        amplitude: float = 1.0,
        frequency: float = 1.0,
        phase: float = 0.0,
    ):
        """
        Initialize cyclic load.

        :param offset: Offset torque (mean value) [N*m], defaults to 0.0
        :type offset: float
        :param amplitude: Amplitude of oscillation [N*m], defaults to 1.0
        :type amplitude: float
        :param frequency: Oscillation frequency [Hz], defaults to 1.0
        :type frequency: float
        :param phase: Initial phase [radians], defaults to 0.0
        :type phase: float

        :raises ValueError: If frequency < 0
        """
        if frequency < 0:
            raise ValueError("Frequency must be non-negative")

        self.offset = offset
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase = phase

    def get_torque(self, time: float) -> float:
        """
        Get cyclic load torque.

        :param time: Simulation time [seconds]
        :type time: float
        :return: Load torque [N*m]
        :rtype: float
        """
        return self.offset + self.amplitude * np.sin(
            2 * np.pi * self.frequency * time + self.phase
        )
