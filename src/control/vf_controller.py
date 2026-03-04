"""
V/f Controller Module
=====================

This module implements a Voltage-to-Frequency (V/f) control block for BLDC motor.

V/f control provides a simple, effective way to control motor speed without
complex feedback systems. It generates a voltage ramp profile proportional to
desired frequency/speed.

Commonly used for:
- Motor soft-start
- Speed ramping
- Simple open-loop speed control
- Sequential voltage buildup

:author: BLDC Control Team
:version: 1.0.0

.. versionadded:: 1.0.0
    Initial V/f controller implementation
"""

import numpy as np
from .base_controller import BaseController


class VFController(BaseController):
    """
    Voltage-to-Frequency (V/f) Controller
    ======================================

    Implements V/f characteristic for open-loop motor control.

    **V/f Relationship:**

    .. math::
        V(f) = V_0 + K_{vf} \\cdot f

    Where:
        - V₀: Starting voltage (to overcome friction)
        - K_vf: V/f slope constant
        - f: Desired frequency [Hz]

    **Features:**
    - Linear V/f characteristic
    - Voltage ramp limiting (soft-start)
    - Frequency slew-rate limiting
    - Voltage saturation
    - Real-time modulation output

    Example:
        >>> controller = VFController(
        ...     v_nominal=48.0,
        ...     f_nominal=100.0,
        ...     dc_voltage=48.0
        ... )
        >>> controller.set_speed_reference(50.0)  # Hz
        >>>
        >>> for i in range(100):
        ...     mag, angle = controller.update(dt=0.001)
        ...     # Apply to SVM
    """

    def __init__(
        self,
        v_nominal: float,
        f_nominal: float,
        dc_voltage: float = 48.0,
        v_startup: float = 1.0,
        ramp_rate: float = 10.0,
    ):
        """
        Initialize V/f controller.

        :param v_nominal: Nominal motor voltage [V]
        :type v_nominal: float
        :param f_nominal: Nominal motor frequency [Hz]
        :type f_nominal: float
        :param dc_voltage: DC bus voltage [V], defaults to 48.0
        :type dc_voltage: float
        :param v_startup: Starting voltage to overcome friction [V], defaults to 1.0
        :type v_startup: float
        :param ramp_rate: Voltage ramp rate limit [V/s], defaults to 10.0
        :type ramp_rate: float

        :raises ValueError: If f_nominal <= 0
        :raises ValueError: If v_nominal <= 0
        """
        super().__init__()

        if f_nominal <= 0:
            raise ValueError("Nominal frequency must be positive")
        if v_nominal <= 0:
            raise ValueError("Nominal voltage must be positive")

        self.v_nominal = v_nominal
        self.f_nominal = f_nominal
        self.dc_voltage = dc_voltage
        self.v_startup = v_startup
        self.ramp_rate = ramp_rate

        # V/f slope
        self.kv_f = (v_nominal - v_startup) / f_nominal if f_nominal != 0 else 0

        # State
        self.frequency_ref = 0.0  # Reference frequency [Hz]
        self.frequency_actual = 0.0  # Actual frequency [Hz]
        self.voltage_actual = 0.0  # Actual voltage magnitude [V]
        self.angle = 0.0  # Voltage angle [rad]
        self.angle_velocity = 0.0  # dθ/dt = 2π*f

        # Limits
        self.max_voltage = (2.0 / 3.0) * dc_voltage  # For linear SVM
        self.frequency_slew_rate = 50.0  # [Hz/s]

        # Startup boost (optional)
        self.use_startup_boost = True
        self.startup_boost_time = 0.5  # Duration to apply boost [s]
        self.startup_timer = 0.0

    def set_speed_reference(self, frequency: float) -> None:
        """
        Set reference frequency (speed).

        :param frequency: Desired frequency [Hz], can be negative for reverse
        :type frequency: float
        """
        self.frequency_ref = float(frequency)

    def get_speed_reference(self) -> float:
        """
        Get current speed reference.

        :return: Reference frequency [Hz]
        :rtype: float
        """
        return self.frequency_ref

    def update(self, dt: float) -> tuple:
        """
        Update controller state and generate voltage command.

        **Algorithm:**
        1. Apply frequency slew-rate limiting
        2. Calculate V/f voltage from frequency
        3. Apply startup boost if needed
        4. Saturate voltage to maximum
        5. Update phase angle
        6. Return voltage magnitude and angle

        :param dt: Time step [seconds]
        :type dt: float
        :return: Tuple of (magnitude [V], angle [rad])
        :rtype: tuple

        .. math::
            f_{actual}(t+dt) = f_{actual} + \\text{clamp}(f_{ref} - f_{actual},
                                                             -\\Delta f, \\Delta f)

        .. math::
            V(t) = V_0 + K_{vf} \\cdot |f_{actual}|

        .. math::
            \\theta(t+dt) = \\theta(t) + 2\\pi \\cdot f_{actual} \\cdot dt
        """
        # Apply frequency slew-rate limiting
        delta_f_max = self.frequency_slew_rate * dt
        delta_f = np.clip(
            self.frequency_ref - self.frequency_actual, -delta_f_max, delta_f_max
        )
        self.frequency_actual += delta_f

        # Calculate V/f voltage
        if self.frequency_actual == 0:
            voltage = 0.0
        else:
            # V = V0 + kv_f * |f|
            voltage = self.v_startup + self.kv_f * np.abs(self.frequency_actual)

            # Apply startup boost (optional)
            if self.use_startup_boost and self.startup_timer < self.startup_boost_time:
                # Add extra voltage during startup
                boost = (self.v_startup * 0.2) * (
                    1.0 - self.startup_timer / self.startup_boost_time
                )
                voltage += boost
                self.startup_timer += dt
            else:
                self.startup_timer = 0.0

        # Clamp voltage to maximum
        voltage = np.clip(voltage, 0, self.max_voltage)
        self.voltage_actual = voltage

        # Update phase angle
        self.angle_velocity = 2.0 * np.pi * self.frequency_actual
        self.angle += self.angle_velocity * dt
        self.angle = self.angle % (2.0 * np.pi)

        return voltage, self.angle

    def set_vf_characteristic(
        self, v_startup: float, v_nominal: float, f_nominal: float
    ) -> None:
        """
        Configure V/f characteristic.

        :param v_startup: Startup voltage [V]
        :type v_startup: float
        :param v_nominal: Nominal voltage [V]
        :type v_nominal: float
        :param f_nominal: Nominal frequency [Hz]
        :type f_nominal: float

        :raises ValueError: If f_nominal <= 0
        """
        if f_nominal <= 0:
            raise ValueError("Nominal frequency must be positive")

        self.v_startup = v_startup
        self.v_nominal = v_nominal
        self.f_nominal = f_nominal
        self.kv_f = (v_nominal - v_startup) / f_nominal

    def set_frequency_slew_rate(self, slew_rate: float) -> None:
        """
        Set frequency ramp rate limit.

        :param slew_rate: Maximum frequency change [Hz/s]
        :type slew_rate: float

        :raises ValueError: If slew_rate < 0
        """
        if slew_rate < 0:
            raise ValueError("Slew rate must be non-negative")
        self.frequency_slew_rate = slew_rate

    def set_voltage_ramp_rate(self, ramp_rate: float) -> None:
        """
        Set voltage ramp rate limit.

        :param ramp_rate: Maximum voltage change [V/s]
        :type ramp_rate: float

        :raises ValueError: If ramp_rate < 0
        """
        if ramp_rate < 0:
            raise ValueError("Ramp rate must be non-negative")
        self.ramp_rate = ramp_rate

    def enable_startup_boost(self, enable: bool = True, duration: float = 0.5) -> None:
        """
        Enable/disable startup boost phase.

        Startup boost applies extra voltage for specified duration to help
        break inertia and accelerate the motor faster at low speeds.

        :param enable: Enable or disable boost
        :type enable: bool
        :param duration: Boost duration [seconds]
        :type duration: float
        """
        self.use_startup_boost = enable
        self.startup_boost_time = duration

    def reset(self) -> None:
        """
        Reset controller to initial state.
        """
        self.frequency_ref = 0.0
        self.frequency_actual = 0.0
        self.voltage_actual = 0.0
        self.angle = 0.0
        self.angle_velocity = 0.0
        self.startup_timer = 0.0

    def get_state(self) -> dict:
        """
        Get controller state variables.

        :return: State dictionary
        :rtype: dict

        **Keys:**
            - 'frequency_ref': Reference frequency [Hz]
            - 'frequency_actual': Actual frequency [Hz]
            - 'voltage': Actual voltage magnitude [V]
            - 'angle': Voltage phase angle [rad]
        """
        return {
            "frequency_ref": self.frequency_ref,
            "frequency_actual": self.frequency_actual,
            "voltage": self.voltage_actual,
            "angle": self.angle,
        }
