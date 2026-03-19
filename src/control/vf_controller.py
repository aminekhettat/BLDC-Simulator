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
:version: 0.8.0

.. versionadded:: 0.8.0
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

        # Standard V/f startup sequence: align -> ramp -> steady run.
        self.startup_sequence_enabled = False
        self.startup_phase = "run"
        self.startup_sequence_elapsed_s = 0.0
        self.startup_phase_elapsed_s = 0.0
        self.startup_align_duration_s = 0.05
        self.startup_align_voltage_v = max(v_startup, 1.5)
        self.startup_align_angle = 0.0
        self.startup_ramp_initial_frequency_hz = 2.0

    def set_startup_sequence(
        self,
        enable: bool = True,
        align_duration_s: float = 0.05,
        align_voltage_v: float = 1.5,
        align_angle_deg: float = 0.0,
        ramp_initial_frequency_hz: float = 2.0,
    ) -> None:
        """Configure the standard V/f startup sequence."""
        if align_duration_s < 0.0:
            raise ValueError("align_duration_s must be non-negative")
        if align_voltage_v < 0.0:
            raise ValueError("align_voltage_v must be non-negative")
        if ramp_initial_frequency_hz < 0.0:
            raise ValueError("ramp_initial_frequency_hz must be non-negative")

        self.startup_sequence_enabled = bool(enable)
        self.startup_align_duration_s = float(align_duration_s)
        self.startup_align_voltage_v = float(align_voltage_v)
        self.startup_align_angle = float(np.deg2rad(align_angle_deg) % (2.0 * np.pi))
        self.startup_ramp_initial_frequency_hz = float(ramp_initial_frequency_hz)
        self._reset_startup_sequence_runtime()

    def _enter_startup_phase(self, phase: str) -> None:
        """Switch V/f startup phase and reset phase-local timers."""
        self.startup_phase = phase
        self.startup_phase_elapsed_s = 0.0
        if phase == "align":
            self.angle = self.startup_align_angle
            self.angle_velocity = 0.0
        elif phase == "open_loop":
            sign = -1.0 if self.frequency_ref < 0.0 else 1.0
            self.frequency_actual = sign * self.startup_ramp_initial_frequency_hz
        else:
            self.startup_phase = "run"

    def _reset_startup_sequence_runtime(self) -> None:
        """Reset V/f startup runtime state."""
        self.startup_sequence_elapsed_s = 0.0
        self.startup_phase_elapsed_s = 0.0
        if not self.startup_sequence_enabled:
            self.startup_phase = "run"
            return
        if self.startup_align_duration_s > 0.0:
            self._enter_startup_phase("align")
        else:
            self._enter_startup_phase("open_loop")

    def _compute_vf_voltage(self, dt: float) -> float:
        """Compute V/f voltage magnitude including startup boost."""
        if abs(self.frequency_actual) <= 1e-12:
            voltage = 0.0
        else:
            voltage = self.v_startup + self.kv_f * np.abs(self.frequency_actual)
            if self.use_startup_boost and self.startup_timer < self.startup_boost_time:
                boost = (self.v_startup * 0.2) * (
                    1.0 - self.startup_timer / self.startup_boost_time
                )
                voltage += boost
                self.startup_timer += dt
            else:
                self.startup_timer = 0.0
        voltage = float(np.clip(voltage, 0.0, self.max_voltage))
        self.voltage_actual = voltage
        return voltage

    def _update_open_loop_frequency(self, dt: float) -> None:
        """Advance frequency reference toward its target with slew limiting."""
        delta_f_max = self.frequency_slew_rate * dt
        delta_f = np.clip(
            self.frequency_ref - self.frequency_actual, -delta_f_max, delta_f_max
        )
        self.frequency_actual += delta_f

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
        self.startup_sequence_elapsed_s += dt
        self.startup_phase_elapsed_s += dt

        if self.startup_sequence_enabled and self.startup_phase == "align":
            self.voltage_actual = float(
                np.clip(self.startup_align_voltage_v, 0.0, self.max_voltage)
            )
            self.angle = self.startup_align_angle
            self.angle_velocity = 0.0
            if self.startup_phase_elapsed_s >= self.startup_align_duration_s:
                self._enter_startup_phase("open_loop")
            return self.voltage_actual, self.angle

        if self.startup_sequence_enabled and self.startup_phase in {"open_loop", "run"}:
            self._update_open_loop_frequency(dt)
            voltage = self._compute_vf_voltage(dt)
            self.angle_velocity = 2.0 * np.pi * self.frequency_actual
            self.angle = (self.angle + self.angle_velocity * dt) % (2.0 * np.pi)

            if self.startup_phase == "open_loop":
                tol_hz = max(0.5, 0.02 * max(abs(self.frequency_ref), 1.0))
                if abs(self.frequency_actual - self.frequency_ref) <= tol_hz:
                    self._enter_startup_phase("run")
            return voltage, self.angle

        self._update_open_loop_frequency(dt)
        voltage = self._compute_vf_voltage(dt)

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
        self._reset_startup_sequence_runtime()

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
            "startup_sequence_enabled": self.startup_sequence_enabled,
            "startup_phase": self.startup_phase,
            "startup_phase_code": {"align": 1.0, "open_loop": 2.0, "run": 3.0}.get(
                self.startup_phase, 0.0
            ),
            "startup_sequence_elapsed_s": self.startup_sequence_elapsed_s,
            "startup_phase_elapsed_s": self.startup_phase_elapsed_s,
            "startup_align_duration_s": self.startup_align_duration_s,
            "startup_align_voltage_v": self.startup_align_voltage_v,
            "startup_align_angle_rad": self.startup_align_angle,
            "startup_ramp_initial_frequency_hz": self.startup_ramp_initial_frequency_hz,
        }
