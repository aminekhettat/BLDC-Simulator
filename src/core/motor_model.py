"""
BLDC Motor Model
================

This module implements the mathematical model of a Brushless DC (BLDC) motor,
including electrical and mechanical dynamics equations. The model accepts 3-phase
voltages and produces motor variables (currents, EMF, torque, speed, position).

The electrical model is based on:
- Phase voltage equation: v_phase = R*i_phase + L*di_phase/dt + e_back_phase
- Back-EMF generation from rotor position

The mechanical model includes:
- Newton's second law: tau_electro - tau_load - f*omega = J*d(omega)/dt
- Position integration: d(theta)/dt = omega

:author: BLDC Control Team
:version: 0.8.0

.. versionadded:: 0.8.0
    Initial implementation of BLDC motor model
"""

from dataclasses import dataclass

import numpy as np

try:
    from numba import njit

    HAS_NUMBA = True
except ImportError:
    HAS_NUMBA = False

    # Dummy decorator if numba not available
    def njit(*args, **kwargs):
        def decorator(f):
            return f

        return decorator if callable(args[0]) is False else args[0]


@dataclass
class MotorParameters:
    """Data class containing BLDC motor parameters.

    :ivar model_type: Type of motor model ('scalar' or 'dq')

    :ivar nominal_voltage: Nominal DC voltage [V]
    :ivar phase_resistance: Phase winding resistance [Ohms]
    :ivar phase_inductance: Phase winding inductance [H]
    :ivar back_emf_constant: Back-EMF constant [V*s/rad]
    :ivar torque_constant: Electromagnetic torque constant [N*m/A]
    :ivar rotor_inertia: Rotor moment of inertia [kg*m^2]
    :ivar friction_coefficient: Viscous friction coefficient [N*m*s/rad]
    :ivar num_poles: Number of magnetic poles
    :ivar poles_pairs: Number of pole pairs (num_poles/2)
    """

    model_type: str = "scalar"
    emf_shape: str = "trapezoidal"

    nominal_voltage: float = 48.0
    phase_resistance: float = 2.5
    phase_inductance: float = 0.005
    back_emf_constant: float = 0.1
    torque_constant: float = 0.1
    rotor_inertia: float = 0.0005
    friction_coefficient: float = 0.001
    num_poles: int = 8
    poles_pairs: int = 4
    # d/q inductances (optional). If None, defaults to phase_inductance
    ld: float | None = None
    lq: float | None = None
    # Optional Id-dependent PM flux weakening model for dq simulation.
    # Effective PM flux ratio is clamped as:
    #   ratio = clip(1 + k_fw * min(id, 0), min_ratio, 1)
    # where k_fw is in 1/A. Defaults keep legacy behavior.
    flux_weakening_id_coefficient: float = 0.0
    flux_weakening_min_ratio: float = 0.2

    def __post_init__(self):
        # Ensure ld/lq default to phase_inductance when not provided
        if self.ld is None:
            self.ld = self.phase_inductance
        if self.lq is None:
            self.lq = self.phase_inductance
        if self.flux_weakening_id_coefficient < 0.0:
            raise ValueError("flux_weakening_id_coefficient must be non-negative")
        if not (0.0 < self.flux_weakening_min_ratio <= 1.0):
            raise ValueError("flux_weakening_min_ratio must be in (0, 1]")


class BLDCMotor:
    """
    BLDC Motor Model
    ================

    Implements a 3-phase BLDC motor model with electrical and mechanical dynamics.
    Uses numerical integration (4th-order Runge-Kutta) for state evolution.

    The motor model tracks:
    - Three phase currents: i_a, i_b, i_c [A]
    - Rotor angular velocity: omega [rad/s]
    - Rotor angular position: theta [rad]
    - Phase voltages (input): v_a, v_b, v_c [V]

    **State Vector:** [i_a, i_b, i_c, omega, theta]

    Example:
        >>> params = MotorParameters(
        ...     nominal_voltage=48.0,
        ...     phase_resistance=2.5,
        ...     phase_inductance=0.005,
        ...     back_emf_constant=0.1,
        ...     torque_constant=0.1,
        ...     rotor_inertia=0.0005,
        ...     friction_coefficient=0.001,
        ...     num_poles=8,
        ...     poles_pairs=4
        ... )
        >>> motor = BLDCMotor(params, dt=0.0001)
        >>> voltages = np.array([10.0, -5.0, -5.0])
        >>> motor.step(voltages, load_torque=0.5)
    """

    def __init__(self, parameters: MotorParameters, dt: float = 0.0001):
        """
        Initialize the BLDC motor model.

        :param parameters: Motor parameters (:class:`MotorParameters`)
        :param dt: Simulation time step [seconds], defaults to 0.0001
        :type dt: float

        :raises ValueError: If dt <= 0
        :raises ValueError: If phase_inductance <= 0
        """
        if dt <= 0:
            raise ValueError("Time step dt must be positive")
        if parameters.phase_inductance <= 0:
            raise ValueError("Phase inductance must be positive")
        if parameters.model_type == "dq":
            ld_eff = (
                float(parameters.ld)
                if parameters.ld is not None
                else float(parameters.phase_inductance)
            )
            lq_eff = (
                float(parameters.lq)
                if parameters.lq is not None
                else float(parameters.phase_inductance)
            )
            if ld_eff <= 0:
                raise ValueError("d-axis inductance (Ld) must be positive for dq model")
            if lq_eff <= 0:
                raise ValueError("q-axis inductance (Lq) must be positive for dq model")

        self.params = parameters
        self.dt = dt

        # State vector: [i_a, i_b, i_c, omega, theta]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        # Store last measured values for output
        self._last_emf = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self._last_torque = 0.0
        self._last_currents = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    @property
    def currents(self) -> np.ndarray:
        """
        Get phase currents [A].

        :return: Array of [i_a, i_b, i_c]
        :rtype: np.ndarray
        """
        return self.state[0:3].copy()

    @property
    def omega(self) -> float:
        """
        Get rotor angular velocity [rad/s].

        :return: Angular velocity
        :rtype: float
        """
        return float(self.state[3])

    @property
    def theta(self) -> float:
        """
        Get rotor angular position [rad].

        :return: Angular position (wrapped to [0, 2*pi])
        :rtype: float
        """
        return float(self.state[4] % (2 * np.pi))

    @property
    def speed_rpm(self) -> float:
        """
        Get rotor speed in revolutions per minute [RPM].

        :return: Speed in RPM
        :rtype: float
        """
        return self.omega * 60.0 / (2 * np.pi)

    @property
    def back_emf(self) -> np.ndarray:
        """
        Get back-EMF for each phase [V].

        :return: Array of [e_a, e_b, e_c]
        :rtype: np.ndarray
        """
        return self._last_emf.copy()

    @property
    def electromagnetic_torque(self) -> float:
        """
        Get electromagnetic torque [N*m].

        :return: Torque
        :rtype: float
        """
        return self._last_torque

    @staticmethod
    @njit
    def _trapezoidal_fast(theta_elec: float) -> float:
        """
        Fast trapezoidal back-EMF lookup (numba-compiled).
        """
        theta_norm = theta_elec % (2 * np.pi)
        pi_6 = np.pi / 6.0

        if theta_norm < pi_6 or theta_norm < np.pi - pi_6:
            return 1.0
        if theta_norm < np.pi:
            return (np.pi - theta_norm) / pi_6
        if theta_norm < np.pi + pi_6 or theta_norm < 2 * np.pi - pi_6:
            return -1.0
        return (theta_norm - 2 * np.pi) / pi_6

    @staticmethod
    @njit
    def _sinusoidal_fast(theta_elec: float) -> float:
        """
        Fast sinusoidal back-EMF lookup (numba-compiled).
        """
        return float(np.sin(theta_elec))

    def _calculate_back_emf_sinusoidal(self, theta: float) -> np.ndarray:
        theta_elec = self.params.poles_pairs * theta
        k_emf = self.params.back_emf_constant * self.state[3]  # K * omega
        emf_a = k_emf * self._sinusoidal_fast(theta_elec)
        emf_b = k_emf * self._sinusoidal_fast(theta_elec - 2 * np.pi / 3)
        emf_c = k_emf * self._sinusoidal_fast(theta_elec - 4 * np.pi / 3)
        return np.array([emf_a, emf_b, emf_c], dtype=np.float64)

    def _calculate_back_emf(self, theta: float) -> np.ndarray:
        """
        Calculate back-EMF for three phases using trapezoidal EMF model.

        For a BLDC motor with trapezoidal back-EMF:
        - Phase A: starts at 0°, continues in 120° increments
        - Phase B: starts at 120°
        - Phase C: starts at 240°

        :param theta: Rotor position [rad]
        :type theta: float
        :return: Back-EMF for each phase [V]
        :rtype: np.ndarray

        .. note::
            This uses a simplified trapezoidal model. For sinusoidal motors,
            use e = K * omega * sin(pole_pairs * theta + phase_offset)
        """
        if self.params.emf_shape == "sinusoidal":
            return self._calculate_back_emf_sinusoidal(theta)
        # Default to trapezoidal
        # Convert to electrical angle
        theta_elec = self.params.poles_pairs * theta

        # Trapezoidal EMF model (three 120° shifts)
        k_emf = self.params.back_emf_constant * self.state[3]  # K * omega

        emf_a = k_emf * self._trapezoidal_fast(theta_elec)
        emf_b = k_emf * self._trapezoidal_fast(theta_elec - 2 * np.pi / 3)
        emf_c = k_emf * self._trapezoidal_fast(theta_elec - 4 * np.pi / 3)
        return np.array([emf_a, emf_b, emf_c], dtype=np.float64)

    @staticmethod
    def _trapezoidal(angle: float) -> float:
        """
        Trapezoidal waveform function used for back-EMF.

        Generates trapezoidal waveform with:
        - Positive flat region: 0 to 120°
        - Negative flat region: 180° to 300°
        - Linear transitions between them

        :param angle: Phase angle [rad]
        :type angle: float
        :return: Trapezoidal value in range [-1, 1]
        :rtype: float
        """
        # Normalize angle to [0, 2*pi]
        angle = angle % (2 * np.pi)

        if angle < 2 * np.pi / 3:  # 0 to 120°
            return 1.0
        if angle < np.pi:  # 120° to 180°
            return (np.pi - angle) / (np.pi / 3) - 1.0
        if angle < 5 * np.pi / 3:  # 180° to 300°
            return -1.0
        # 300° to 360°
        return (angle - 5 * np.pi / 3) / (np.pi / 3) - 1.0

    @staticmethod
    @njit
    def _calculate_electromagnetic_torque_fast(
        kt: float, currents_a: float, currents_b: float, currents_c: float
    ) -> float:
        """
        Fast electromagnetic torque calculation (numba-compiled).
        For BLDC: tau = K_t * (i_a + i_b + i_c) / 3
        """
        return kt * (currents_a + currents_b + currents_c) / 3.0

    def _calculate_electromagnetic_torque(self, currents: np.ndarray) -> float:
        """
        Calculate electromagnetic torque from phase currents.

        For a BLDC motor: tau = K_t * i_phase_active

        :param currents: Phase currents [A]
        :type currents: np.ndarray
        :return: Electromagnetic torque [N*m]
        :rtype: float
        """
        # Simplified: torque proportional to active phase current
        # For trapezoidal BLDC, active phase = max current in conducting phases
        i_active = np.max(np.abs(currents))
        return float(self.params.torque_constant * i_active)

    def _calculate_electromagnetic_torque_dq(self, currents: np.ndarray, theta: float) -> float:
        """
        Calculate electromagnetic torque from d-q currents.
        Torque = 3/2 * P * ( (Ld-Lq)*id*iq + iq*lambda_pm )
        """
        from src.control.transforms import clarke_transform, park_transform

        ia, ib, ic = currents
        i_alpha, i_beta = clarke_transform(ia, ib, ic)

        theta_elec = self.params.poles_pairs * theta
        i_d, i_q = park_transform(i_alpha, i_beta, theta_elec)

        # For non-salient motors (Ld=Lq), torque is proportional to iq
        # Torque = (3/2) * P * lambda_pm * iq
        # Since Kt = (3/2) * P * lambda_pm for non-salient PMSM,
        # Torque = Kt * iq
        # We use a simplified model here that is more general.
        # For a surface-mount PMSM (non-salient), Ld=Lq, so the first term is zero.
        # The permanent magnet flux linkage lambda_pm is related to Kt.
        # lambda_pm = Kt / ( (3/2) * P )

        P = self.params.poles_pairs
        lambda_pm = self.params.torque_constant / (1.5 * P)
        flux_ratio = self._flux_weakening_ratio(i_d)
        lambda_pm_eff = lambda_pm * flux_ratio
        ld_eff = (
            float(self.params.ld)
            if self.params.ld is not None
            else float(self.params.phase_inductance)
        )
        lq_eff = (
            float(self.params.lq)
            if self.params.lq is not None
            else float(self.params.phase_inductance)
        )

        torque = 1.5 * P * ((ld_eff - lq_eff) * i_d * i_q + i_q * lambda_pm_eff)
        return torque

    def _flux_weakening_ratio(self, i_d: float) -> float:
        """Return effective PM flux ratio after Id-based weakening."""
        id_neg = min(float(i_d), 0.0)
        ratio = 1.0 + self.params.flux_weakening_id_coefficient * id_neg
        return float(np.clip(ratio, self.params.flux_weakening_min_ratio, 1.0))

    def _calculate_torque(self, currents: np.ndarray, theta: float) -> float:
        if self.params.model_type == "dq":
            return self._calculate_electromagnetic_torque_dq(currents, theta)
        # scalar
        return self._calculate_electromagnetic_torque(currents)

    def _state_derivatives(self, state: np.ndarray, voltages: np.ndarray) -> np.ndarray:
        """
        Calculate state derivatives: d(state)/dt.

        Uses Kirchhoff's voltage law and Newton's second law:

        **Electrical equation (for each phase i):**

        .. math::
            v_i = R*i_i + L*\\frac{di_i}{dt} + e_i

        Rearranged:

        .. math::
            \\frac{di_i}{dt} = \\frac{1}{L}(v_i - R*i_i - e_i)

        **Mechanical equation:**

        .. math::
            \\frac{d\\omega}{dt} = \\frac{1}{J}(\\tau_{em} - \\tau_{load} - f*\\omega)

        .. math::
            \\frac{d\\theta}{dt} = \\omega

        :param state: Current state vector [i_a, i_b, i_c, omega, theta]
        :type state: np.ndarray
        :param voltages: Applied phase voltages [v_a, v_b, v_c]
        :type voltages: np.ndarray
        :return: State derivatives
        :rtype: np.ndarray

        :raises AssertionError: If voltages array size != 3
        """
        assert voltages.shape == (3,), "Voltages must be 3-phase array"

        currents = state[0:3]
        omega = state[3]
        theta = state[4]

        # Calculate back-EMF
        emf = self._calculate_back_emf(theta)
        self._last_emf = emf.copy()

        if self.params.model_type == "dq":
            from src.control.transforms import (
                clarke_transform,
                inverse_clarke,
                inverse_park,
                park_transform,
            )

            # d-q model requires transforming voltages and currents
            theta_elec = self.params.poles_pairs * theta
            omega_elec = self.params.poles_pairs * omega

            # To get Vd, Vq from Va,Vb,Vc
            v_alpha, v_beta = clarke_transform(voltages[0], voltages[1], voltages[2])
            v_d, v_q = park_transform(v_alpha, v_beta, theta_elec)

            # To get id, iq from ia,ib,ic
            i_alpha, i_beta = clarke_transform(currents[0], currents[1], currents[2])
            i_d, i_q = park_transform(i_alpha, i_beta, theta_elec)

            # d-q voltage equations
            # back_emf_constant is Ke [V·s/mech.rad]; flux linkage λ_pm = Ke/P.
            # BEMF term in q-axis: ωe × λ_pm = ωe × Ke/P = ωmech × Ke = omega × Ke.
            lambda_pm = self.params.back_emf_constant / max(float(self.params.poles_pairs), 1.0)
            flux_ratio = self._flux_weakening_ratio(i_d)
            lambda_pm_eff = lambda_pm * flux_ratio
            di_d_dt = (
                v_d - self.params.phase_resistance * i_d + omega_elec * self.params.lq * i_q
            ) / self.params.ld
            di_q_dt = (
                v_q
                - self.params.phase_resistance * i_q
                - omega_elec * self.params.ld * i_d
                - omega_elec * lambda_pm_eff
            ) / self.params.lq

            # Transform DQ derivatives back to stationary-frame (ABC) derivatives.
            # Full chain-rule: d/dt(iPark(id, iq, θe)) has a rotational correction
            # from dθe/dt = ωe:
            #   d(iα)/dt = iPark(diᵈ/dt, diᵍ/dt, θe)[α] − ωe · iβ
            #   d(iβ)/dt = iPark(diᵈ/dt, diᵍ/dt, θe)[β] + ωe · iα
            # Without this term the ABC state "freezes" while Park transform rotates,
            # producing a spurious limit cycle at the electrical frequency.
            di_alpha_dt_base, di_beta_dt_base = inverse_park(di_d_dt, di_q_dt, theta_elec)
            di_alpha_dt = di_alpha_dt_base - omega_elec * i_beta
            di_beta_dt = di_beta_dt_base + omega_elec * i_alpha
            di_dt_a, di_dt_b, di_dt_c = inverse_clarke(di_alpha_dt, di_beta_dt)
            di_dt = np.array([di_dt_a, di_dt_b, di_dt_c])
        else:  # scalar model
            # Electrical dynamics: di/dt = (V - R*i - E) / L
            di_dt = (
                voltages - self.params.phase_resistance * currents - emf
            ) / self.params.phase_inductance

        # Electromagnetic torque
        tau_em = self._calculate_torque(currents, theta)
        self._last_torque = tau_em

        # Mechanical dynamics: dω/dt = (τ_em - τ_load - f*ω) / J
        # Load torque is added in step() method

        # Create derivatives array
        derivatives = np.zeros(5, dtype=np.float64)
        derivatives[0:3] = di_dt
        # omega derivative will be set in step() with load torque
        derivatives[4] = omega  # dtheta/dt = omega

        return derivatives

    def step(self, voltages: np.ndarray, load_torque: float = 0.0) -> None:
        """
        Execute one simulation step using 4th-order Runge-Kutta integration.

        Updates internal state: [i_a, i_b, i_c, omega, theta]

        **Runge-Kutta 4th Order (RK4):**

        .. math::
            k_1 = f(t_n, y_n)

        .. math::
            k_2 = f(t_n + \\frac{h}{2}, y_n + \\frac{h}{2}k_1)

        .. math::
            k_3 = f(t_n + \\frac{h}{2}, y_n + \\frac{h}{2}k_2)

        .. math::
            k_4 = f(t_n + h, y_n + h*k_3)

        .. math::
            y_{n+1} = y_n + \\frac{h}{6}(k_1 + 2k_2 + 2k_3 + k_4)

        :param voltages: Applied phase voltages [v_a, v_b, v_c] [V]
        :type voltages: np.ndarray
        :param load_torque: Load torque acting on motor shaft [N*m], defaults to 0.0
        :type load_torque: float

        .. warning::
            Voltages must be 3-element array. Use SVM generator for modulated voltages.
        """
        # RK4 integration
        k1_derivatives = self._state_derivatives(self.state, voltages)

        # Modify k1 to include load torque effect on omega
        k1_derivatives[3] = (
            self._last_torque - load_torque - self.params.friction_coefficient * self.state[3]
        ) / self.params.rotor_inertia

        # k2
        state_k2 = self.state + 0.5 * self.dt * k1_derivatives
        k2_derivatives = self._state_derivatives(state_k2, voltages)
        k2_derivatives[3] = (
            self._last_torque - load_torque - self.params.friction_coefficient * state_k2[3]
        ) / self.params.rotor_inertia

        # k3
        state_k3 = self.state + 0.5 * self.dt * k2_derivatives
        k3_derivatives = self._state_derivatives(state_k3, voltages)
        k3_derivatives[3] = (
            self._last_torque - load_torque - self.params.friction_coefficient * state_k3[3]
        ) / self.params.rotor_inertia

        # k4
        state_k4 = self.state + self.dt * k3_derivatives
        k4_derivatives = self._state_derivatives(state_k4, voltages)
        k4_derivatives[3] = (
            self._last_torque - load_torque - self.params.friction_coefficient * state_k4[3]
        ) / self.params.rotor_inertia

        # Final update
        self.state += (self.dt / 6.0) * (
            k1_derivatives + 2 * k2_derivatives + 2 * k3_derivatives + k4_derivatives
        )

    def reset(self, initial_speed: float = 0.0) -> None:
        """
        Reset motor to initial conditions.

        :param initial_speed: Initial rotor speed [rad/s], defaults to 0.0
        :type initial_speed: float
        """
        self.state = np.array([0.0, 0.0, 0.0, initial_speed, 0.0], dtype=np.float64)
        self._last_emf = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self._last_torque = 0.0
        self._last_currents = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    def get_state_dict(self) -> dict:
        """
        Get all motor state variables as a dictionary.

        :return: Dictionary containing all motor variables
        :rtype: dict

        **Keys:**
            - 'currents_a': Phase A current [A]
            - 'currents_b': Phase B current [A]
            - 'currents_c': Phase C current [A]
            - 'omega': Angular velocity [rad/s]
            - 'theta': Angular position [rad]
            - 'speed_rpm': Speed [RPM]
            - 'back_emf_a': Phase A back-EMF [V]
            - 'back_emf_b': Phase B back-EMF [V]
            - 'back_emf_c': Phase C back-EMF [V]
            - 'torque': Electromagnetic torque [N*m]
        """
        return {
            "currents_a": self.state[0],
            "currents_b": self.state[1],
            "currents_c": self.state[2],
            "omega": self.state[3],
            "theta": self.theta,
            "speed_rpm": self.speed_rpm,
            "back_emf_a": self._last_emf[0],
            "back_emf_b": self._last_emf[1],
            "back_emf_c": self._last_emf[2],
            "torque": self._last_torque,
        }
