"""
Simulation Engine Module
========================

This module implements the main simulation loop that coordinates
motor model, control blocks, load profile, and data logging.

Provides efficient real-time simulation with optimized calculations.

:author: BLDC Control Team
:version: 1.0.0

.. versionadded:: 1.0.0
    Initial implementation of simulation engine
"""

import numpy as np
from typing import Optional, Dict, List, Tuple, Any
from .motor_model import BLDCMotor, MotorParameters
from .load_model import LoadProfile
from .power_model import (
    SupplyProfile,
    ConstantSupply,
    PowerFactorController,
    compute_power_metrics,
)


class SimulationEngine:
    """
    Main simulation engine for BLDC motor control.

    Orchestrates motor simulation with:
    - Motor model state evolution
    - Load profile application
    - Voltage input (from SVM or control blocks)
    - Data logging and history tracking

    **Features:**
    - Efficient NumPy-based calculations
    - Real-time data logging
    - State history tracking
    - Simulation time management

    Example:
        >>> params = MotorParameters(...)
        >>> motor = BLDCMotor(params, dt=0.0001)
        >>> load = ConstantLoad(torque=1.0)
        >>> engine = SimulationEngine(motor, load, dt=0.0001)
        >>>
        >>> for i in range(10000):
        ...     voltages = np.array([10, -5, -5])
        ...     engine.step(voltages)
        >>>
        >>> history = engine.get_history()
    """

    def __init__(
        self,
        motor: BLDCMotor,
        load_profile: LoadProfile,
        dt: float = 0.0001,
        max_history: int = 100000,
        supply_profile: Optional[SupplyProfile] = None,
        pfc_controller: Optional[PowerFactorController] = None,
        pfc_window_samples: int = 128,
    ):
        """
        Initialize simulation engine.

        :param motor: BLDC motor model (:class:`BLDCMotor`)
        :type motor: BLDCMotor
        :param load_profile: Load profile (:class:`LoadProfile`)
        :type load_profile: LoadProfile
        :param dt: Simulation time step [seconds], defaults to 0.0001
        :type dt: float
        :param max_history: Maximum history samples to store, defaults to 100000
        :type max_history: int

        :raises ValueError: If dt <= 0
        """
        if dt <= 0:
            raise ValueError("Time step dt must be positive")

        self.motor = motor
        self.load_profile = load_profile
        self.dt = dt
        self.max_history = max_history

        # supply profile (dc bus voltage)
        if supply_profile is None:
            # default uses nominal constant supply
            self.supply_profile = ConstantSupply()
        else:
            self.supply_profile = supply_profile

        self.pfc_controller = pfc_controller
        self.pfc_window_samples = max(int(pfc_window_samples), 8)
        self._pfc_enabled = pfc_controller is not None
        self._pfc_metrics: Dict[str, float] = {
            "power_factor": 0.0,
            "active_power_w": 0.0,
            "apparent_power_va": 0.0,
            "reactive_power_var": 0.0,
            "compensation_command_var": 0.0,
            "target_pf": float(pfc_controller.target_pf) if pfc_controller else 0.0,
        }

        # Simulation state
        self.time = 0.0
        self.step_count = 0

        # History buffers (pre-allocated for efficiency)
        self._history_times: List[float] = []
        self._history_currents_a: List[float] = []
        self._history_currents_b: List[float] = []
        self._history_currents_c: List[float] = []
        self._history_omega: List[float] = []
        self._history_theta: List[float] = []
        self._history_speed: List[float] = []
        self._history_emf_a: List[float] = []
        self._history_emf_b: List[float] = []
        self._history_emf_c: List[float] = []
        self._history_torque: List[float] = []
        self._history_load_torque: List[float] = []
        self._history_voltages_a: List[float] = []
        self._history_voltages_b: List[float] = []
        self._history_voltages_c: List[float] = []
        self._history_supply_voltage: List[float] = []
        self._history_input_current: List[float] = []
        self._history_input_power: List[float] = []
        self._history_pf: List[float] = []
        self._history_pfc_command: List[float] = []

        # State variables cache
        self._last_state_dict = {}

    def step(self, voltages: np.ndarray, log_data: bool = True) -> None:
        """
        Execute one simulation step.

        Updates motor state, applies load, and optionally logs data.

        :param voltages: 3-phase input voltages [v_a, v_b, v_c] [V]
        :type voltages: np.ndarray
        :param log_data: Whether to log data to history, defaults to True
        :type log_data: bool

        .. warning::
            Set log_data=False during high-frequency real-time control loops,
            then call :meth:`log_data` periodically to reduce overhead.
        """
        # supply voltage at this time
        supply_v = self.supply_profile.get_voltage(self.time)

        # Get load torque at current simulation time
        load_torque = self.load_profile.get_torque(self.time)

        # Execute motor dynamics step
        self.motor.step(voltages, load_torque)

        # Log data if requested
        if log_data:
            self._log_data(voltages, load_torque, supply_v)

        # Advance simulation time
        self.time += self.dt
        self.step_count += 1

    def _log_data(
        self, voltages: np.ndarray, load_torque: float, supply_voltage: float
    ) -> None:
        """
        Log simulation data to history buffers.

        :param voltages: Applied 3-phase voltages [V]
        :type voltages: np.ndarray
        :param load_torque: Load torque [N*m]
        :type load_torque: float
        :param supply_voltage: DC bus voltage [V]
        :type supply_voltage: float

        .. note::
            History buffers are limited to max_history size.
            When full, oldest data is dropped.
        """
        # Check if buffers are full
        if len(self._history_times) >= self.max_history:
            # Remove first element from all buffers
            self._history_times.pop(0)
            self._history_currents_a.pop(0)
            self._history_currents_b.pop(0)
            self._history_currents_c.pop(0)
            self._history_omega.pop(0)
            self._history_theta.pop(0)
            self._history_speed.pop(0)
            self._history_emf_a.pop(0)
            self._history_emf_b.pop(0)
            self._history_emf_c.pop(0)
            self._history_torque.pop(0)
            self._history_load_torque.pop(0)
            self._history_voltages_a.pop(0)
            self._history_voltages_b.pop(0)
            self._history_voltages_c.pop(0)
            self._history_supply_voltage.pop(0)
            self._history_input_current.pop(0)
            self._history_input_power.pop(0)
            self._history_pf.pop(0)
            self._history_pfc_command.pop(0)

        # Get motor state
        state = self.motor.get_state_dict()
        self._last_state_dict = state

        # Append to history
        self._history_times.append(self.time)
        self._history_currents_a.append(state["currents_a"])
        self._history_currents_b.append(state["currents_b"])
        self._history_currents_c.append(state["currents_c"])
        self._history_omega.append(state["omega"])
        self._history_theta.append(state["theta"])
        self._history_speed.append(state["speed_rpm"])
        self._history_emf_a.append(state["back_emf_a"])
        self._history_emf_b.append(state["back_emf_b"])
        self._history_emf_c.append(state["back_emf_c"])
        self._history_torque.append(state["torque"])
        self._history_load_torque.append(load_torque)
        self._history_voltages_a.append(voltages[0])
        self._history_voltages_b.append(voltages[1])
        self._history_voltages_c.append(voltages[2])
        self._history_supply_voltage.append(supply_voltage)

        i_abc = np.array(
            [state["currents_a"], state["currents_b"], state["currents_c"]],
            dtype=np.float64,
        )
        p_instant = float(np.dot(voltages, i_abc))
        if abs(supply_voltage) > 1e-9:
            i_in = p_instant / float(supply_voltage)
        else:
            i_in = 0.0

        self._history_input_power.append(p_instant)
        self._history_input_current.append(i_in)

        pf_now = self._pfc_metrics["power_factor"] if self._history_pf else 0.0
        cmd_var = self._pfc_metrics["compensation_command_var"]

        if len(self._history_input_current) >= 8:
            w = min(self.pfc_window_samples, len(self._history_input_current))
            metrics = compute_power_metrics(
                np.array(self._history_supply_voltage[-w:], dtype=np.float64),
                np.array(self._history_input_current[-w:], dtype=np.float64),
            )
            pf_now = float(metrics["power_factor"])
            self._pfc_metrics["power_factor"] = pf_now
            self._pfc_metrics["active_power_w"] = float(metrics["active_power_w"])
            self._pfc_metrics["apparent_power_va"] = float(metrics["apparent_power_va"])
            self._pfc_metrics["reactive_power_var"] = float(
                metrics["reactive_power_var"]
            )

            if self._pfc_enabled and self.pfc_controller is not None:
                cmd_var = self.pfc_controller.update(
                    current_pf=pf_now,
                    active_power_w=float(metrics["active_power_w"]),
                    dt=self.dt,
                )
                self._pfc_metrics["compensation_command_var"] = float(cmd_var)
                self._pfc_metrics["target_pf"] = float(self.pfc_controller.target_pf)

        self._history_pf.append(pf_now)
        self._history_pfc_command.append(cmd_var)

    def configure_power_factor_control(
        self,
        enabled: bool,
        target_pf: float = 0.95,
        kp: float = 0.10,
        ki: float = 1.0,
        max_compensation_var: float = 10000.0,
        window_samples: int = 128,
    ) -> None:
        """Enable/disable closed-loop PFC telemetry hook."""
        self._pfc_enabled = bool(enabled)
        self.pfc_window_samples = max(int(window_samples), 8)

        if not self._pfc_enabled:
            self.pfc_controller = None
            self._pfc_metrics["target_pf"] = 0.0
            self._pfc_metrics["compensation_command_var"] = 0.0
            return

        self.pfc_controller = PowerFactorController(
            target_pf=target_pf,
            kp=kp,
            ki=ki,
            max_compensation_var=max_compensation_var,
        )
        self._pfc_metrics["target_pf"] = float(target_pf)

    def get_power_factor_control_state(self) -> Dict[str, float | bool]:
        """Return current PFC telemetry and controller state."""
        return {
            "enabled": self._pfc_enabled,
            **self._pfc_metrics,
        }

    def log_data(self, load_torque: Optional[float] = None) -> None:
        """
        Manual data logging (for external control loops).

        Call this periodically when using :meth:`step` with log_data=False.

        :param load_torque: Load torque to log. If None, uses current load profile value.
        :type load_torque: float, optional
        """
        if load_torque is None:
            load_torque = self.load_profile.get_torque(self.time)

        # Get last voltages (assume still applied)
        voltages = np.array(
            [
                self._history_voltages_a[-1] if self._history_voltages_a else 0.0,
                self._history_voltages_b[-1] if self._history_voltages_b else 0.0,
                self._history_voltages_c[-1] if self._history_voltages_c else 0.0,
            ]
        )

        # in manual logging we may not know supply; use current supply_profile value
        supply_v = self.supply_profile.get_voltage(self.time)
        self._log_data(voltages, load_torque, supply_v)

    def reset(self, initial_speed: float = 0.0) -> None:
        """
        Reset simulation to initial state.

        Clears all history and resets motor and load profile.

        :param initial_speed: Initial motor speed [rad/s], defaults to 0.0
        :type initial_speed: float
        """
        self.motor.reset(initial_speed)
        self.load_profile.reset()

        # Clear history
        self._history_times.clear()
        self._history_currents_a.clear()
        self._history_currents_b.clear()
        self._history_currents_c.clear()
        self._history_omega.clear()
        self._history_theta.clear()
        self._history_speed.clear()
        self._history_emf_a.clear()
        self._history_emf_b.clear()
        self._history_emf_c.clear()
        self._history_torque.clear()
        self._history_load_torque.clear()
        self._history_voltages_a.clear()
        self._history_voltages_b.clear()
        self._history_voltages_c.clear()
        self._history_supply_voltage.clear()
        self._history_input_current.clear()
        self._history_input_power.clear()
        self._history_pf.clear()
        self._history_pfc_command.clear()

        self._pfc_metrics.update(
            {
                "power_factor": 0.0,
                "active_power_w": 0.0,
                "apparent_power_va": 0.0,
                "reactive_power_var": 0.0,
                "compensation_command_var": 0.0,
                "target_pf": float(self.pfc_controller.target_pf)
                if self.pfc_controller
                else 0.0,
            }
        )

        if self.pfc_controller is not None:
            self.pfc_controller.reset()

        self.time = 0.0
        self.step_count = 0

    def get_history(self) -> Dict[str, np.ndarray]:
        """
        Get all simulation history as numpy arrays.

        :return: Dictionary with history data arrays
        :rtype: dict

        **Keys:**
            - 'time': Simulation time points [s]
            - 'currents_a': Phase A currents [A]
            - 'currents_b': Phase B currents [A]
            - 'currents_c': Phase C currents [A]
            - 'omega': Angular velocity [rad/s]
            - 'theta': Angular position [rad]
            - 'speed': Speed [RPM]
            - 'emf_a': Back-EMF phase A [V]
            - 'emf_b': Back-EMF phase B [V]
            - 'emf_c': Back-EMF phase C [V]
            - 'torque': Electromagnetic torque [N*m]
            - 'load_torque': Load torque [N*m]
            - 'voltages_a': Applied voltage phase A [V]
            - 'voltages_b': Applied voltage phase B [V]
            - 'voltages_c': Applied voltage phase C [V]
        """
        return {
            "time": np.array(self._history_times, dtype=np.float64),
            "currents_a": np.array(self._history_currents_a, dtype=np.float64),
            "currents_b": np.array(self._history_currents_b, dtype=np.float64),
            "currents_c": np.array(self._history_currents_c, dtype=np.float64),
            "omega": np.array(self._history_omega, dtype=np.float64),
            "theta": np.array(self._history_theta, dtype=np.float64),
            "speed": np.array(self._history_speed, dtype=np.float64),
            "emf_a": np.array(self._history_emf_a, dtype=np.float64),
            "emf_b": np.array(self._history_emf_b, dtype=np.float64),
            "emf_c": np.array(self._history_emf_c, dtype=np.float64),
            "torque": np.array(self._history_torque, dtype=np.float64),
            "load_torque": np.array(self._history_load_torque, dtype=np.float64),
            "voltages_a": np.array(self._history_voltages_a, dtype=np.float64),
            "voltages_b": np.array(self._history_voltages_b, dtype=np.float64),
            "voltages_c": np.array(self._history_voltages_c, dtype=np.float64),
            "supply_voltage": np.array(self._history_supply_voltage, dtype=np.float64),
            "input_current": np.array(self._history_input_current, dtype=np.float64),
            "input_power": np.array(self._history_input_power, dtype=np.float64),
            "power_factor": np.array(self._history_pf, dtype=np.float64),
            "pfc_command_var": np.array(self._history_pfc_command, dtype=np.float64),
        }

    def get_current_state(self) -> Dict[str, float]:
        """
        Get current motor state.

        :return: Current motor state variables
        :rtype: dict
        """
        return self.motor.get_state_dict()

    def get_simulation_info(self) -> Dict[str, Any]:
        """
        Get simulation metadata.

        :return: Simulation information
        :rtype: dict

        **Keys:**
            - 'time': Current simulation time [s]
            - 'step_count': Number of steps executed
            - 'dt': Time step [s]
            - 'history_size': Number of logged data points
        """
        return {
            "time": self.time,
            "step_count": self.step_count,
            "dt": self.dt,
            "history_size": len(self._history_times),
            "pfc": self.get_power_factor_control_state(),
        }
