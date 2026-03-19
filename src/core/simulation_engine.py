"""
Simulation Engine Module
========================

This module implements the main simulation loop that coordinates
motor model, control blocks, load profile, and data logging.

Provides efficient real-time simulation with optimized calculations.

:author: BLDC Control Team
:version: 0.8.0

.. versionadded:: 0.8.0
    Initial implementation of simulation engine
"""

import numpy as np
from typing import Optional, Dict, Any
from collections import deque
from .motor_model import BLDCMotor
from .load_model import LoadProfile
from src.hardware import HardwareInterface, InverterCurrentSense
from src.utils.compute_backend import resolve_compute_backend, as_dict
from .power_model import (
    SupplyProfile,
    ConstantSupply,
    PowerFactorController,
    compute_efficiency_metrics,
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
        hardware_interface: Optional[HardwareInterface] = None,
        current_sense: Optional[InverterCurrentSense] = None,
        compute_backend: str = "auto",
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
        self.hardware_interface = hardware_interface
        self.current_sense = current_sense
        self._hardware_enabled = hardware_interface is not None
        self._hardware_state: Dict[str, Any] = {
            "enabled": self._hardware_enabled,
            "connected": False,
            "backend": hardware_interface.name if hardware_interface else "none",
            "write_count": 0,
            "read_count": 0,
            "last_io_error": "",
            "last_feedback": {},
        }
        self._compute_backend_state = resolve_compute_backend(compute_backend)
        self._measured_phase_currents = np.zeros(3, dtype=np.float64)
        self._last_effective_voltages = np.zeros(3, dtype=np.float64)
        self._current_sense_state: Dict[str, Any] = {
            "enabled": self.current_sense is not None,
            "topology": self.current_sense.topology if self.current_sense else "none",
            "phase_voltage_drop_v": [0.0, 0.0, 0.0],
            "amplifier_outputs_v": [],
            "adc_saturated": [],
        }

        if self.hardware_interface is not None:
            try:
                self.hardware_interface.connect()
                self._hardware_state["connected"] = self.hardware_interface.is_connected
            except Exception as exc:
                self._hardware_state["enabled"] = False
                self._hardware_state["connected"] = False
                self._hardware_state["last_io_error"] = str(exc)
        self._pfc_metrics: Dict[str, float] = {
            "power_factor": 0.0,
            "active_power_w": 0.0,
            "apparent_power_va": 0.0,
            "reactive_power_var": 0.0,
            "compensation_command_var": 0.0,
            "target_pf": float(pfc_controller.target_pf) if pfc_controller else 0.0,
        }
        self._efficiency_metrics: Dict[str, float] = {
            "electrical_input_power_w": 0.0,
            "mechanical_output_power_w": 0.0,
            "regenerative_power_w": 0.0,
            "total_loss_power_w": 0.0,
            "efficiency": 0.0,
            "inverter_total_loss_power_w": 0.0,
        }
        self._inverter_metrics: Dict[str, float | int | bool] = {
            "effective_dc_voltage": 0.0,
            "dc_link_ripple_v": 0.0,
            "dc_link_bus_current_a": 0.0,
            "motor_terminal_power_w": 0.0,
            "device_loss_power_w": 0.0,
            "conduction_loss_power_w": 0.0,
            "switching_loss_power_w": 0.0,
            "dead_time_loss_power_w": 0.0,
            "diode_loss_power_w": 0.0,
            "total_inverter_loss_power_w": 0.0,
            "junction_temperature_c": 0.0,
            "common_mode_voltage": 0.0,
            "min_pulse_event_count": 0,
        }
        self._control_timing_metrics: Dict[str, float] = {
            "pwm_frequency_hz": 1.0 / self.dt,
            "control_period_s": self.dt,
            "calc_duration_s": 0.0,
            "calc_duration_avg_s": 0.0,
            "calc_duration_max_s": 0.0,
            "cpu_load_pct": 0.0,
            "cpu_load_avg_pct": 0.0,
            "sample_count": 0.0,
        }

        # Simulation state
        self.time = 0.0
        self.step_count = 0

        # History buffers (pre-allocated for efficiency)
        self._history_times: deque[float] = deque(maxlen=self.max_history)
        self._history_currents_a: deque[float] = deque(maxlen=self.max_history)
        self._history_currents_b: deque[float] = deque(maxlen=self.max_history)
        self._history_currents_c: deque[float] = deque(maxlen=self.max_history)
        self._history_omega: deque[float] = deque(maxlen=self.max_history)
        self._history_theta: deque[float] = deque(maxlen=self.max_history)
        self._history_speed: deque[float] = deque(maxlen=self.max_history)
        self._history_emf_a: deque[float] = deque(maxlen=self.max_history)
        self._history_emf_b: deque[float] = deque(maxlen=self.max_history)
        self._history_emf_c: deque[float] = deque(maxlen=self.max_history)
        self._history_torque: deque[float] = deque(maxlen=self.max_history)
        self._history_load_torque: deque[float] = deque(maxlen=self.max_history)
        self._history_voltages_a: deque[float] = deque(maxlen=self.max_history)
        self._history_voltages_b: deque[float] = deque(maxlen=self.max_history)
        self._history_voltages_c: deque[float] = deque(maxlen=self.max_history)
        self._history_supply_voltage: deque[float] = deque(maxlen=self.max_history)
        self._history_input_current: deque[float] = deque(maxlen=self.max_history)
        self._history_input_power: deque[float] = deque(maxlen=self.max_history)
        self._history_pf: deque[float] = deque(maxlen=self.max_history)
        self._history_pfc_command: deque[float] = deque(maxlen=self.max_history)
        self._history_mechanical_output_power: deque[float] = deque(
            maxlen=self.max_history
        )
        self._history_total_loss_power: deque[float] = deque(maxlen=self.max_history)
        self._history_efficiency: deque[float] = deque(maxlen=self.max_history)
        self._history_effective_dc_voltage: deque[float] = deque(
            maxlen=self.max_history
        )
        self._history_dc_link_ripple_v: deque[float] = deque(maxlen=self.max_history)
        self._history_dc_link_bus_current_a: deque[float] = deque(
            maxlen=self.max_history
        )
        self._history_device_loss_power: deque[float] = deque(maxlen=self.max_history)
        self._history_conduction_loss_power: deque[float] = deque(
            maxlen=self.max_history
        )
        self._history_switching_loss_power: deque[float] = deque(
            maxlen=self.max_history
        )
        self._history_dead_time_loss_power: deque[float] = deque(
            maxlen=self.max_history
        )
        self._history_diode_loss_power: deque[float] = deque(maxlen=self.max_history)
        self._history_inverter_total_loss_power: deque[float] = deque(
            maxlen=self.max_history
        )
        self._history_inverter_junction_temp_c: deque[float] = deque(
            maxlen=self.max_history
        )
        self._history_common_mode_voltage: deque[float] = deque(maxlen=self.max_history)
        self._history_min_pulse_event_count: deque[float] = deque(
            maxlen=self.max_history
        )
        self._history_control_calc_duration_s: deque[float] = deque(
            maxlen=self.max_history
        )
        self._history_control_cpu_load_pct: deque[float] = deque(
            maxlen=self.max_history
        )

        # State variables cache
        self._last_state_dict = {}

        # True physics phase-current history (parallel to measured _history_currents_*)
        self._history_currents_a_true: deque[float] = deque(maxlen=self.max_history)
        self._history_currents_b_true: deque[float] = deque(maxlen=self.max_history)
        self._history_currents_c_true: deque[float] = deque(maxlen=self.max_history)

    def set_inverter_telemetry(self, telemetry: Optional[Dict[str, Any]]) -> None:
        """Update latest inverter telemetry packet from the modulation stage."""
        if not telemetry:
            return
        self._inverter_metrics.update(dict(telemetry))

    def get_inverter_state(self) -> Dict[str, float | int | bool]:
        """Return latest inverter telemetry and runtime diagnostics."""
        return dict(self._inverter_metrics)

    def set_pwm_frequency(self, pwm_frequency_hz: float) -> None:
        """Configure PWM/control period metadata used for timing telemetry."""
        if pwm_frequency_hz <= 0.0:
            raise ValueError("pwm_frequency_hz must be positive")
        self._control_timing_metrics["pwm_frequency_hz"] = float(pwm_frequency_hz)
        self._control_timing_metrics["control_period_s"] = 1.0 / float(pwm_frequency_hz)

    def record_control_timing(
        self, calc_duration_s: float, control_period_s: Optional[float] = None
    ) -> None:
        """Record controller compute duration and update CPU-load estimates."""
        duration = max(float(calc_duration_s), 0.0)
        if control_period_s is not None:
            if control_period_s <= 0.0:
                raise ValueError("control_period_s must be positive")
            period = float(control_period_s)
            self._control_timing_metrics["control_period_s"] = period
            self._control_timing_metrics["pwm_frequency_hz"] = 1.0 / period
        else:
            period = max(float(self._control_timing_metrics["control_period_s"]), 1e-12)

        cpu_load_pct = 100.0 * duration / period
        count = self._control_timing_metrics["sample_count"] + 1.0
        avg_duration = self._control_timing_metrics["calc_duration_avg_s"]
        avg_cpu_load = self._control_timing_metrics["cpu_load_avg_pct"]

        self._control_timing_metrics["calc_duration_s"] = duration
        self._control_timing_metrics["calc_duration_avg_s"] = (
            avg_duration + (duration - avg_duration) / count
        )
        self._control_timing_metrics["calc_duration_max_s"] = max(
            self._control_timing_metrics["calc_duration_max_s"], duration
        )
        self._control_timing_metrics["cpu_load_pct"] = cpu_load_pct
        self._control_timing_metrics["cpu_load_avg_pct"] = (
            avg_cpu_load + (cpu_load_pct - avg_cpu_load) / count
        )
        self._control_timing_metrics["sample_count"] = count

    def get_control_timing_state(self) -> Dict[str, float]:
        """Return runtime control-timing and CPU-load telemetry."""
        return dict(self._control_timing_metrics)

    def set_compute_backend(self, backend: str) -> None:
        """Set compute backend policy: auto, cpu, or gpu."""
        self._compute_backend_state = resolve_compute_backend(backend)

    def get_compute_backend_state(self) -> Dict[str, Any]:
        """Return currently selected compute backend state."""
        return as_dict(self._compute_backend_state)

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

        effective_voltages = np.asarray(voltages, dtype=np.float64)

        if self.current_sense is not None:
            effective_voltages, phase_drop = (
                self.current_sense.apply_shunt_voltage_drop(
                    effective_voltages, self.motor.currents
                )
            )
            self._current_sense_state["phase_voltage_drop_v"] = phase_drop.tolist()

        if (
            self._hardware_enabled
            and self.hardware_interface is not None
            and self.hardware_interface.is_connected
        ):
            try:
                self.hardware_interface.write_phase_voltages(
                    effective_voltages, self.time
                )
                self._hardware_state["write_count"] += 1

                feedback = self.hardware_interface.read_feedback(self.time)
                self._hardware_state["read_count"] += 1
                self._hardware_state["last_feedback"] = dict(feedback)

                fb_voltages = feedback.get("applied_voltages")
                if fb_voltages is not None:
                    candidate = np.asarray(fb_voltages, dtype=np.float64)
                    if candidate.shape == (3,):
                        effective_voltages = candidate

                fb_load_torque = feedback.get("load_torque")
                if fb_load_torque is not None:
                    load_torque = float(fb_load_torque)

                self._hardware_state["connected"] = True
                self._hardware_state["last_io_error"] = ""
            except Exception as exc:
                # Fallback to pure simulation path if hardware I/O fails.
                self._hardware_state["connected"] = False
                self._hardware_state["last_io_error"] = str(exc)

        # Execute motor dynamics step
        self._last_effective_voltages = np.asarray(effective_voltages, dtype=np.float64)
        self.motor.step(effective_voltages, load_torque)

        if self.current_sense is not None:
            svm_sector = self.current_sense.sector_from_voltages(effective_voltages)
            sensed = self.current_sense.measure(
                self.motor.currents, self.dt, svm_sector=svm_sector
            )
            self._measured_phase_currents = np.asarray(
                sensed["currents_abc_measured"], dtype=np.float64
            )
            self._current_sense_state.update(
                {
                    "enabled": True,
                    "topology": sensed["topology"],
                    "amplifier_outputs_v": np.asarray(
                        sensed["amplifier_outputs_v"], dtype=np.float64
                    ).tolist(),
                    "adc_saturated": np.asarray(
                        sensed["adc_saturated"], dtype=bool
                    ).tolist(),
                }
            )
        else:
            self._measured_phase_currents = np.asarray(
                self.motor.currents, dtype=np.float64
            )

        # Log data if requested
        if log_data:
            self._log_data(effective_voltages, load_torque, supply_v)

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
        # Get motor state
        state = self.motor.get_state_dict()
        self._last_state_dict = state

        # Append to history
        self._history_times.append(self.time)
        self._history_currents_a.append(float(self._measured_phase_currents[0]))
        self._history_currents_b.append(float(self._measured_phase_currents[1]))
        self._history_currents_c.append(float(self._measured_phase_currents[2]))
        # True (physics) phase currents — always from motor ODE, never overridden
        self._history_currents_a_true.append(state["currents_a"])
        self._history_currents_b_true.append(state["currents_b"])
        self._history_currents_c_true.append(state["currents_c"])
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
        self._history_effective_dc_voltage.append(
            float(self._inverter_metrics.get("effective_dc_voltage", supply_voltage))
        )
        self._history_dc_link_ripple_v.append(
            float(self._inverter_metrics.get("dc_link_ripple_v", 0.0))
        )
        self._history_dc_link_bus_current_a.append(
            float(self._inverter_metrics.get("dc_link_bus_current_a", 0.0))
        )
        self._history_device_loss_power.append(
            float(self._inverter_metrics.get("device_loss_power_w", 0.0))
        )
        self._history_conduction_loss_power.append(
            float(self._inverter_metrics.get("conduction_loss_power_w", 0.0))
        )
        self._history_switching_loss_power.append(
            float(self._inverter_metrics.get("switching_loss_power_w", 0.0))
        )
        self._history_dead_time_loss_power.append(
            float(self._inverter_metrics.get("dead_time_loss_power_w", 0.0))
        )
        self._history_diode_loss_power.append(
            float(self._inverter_metrics.get("diode_loss_power_w", 0.0))
        )
        self._history_inverter_total_loss_power.append(
            float(self._inverter_metrics.get("total_inverter_loss_power_w", 0.0))
        )
        self._history_inverter_junction_temp_c.append(
            float(self._inverter_metrics.get("junction_temperature_c", 0.0))
        )
        self._history_common_mode_voltage.append(
            float(self._inverter_metrics.get("common_mode_voltage", 0.0))
        )
        self._history_min_pulse_event_count.append(
            float(self._inverter_metrics.get("min_pulse_event_count", 0.0))
        )
        self._history_control_calc_duration_s.append(
            float(self._control_timing_metrics.get("calc_duration_s", 0.0))
        )
        self._history_control_cpu_load_pct.append(
            float(self._control_timing_metrics.get("cpu_load_pct", 0.0))
        )

        i_abc = np.array(
            [state["currents_a"], state["currents_b"], state["currents_c"]],
            dtype=np.float64,
        )
        p_motor_terminal = float(np.dot(voltages, i_abc))
        inverter_loss = float(
            self._inverter_metrics.get("total_inverter_loss_power_w", 0.0)
        )
        p_instant = p_motor_terminal + inverter_loss
        if abs(supply_voltage) > 1e-9:
            i_in = p_instant / float(supply_voltage)
        else:
            i_in = 0.0

        self._history_input_power.append(p_instant)
        self._history_input_current.append(i_in)

        efficiency_metrics = compute_efficiency_metrics(
            input_power_w=p_instant,
            torque_nm=state["torque"],
            omega_rad_s=state["omega"],
        )
        efficiency_metrics["inverter_total_loss_power_w"] = inverter_loss
        self._efficiency_metrics.update(efficiency_metrics)
        self._history_mechanical_output_power.append(
            efficiency_metrics["mechanical_output_power_w"]
        )
        self._history_total_loss_power.append(efficiency_metrics["total_loss_power_w"])
        self._history_efficiency.append(efficiency_metrics["efficiency"])

        pf_now = self._pfc_metrics["power_factor"] if self._history_pf else 0.0
        cmd_var = self._pfc_metrics["compensation_command_var"]

        if len(self._history_input_current) >= 8:
            w = min(self.pfc_window_samples, len(self._history_input_current))
            recent_supply = np.array(
                list(self._history_supply_voltage)[-w:], dtype=np.float64
            )
            recent_input_current = np.array(
                list(self._history_input_current)[-w:], dtype=np.float64
            )
            metrics = compute_power_metrics(
                recent_supply,
                recent_input_current,
                backend=self._compute_backend_state.selected,
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

    def get_efficiency_state(self) -> Dict[str, float]:
        """Return current efficiency and loss telemetry."""
        return dict(self._efficiency_metrics)

    def configure_hardware_interface(self, enabled: bool) -> None:
        """Enable or disable hardware I/O execution path at runtime."""
        self._hardware_enabled = bool(enabled) and self.hardware_interface is not None
        self._hardware_state["enabled"] = self._hardware_enabled

        if self.hardware_interface is None:
            self._hardware_state["connected"] = False
            return

        if self._hardware_enabled and not self.hardware_interface.is_connected:
            try:
                self.hardware_interface.connect()
                self._hardware_state["connected"] = self.hardware_interface.is_connected
                self._hardware_state["last_io_error"] = ""
            except Exception as exc:
                self._hardware_state["connected"] = False
                self._hardware_state["last_io_error"] = str(exc)
        elif not self._hardware_enabled and self.hardware_interface.is_connected:
            self.hardware_interface.disconnect()
            self._hardware_state["connected"] = False

    def get_hardware_state(self) -> Dict[str, Any]:
        """Return hardware backend execution status and diagnostics."""
        return dict(self._hardware_state)

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
        self._history_mechanical_output_power.clear()
        self._history_total_loss_power.clear()
        self._history_efficiency.clear()
        self._history_effective_dc_voltage.clear()
        self._history_dc_link_ripple_v.clear()
        self._history_dc_link_bus_current_a.clear()
        self._history_device_loss_power.clear()
        self._history_conduction_loss_power.clear()
        self._history_switching_loss_power.clear()
        self._history_dead_time_loss_power.clear()
        self._history_diode_loss_power.clear()
        self._history_inverter_total_loss_power.clear()
        self._history_inverter_junction_temp_c.clear()
        self._history_common_mode_voltage.clear()
        self._history_min_pulse_event_count.clear()
        self._history_control_calc_duration_s.clear()
        self._history_control_cpu_load_pct.clear()

        self._history_currents_a_true.clear()
        self._history_currents_b_true.clear()
        self._history_currents_c_true.clear()

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
        self._efficiency_metrics.update(
            {
                "electrical_input_power_w": 0.0,
                "mechanical_output_power_w": 0.0,
                "regenerative_power_w": 0.0,
                "total_loss_power_w": 0.0,
                "efficiency": 0.0,
                "inverter_total_loss_power_w": 0.0,
            }
        )
        self._inverter_metrics.update(
            {
                "effective_dc_voltage": 0.0,
                "dc_link_ripple_v": 0.0,
                "dc_link_bus_current_a": 0.0,
                "motor_terminal_power_w": 0.0,
                "device_loss_power_w": 0.0,
                "conduction_loss_power_w": 0.0,
                "switching_loss_power_w": 0.0,
                "dead_time_loss_power_w": 0.0,
                "diode_loss_power_w": 0.0,
                "total_inverter_loss_power_w": 0.0,
                "junction_temperature_c": 0.0,
                "common_mode_voltage": 0.0,
                "min_pulse_event_count": 0,
            }
        )
        self._control_timing_metrics.update(
            {
                "pwm_frequency_hz": 1.0 / self.dt,
                "control_period_s": self.dt,
                "calc_duration_s": 0.0,
                "calc_duration_avg_s": 0.0,
                "calc_duration_max_s": 0.0,
                "cpu_load_pct": 0.0,
                "cpu_load_avg_pct": 0.0,
                "sample_count": 0.0,
            }
        )

        if self.pfc_controller is not None:
            self.pfc_controller.reset()

        if self.hardware_interface is not None:
            self._hardware_state["connected"] = self.hardware_interface.is_connected
            self._hardware_state["last_feedback"] = {}
            self._hardware_state["last_io_error"] = ""

        self.time = 0.0
        self.step_count = 0
        self._last_effective_voltages = np.zeros(3, dtype=np.float64)

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
            "currents_a_true": np.array(
                self._history_currents_a_true, dtype=np.float64
            ),
            "currents_b_true": np.array(
                self._history_currents_b_true, dtype=np.float64
            ),
            "currents_c_true": np.array(
                self._history_currents_c_true, dtype=np.float64
            ),
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
            "effective_dc_voltage": np.array(
                self._history_effective_dc_voltage, dtype=np.float64
            ),
            "dc_link_ripple_v": np.array(
                self._history_dc_link_ripple_v, dtype=np.float64
            ),
            "dc_link_bus_current_a": np.array(
                self._history_dc_link_bus_current_a, dtype=np.float64
            ),
            "input_current": np.array(self._history_input_current, dtype=np.float64),
            "input_power": np.array(self._history_input_power, dtype=np.float64),
            "power_factor": np.array(self._history_pf, dtype=np.float64),
            "pfc_command_var": np.array(self._history_pfc_command, dtype=np.float64),
            "device_loss_power": np.array(
                self._history_device_loss_power, dtype=np.float64
            ),
            "conduction_loss_power": np.array(
                self._history_conduction_loss_power, dtype=np.float64
            ),
            "switching_loss_power": np.array(
                self._history_switching_loss_power, dtype=np.float64
            ),
            "dead_time_loss_power": np.array(
                self._history_dead_time_loss_power, dtype=np.float64
            ),
            "diode_loss_power": np.array(
                self._history_diode_loss_power, dtype=np.float64
            ),
            "inverter_total_loss_power": np.array(
                self._history_inverter_total_loss_power, dtype=np.float64
            ),
            "inverter_junction_temp_c": np.array(
                self._history_inverter_junction_temp_c, dtype=np.float64
            ),
            "common_mode_voltage": np.array(
                self._history_common_mode_voltage, dtype=np.float64
            ),
            "min_pulse_event_count": np.array(
                self._history_min_pulse_event_count, dtype=np.float64
            ),
            "control_calc_duration_s": np.array(
                self._history_control_calc_duration_s, dtype=np.float64
            ),
            "control_cpu_load_pct": np.array(
                self._history_control_cpu_load_pct, dtype=np.float64
            ),
            "mechanical_output_power": np.array(
                self._history_mechanical_output_power, dtype=np.float64
            ),
            "total_loss_power": np.array(
                self._history_total_loss_power, dtype=np.float64
            ),
            "efficiency": np.array(self._history_efficiency, dtype=np.float64),
        }

    def get_current_state(self) -> Dict[str, float]:
        """
        Get current motor state.

        :return: Current motor state variables
        :rtype: dict
        """
        state = self.motor.get_state_dict()
        if self.current_sense is not None:
            state["currents_a_true"] = state["currents_a"]
            state["currents_b_true"] = state["currents_b"]
            state["currents_c_true"] = state["currents_c"]
            state["currents_a"] = float(self._measured_phase_currents[0])
            state["currents_b"] = float(self._measured_phase_currents[1])
            state["currents_c"] = float(self._measured_phase_currents[2])
            state["current_measurement"] = dict(self._current_sense_state)
        state["voltages_a"] = float(self._last_effective_voltages[0])
        state["voltages_b"] = float(self._last_effective_voltages[1])
        state["voltages_c"] = float(self._last_effective_voltages[2])
        return state

    def get_controller_phase_currents(self) -> np.ndarray:
        """Return phase currents as seen by the control algorithm."""
        if self.current_sense is not None:
            return np.asarray(self._measured_phase_currents, dtype=np.float64)
        return np.asarray(self.motor.currents, dtype=np.float64)

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
            "efficiency_metrics": self.get_efficiency_state(),
            "inverter": self.get_inverter_state(),
            "control_timing": self.get_control_timing_state(),
            "compute_backend": self.get_compute_backend_state(),
            "hardware": self.get_hardware_state(),
            "current_measurement": dict(self._current_sense_state),
        }
