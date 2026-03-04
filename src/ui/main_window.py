"""
Main GUI Window Module
======================

Main application window for BLDC motor control simulator.

Provides comprehensive GUI for:
- Motor parameter configuration
- Load profile definition
- Control and monitoring
- Real-time plotting
- Data export

:author: BLDC Control Team
:version: 1.0.0
"""

import numpy as np
from typing import Optional
import time
import logging

from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QFileDialog,
    QTableWidget,
    QTableWidgetItem,
    QScrollArea,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, QAccessible
from PyQt6.QtGui import QFont

from src.ui.widgets.accessible_widgets import (
    LabeledSpinBox,
    LabeledComboBox,
    AccessibleButton,
    AccessibleGroupBox,
    AccessibleTabWidget,
    AccessibleTableWidget,
)
from src.core import (
    BLDCMotor,
    MotorParameters,
    SimulationEngine,
    ConstantLoad,
    RampLoad,
)
from src.control import SVMGenerator, VFController, BaseController, FOCController
from src.control.transforms import inverse_clarke
from src.utils.config import (
    DEFAULT_MOTOR_PARAMS,
    SIMULATION_PARAMS,
    VF_CONTROLLER_PARAMS,
    PLOT_STYLE,
    DEFAULT_LOAD_PROFILE,
)
from src.utils.speech import speak
from src.utils.data_logger import DataLogger
from src.visualization.visualization import SimulationPlotter

logger = logging.getLogger(__name__)


class SimulationThread(QThread):
    """Background simulation thread."""

    update_signal = pyqtSignal(dict)  # Emit current state
    finished_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.running = False
        self.engine: Optional[SimulationEngine] = None
        self.svm: Optional[SVMGenerator] = None
        # controller may be VFController or FOCController or any BaseController
        self.controller: Optional[BaseController] = None
        self.update_interval = 0.1  # Update GUI every 100ms
        self.max_duration = 0.0  # 0 = infinite
        # start flag will be set when set_simulation is called

    def set_simulation(
        self,
        engine: SimulationEngine,
        svm: SVMGenerator,
        controller: BaseController,
        max_duration: float = 0.0,
    ):
        """Assign engine, svm and controller before running thread."""
        self.engine = engine
        self.svm = svm
        self.controller = controller
        self.max_duration = max_duration

    def start_simulation(self):
        """Flag thread to begin executing the simulation loop."""
        self.running = True
        if not self.isRunning():
            self.start()

    def stop_simulation(self):
        """Stop simulation loop."""
        self.running = False

    def run(self):
        """Main simulation loop."""
        if not self.engine or not self.svm or not self.controller:
            return

        dt = self.engine.dt
        last_update = time.time()
        step_count = 0
        sim_start_time = self.engine.time

        while self.running:
            # Check if max duration reached (only if > 0)
            if (
                self.max_duration > 0
                and (self.engine.time - sim_start_time) >= self.max_duration
            ):
                break

            # update supply voltage on svm from engine
            supply_v = self.engine.supply_profile.get_voltage(self.engine.time)
            if self.svm:
                try:
                    self.svm.set_dc_voltage(supply_v)
                except AttributeError:
                    pass

            # Get control output (could be polar or cartesian)
            ctrl_out = self.controller.update(dt)
            voltages = None
            if isinstance(ctrl_out, tuple) and len(ctrl_out) == 2:
                a, b = ctrl_out
                # determine whether cartesian
                if hasattr(self.controller, "output_cartesian") and getattr(
                    self.controller, "output_cartesian"
                ):
                    # use cartesian modulate if available
                    try:
                        voltages = self.svm.modulate_cartesian(valpha=a, vbeta=b)
                    except Exception:
                        # fallback to manual inverse clarke
                        va, vb, vc = inverse_clarke(a, b)
                        voltages = np.array([va, vb, vc])
                else:
                    # polar
                    voltages = self.svm.modulate(a, b)
            else:
                raise ValueError("Controller returned unexpected output format")
            # Execute motor step
            self.engine.step(voltages, log_data=True)

            step_count += 1

            # Periodic GUI update
            current_time = time.time()
            if (current_time - last_update) >= self.update_interval:
                state = self.engine.get_current_state()
                info = self.engine.get_simulation_info()
                self.update_signal.emit({**state, **info})
                last_update = current_time

            # Real-time efficiency: small sleep to prevent 100% CPU
            time.sleep(dt / 4)  # Sleep for 1/4 of simulation step

        self.finished_signal.emit()


class BLDCMotorControlGUI(QMainWindow):
    """
    Main BLDC Motor Control GUI
    ===========================

    Comprehensive application for BLDC motor simulation and control.

    Features:
    - Screen reader accessible
    - Real-time motor simulation
    - V/f speed control
    - Data logging and export
    - Visualization tools
    """

    def __init__(self):
        super().__init__()

        # Fun application name and version
        APP_NAME = "⚡ SPIN DOCTOR"
        APP_VERSION = "2.0.0"

        self.setWindowTitle(
            f"{APP_NAME} - BLDC Motor Control Simulator (v{APP_VERSION})"
        )
        self.setGeometry(100, 100, 1500, 950)

        # Accessibility
        self.setAccessibleName("SPIN DOCTOR - BLDC Motor Control Application")
        self.setAccessibleDescription(
            "Comprehensive BLDC motor simulator with V/f and FOC control. "
            "Use Tab to navigate between sections, arrow keys in list views."
        )

        # Simulation components
        self.motor: Optional[BLDCMotor] = None
        self.engine: Optional[SimulationEngine] = None
        self.svm: Optional[SVMGenerator] = None
        self.controller: Optional[VFController] = None
        self.sim_thread: Optional[SimulationThread] = None
        self.logger = DataLogger()

        # UI state
        self.is_running = False

        # Speed curve history for live plotting
        self.speed_history_time = []
        self.speed_history_rpm = []

        # Create UI
        self._create_ui()
        self._initialize_defaults()

        # Timer for GUI updates when not using thread
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_display)

    def _create_ui(self):
        """Create main UI layout."""
        # Create menu bar
        self._create_menu_bar()
        # Central widget with permanent info panel (left: main UI, right: info)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout()

        # Left column: title, tabs, buttons
        left_layout = QVBoxLayout()

        title = QLabel("⚡ SPIN DOCTOR - Advanced BLDC Motor Control Simulator")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAccessibleName("Application Title")
        left_layout.addWidget(title)

        # Tab widget
        self.tabs = AccessibleTabWidget()
        self._create_parameters_tab()
        self._create_load_tab()
        self._create_supply_tab()
        self._create_control_tab()
        self._create_monitoring_tab()
        self._create_plotting_tab()

        left_layout.addWidget(self.tabs, 1)

        # Control buttons
        button_layout = QHBoxLayout()

        self.btn_start = AccessibleButton(
            "Start Simulation (F5)",
            "Begin BLDC motor simulation with current parameters",
        )
        self.btn_start.setShortcut("F5")
        self.btn_start.clicked.connect(self._start_simulation)
        button_layout.addWidget(self.btn_start)

        self.btn_stop = AccessibleButton(
            "Stop Simulation (F6)", "Stop running simulation"
        )
        self.btn_stop.setShortcut("F6")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self._stop_simulation)
        button_layout.addWidget(self.btn_stop)

        self.btn_reset = AccessibleButton(
            "Reset (F7)", "Reset simulation to initial state"
        )
        self.btn_reset.setShortcut("F7")
        self.btn_reset.clicked.connect(self._reset_simulation)
        button_layout.addWidget(self.btn_reset)

        btn_export = AccessibleButton(
            "Export Data (Ctrl+S)", "Save simulation results to CSV and metadata"
        )
        btn_export.setShortcut("Ctrl+S")
        btn_export.clicked.connect(self._export_data)
        button_layout.addWidget(btn_export)

        button_layout.addStretch()
        left_layout.addLayout(button_layout)

        main_layout.addLayout(left_layout, 3)

        # Right column: permanent info panel
        info_group = AccessibleGroupBox(
            "Quick Info", "Persistent simulation parameters and units"
        )
        info_layout = QVBoxLayout()

        # dt and time constants
        self.lbl_dt = QLabel("dt: -- s")
        self.lbl_dt.setAccessibleName("Time step")
        info_layout.addWidget(self.lbl_dt)

        self.lbl_tau_e = QLabel("Electrical time constant (L/R): -- s")
        self.lbl_tau_e.setAccessibleName("Electrical time constant")
        info_layout.addWidget(self.lbl_tau_e)

        self.lbl_tau_m = QLabel("Mechanical time constant (J/b): -- s")
        self.lbl_tau_m.setAccessibleName("Mechanical time constant")
        info_layout.addWidget(self.lbl_tau_m)

        # Parameter units summary
        self.lbl_param_units = QLabel()
        self.lbl_param_units.setWordWrap(True)
        self.lbl_param_units.setAccessibleName("Parameter units summary")
        info_layout.addWidget(self.lbl_param_units)

        # Ld / Lq quick entry note will be below parameters tab; show current values here
        self.lbl_ld_lq = QLabel("Ld: -- H, Lq: -- H")
        self.lbl_ld_lq.setAccessibleName("Ld and Lq values")
        info_layout.addWidget(self.lbl_ld_lq)

        info_layout.addStretch()
        info_group.setLayout(info_layout)
        main_layout.addWidget(info_group, 1)

        central_widget.setLayout(main_layout)

        # Add status bar with simulation parameters
        self.status_bar_dt = QLabel("dt: -- s")
        self.status_bar_tau_e = QLabel("τ_e: -- s")
        self.status_bar_tau_m = QLabel("τ_m: -- s")

        self.statusBar().addWidget(self.status_bar_dt)
        self.statusBar().addWidget(QLabel("|"))  # Separator
        self.statusBar().addWidget(self.status_bar_tau_e)
        self.statusBar().addWidget(QLabel("|"))  # Separator
        self.statusBar().addWidget(self.status_bar_tau_m)

    def _create_menu_bar(self):
        """Create application menu bar with File, Tools, and Help menus."""
        menubar = self.menuBar()

        # File Menu (mnemonic Alt+F)
        file_menu = menubar.addMenu("&File")

        export_action = file_menu.addAction("&Export Simulation Data")
        export_action.setShortcut("Ctrl+S")
        export_action.triggered.connect(self._export_data)

        file_menu.addSeparator()

        params_action = file_menu.addAction("&Simulation Parameters")
        params_action.triggered.connect(self._show_simulation_params)

        file_menu.addSeparator()

        quit_action = file_menu.addAction("&Quit")
        quit_action.setShortcut("Ctrl+Q")
        quit_action.triggered.connect(self.close)

        # Tools Menu (mnemonic Alt+T)
        tools_menu = menubar.addMenu("&Tools")
        reset_action = tools_menu.addAction("Reset Simulation (F7)")
        reset_action.setShortcut("F7")
        reset_action.triggered.connect(self._reset_simulation)

        # Help Menu (mnemonic Alt+H)
        help_menu = menubar.addMenu("&Help")
        about_action = help_menu.addAction("About SPIN DOCTOR")
        about_action.triggered.connect(self._show_about)

        guide_action = help_menu.addAction("Quick Start Guide")
        guide_action.triggered.connect(self._show_guide)

    def _show_simulation_params(self):
        """Show current simulation time step and motor time constants."""
        msg = self.get_simulation_params_info()
        QMessageBox.information(self, "Simulation Parameters", msg)
        speak("Simulation parameters displayed.")

    def get_simulation_params_info(self) -> str:
        """Return a string with current dt and motor time constants (for testing)."""
        # Determine dt
        if self.engine:
            dt_val = self.engine.dt
            motor_params = self.engine.motor.params
        else:
            from src.core.motor_model import MotorParameters

            dt_val = SIMULATION_PARAMS.get("dt", 0.0001)
            motor_params = MotorParameters()

        # Electrical time constant (L/R) and mechanical (J/b)
        try:
            tau_e = motor_params.phase_inductance / motor_params.phase_resistance
        except Exception:
            tau_e = None

        try:
            tau_m = motor_params.rotor_inertia / motor_params.friction_coefficient
        except Exception:
            tau_m = None

        msg = f"Simulation time step (dt): {dt_val} s\n"
        if tau_e is not None:
            msg += f"Electrical time constant (L/R): {tau_e:.6f} s\n"
        else:
            msg += "Electrical time constant (L/R): n/a\n"
        if tau_m is not None:
            msg += f"Mechanical time constant (J/b): {tau_m:.6f} s\n"
        else:
            msg += "Mechanical time constant (J/b): n/a\n"

        return msg

    def _show_about(self):
        """Show about dialog."""
        about_text = (
            "<h2>⚡ SPIN DOCTOR v2.0.0</h2>"
            "<p><b>Advanced BLDC Motor Control Simulator</b></p>"
            "<p>A comprehensive tool for simulating and controlling Brushless DC motors with:</p>"
            "<ul>"
            "<li>V/f (Voltage-to-Frequency) control</li>"
            "<li>FOC (Field-Oriented Control) algorithm</li>"
            "<li>Clarke/Concordia coordinate transforms</li>"
            "<li>SVM (Space Vector Modulation) generation</li>"
            "<li>Real-time monitoring and visualization</li>"
            "<li>Screen reader accessibility</li>"
            "</ul>"
            "<p><b>Author:</b> BLDC Control Team</p>"
            "<p><b>Version:</b> 2.0.0</p>"
        )
        QMessageBox.about(self, "About SPIN DOCTOR", about_text)

    def _show_guide(self):
        """Show quick start guide."""
        guide_text = (
            "<h3>Quick Start Guide - SPIN DOCTOR</h3>"
            "<p><b>1. Configure Motor Parameters:</b></p>"
            "<ul><li>Set nominal voltage, resistance, inductance, pole pairs, Back-EMF constant</li></ul>"
            "<p><b>2. Set Load Profile:</b></p>"
            "<ul><li>Choose load type (Constant, Ramp, Inertial) and configure parameters</li></ul>"
            "<p><b>3. Setup Supply Profile (Optional):</b></p>"
            "<ul><li>Select supply type and voltage variation parameters</li></ul>"
            "<p><b>4. Select Control Mode:</b></p>"
            "<ul><li>Choose V/f or FOC control, configure parameters</li></ul>"
            "<p><b>5. Set Simulation Duration:</b></p>"
            "<ul><li>Enter duration (0 = infinite) and click Start (F5)</li></ul>"
            "<p><b>6. Monitor & Analyze:</b></p>"
            "<ul><li>Watch Monitoring tab for real-time values</li>"
            "<li>Use Plotting tab to visualize results</li></ul>"
            "<p><b>7. Export Results:</b></p>"
            "<ul><li>Use File → Export or Ctrl+S to save data</li></ul>"
        )
        QMessageBox.information(self, "Quick Start Guide", guide_text)

    def _create_parameters_tab(self):
        """Create motor parameters configuration tab."""
        widget = QWidget()
        layout = QVBoxLayout()

        # Motor parameters group
        group = AccessibleGroupBox(
            "Motor Parameters",
            "Configure BLDC motor specifications. Default values from literature.",
        )
        group_layout = QVBoxLayout()

        self.param_voltage = LabeledSpinBox(
            "Nominal Voltage",
            min_val=12,
            max_val=300,
            initial=DEFAULT_MOTOR_PARAMS["nominal_voltage"],
            step=1,
            decimals=1,
            suffix=" V",
            description="Motor nominal DC voltage. Typical: 24-48V for hobby, 48-400V for industrial.",
        )
        group_layout.addWidget(self.param_voltage)

        self.param_resistance = LabeledSpinBox(
            "Phase Resistance",
            min_val=0.1,
            max_val=50,
            initial=DEFAULT_MOTOR_PARAMS["phase_resistance"],
            step=0.1,
            decimals=3,
            suffix=" Ω",
            description="Winding resistance per phase. Affects current and losses.",
        )
        group_layout.addWidget(self.param_resistance)

        self.param_inductance = LabeledSpinBox(
            "Phase Inductance",
            min_val=0.0001,
            max_val=0.1,
            initial=DEFAULT_MOTOR_PARAMS["phase_inductance"],
            step=0.0001,
            decimals=5,
            suffix=" H",
            description="Winding inductance per phase. Affects current rise time.",
        )
        group_layout.addWidget(self.param_inductance)

        # d/q inductances (Ld/Lq) for FOC users
        self.param_ld = LabeledSpinBox(
            "Ld (d-axis inductance)",
            min_val=0.00005,
            max_val=0.2,
            initial=DEFAULT_MOTOR_PARAMS.get("phase_inductance", 0.005),
            step=0.00001,
            decimals=6,
            suffix=" H",
            description="d-axis inductance. Use for FOC models (Ld).",
        )
        group_layout.addWidget(self.param_ld)

        self.param_lq = LabeledSpinBox(
            "Lq (q-axis inductance)",
            min_val=0.00005,
            max_val=0.2,
            initial=DEFAULT_MOTOR_PARAMS.get("phase_inductance", 0.005),
            step=0.00001,
            decimals=6,
            suffix=" H",
            description="q-axis inductance. Use for FOC models (Lq).",
        )
        group_layout.addWidget(self.param_lq)

        self.param_emf = LabeledSpinBox(
            "Back-EMF Constant",
            min_val=0.01,
            max_val=1.0,
            initial=DEFAULT_MOTOR_PARAMS["back_emf_constant"],
            step=0.01,
            decimals=4,
            suffix=" V·s/rad",
            description="Back-EMF voltage per angular velocity unit.",
        )
        group_layout.addWidget(self.param_emf)

        self.param_kt = LabeledSpinBox(
            "Torque Constant",
            min_val=0.01,
            max_val=1.0,
            initial=DEFAULT_MOTOR_PARAMS["torque_constant"],
            step=0.01,
            decimals=4,
            suffix=" N·m/A",
            description="Torque produced per unit current.",
        )
        group_layout.addWidget(self.param_kt)

        self.param_inertia = LabeledSpinBox(
            "Rotor Inertia",
            min_val=0.00001,
            max_val=0.01,
            initial=DEFAULT_MOTOR_PARAMS["rotor_inertia"],
            step=0.00001,
            decimals=6,
            suffix=" kg·m²",
            description="Moment of inertia. Affects acceleration response.",
        )
        group_layout.addWidget(self.param_inertia)

        self.param_friction = LabeledSpinBox(
            "Friction Coefficient",
            min_val=0,
            max_val=0.1,
            initial=DEFAULT_MOTOR_PARAMS["friction_coefficient"],
            step=0.001,
            decimals=5,
            suffix=" N·m·s/rad",
            description="Viscous friction damping.",
        )
        group_layout.addWidget(self.param_friction)

        self.param_poles = LabeledSpinBox(
            "Number of Poles",
            min_val=2,
            max_val=20,
            initial=DEFAULT_MOTOR_PARAMS["num_poles"],
            step=1,
            decimals=0,
            suffix="",
            description="Total magnetic poles in motor (must be even).",
        )
        group_layout.addWidget(self.param_poles)

        self.param_model_type = LabeledComboBox(
            "Motor Model",
            items=["scalar", "dq"],
            description="Select motor model type. 'scalar' for 3-phase model, 'dq' for d-q axis model.",
        )
        group_layout.addWidget(self.param_model_type)

        self.param_emf_shape = LabeledComboBox(
            "Back-EMF Shape",
            items=["trapezoidal", "sinusoidal"],
            description="Select back-EMF waveform shape. 'trapezoidal' for BLDC, 'sinusoidal' for PMSM.",
        )
        group_layout.addWidget(self.param_emf_shape)

        group.setLayout(group_layout)
        layout.addWidget(group)
        layout.addStretch()

        widget.setLayout(layout)
        self.tabs.addTab(widget, "Motor Parameters")

    def _create_load_tab(self):
        """Create load profile configuration tab."""
        widget = QWidget()
        layout = QVBoxLayout()

        group = AccessibleGroupBox(
            "Load Profile", "Define mechanical load applied to motor shaft."
        )
        group_layout = QVBoxLayout()

        self.load_type = LabeledComboBox(
            "Load Type",
            items=["Constant", "Ramp", "Variable"],
            description="Select load profile type. Constant: fixed torque. "
            "Ramp: linear increase. Variable: custom profile.",
        )
        self.load_type.currentTextChanged.connect(self._on_load_type_changed)
        group_layout.addWidget(self.load_type)

        self.load_constant_torque = LabeledSpinBox(
            "Constant Load Torque",
            min_val=0,
            max_val=10,
            initial=DEFAULT_LOAD_PROFILE["torque"],
            step=0.1,
            decimals=2,
            suffix=" N·m",
            description="Constant load torque applied to motor shaft.",
        )
        group_layout.addWidget(self.load_constant_torque)

        self.load_initial_torque = LabeledSpinBox(
            "Initial Ramp Torque",
            min_val=0,
            max_val=10,
            initial=DEFAULT_LOAD_PROFILE["initial_torque"],
            step=0.1,
            decimals=2,
            suffix=" N·m",
            description="Starting load torque for ramp profile.",
        )
        group_layout.addWidget(self.load_initial_torque)

        self.load_final_torque = LabeledSpinBox(
            "Final Ramp Torque",
            min_val=0,
            max_val=10,
            initial=DEFAULT_LOAD_PROFILE["final_torque"],
            step=0.1,
            decimals=2,
            suffix=" N·m",
            description="Ending load torque for ramp profile.",
        )
        group_layout.addWidget(self.load_final_torque)

        self.load_ramp_duration = LabeledSpinBox(
            "Ramp Duration",
            min_val=0.1,
            max_val=10,
            initial=DEFAULT_LOAD_PROFILE["ramp_duration"],
            step=0.1,
            decimals=2,
            suffix=" s",
            description="Time to complete load ramp (affects ramp slope).",
        )
        group_layout.addWidget(self.load_ramp_duration)

        group.setLayout(group_layout)
        layout.addWidget(group)
        layout.addStretch()

        widget.setLayout(layout)
        self.tabs.addTab(widget, "Load Profile")
        # also create supply tab right after load
        self._create_supply_tab()

    def _create_supply_tab(self):
        """Create power supply configuration tab."""
        widget = QWidget()
        layout = QVBoxLayout()

        group = AccessibleGroupBox(
            "Supply Profile", "Define DC bus voltage profile feeding the inverter."
        )
        group_layout = QVBoxLayout()

        self.supply_type = LabeledComboBox(
            "Profile Type",
            items=["Constant", "Ramp"],
            description="Select supply voltage profile type. Variable/custom profiles not implemented yet.",
        )
        self.supply_type.currentTextChanged.connect(self._on_supply_type_changed)
        group_layout.addWidget(self.supply_type)

        self.supply_constant_voltage = LabeledSpinBox(
            "Constant Voltage",
            min_val=0,
            max_val=500,
            initial=SIMULATION_PARAMS.get("dc_voltage", 48.0),
            step=1,
            decimals=1,
            suffix=" V",
            description="Fixed DC bus voltage.",
        )
        group_layout.addWidget(self.supply_constant_voltage)

        self.supply_ramp_initial = LabeledSpinBox(
            "Initial Voltage",
            min_val=0,
            max_val=500,
            initial=SIMULATION_PARAMS.get("dc_voltage", 48.0),
            step=1,
            decimals=1,
            suffix=" V",
            description="Start voltage for ramp profile.",
        )
        group_layout.addWidget(self.supply_ramp_initial)

        self.supply_ramp_final = LabeledSpinBox(
            "Final Voltage",
            min_val=0,
            max_val=500,
            initial=SIMULATION_PARAMS.get("dc_voltage", 48.0),
            step=1,
            decimals=1,
            suffix=" V",
            description="End voltage for ramp profile.",
        )
        group_layout.addWidget(self.supply_ramp_final)

        self.supply_ramp_duration = LabeledSpinBox(
            "Ramp Duration",
            min_val=0.1,
            max_val=60,
            initial=1.0,
            step=0.1,
            decimals=2,
            suffix=" s",
            description="Time to complete voltage ramp.",
        )
        group_layout.addWidget(self.supply_ramp_duration)

        group.setLayout(group_layout)
        layout.addWidget(group)
        layout.addStretch()
        widget.setLayout(layout)
        self.tabs.addTab(widget, "Supply Profile")

    def _on_supply_type_changed(self, text: str) -> None:
        """Show/hide appropriate supply parameters."""
        if text == "Constant":
            self.supply_constant_voltage.setVisible(True)
            self.supply_ramp_initial.setVisible(False)
            self.supply_ramp_final.setVisible(False)
            self.supply_ramp_duration.setVisible(False)
        else:
            self.supply_constant_voltage.setVisible(False)
            self.supply_ramp_initial.setVisible(True)
            self.supply_ramp_final.setVisible(True)
            self.supply_ramp_duration.setVisible(True)

    def _create_control_tab(self):
        """Create controller configuration tab with mode selection."""
        widget = QWidget()
        layout = QVBoxLayout()

        # Simulation duration control
        duration_group = AccessibleGroupBox(
            "Simulation Duration", "Set how long the simulation runs (0 = infinite)"
        )
        duration_layout = QVBoxLayout()

        self.sim_duration = LabeledSpinBox(
            "Duration",
            min_val=0,
            max_val=300,
            initial=10.0,
            step=0.5,
            decimals=1,
            suffix=" s",
            description="Simulation runtime in seconds. Set to 0 for infinite/continuous simulation.",
        )
        duration_layout.addWidget(self.sim_duration)

        info_label = QLabel(
            "💡 Tip: Set to 0 seconds for infinite simulation (run until you press Stop)"
        )
        info_label.setWordWrap(True)
        duration_layout.addWidget(info_label)

        duration_group.setLayout(duration_layout)
        layout.addWidget(duration_group)

        # allow choice between control algorithms
        self.ctrl_mode = LabeledComboBox(
            "Control Mode",
            items=["V/f", "FOC"],
            description="Select the control algorithm to use for simulation.",
        )
        self.ctrl_mode.currentTextChanged.connect(self._on_control_mode_changed)
        layout.addWidget(self.ctrl_mode)

        # V/f parameters group (shown when mode == "V/f")
        self.vf_group = AccessibleGroupBox(
            "V/f Speed Controller", "Configure Voltage-to-Frequency control algorithm."
        )
        self.vf_group_layout = QVBoxLayout()

        self.vf_v_nominal = LabeledSpinBox(
            "Nominal Voltage",
            min_val=1,
            max_val=100,
            initial=VF_CONTROLLER_PARAMS["v_nominal"],
            step=1,
            decimals=1,
            suffix=" V",
            description="Motor rated voltage (volts at nominal frequency).",
        )
        self.vf_group_layout.addWidget(self.vf_v_nominal)

        self.vf_f_nominal = LabeledSpinBox(
            "Nominal Frequency",
            min_val=1,
            max_val=500,
            initial=VF_CONTROLLER_PARAMS["f_nominal"],
            step=1,
            decimals=1,
            suffix=" Hz",
            description="Motor rated frequency (frequency at rated voltage).",
        )
        self.vf_group_layout.addWidget(self.vf_f_nominal)

        self.vf_startup_voltage = LabeledSpinBox(
            "Startup Voltage",
            min_val=0,
            max_val=20,
            initial=VF_CONTROLLER_PARAMS["v_startup"],
            step=0.1,
            decimals=2,
            suffix=" V",
            description="Initial voltage to overcome static friction. Typical: 0.5-2V.",
        )
        self.vf_group_layout.addWidget(self.vf_startup_voltage)

        self.vf_freq_slew = LabeledSpinBox(
            "Frequency Slew Rate",
            min_val=1,
            max_val=500,
            initial=VF_CONTROLLER_PARAMS["frequency_slew_rate"],
            step=1,
            decimals=1,
            suffix=" Hz/s",
            description="Maximum frequency change rate. Higher = faster acceleration.",
        )
        self.vf_group_layout.addWidget(self.vf_freq_slew)

        # Speed reference (will be shown in monitoring tab)
        self.vf_speed_ref = LabeledSpinBox(
            "Speed Reference",
            min_val=0,
            max_val=500,
            initial=50.0,
            step=1,
            decimals=1,
            suffix=" Hz",
            description="Desired motor operating frequency (speed command).",
        )
        self.vf_group_layout.addWidget(self.vf_speed_ref)

        self.vf_group.setLayout(self.vf_group_layout)
        layout.addWidget(self.vf_group)

        # FOC parameters group (hidden by default)
        self.foc_group = AccessibleGroupBox(
            "FOC Controller", "Configure Field-Oriented Control algorithm."
        )
        self.foc_group_layout = QVBoxLayout()

        self.foc_transform = LabeledComboBox(
            "Transform",
            items=["Clarke", "Concordia"],
            description="Choose coordinate transform for current measurement.",
        )
        self.foc_group_layout.addWidget(self.foc_transform)

        self.foc_output_mode = LabeledComboBox(
            "Output Mode",
            items=["Polar", "Cartesian"],
            description="Polar returns (mag,angle), Cartesian returns (v_alpha,v_beta).",
        )
        self.foc_group_layout.addWidget(self.foc_output_mode)

        self.foc_id_ref = LabeledSpinBox(
            "D-axis Current Ref",
            min_val=-10,
            max_val=10,
            initial=0.0,
            step=0.1,
            decimals=2,
            suffix=" A",
            description="Reference for d-axis current (field weakening).",
        )
        self.foc_group_layout.addWidget(self.foc_id_ref)

        self.foc_iq_ref = LabeledSpinBox(
            "Q-axis Current Ref",
            min_val=-10,
            max_val=10,
            initial=0.0,
            step=0.1,
            decimals=2,
            suffix=" A",
            description="Reference for q-axis current (torque-producing).",
        )
        self.foc_group_layout.addWidget(self.foc_iq_ref)

        self.foc_speed_ref = LabeledSpinBox(
            "Speed Ref (rpm)",
            min_val=0,
            max_val=10000,
            initial=0.0,
            step=10,
            decimals=0,
            suffix=" RPM",
            description="Speed reference for closed-loop control (mapped to iq).",
        )
        self.foc_group_layout.addWidget(self.foc_speed_ref)

        btn_auto_d = AccessibleButton(
            "Auto-tune d-axis", "Auto-tune d-axis PI controller"
        )
        btn_auto_d.clicked.connect(lambda: self._auto_tune_axis("d"))
        self.foc_group_layout.addWidget(btn_auto_d)

        btn_auto_q = AccessibleButton(
            "Auto-tune q-axis", "Auto-tune q-axis PI controller"
        )
        btn_auto_q.clicked.connect(lambda: self._auto_tune_axis("q"))
        self.foc_group_layout.addWidget(btn_auto_q)

        self.foc_group.setLayout(self.foc_group_layout)
        layout.addWidget(self.foc_group)

        layout.addStretch()

        widget.setLayout(layout)
        self.tabs.addTab(widget, "Control")

    def _create_monitoring_tab(self):
        """Create real-time monitoring tab with speed curve."""
        widget = QWidget()
        layout = QVBoxLayout()

        # Navigation instructions for accessibility
        instructions = QLabel(
            "📋 Monitoring Tab: Real-time motor values on the left, Speed profile on the right.\n"
            "Navigate with Tab key. In the monitoring list, use arrow keys or Tab+Shift to navigate values."
        )
        instructions.setWordWrap(True)
        instructions.setAccessibleName("Navigation Instructions")
        layout.addWidget(instructions)

        # Status group with more comprehensive monitoring
        group = AccessibleGroupBox(
            "Real-Time Monitoring",
            "Current motor state and performance metrics. Use arrow keys to navigate through values.",
        )
        group_layout = QHBoxLayout()

        # Left side: Status labels in accessible list
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setAccessibleName("Monitoring Values List")
        scroll.setAccessibleDescription(
            "Scrollable list of current motor state values. Use arrow keys to navigate."
        )

        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout()

        self.status_labels = {}
        status_items = [
            ("speed_rpm", "⚡ Rotor Speed", "RPM"),
            ("omega", "Angular Velocity", "rad/s"),
            ("theta", "Rotor Position", "rad"),
            ("currents_a", "Phase A Current", "A"),
            ("currents_b", "Phase B Current", "A"),
            ("currents_c", "Phase C Current", "A"),
            ("torque", "Electromagnetic Torque", "N·m"),
            ("back_emf_a", "Back-EMF Phase A", "V"),
            ("back_emf_b", "Back-EMF Phase B", "V"),
            ("back_emf_c", "Back-EMF Phase C", "V"),
            ("id_ref", "d-axis Current Ref", "A"),
            ("iq_ref", "q-axis Current Ref", "A"),
            ("time", "Simulation Time", "s"),
        ]

        for idx, (key, name, unit) in enumerate(status_items):
            label = QLabel(f"{name}: -- {unit}")
            label.setAccessibleName(name)
            label.setAccessibleDescription(
                f"Monitoring value {idx + 1} of {len(status_items)}: {name} (Unit: {unit})"
            )
            label.setFocusPolicy(
                Qt.FocusPolicy.StrongFocus
            )  # Make focusable for accessibility
            self.status_labels[key] = label
            scroll_layout.addWidget(label)

        scroll_layout.addStretch()
        scroll_widget.setLayout(scroll_layout)
        scroll.setWidget(scroll_widget)
        group_layout.addWidget(scroll, 1)

        # Right side: Speed curve display (using matplotlib)
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

        self.speed_figure = Figure(figsize=(5, 4), dpi=80)
        self.speed_canvas = FigureCanvas(self.speed_figure)
        self.speed_ax = self.speed_figure.add_subplot(111)
        self.speed_ax.set_xlabel("Time (s)")
        self.speed_ax.set_ylabel("Speed (RPM)")
        self.speed_ax.set_title("Rotor Speed Profile")
        self.speed_ax.grid(True, alpha=0.3)
        self.speed_line = None
        self.speed_canvas.setAccessibleName("Speed Profile Graph")
        self.speed_canvas.setAccessibleDescription(
            "Real-time plot of motor rotor speed vs simulation time"
        )
        group_layout.addWidget(self.speed_canvas, 1)

        group.setLayout(group_layout)
        layout.addWidget(group)

        widget.setLayout(layout)
        self.tabs.addTab(widget, "Monitoring")

    def _create_plotting_tab(self):
        """Create plotting and visualization tab."""
        widget = QWidget()
        layout = QVBoxLayout()

        group = AccessibleGroupBox(
            "Visualization", "Generate plots of simulation results."
        )
        group_layout = QVBoxLayout()

        info = QLabel(
            "Plots are generated from recorded simulation data.\n"
            "Run simulation first, then select plot type or custom variables to generate visualization."
        )
        info.setWordWrap(True)
        group_layout.addWidget(info)

        # Grid controls for plots
        grid_layout = QHBoxLayout()
        from PyQt6.QtWidgets import QCheckBox

        self.plot_grid_checkbox = QCheckBox("Show Grid")
        self.plot_grid_checkbox.setAccessibleName("Show Grid")
        self.plot_grid_checkbox.setChecked(True)
        grid_layout.addWidget(self.plot_grid_checkbox)

        self.plot_minor_grid_checkbox = QCheckBox("Minor Grid")
        self.plot_minor_grid_checkbox.setAccessibleName("Show Minor Grid")
        self.plot_minor_grid_checkbox.setChecked(False)
        grid_layout.addWidget(self.plot_minor_grid_checkbox)

        self.plot_grid_spacing = LabeledSpinBox(
            "Grid Spacing (s)",
            0.001,
            10.0,
            0.5,
            0.001,
            3,
            " s",
            "Grid spacing in seconds (X-axis)",
        )
        grid_layout.addWidget(self.plot_grid_spacing)

        self.plot_grid_spacing_y = LabeledSpinBox(
            "Y Spacing",
            0.0,
            100.0,
            0.0,
            0.1,
            2,
            " units",
            "Grid spacing for Y-axis (0=auto)",
        )
        grid_layout.addWidget(self.plot_grid_spacing_y)

        group_layout.addLayout(grid_layout)

        # custom variable selector
        self.plot_var_list = AccessibleTableWidget(
            "Variable Selection",
            "Select variables to plot. Use arrow keys to navigate, Space or Enter to select/deselect items.",
        )
        self.plot_var_list.setColumnCount(1)
        self.plot_var_list.setHorizontalHeaderLabels(["Variable"])
        self.plot_var_list.setSelectionBehavior(
            QTableWidget.SelectionBehavior.SelectRows
        )
        self.plot_var_list.setSelectionMode(QTableWidget.SelectionMode.MultiSelection)
        # will populate when simulation ends / when plotting
        group_layout.addWidget(self.plot_var_list)

        button_layout = QHBoxLayout()

        btn_plot_3phase = AccessibleButton(
            "Plot 3-Phase Overview",
            "Generate comprehensive 3-phase motor variables plot",
        )
        btn_plot_3phase.clicked.connect(self._plot_3phase)
        button_layout.addWidget(btn_plot_3phase)

        btn_plot_current = AccessibleButton(
            "Plot Currents", "Generate detailed 3-phase current analysis plot"
        )
        btn_plot_current.clicked.connect(self._plot_currents)
        button_layout.addWidget(btn_plot_current)

        btn_plot_custom = AccessibleButton(
            "Plot Selected", "Generate plot for user-selected variables"
        )
        btn_plot_custom.clicked.connect(self._plot_custom)
        button_layout.addWidget(btn_plot_custom)

        group_layout.addLayout(button_layout)
        group_layout.addStretch()

        group.setLayout(group_layout)
        layout.addWidget(group)

        widget.setLayout(layout)
        self.tabs.addTab(widget, "Plotting")

    def _initialize_defaults(self):
        """Initialize simulation with default values."""
        self.ctrl_mode.setCurrentText("V/f")
        self.foc_group.setVisible(False)
        # initialize supply controls visibility
        if hasattr(self, "supply_type"):
            self.supply_type.setCurrentText("Constant")
            self._on_supply_type_changed("Constant")
        self._apply_to_simulation()

    def _apply_to_simulation(self):
        """Apply current UI parameters to simulation."""
        # Create motor
        params = MotorParameters(
            nominal_voltage=self.param_voltage.value(),
            phase_resistance=self.param_resistance.value(),
            phase_inductance=self.param_inductance.value(),
            back_emf_constant=self.param_emf.value(),
            torque_constant=self.param_kt.value(),
            rotor_inertia=self.param_inertia.value(),
            friction_coefficient=self.param_friction.value(),
            num_poles=int(self.param_poles.value()),
            ld=self.param_ld.value(),
            lq=self.param_lq.value(),
            poles_pairs=int(self.param_poles.value() / 2),
            model_type=self.param_model_type.currentText(),
            emf_shape=self.param_emf_shape.currentText(),
        )

        self.motor = BLDCMotor(params, dt=SIMULATION_PARAMS["dt"])
        # If Ld/Lq provided in UI, update params
        try:
            params.ld = float(getattr(self, "param_ld", None).value())
            params.lq = float(getattr(self, "param_lq", None).value())
        except Exception:
            pass

        # Create load profile
        load_type = self.load_type.currentText()
        if load_type == "Constant":
            load = ConstantLoad(torque=self.load_constant_torque.value())
        elif load_type == "Ramp":
            load = RampLoad(
                initial=self.load_initial_torque.value(),
                final=self.load_final_torque.value(),
                duration=self.load_ramp_duration.value(),
            )
        else:
            load = ConstantLoad(0.0)

        # Create supply profile based on UI selection
        supply_type = getattr(self, "supply_type", None)
        if supply_type and supply_type.currentText() == "Ramp":
            from src.core.power_model import RampSupply

            supply = RampSupply(
                initial=self.supply_ramp_initial.value(),
                final=self.supply_ramp_final.value(),
                duration=self.supply_ramp_duration.value(),
            )
        else:
            from src.core.power_model import ConstantSupply

            supply = ConstantSupply(voltage=self.supply_constant_voltage.value())

        # Create simulation engine
        self.engine = SimulationEngine(
            self.motor, load, dt=SIMULATION_PARAMS["dt"], supply_profile=supply
        )

        # Create SVM generator (cartesian capable if needed later)
        self.svm = SVMGenerator(dc_voltage=SIMULATION_PARAMS["dc_voltage"])

        # Determine controller type
        if self.ctrl_mode.currentText() == "V/f":
            self.controller = VFController(
                v_nominal=self.vf_v_nominal.value(),
                f_nominal=self.vf_f_nominal.value(),
                dc_voltage=SIMULATION_PARAMS["dc_voltage"],
                v_startup=self.vf_startup_voltage.value(),
                ramp_rate=10.0,
            )
            self.controller.set_frequency_slew_rate(self.vf_freq_slew.value())
            self.controller.set_speed_reference(self.vf_speed_ref.value())
        else:
            # FOC controller configuration
            use_conc = self.foc_transform.currentText() == "Concordia"
            out_cart = self.foc_output_mode.currentText() == "Cartesian"
            self.controller = FOCController(
                motor=self.motor,
                use_concordia=use_conc,
                output_cartesian=out_cart,
            )
            # set references
            self.controller.set_current_references(
                id_ref=self.foc_id_ref.value(),
                iq_ref=self.foc_iq_ref.value(),
            )
            self.controller.set_speed_reference(self.foc_speed_ref.value())

    def _start_simulation(self):
        """Start simulation."""
        if self.is_running:
            QMessageBox.warning(self, "Warning", "Simulation already running!")
            return

        self._apply_to_simulation()

        self.is_running = True
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

        # Reset speed curve data
        self.speed_history_time = []
        self.speed_history_rpm = []

        # Create and start simulation thread
        self.sim_thread = SimulationThread()
        duration = self.sim_duration.value()
        self.sim_thread.set_simulation(
            self.engine, self.svm, self.controller, max_duration=duration
        )
        self.sim_thread.update_signal.connect(self._update_monitoring)
        self.sim_thread.finished_signal.connect(self._on_simulation_finished)
        self.sim_thread.start_simulation()

        msg = f"Simulation started. Duration: {'∞ (infinite)' if duration == 0 else f'{duration}s'}"
        speak(msg)

    def _stop_simulation(self):
        """Stop simulation."""
        if not self.is_running:
            return
        speak("Simulation stopped.")

        self.is_running = False
        if self.sim_thread:
            self.sim_thread.stop_simulation()
            self.sim_thread.wait()

        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

        QMessageBox.information(self, "Info", "Simulation stopped.")

    def _reset_simulation(self):
        """Reset simulation."""
        if self._stop_simulation() is not None:
            return

        if self.engine:
            self.engine.reset()
            self._update_display()
            QMessageBox.information(self, "Info", "Simulation reset to initial state.")

    def _on_control_mode_changed(self, text: str) -> None:
        """Show/hide parameter groups depending on selected control mode."""
        if text == "V/f":
            self.vf_group.setVisible(True)
            self.foc_group.setVisible(False)
        else:
            self.vf_group.setVisible(False)
            self.foc_group.setVisible(True)

    def _auto_tune_axis(self, axis: str) -> None:
        """Trigger FOC controller auto-tuning for a specific axis."""
        if not isinstance(self.controller, FOCController):
            QMessageBox.warning(self, "Warning", "FOC controller is not active.")
            return
        self.controller.auto_tune_pi(axis=axis)
        QMessageBox.information(self, "Info", f"Auto-tuned {axis}-axis PI parameters.")

    def _update_monitoring(self, state: dict):
        """Update monitoring display with real-time values and speed curve."""
        speed_val = state.get("speed_rpm", 0)
        time_val = state.get("time", 0)

        self.status_labels["speed_rpm"].setText(f"⚡ Rotor Speed: {speed_val:.2f} RPM")
        self.status_labels["omega"].setText(
            f"Angular Velocity: {state.get('omega', 0):.4f} rad/s"
        )
        self.status_labels["theta"].setText(
            f"Rotor Position: {state.get('theta', 0):.4f} rad"
        )
        self.status_labels["currents_a"].setText(
            f"Phase A Current: {state.get('currents_a', 0):.3f} A"
        )
        self.status_labels["currents_b"].setText(
            f"Phase B Current: {state.get('currents_b', 0):.3f} A"
        )
        self.status_labels["currents_c"].setText(
            f"Phase C Current: {state.get('currents_c', 0):.3f} A"
        )
        self.status_labels["torque"].setText(
            f"Electromagnetic Torque: {state.get('torque', 0):.4f} N·m"
        )
        self.status_labels["back_emf_a"].setText(
            f"Back-EMF Phase A: {state.get('back_emf_a', 0):.3f} V"
        )
        self.status_labels["back_emf_b"].setText(
            f"Back-EMF Phase B: {state.get('back_emf_b', 0):.3f} V"
        )
        self.status_labels["back_emf_c"].setText(
            f"Back-EMF Phase C: {state.get('back_emf_c', 0):.3f} V"
        )
        self.status_labels["time"].setText(f"Simulation Time: {time_val:.3f} s")

        # Update accessible descriptions so screen readers announce latest values
        # Extract and format numerical values for better screen reader access
        for lbl in self.status_labels.values():
            try:
                # Get the full text which includes the numerical value
                full_text = lbl.text()
                # Set tooltip, which is often read by screen readers on focus
                lbl.setToolTip(full_text)
                # Crucially, emit an accessibility event to notify screen readers of the update
                QAccessible.updateAccessibility(lbl, 0, QAccessible.Event.ValueChange)
            except Exception:
                pass

        # Update permanent info panel values
        try:
            if self.engine:
                dt_val = self.engine.dt
                motor_params = self.engine.motor.params
            else:
                dt_val = SIMULATION_PARAMS.get("dt", 0.0001)
                from src.core.motor_model import MotorParameters

                motor_params = MotorParameters()

            self.lbl_dt.setText(f"dt: {dt_val} s")
            # electrical & mechanical time constants
            try:
                tau_e = motor_params.phase_inductance / motor_params.phase_resistance
            except Exception:
                tau_e = None
            try:
                tau_m = motor_params.rotor_inertia / motor_params.friction_coefficient
            except Exception:
                tau_m = None
            self.lbl_tau_e.setText(
                f"Electrical time constant (L/R): {tau_e if tau_e is not None else '--'} s"
            )
            self.lbl_tau_m.setText(
                f"Mechanical time constant (J/b): {tau_m if tau_m is not None else '--'} s"
            )

            # update Ld/Lq display
            ld_val = (
                getattr(self, "param_ld", None).value()
                if getattr(self, "param_ld", None)
                else motor_params.ld
            )
            lq_val = (
                getattr(self, "param_lq", None).value()
                if getattr(self, "param_lq", None)
                else motor_params.lq
            )
            self.lbl_ld_lq.setText(f"Ld: {ld_val:.6f} H, Lq: {lq_val:.6f} H")

            # Update parameter units summary
            units_html = (
                "<b>Parameters (units):</b><br>"
                + "Nominal Voltage (V), Phase Resistance (Ω), Phase Inductance (H),<br>"
                + "Back-EMF (V·s/rad), Torque Constant (N·m/A), Inertia (kg·m²), Friction (N·m·s/rad),<br>"
                + "Poles (count), Ld (H), Lq (H)"
            )
            self.lbl_param_units.setText(units_html)

            # Update status bar with simulation parameters
            self.status_bar_dt.setText(f"dt: {dt_val} s")
            self.status_bar_tau_e.setText(
                f"τ_e: {tau_e if tau_e is not None else '--'} s"
            )
            self.status_bar_tau_m.setText(
                f"τ_m: {tau_m if tau_m is not None else '--'} s"
            )
        except Exception:
            pass

        # Update speed curve
        self.speed_history_time.append(time_val)
        self.speed_history_rpm.append(speed_val)

        # Every 10 updates, redraw the curve (to avoid too frequent redraws)
        if len(self.speed_history_time) % 10 == 0:
            self.speed_ax.clear()
            self.speed_ax.plot(
                self.speed_history_time,
                self.speed_history_rpm,
                color=PLOT_STYLE.get("live_speed_color", "#FF6B9D"),
                linewidth=2,
                label="Speed",
            )
            self.speed_ax.set_xlabel("Time (s)")
            self.speed_ax.set_ylabel("Speed (RPM)")
            self.speed_ax.set_title("⚡ Rotor Speed Profile")
            self.speed_ax.grid(True, alpha=0.3)
            self.speed_ax.legend()
            self.speed_figure.tight_layout()
            self.speed_canvas.draw()

        # if using FOC, show additional controller state
        if isinstance(self.controller, FOCController):
            ctrl_state = self.controller.get_state()
            id_val = ctrl_state.get("id_ref", 0)
            iq_val = ctrl_state.get("iq_ref", 0)
            self.status_labels.setdefault("id_ref", QLabel()).setText(
                f"d-axis ref: {id_val:.2f} A"
            )
            self.status_labels.setdefault("iq_ref", QLabel()).setText(
                f"q-axis ref: {iq_val:.2f} A"
            )

    def _update_display(self):
        """Update display (for manual updates)."""
        if self.engine:
            state = self.engine.get_current_state()
            self._update_monitoring(state)

    def _on_simulation_finished(self):
        """Handle simulation thread completion."""
        self.is_running = False
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

    def _on_load_type_changed(self):
        """Handle load type change."""
        load_type = self.load_type.currentText()
        self.load_constant_torque.setVisible(load_type == "Constant")
        self.load_initial_torque.setVisible(load_type == "Ramp")
        self.load_final_torque.setVisible(load_type == "Ramp")
        self.load_ramp_duration.setVisible(load_type == "Ramp")

    def _export_data(self):
        """Export simulation data to CSV."""
        if not self.engine or len(self.engine.get_history()["time"]) == 0:
            QMessageBox.warning(self, "Warning", "No simulation data to export!")
            return

        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Simulation Data", "", "CSV Files (*.csv)"
        )

        if filename:
            metadata = {
                "motor_params": {
                    "nominal_voltage": self.param_voltage.value(),
                    "resistance": self.param_resistance.value(),
                    "inductance": self.param_inductance.value(),
                },
                "control_params": {
                    "v_nominal": self.vf_v_nominal.value(),
                    "f_nominal": self.vf_f_nominal.value(),
                },
            }

            try:
                self.logger.save_simulation_data(
                    self.engine.get_history(), metadata, filename, use_custom_path=True
                )
                QMessageBox.information(
                    self,
                    "Success",
                    f"Data saved to {filename}\nMetadata saved with _metadata suffix",
                )
                speak("Data exported successfully.")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to export data: {str(e)}")
                logger.error(f"Export failed: {str(e)}")

    def _plot_3phase(self):
        """Generate 3-phase plot."""
        if not self.engine or len(self.engine.get_history()["time"]) == 0:
            QMessageBox.warning(self, "Warning", "No data to plot!")
            return

        history = self.engine.get_history()
        grid_on = (
            getattr(self, "plot_grid_checkbox", None)
            and self.plot_grid_checkbox.isChecked()
        )
        grid_spacing = (
            getattr(self, "plot_grid_spacing", None) and self.plot_grid_spacing.value()
        )
        minor_grid = (
            getattr(self, "plot_minor_grid_checkbox", None)
            and self.plot_minor_grid_checkbox.isChecked()
        )
        grid_spacing_y = (
            getattr(self, "plot_grid_spacing_y", None)
            and self.plot_grid_spacing_y.value()
        )
        figure = SimulationPlotter.create_3phase_plot(
            history,
            grid_on=grid_on,
            grid_spacing=grid_spacing,
            minor_grid=minor_grid,
            grid_spacing_y=grid_spacing_y,
        )
        figure.show()
        speak("3-phase plot generated.")

    def _plot_currents(self):
        """Generate current plot."""
        if not self.engine or len(self.engine.get_history()["time"]) == 0:
            QMessageBox.warning(self, "Warning", "No data to plot!")
            return

        history = self.engine.get_history()
        grid_on = (
            getattr(self, "plot_grid_checkbox", None)
            and self.plot_grid_checkbox.isChecked()
        )
        grid_spacing = (
            getattr(self, "plot_grid_spacing", None) and self.plot_grid_spacing.value()
        )
        minor_grid = (
            getattr(self, "plot_minor_grid_checkbox", None)
            and self.plot_minor_grid_checkbox.isChecked()
        )
        grid_spacing_y = (
            getattr(self, "plot_grid_spacing_y", None)
            and self.plot_grid_spacing_y.value()
        )
        figure = SimulationPlotter.create_current_plot(
            history,
            grid_on=grid_on,
            grid_spacing=grid_spacing,
            minor_grid=minor_grid,
            grid_spacing_y=grid_spacing_y,
        )
        figure.show()
        speak("Current plot generated.")

    def _plot_custom(self):
        """Generate a custom multi-axis plot from selected variables."""
        if not self.engine or len(self.engine.get_history()["time"]) == 0:
            QMessageBox.warning(self, "Warning", "No data to plot!")
            return
        history = self.engine.get_history()
        # populate variable list if not done yet
        if self.plot_var_list.rowCount() == 0:
            keys = [k for k in history.keys() if k != "time"]
            self.plot_var_list.setRowCount(len(keys))
            for i, k in enumerate(keys):
                item = QTableWidgetItem(k)
                item.setFlags(item.flags() | Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                self.plot_var_list.setItem(i, 0, item)
        selected = [item.text() for item in self.plot_var_list.selectedItems()]
        if not selected:
            QMessageBox.warning(self, "Warning", "No variables selected for plotting.")
            return
        grid_on = (
            getattr(self, "plot_grid_checkbox", None)
            and self.plot_grid_checkbox.isChecked()
        )
        grid_spacing = (
            getattr(self, "plot_grid_spacing", None) and self.plot_grid_spacing.value()
        )
        minor_grid = (
            getattr(self, "plot_minor_grid_checkbox", None)
            and self.plot_minor_grid_checkbox.isChecked()
        )
        grid_spacing_y = (
            getattr(self, "plot_grid_spacing_y", None)
            and self.plot_grid_spacing_y.value()
        )
        figure = SimulationPlotter.create_multi_axis_plot(
            history,
            selected,
            grid_on=grid_on,
            grid_spacing=grid_spacing,
            minor_grid=minor_grid,
            grid_spacing_y=grid_spacing_y,
        )
        figure.show()
        speak("Custom plot generated for variables: " + ", ".join(selected))
