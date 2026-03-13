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
    QFrame,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread

# QAccessible was removed from PyQt6.QtCore in some builds of PyQt6.
# We still want the code to run even if accessibility support is not available.
try:
    from PyQt6.QtCore import QAccessible
except ImportError:  # pragma: no cover

    class _DummyQAccessible:
        class Event:
            NameChange = 0
            ValueChange = 1

        @staticmethod
        def updateAccessibility(*args, **kwargs):
            return

    QAccessible = _DummyQAccessible

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
    recommend_efficiency_adjustments,
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


class AccessibleTextBlock(QFrame):
    """
    Accessible visual text block component for monitoring values.
    Each block displays a labeled monitoring value with units.
    Optimized for screen reader detection and visual distinction.
    """

    def __init__(
        self,
        parameter_name: str,
        unit: str,
        index: int,
        total_count: int,
        parent=None,
    ):
        super().__init__(parent)

        self.parameter_name = parameter_name
        self.unit = unit
        self.index = index
        self.total_count = total_count
        self.current_value = 0.0

        # Visual styling for text block
        self.setFrameShape(QFrame.Shape.Box)
        self.setFrameShadow(QFrame.Shadow.Raised)
        self.setLineWidth(2)
        self.setStyleSheet(
            """
            AccessibleTextBlock {
                background-color: #f0f0f0;
                border: 2px solid #4CAF50;
                border-radius: 5px;
                padding: 8px;
                margin: 4px;
            }
            AccessibleTextBlock:focus {
                border: 2px solid #2196F3;
                background-color: #e8f4f8;
            }
        """
        )

        # Make focusable for keyboard navigation and screen readers
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # Layout
        layout = QVBoxLayout()
        layout.setContentsMargins(8, 4, 8, 4)

        # Parameter name label (bold, prominent)
        self.name_label = QLabel(parameter_name)
        name_font = QFont()
        name_font.setBold(True)
        name_font.setPointSize(10)
        self.name_label.setFont(name_font)
        self.name_label.setAccessibleName(
            f"Parameter {self.index + 1}: {parameter_name}"
        )
        layout.addWidget(self.name_label)

        # Value display (large, readable)
        self.value_label = QLabel("-- " + unit)
        value_font = QFont()
        value_font.setPointSize(12)
        value_font.setBold(True)
        value_font.setFamily("Courier")  # Monospace for alignment
        self.value_label.setFont(value_font)
        self.value_label.setStyleSheet("color: #1976D2;")
        self.value_label.setAccessibleName(f"Value: {parameter_name} in {unit}")
        layout.addWidget(self.value_label)

        self.setLayout(layout)

        # Set accessible properties for screen readers
        self.setAccessibleName(
            f"Monitoring Block {self.index + 1} of {self.total_count}: {parameter_name}"
        )
        self.setAccessibleDescription(
            f"{parameter_name} current value (Unit: {unit}). Navigation: Tab to next block, Shift+Tab to previous."
        )

    def update_value(self, value: float):
        """Update the displayed value."""
        self.current_value = value
        formatted_value = f"{value:.4g}"
        self.value_label.setText(f"{formatted_value} {self.unit}")
        self.value_label.setToolTip(
            f"{self.parameter_name}: {formatted_value} {self.unit}"
        )

        # Notify screen readers of value change
        QAccessible.updateAccessibility(
            self.value_label, 0, QAccessible.Event.NameChange
        )

    def keyPressEvent(self, event):
        """Handle keyboard navigation for accessibility."""
        if event.key() in (Qt.Key.Key_Up, Qt.Key.Key_Left):
            self.focusPreviousChild()
            event.accept()
        elif event.key() in (Qt.Key.Key_Down, Qt.Key.Key_Right):
            self.focusNextChild()
            event.accept()
        else:
            super().keyPressEvent(event)


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
                try:
                    self.svm.set_phase_currents(self.engine.motor.currents)
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

        self.foc_speed_loop_mode = LabeledComboBox(
            "Speed Loop Mode",
            items=["Legacy iq mapping", "Cascaded PI"],
            description="Legacy keeps previous iq mapping behavior. Cascaded PI enables speed-loop to iq generation.",
        )
        self.foc_group_layout.addWidget(self.foc_speed_loop_mode)

        self.foc_iq_limit = LabeledSpinBox(
            "Iq Limit",
            min_val=0.1,
            max_val=100.0,
            initial=30.0,
            step=0.1,
            decimals=2,
            suffix=" A",
            description="Absolute q-axis current clamp used by cascaded speed loop.",
        )
        self.foc_group_layout.addWidget(self.foc_iq_limit)

        self.foc_speed_kp = LabeledSpinBox(
            "Speed PI Kp",
            min_val=0.0,
            max_val=10.0,
            initial=0.02,
            step=0.001,
            decimals=4,
            suffix="",
            description="Proportional gain for cascaded speed PI loop.",
        )
        self.foc_group_layout.addWidget(self.foc_speed_kp)

        self.foc_speed_ki = LabeledSpinBox(
            "Speed PI Ki",
            min_val=0.0,
            max_val=200.0,
            initial=1.0,
            step=0.1,
            decimals=3,
            suffix="",
            description="Integral gain for cascaded speed PI loop.",
        )
        self.foc_group_layout.addWidget(self.foc_speed_ki)

        self.foc_d_kp = LabeledSpinBox(
            "D-axis PI Kp",
            min_val=0.0,
            max_val=50.0,
            initial=1.0,
            step=0.01,
            decimals=3,
            suffix="",
            description="Proportional gain for d-axis current PI loop.",
        )
        self.foc_group_layout.addWidget(self.foc_d_kp)

        self.foc_d_ki = LabeledSpinBox(
            "D-axis PI Ki",
            min_val=0.0,
            max_val=500.0,
            initial=0.1,
            step=0.01,
            decimals=3,
            suffix="",
            description="Integral gain for d-axis current PI loop.",
        )
        self.foc_group_layout.addWidget(self.foc_d_ki)

        self.foc_q_kp = LabeledSpinBox(
            "Q-axis PI Kp",
            min_val=0.0,
            max_val=50.0,
            initial=1.0,
            step=0.01,
            decimals=3,
            suffix="",
            description="Proportional gain for q-axis current PI loop.",
        )
        self.foc_group_layout.addWidget(self.foc_q_kp)

        self.foc_q_ki = LabeledSpinBox(
            "Q-axis PI Ki",
            min_val=0.0,
            max_val=500.0,
            initial=0.1,
            step=0.01,
            decimals=3,
            suffix="",
            description="Integral gain for q-axis current PI loop.",
        )
        self.foc_group_layout.addWidget(self.foc_q_ki)

        self.foc_decouple_d_mode = LabeledComboBox(
            "D-axis Decoupling",
            items=["Disabled", "Enabled"],
            description="Enable d-axis feed-forward compensation term (-omega*Lq*iq).",
        )
        self.foc_group_layout.addWidget(self.foc_decouple_d_mode)

        self.foc_decouple_q_mode = LabeledComboBox(
            "Q-axis Decoupling",
            items=["Disabled", "Enabled"],
            description="Enable q-axis feed-forward compensation term (omega*Ld*id).",
        )
        self.foc_group_layout.addWidget(self.foc_decouple_q_mode)

        self.foc_angle_observer_mode = LabeledComboBox(
            "Angle Observer",
            items=["Measured", "PLL", "SMO"],
            description="Select rotor electrical angle source: direct model angle, PLL on back-EMF, or sliding-mode observer variant.",
        )
        self.foc_group_layout.addWidget(self.foc_angle_observer_mode)

        self.foc_pll_kp = LabeledSpinBox(
            "PLL Kp",
            min_val=0.0,
            max_val=5000.0,
            initial=80.0,
            step=1.0,
            decimals=2,
            suffix="",
            description="Proportional gain for back-EMF PLL angle observer.",
        )
        self.foc_group_layout.addWidget(self.foc_pll_kp)

        self.foc_pll_ki = LabeledSpinBox(
            "PLL Ki",
            min_val=0.0,
            max_val=50000.0,
            initial=2000.0,
            step=10.0,
            decimals=2,
            suffix="",
            description="Integral gain for back-EMF PLL angle observer.",
        )
        self.foc_group_layout.addWidget(self.foc_pll_ki)

        self.foc_smo_k_slide = LabeledSpinBox(
            "SMO Kslide",
            min_val=0.0,
            max_val=10000.0,
            initial=600.0,
            step=5.0,
            decimals=2,
            suffix="",
            description="Sliding gain for SMO-inspired angle observer correction.",
        )
        self.foc_group_layout.addWidget(self.foc_smo_k_slide)

        self.foc_smo_lpf_alpha = LabeledSpinBox(
            "SMO LPF Alpha",
            min_val=0.001,
            max_val=1.0,
            initial=0.08,
            step=0.001,
            decimals=3,
            suffix="",
            description="Low-pass blending factor for SMO estimated electrical speed.",
        )
        self.foc_group_layout.addWidget(self.foc_smo_lpf_alpha)

        self.foc_smo_boundary = LabeledSpinBox(
            "SMO Boundary",
            min_val=0.001,
            max_val=1.0,
            initial=0.06,
            step=0.001,
            decimals=3,
            suffix=" rad",
            description="Boundary layer width used in SMO switching nonlinearity.",
        )
        self.foc_group_layout.addWidget(self.foc_smo_boundary)

        self.foc_startup_transition_mode = LabeledComboBox(
            "Observer Startup Transition",
            items=["Disabled", "Enabled"],
            description="When enabled, start with an initial observer mode and automatically hand off to selected observer when startup conditions are met.",
        )
        self.foc_group_layout.addWidget(self.foc_startup_transition_mode)

        self.foc_startup_initial_observer = LabeledComboBox(
            "Startup Initial Observer",
            items=["Measured", "PLL", "SMO"],
            description="Observer mode used during startup before handoff.",
        )
        self.foc_group_layout.addWidget(self.foc_startup_initial_observer)

        self.foc_startup_min_speed = LabeledSpinBox(
            "Startup Min Speed",
            min_val=0.0,
            max_val=20000.0,
            initial=300.0,
            step=10.0,
            decimals=1,
            suffix=" RPM",
            description="Minimum speed required before observer handoff.",
        )
        self.foc_group_layout.addWidget(self.foc_startup_min_speed)

        self.foc_startup_min_time = LabeledSpinBox(
            "Startup Min Time",
            min_val=0.0,
            max_val=5.0,
            initial=0.05,
            step=0.005,
            decimals=3,
            suffix=" s",
            description="Minimum startup dwell time before observer handoff.",
        )
        self.foc_group_layout.addWidget(self.foc_startup_min_time)

        self.foc_startup_min_emf = LabeledSpinBox(
            "Startup Min Back-EMF",
            min_val=0.0,
            max_val=50.0,
            initial=0.5,
            step=0.05,
            decimals=3,
            suffix=" V",
            description="Minimum back-EMF magnitude required before observer handoff.",
        )
        self.foc_group_layout.addWidget(self.foc_startup_min_emf)

        self.foc_startup_min_confidence = LabeledSpinBox(
            "Startup Min Confidence",
            min_val=0.0,
            max_val=1.0,
            initial=0.6,
            step=0.01,
            decimals=2,
            suffix="",
            description="Minimum observer confidence required before handoff.",
        )
        self.foc_group_layout.addWidget(self.foc_startup_min_confidence)

        self.foc_startup_confidence_hold = LabeledSpinBox(
            "Confidence Hold Time",
            min_val=0.0,
            max_val=1.0,
            initial=0.02,
            step=0.005,
            decimals=3,
            suffix=" s",
            description="Time confidence must stay above threshold before handoff.",
        )
        self.foc_group_layout.addWidget(self.foc_startup_confidence_hold)

        self.foc_startup_confidence_hysteresis = LabeledSpinBox(
            "Confidence Hysteresis",
            min_val=0.0,
            max_val=1.0,
            initial=0.1,
            step=0.01,
            decimals=2,
            suffix="",
            description="Lower-confidence margin before fallback to startup observer.",
        )
        self.foc_group_layout.addWidget(self.foc_startup_confidence_hysteresis)

        self.foc_startup_fallback_mode = LabeledComboBox(
            "Observer Fallback",
            items=["Enabled", "Disabled"],
            description="Allow reverting to startup observer when confidence degrades.",
        )
        self.foc_group_layout.addWidget(self.foc_startup_fallback_mode)

        self.foc_startup_fallback_hold = LabeledSpinBox(
            "Fallback Hold Time",
            min_val=0.0,
            max_val=1.0,
            initial=0.03,
            step=0.005,
            decimals=3,
            suffix=" s",
            description="Time degraded confidence must persist before fallback.",
        )
        self.foc_group_layout.addWidget(self.foc_startup_fallback_hold)

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

        self.inverter_group = AccessibleGroupBox(
            "Inverter Non-Idealities",
            "Optional inverter realism settings. Set to zero to keep ideal inverter behavior.",
        )
        inverter_layout = QVBoxLayout()

        self.inverter_device_drop = LabeledSpinBox(
            "Device Drop",
            min_val=0.0,
            max_val=10.0,
            initial=0.0,
            step=0.01,
            decimals=3,
            suffix=" V",
            description="Approximate per-phase effective voltage drop from switching devices.",
        )
        inverter_layout.addWidget(self.inverter_device_drop)

        self.inverter_dead_time_fraction = LabeledSpinBox(
            "Dead-Time Loss",
            min_val=0.0,
            max_val=0.2,
            initial=0.0,
            step=0.001,
            decimals=4,
            suffix=" pu",
            description="Duty loss fraction from dead-time effects. 0 keeps ideal modulation.",
        )
        inverter_layout.addWidget(self.inverter_dead_time_fraction)

        self.inverter_conduction_resistance = LabeledSpinBox(
            "Conduction Resistance",
            min_val=0.0,
            max_val=5.0,
            initial=0.0,
            step=0.001,
            decimals=4,
            suffix=" Ohm",
            description="Effective inverter conduction path resistance used for current-dependent voltage drop.",
        )
        inverter_layout.addWidget(self.inverter_conduction_resistance)

        self.inverter_switching_frequency = LabeledSpinBox(
            "Switching Frequency",
            min_val=0.0,
            max_val=200000.0,
            initial=0.0,
            step=500.0,
            decimals=1,
            suffix=" Hz",
            description="PWM switching frequency used by switching-loss approximation (0 disables).",
        )
        inverter_layout.addWidget(self.inverter_switching_frequency)

        self.inverter_switching_loss_coeff = LabeledSpinBox(
            "Switching Loss Coeff",
            min_val=0.0,
            max_val=1.0,
            initial=0.0,
            step=0.001,
            decimals=4,
            suffix=" V/A/kHz",
            description="Voltage-loss coefficient per ampere and kHz for switching-dependent drop modeling.",
        )
        inverter_layout.addWidget(self.inverter_switching_loss_coeff)

        self.inverter_group.setLayout(inverter_layout)
        layout.addWidget(self.inverter_group)

        self.pfc_group = AccessibleGroupBox(
            "Power Factor Correction",
            "Configure simulation-level closed-loop power factor telemetry control.",
        )
        pfc_layout = QVBoxLayout()

        self.pfc_mode = LabeledComboBox(
            "PFC Mode",
            items=["Disabled", "Enabled"],
            description="Enable or disable closed-loop power factor correction telemetry.",
        )
        pfc_layout.addWidget(self.pfc_mode)

        self.pfc_target_pf = LabeledSpinBox(
            "Target Power Factor",
            min_val=0.50,
            max_val=1.00,
            initial=0.95,
            step=0.01,
            decimals=3,
            suffix="",
            description="Desired target power factor for the PFC controller.",
        )
        pfc_layout.addWidget(self.pfc_target_pf)

        self.pfc_kp = LabeledSpinBox(
            "PFC Kp",
            min_val=0.0,
            max_val=10.0,
            initial=0.10,
            step=0.01,
            decimals=3,
            suffix="",
            description="Proportional gain for PFC compensation command.",
        )
        pfc_layout.addWidget(self.pfc_kp)

        self.pfc_ki = LabeledSpinBox(
            "PFC Ki",
            min_val=0.0,
            max_val=50.0,
            initial=1.0,
            step=0.1,
            decimals=3,
            suffix="",
            description="Integral gain for PFC compensation command.",
        )
        pfc_layout.addWidget(self.pfc_ki)

        self.pfc_max_var = LabeledSpinBox(
            "Max Compensation",
            min_val=10.0,
            max_val=200000.0,
            initial=10000.0,
            step=10.0,
            decimals=1,
            suffix=" VAR",
            description="Upper clamp for reactive compensation command.",
        )
        pfc_layout.addWidget(self.pfc_max_var)

        self.pfc_window_samples = LabeledSpinBox(
            "PF Window Samples",
            min_val=8,
            max_val=5000,
            initial=128,
            step=1,
            decimals=0,
            suffix="",
            description="Rolling window size used for power-factor metric estimation.",
        )
        pfc_layout.addWidget(self.pfc_window_samples)

        self.pfc_group.setLayout(pfc_layout)
        layout.addWidget(self.pfc_group)

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
        self.status_blocks = {}  # Store text blocks for easier access
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
            ("speed_error", "Speed Error", "rad/s"),
            ("v_d_ff", "d-axis Feedforward", "V"),
            ("v_q_ff", "q-axis Feedforward", "V"),
            ("speed_loop_enabled", "Speed Loop Enabled", "0/1"),
            ("decouple_d_enabled", "D-axis Decoupling", "0/1"),
            ("decouple_q_enabled", "Q-axis Decoupling", "0/1"),
            ("observer_mode_code", "Observer Mode Code", "0/1/2"),
            ("theta_electrical", "Estimated Electrical Angle", "rad"),
            ("theta_meas_emf", "Back-EMF Angle", "rad"),
            ("theta_error_pll", "PLL Angle Error", "rad"),
            ("theta_error_smo", "SMO Angle Error", "rad"),
            ("smo_omega_est", "SMO Estimated Speed", "rad/s"),
            ("observer_confidence", "Observer Confidence", "0-1"),
            ("observer_confidence_emf", "Confidence from EMF", "0-1"),
            ("observer_confidence_speed", "Confidence from Speed", "0-1"),
            ("observer_confidence_coherence", "Confidence from Coherence", "0-1"),
            ("observer_confidence_ema", "Confidence EMA", "0-1"),
            ("observer_confidence_trend", "Confidence Trend", "delta"),
            (
                "observer_confidence_above_threshold_time_s",
                "Confidence Above Threshold Time",
                "s",
            ),
            (
                "observer_confidence_below_threshold_time_s",
                "Confidence Below Threshold Time",
                "s",
            ),
            ("observer_confidence_crossings_up", "Confidence Crossings Up", "count"),
            (
                "observer_confidence_crossings_down",
                "Confidence Crossings Down",
                "count",
            ),
            ("startup_handoff_count", "Startup Handoff Count", "count"),
            (
                "startup_last_handoff_time_s",
                "Last Handoff Time",
                "s",
            ),
            (
                "startup_last_handoff_confidence",
                "Last Handoff Confidence",
                "0-1",
            ),
            (
                "startup_handoff_confidence_peak",
                "Handoff Confidence Peak",
                "0-1",
            ),
            ("startup_handoff_quality", "Handoff Quality KPI", "0-1"),
            (
                "startup_handoff_stability_ratio",
                "Handoff Stability Ratio",
                "0-1",
            ),
            ("pfc_enabled", "PFC Enabled", "0/1"),
            ("pfc_target_pf", "PFC Target PF", "0-1"),
            ("pfc_power_factor", "Input Power Factor", "-1..1"),
            ("pfc_active_power_w", "Input Active Power", "W"),
            ("pfc_reactive_power_var", "Input Reactive Power", "VAR"),
            ("pfc_command_var", "PFC Compensation Command", "VAR"),
            ("efficiency", "System Efficiency", "0-1"),
            ("mechanical_output_power_w", "Mechanical Output Power", "W"),
            ("total_loss_power_w", "Estimated Total Loss", "W"),
            ("time", "Simulation Time", "s"),
        ]

        total_items = len(status_items)
        for idx, (key, name, unit) in enumerate(status_items):
            # Create accessible text block component
            text_block = AccessibleTextBlock(name, unit, idx, total_items)
            self.status_blocks[key] = text_block
            self.status_labels[key] = text_block.value_label  # Keep for backward compat
            scroll_layout.addWidget(text_block)

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

        btn_plot_pfc = AccessibleButton(
            "Plot PFC Analysis",
            "Generate power-factor, active/reactive power, and command trends",
        )
        btn_plot_pfc.clicked.connect(self._plot_pfc_analysis)
        button_layout.addWidget(btn_plot_pfc)

        btn_plot_efficiency = AccessibleButton(
            "Plot Efficiency Analysis",
            "Generate input power, output power, loss, and efficiency trends",
        )
        btn_plot_efficiency.clicked.connect(self._plot_efficiency_analysis)
        button_layout.addWidget(btn_plot_efficiency)

        btn_efficiency_tips = AccessibleButton(
            "Suggest Efficiency Tuning",
            "Show heuristic efficiency recommendations from current settings",
        )
        btn_efficiency_tips.clicked.connect(self._show_efficiency_recommendations)
        button_layout.addWidget(btn_efficiency_tips)

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
        if hasattr(self, "pfc_mode"):
            self.pfc_mode.setCurrentText("Disabled")
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
        self.engine.configure_power_factor_control(
            enabled=self.pfc_mode.currentText() == "Enabled",
            target_pf=self.pfc_target_pf.value(),
            kp=self.pfc_kp.value(),
            ki=self.pfc_ki.value(),
            max_compensation_var=self.pfc_max_var.value(),
            window_samples=int(self.pfc_window_samples.value()),
        )

        # Create SVM generator (cartesian capable if needed later)
        self.svm = SVMGenerator(dc_voltage=SIMULATION_PARAMS["dc_voltage"])
        self.svm.set_nonidealities(
            device_drop_v=self.inverter_device_drop.value(),
            dead_time_fraction=self.inverter_dead_time_fraction.value(),
            conduction_resistance_ohm=self.inverter_conduction_resistance.value(),
            switching_frequency_hz=self.inverter_switching_frequency.value(),
            switching_loss_coeff_v_per_a_khz=self.inverter_switching_loss_coeff.value(),
        )

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
            speed_loop_enabled = self.foc_speed_loop_mode.currentText() == "Cascaded PI"
            self.controller = FOCController(
                motor=self.motor,
                use_concordia=use_conc,
                output_cartesian=out_cart,
                enable_speed_loop=speed_loop_enabled,
            )
            # set references
            self.controller.set_current_references(
                id_ref=self.foc_id_ref.value(),
                iq_ref=self.foc_iq_ref.value(),
            )
            self.controller.set_speed_reference(self.foc_speed_ref.value())
            self.controller.set_cascaded_speed_loop(
                enabled=speed_loop_enabled,
                iq_limit_a=self.foc_iq_limit.value(),
            )
            self.controller.set_speed_pi_gains(
                kp=self.foc_speed_kp.value(),
                ki=self.foc_speed_ki.value(),
            )
            self.controller.set_current_pi_gains(
                d_kp=self.foc_d_kp.value(),
                d_ki=self.foc_d_ki.value(),
                q_kp=self.foc_q_kp.value(),
                q_ki=self.foc_q_ki.value(),
            )
            self.controller.set_decoupling(
                enable_d=self.foc_decouple_d_mode.currentText() == "Enabled",
                enable_q=self.foc_decouple_q_mode.currentText() == "Enabled",
            )
            self.controller.set_angle_observer(
                self.foc_angle_observer_mode.currentText()
            )
            self.controller.set_pll_gains(
                kp=self.foc_pll_kp.value(),
                ki=self.foc_pll_ki.value(),
            )
            self.controller.set_smo_gains(
                k_slide=self.foc_smo_k_slide.value(),
                lpf_alpha=self.foc_smo_lpf_alpha.value(),
                boundary=self.foc_smo_boundary.value(),
            )
            self.controller.set_startup_transition(
                enabled=self.foc_startup_transition_mode.currentText() == "Enabled",
                initial_mode=self.foc_startup_initial_observer.currentText(),
                min_speed_rpm=self.foc_startup_min_speed.value(),
                min_elapsed_s=self.foc_startup_min_time.value(),
                min_emf_v=self.foc_startup_min_emf.value(),
                min_confidence=self.foc_startup_min_confidence.value(),
                confidence_hold_s=self.foc_startup_confidence_hold.value(),
                confidence_hysteresis=self.foc_startup_confidence_hysteresis.value(),
                fallback_enabled=self.foc_startup_fallback_mode.currentText()
                == "Enabled",
                fallback_hold_s=self.foc_startup_fallback_hold.value(),
            )

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

        # Pull advanced controller telemetry when FOC is active.
        ctrl_state = {}
        pfc_state = (
            state.get("pfc", {}) if isinstance(state.get("pfc", {}), dict) else {}
        )
        efficiency_state = (
            state.get("efficiency_metrics", {})
            if isinstance(state.get("efficiency_metrics", {}), dict)
            else {}
        )
        if not pfc_state and self.engine is not None:
            pfc_state = self.engine.get_power_factor_control_state()
        if not efficiency_state and self.engine is not None:
            efficiency_state = self.engine.get_efficiency_state()
        observer_mode_code = 0.0
        if isinstance(self.controller, FOCController):
            ctrl_state = self.controller.get_state()
            observer_mode = str(ctrl_state.get("angle_observer_mode", "Measured"))
            mode_to_code = {"Measured": 0.0, "PLL": 1.0, "SMO": 2.0}
            observer_mode_code = mode_to_code.get(observer_mode, -1.0)

        # Update accessible text blocks with new values
        if hasattr(self, "status_blocks"):
            self.status_blocks["speed_rpm"].update_value(speed_val)
            self.status_blocks["omega"].update_value(state.get("omega", 0))
            self.status_blocks["theta"].update_value(state.get("theta", 0))
            self.status_blocks["currents_a"].update_value(state.get("currents_a", 0))
            self.status_blocks["currents_b"].update_value(state.get("currents_b", 0))
            self.status_blocks["currents_c"].update_value(state.get("currents_c", 0))
            self.status_blocks["torque"].update_value(state.get("torque", 0))
            self.status_blocks["back_emf_a"].update_value(state.get("back_emf_a", 0))
            self.status_blocks["back_emf_b"].update_value(state.get("back_emf_b", 0))
            self.status_blocks["back_emf_c"].update_value(state.get("back_emf_c", 0))
            self.status_blocks["id_ref"].update_value(ctrl_state.get("id_ref", 0.0))
            self.status_blocks["iq_ref"].update_value(ctrl_state.get("iq_ref", 0.0))
            self.status_blocks["speed_error"].update_value(
                ctrl_state.get("speed_error", 0.0)
            )
            self.status_blocks["v_d_ff"].update_value(ctrl_state.get("v_d_ff", 0.0))
            self.status_blocks["v_q_ff"].update_value(ctrl_state.get("v_q_ff", 0.0))
            self.status_blocks["speed_loop_enabled"].update_value(
                1.0 if ctrl_state.get("speed_loop_enabled", False) else 0.0
            )
            self.status_blocks["decouple_d_enabled"].update_value(
                1.0 if ctrl_state.get("decouple_d_enabled", False) else 0.0
            )
            self.status_blocks["decouple_q_enabled"].update_value(
                1.0 if ctrl_state.get("decouple_q_enabled", False) else 0.0
            )
            self.status_blocks["observer_mode_code"].update_value(observer_mode_code)
            self.status_blocks["theta_electrical"].update_value(
                ctrl_state.get("theta_electrical", 0.0)
            )
            self.status_blocks["theta_meas_emf"].update_value(
                ctrl_state.get("theta_meas_emf", 0.0)
            )
            self.status_blocks["theta_error_pll"].update_value(
                ctrl_state.get("theta_error_pll", 0.0)
            )
            self.status_blocks["theta_error_smo"].update_value(
                ctrl_state.get("theta_error_smo", 0.0)
            )
            self.status_blocks["smo_omega_est"].update_value(
                ctrl_state.get("smo", {}).get("omega_est", 0.0)
            )
            self.status_blocks["observer_confidence"].update_value(
                ctrl_state.get("observer_confidence", 0.0)
            )
            self.status_blocks["observer_confidence_emf"].update_value(
                ctrl_state.get("observer_confidence_emf", 0.0)
            )
            self.status_blocks["observer_confidence_speed"].update_value(
                ctrl_state.get("observer_confidence_speed", 0.0)
            )
            self.status_blocks["observer_confidence_coherence"].update_value(
                ctrl_state.get("observer_confidence_coherence", 0.0)
            )
            self.status_blocks["observer_confidence_ema"].update_value(
                ctrl_state.get("observer_confidence_ema", 0.0)
            )
            self.status_blocks["observer_confidence_trend"].update_value(
                ctrl_state.get("observer_confidence_trend", 0.0)
            )
            self.status_blocks[
                "observer_confidence_above_threshold_time_s"
            ].update_value(
                ctrl_state.get("observer_confidence_above_threshold_time_s", 0.0)
            )
            self.status_blocks[
                "observer_confidence_below_threshold_time_s"
            ].update_value(
                ctrl_state.get("observer_confidence_below_threshold_time_s", 0.0)
            )
            self.status_blocks["observer_confidence_crossings_up"].update_value(
                ctrl_state.get("observer_confidence_crossings_up", 0.0)
            )
            self.status_blocks["observer_confidence_crossings_down"].update_value(
                ctrl_state.get("observer_confidence_crossings_down", 0.0)
            )
            self.status_blocks["startup_handoff_count"].update_value(
                ctrl_state.get("startup_handoff_count", 0.0)
            )
            self.status_blocks["startup_last_handoff_time_s"].update_value(
                ctrl_state.get("startup_last_handoff_time_s", 0.0)
            )
            self.status_blocks["startup_last_handoff_confidence"].update_value(
                ctrl_state.get("startup_last_handoff_confidence", 0.0)
            )
            self.status_blocks["startup_handoff_confidence_peak"].update_value(
                ctrl_state.get("startup_handoff_confidence_peak", 0.0)
            )
            self.status_blocks["startup_handoff_quality"].update_value(
                ctrl_state.get("startup_handoff_quality", 0.0)
            )
            self.status_blocks["startup_handoff_stability_ratio"].update_value(
                ctrl_state.get("startup_handoff_stability_ratio", 1.0)
            )
            self.status_blocks["pfc_enabled"].update_value(
                1.0 if pfc_state.get("enabled", False) else 0.0
            )
            self.status_blocks["pfc_target_pf"].update_value(
                float(pfc_state.get("target_pf", 0.0))
            )
            self.status_blocks["pfc_power_factor"].update_value(
                float(pfc_state.get("power_factor", 0.0))
            )
            self.status_blocks["pfc_active_power_w"].update_value(
                float(pfc_state.get("active_power_w", 0.0))
            )
            self.status_blocks["pfc_reactive_power_var"].update_value(
                float(pfc_state.get("reactive_power_var", 0.0))
            )
            self.status_blocks["pfc_command_var"].update_value(
                float(pfc_state.get("compensation_command_var", 0.0))
            )
            self.status_blocks["efficiency"].update_value(
                float(efficiency_state.get("efficiency", 0.0))
            )
            self.status_blocks["mechanical_output_power_w"].update_value(
                float(efficiency_state.get("mechanical_output_power_w", 0.0))
            )
            self.status_blocks["total_loss_power_w"].update_value(
                float(efficiency_state.get("total_loss_power_w", 0.0))
            )
            self.status_blocks["time"].update_value(time_val)
        else:
            # Fallback for backward compatibility with old label-based system
            self.status_labels["speed_rpm"].setText(
                f"⚡ Rotor Speed: {speed_val:.2f} RPM"
            )
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
            self.status_labels["id_ref"].setText(
                f"d-axis Current Ref: {ctrl_state.get('id_ref', 0.0):.3f} A"
            )
            self.status_labels["iq_ref"].setText(
                f"q-axis Current Ref: {ctrl_state.get('iq_ref', 0.0):.3f} A"
            )
            self.status_labels["speed_error"].setText(
                f"Speed Error: {ctrl_state.get('speed_error', 0.0):.4f} rad/s"
            )
            self.status_labels["v_d_ff"].setText(
                f"d-axis Feedforward: {ctrl_state.get('v_d_ff', 0.0):.4f} V"
            )
            self.status_labels["v_q_ff"].setText(
                f"q-axis Feedforward: {ctrl_state.get('v_q_ff', 0.0):.4f} V"
            )
            self.status_labels["speed_loop_enabled"].setText(
                f"Speed Loop Enabled: {1 if ctrl_state.get('speed_loop_enabled', False) else 0}"
            )
            self.status_labels["decouple_d_enabled"].setText(
                f"D-axis Decoupling: {1 if ctrl_state.get('decouple_d_enabled', False) else 0}"
            )
            self.status_labels["decouple_q_enabled"].setText(
                f"Q-axis Decoupling: {1 if ctrl_state.get('decouple_q_enabled', False) else 0}"
            )
            self.status_labels["observer_mode_code"].setText(
                f"Observer Mode Code: {observer_mode_code:.0f}"
            )
            self.status_labels["theta_electrical"].setText(
                f"Estimated Electrical Angle: {ctrl_state.get('theta_electrical', 0.0):.4f} rad"
            )
            self.status_labels["theta_meas_emf"].setText(
                f"Back-EMF Angle: {ctrl_state.get('theta_meas_emf', 0.0):.4f} rad"
            )
            self.status_labels["theta_error_pll"].setText(
                f"PLL Angle Error: {ctrl_state.get('theta_error_pll', 0.0):.4f} rad"
            )
            self.status_labels["theta_error_smo"].setText(
                f"SMO Angle Error: {ctrl_state.get('theta_error_smo', 0.0):.4f} rad"
            )
            self.status_labels["smo_omega_est"].setText(
                f"SMO Estimated Speed: {ctrl_state.get('smo', {}).get('omega_est', 0.0):.4f} rad/s"
            )
            self.status_labels["observer_confidence"].setText(
                f"Observer Confidence: {ctrl_state.get('observer_confidence', 0.0):.3f}"
            )
            self.status_labels["observer_confidence_emf"].setText(
                f"Confidence from EMF: {ctrl_state.get('observer_confidence_emf', 0.0):.3f}"
            )
            self.status_labels["observer_confidence_speed"].setText(
                f"Confidence from Speed: {ctrl_state.get('observer_confidence_speed', 0.0):.3f}"
            )
            self.status_labels["observer_confidence_coherence"].setText(
                f"Confidence from Coherence: {ctrl_state.get('observer_confidence_coherence', 0.0):.3f}"
            )
            self.status_labels["observer_confidence_ema"].setText(
                f"Confidence EMA: {ctrl_state.get('observer_confidence_ema', 0.0):.3f}"
            )
            self.status_labels["observer_confidence_trend"].setText(
                f"Confidence Trend: {ctrl_state.get('observer_confidence_trend', 0.0):.4f}"
            )
            self.status_labels["observer_confidence_above_threshold_time_s"].setText(
                f"Confidence Above Threshold Time: {ctrl_state.get('observer_confidence_above_threshold_time_s', 0.0):.3f} s"
            )
            self.status_labels["observer_confidence_below_threshold_time_s"].setText(
                f"Confidence Below Threshold Time: {ctrl_state.get('observer_confidence_below_threshold_time_s', 0.0):.3f} s"
            )
            self.status_labels["observer_confidence_crossings_up"].setText(
                f"Confidence Crossings Up: {int(ctrl_state.get('observer_confidence_crossings_up', 0))}"
            )
            self.status_labels["observer_confidence_crossings_down"].setText(
                f"Confidence Crossings Down: {int(ctrl_state.get('observer_confidence_crossings_down', 0))}"
            )
            self.status_labels["startup_handoff_count"].setText(
                f"Startup Handoff Count: {int(ctrl_state.get('startup_handoff_count', 0))}"
            )
            self.status_labels["startup_last_handoff_time_s"].setText(
                f"Last Handoff Time: {ctrl_state.get('startup_last_handoff_time_s', 0.0):.3f} s"
            )
            self.status_labels["startup_last_handoff_confidence"].setText(
                f"Last Handoff Confidence: {ctrl_state.get('startup_last_handoff_confidence', 0.0):.3f}"
            )
            self.status_labels["startup_handoff_confidence_peak"].setText(
                f"Handoff Confidence Peak: {ctrl_state.get('startup_handoff_confidence_peak', 0.0):.3f}"
            )
            self.status_labels["startup_handoff_quality"].setText(
                f"Handoff Quality KPI: {ctrl_state.get('startup_handoff_quality', 0.0):.3f}"
            )
            self.status_labels["startup_handoff_stability_ratio"].setText(
                f"Handoff Stability Ratio: {ctrl_state.get('startup_handoff_stability_ratio', 1.0):.3f}"
            )
            self.status_labels["efficiency"].setText(
                f"System Efficiency: {efficiency_state.get('efficiency', 0.0):.3f}"
            )
            self.status_labels["mechanical_output_power_w"].setText(
                f"Mechanical Output Power: {efficiency_state.get('mechanical_output_power_w', 0.0):.3f} W"
            )
            self.status_labels["total_loss_power_w"].setText(
                f"Estimated Total Loss: {efficiency_state.get('total_loss_power_w', 0.0):.3f} W"
            )
            self.status_labels["time"].setText(f"Simulation Time: {time_val:.3f} s")

            # Update accessible descriptions
            for lbl in self.status_labels.values():
                try:
                    full_text = lbl.text()
                    lbl.setToolTip(full_text)
                    QAccessible.updateAccessibility(
                        lbl, 0, QAccessible.Event.ValueChange
                    )
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

    def _update_display(self):
        """Update display (for manual updates)."""
        if self.engine:
            state = self.engine.get_current_state()
            info = self.engine.get_simulation_info()
            self._update_monitoring({**state, **info})

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

    def _plot_pfc_analysis(self):
        """Generate dedicated PFC telemetry plot."""
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
        figure = SimulationPlotter.create_pfc_analysis_plot(
            history,
            grid_on=grid_on,
            grid_spacing=grid_spacing,
            minor_grid=minor_grid,
            grid_spacing_y=grid_spacing_y,
        )
        figure.show()
        speak("PFC analysis plot generated.")

    def _plot_efficiency_analysis(self):
        """Generate dedicated efficiency telemetry plot."""
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
        figure = SimulationPlotter.create_efficiency_analysis_plot(
            history,
            grid_on=grid_on,
            grid_spacing=grid_spacing,
            minor_grid=minor_grid,
            grid_spacing_y=grid_spacing_y,
        )
        figure.show()
        speak("Efficiency analysis plot generated.")

    def _show_efficiency_recommendations(self):
        """Show heuristic efficiency tuning suggestions for the current setup."""
        if not self.engine or len(self.engine.get_history()["time"]) == 0:
            QMessageBox.warning(
                self,
                "Warning",
                "Run a simulation first to generate efficiency recommendations.",
            )
            return

        efficiency_state = self.engine.get_efficiency_state()
        pfc_state = self.engine.get_power_factor_control_state()
        rec = recommend_efficiency_adjustments(
            efficiency=efficiency_state.get("efficiency", 0.0),
            power_factor=pfc_state.get("power_factor", 0.0),
            device_drop_v=self.inverter_device_drop.value(),
            dead_time_fraction=self.inverter_dead_time_fraction.value(),
            conduction_resistance_ohm=self.inverter_conduction_resistance.value(),
            switching_frequency_hz=self.inverter_switching_frequency.value(),
            switching_loss_coeff_v_per_a_khz=self.inverter_switching_loss_coeff.value(),
        )
        lines = [
            f"Efficiency: {rec['efficiency']:.3f}",
            f"Power factor: {rec['power_factor']:.3f}",
            "",
            "Suggestions:",
        ]
        lines.extend(f"- {item}" for item in rec["suggestions"])
        QMessageBox.information(self, "Efficiency Tuning Suggestions", "\n".join(lines))

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
