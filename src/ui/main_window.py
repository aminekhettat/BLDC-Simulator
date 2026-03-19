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

import sys
import numpy as np
from typing import Optional
import time
import logging
import json
from threading import Lock
from pathlib import Path
from collections import deque

from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QCheckBox,
    QLabel,
    QMessageBox,
    QFileDialog,
    QTableWidget,
    QTableWidgetItem,
    QScrollArea,
    QFrame,
    QComboBox,
    QTextEdit,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, QProcess

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

from PyQt6.QtGui import (
    QFont,
    QDesktopServices,
    QPixmap,
    QPainter,
    QColor,
    QPen,
    QTextDocument,
    QCloseEvent,
)
from PyQt6.QtCore import QUrl
from PyQt6.QtPrintSupport import QPrinter

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
from src.hardware import MockDAQHardware, InverterCurrentSense, ShuntAmplifierChannel
from src.utils.config import (
    DEFAULT_MOTOR_PARAMS,
    FOC_FIELD_WEAKENING_PARAMS,
    FOC_STARTUP_PARAMS,
    MOTOR_PROFILES_DIR,
    SIMULATION_PARAMS,
    VF_CONTROLLER_PARAMS,
    PLOT_STYLE,
    DEFAULT_LOAD_PROFILE,
)
from src.utils.motor_profiles import (
    list_motor_profiles,
    load_motor_profile,
    save_motor_profile,
)
from src.utils.speech import (
    speak,
    is_audio_assistance_enabled,
    set_audio_assistance_enabled,
)
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
        self._latest_state_lock = Lock()
        self._latest_state: dict = {}
        # start flag will be set when set_simulation is called

    def get_latest_state(self) -> dict:
        """Return the latest simulation snapshot without blocking control loop."""
        with self._latest_state_lock:
            return dict(self._latest_state)

    def set_simulation(
        self,
        engine: SimulationEngine,
        svm: SVMGenerator,
        controller: BaseController,
        max_duration: float = 0.0,
        pwm_frequency_hz: Optional[float] = None,
    ):
        """Assign engine, svm and controller before running thread."""
        self.engine = engine
        self.svm = svm
        self.controller = controller
        self.max_duration = max_duration
        self.svm.set_sample_time(engine.dt)
        resolved_pwm_hz = (
            float(pwm_frequency_hz) if pwm_frequency_hz else (1.0 / engine.dt)
        )
        self.engine.set_pwm_frequency(resolved_pwm_hz)

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
        control_period_s = self.engine.get_control_timing_state().get(
            "control_period_s", dt
        )
        control_period_s = max(float(control_period_s), dt)
        next_control_time = self.engine.time
        last_voltages = np.zeros(3, dtype=np.float64)
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
                    if hasattr(self.engine, "get_controller_phase_currents"):
                        phase_currents = self.engine.get_controller_phase_currents()
                    else:
                        phase_currents = self.engine.motor.currents
                    self.svm.set_phase_currents(phase_currents)
                except AttributeError:
                    pass

            if self.engine.time >= (next_control_time - 1e-15):
                calc_start = time.perf_counter()

                # Get control output (could be polar or cartesian)
                ctrl_out = self.controller.update(control_period_s)
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

                calc_duration_s = time.perf_counter() - calc_start
                self.engine.record_control_timing(calc_duration_s, control_period_s)
                last_voltages = voltages
                next_control_time += control_period_s

            if self.engine is not None:
                self.engine.set_inverter_telemetry(self.svm.get_last_telemetry())
            # Execute motor step
            self.engine.step(last_voltages, log_data=True)

            step_count += 1

            # Periodic snapshot update for GUI polling.
            current_time = time.time()
            if (current_time - last_update) >= self.update_interval:
                state = self.engine.get_current_state()
                info = self.engine.get_simulation_info()
                with self._latest_state_lock:
                    self._latest_state = {**state, **info}
                last_update = current_time

            # No GUI operations are executed in this loop; keep the control
            # path free from GUI timing jitter.

        self.finished_signal.emit()


class CurrentSpectrumWindow(QMainWindow):
    """Dedicated FFT window for controller-facing current harmonics."""

    closed = pyqtSignal()

    def __init__(self, window_size_samples: int = 512, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Current Harmonic FFT")
        self.setMinimumSize(820, 520)

        self._window_size_samples = int(max(64, window_size_samples))
        self._time_samples: deque[float] = deque(maxlen=self._window_size_samples)
        self._ia_samples: deque[float] = deque(maxlen=self._window_size_samples)

        central = QWidget()
        layout = QVBoxLayout()

        self.summary_label = QLabel(
            "Awaiting simulation samples. FFT summary will report dominant frequency and THD."
        )
        self.summary_label.setWordWrap(True)
        layout.addWidget(self.summary_label)

        from matplotlib.figure import Figure
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

        self.figure = Figure(figsize=(7, 4), dpi=90)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Phase-A Current Spectrum (Controller-Facing)")
        self.ax.set_xlabel("Frequency (Hz)")
        self.ax.set_ylabel("Magnitude (A)")
        self.ax.grid(True, alpha=0.3)
        layout.addWidget(self.canvas, 1)

        central.setLayout(layout)
        self.setCentralWidget(central)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_plot)
        self._refresh_timer.start(250)

    def set_window_size(self, samples: int) -> None:
        """Update FFT window length and preserve newest samples when possible."""
        samples = int(max(64, samples))
        if samples == self._window_size_samples:
            return
        self._window_size_samples = samples
        self._time_samples = deque(list(self._time_samples)[-samples:], maxlen=samples)
        self._ia_samples = deque(list(self._ia_samples)[-samples:], maxlen=samples)

    def push_snapshot(self, snapshot: dict) -> None:
        """Consume latest simulation snapshot without blocking control loop."""
        if not isinstance(snapshot, dict):
            return
        try:
            time_s = float(snapshot.get("time", 0.0))
            ia = float(snapshot.get("currents_a", 0.0))
        except (TypeError, ValueError):
            return
        self._time_samples.append(time_s)
        self._ia_samples.append(ia)

    def _refresh_plot(self) -> None:
        """Compute and draw FFT from buffered samples in this window thread context."""
        sample_count = len(self._ia_samples)
        if sample_count < 16:
            return

        time_arr = np.asarray(self._time_samples, dtype=np.float64)
        current_arr = np.asarray(self._ia_samples, dtype=np.float64)

        dt = float(np.median(np.diff(time_arr))) if sample_count > 2 else 0.0
        if dt <= 0.0:
            return

        centered = current_arr - np.mean(current_arr)
        spec = np.fft.rfft(centered)
        freq = np.fft.rfftfreq(centered.size, d=dt)
        mag = np.abs(spec) * (2.0 / max(centered.size, 1))

        # Ignore DC when selecting dominant component.
        if mag.size > 1:
            dominant_idx = int(np.argmax(mag[1:]) + 1)
            fundamental_mag = float(max(mag[dominant_idx], 1e-12))
            harmonic_energy = float(np.sqrt(np.sum(np.square(mag[dominant_idx + 1 :]))))
            thd = (harmonic_energy / fundamental_mag) * 100.0
            dominant_freq = float(freq[dominant_idx])
        else:
            dominant_freq = 0.0
            thd = 0.0

        self.ax.clear()
        self.ax.plot(freq, mag, color="#1E88E5", linewidth=1.5)
        self.ax.set_title("Phase-A Current Spectrum (Controller-Facing)")
        self.ax.set_xlabel("Frequency (Hz)")
        self.ax.set_ylabel("Magnitude (A)")
        self.ax.grid(True, alpha=0.3)
        self.figure.tight_layout()
        self.canvas.draw_idle()

        self.summary_label.setText(
            f"FFT window: {sample_count} samples, dominant frequency: {dominant_freq:.1f} Hz, THD: {thd:.2f}%"
        )

    def closeEvent(self, event) -> None:
        self._refresh_timer.stop()
        self.closed.emit()
        super().closeEvent(event)


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

        APP_NAME = "BLIND SYSTEMS BLDC Simulator"
        APP_VERSION = "2.1.0"

        self.setWindowTitle(
            f"{APP_NAME} - BLDC Motor Control Simulator (v{APP_VERSION})"
        )
        self.setGeometry(100, 100, 1500, 950)

        # Accessibility
        self.setAccessibleName("BLIND SYSTEMS BLDC Simulator")
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

        # Calibration process state
        self.calib_process: Optional[QProcess] = None
        self.calib_output_path: Optional[Path] = None

        # UI state
        self.is_running = False

        # Speed curve history for live plotting
        self.speed_history_time = []
        self.speed_history_rpm = []

        # Optional FFT analysis window for measured currents.
        self.current_fft_window: Optional[CurrentSpectrumWindow] = None

        # Process lifecycle tracking (prevent simultaneous simulation/calibration)
        self._task_lock = Lock()  # Protects concurrent access to task state
        self._running_task_name: Optional[str] = None  # "simulation" or "calibration"

        # Create UI
        self._create_ui()
        self._initialize_defaults()

        # Timer for GUI updates when not using thread
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._poll_simulation_state)

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

        title = QLabel("BLIND SYSTEMS BLDC Simulator - Advanced Motor Control")
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
        self._create_calibration_tab()

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

        # Add status bar with simulation parameters and telemetry
        self.status_bar_dt = QLabel("dt: -- s")
        self.status_bar_tau_e = QLabel("τ_e: -- s")
        self.status_bar_tau_m = QLabel("τ_m: -- s")
        self.status_bar_state = QLabel("State: Ready")
        self.status_bar_time_remaining = QLabel("Remaining: -- s")
        self.status_bar_cpu_load = QLabel("CPU: -- %")
        self.status_bar_task = QLabel("Task: None")

        self.statusBar().addWidget(self.status_bar_state)
        self.statusBar().addWidget(QLabel("|"))  # Separator
        self.statusBar().addWidget(self.status_bar_task)
        self.statusBar().addWidget(QLabel("|"))  # Separator
        self.statusBar().addWidget(self.status_bar_time_remaining)
        self.statusBar().addWidget(QLabel("|"))  # Separator
        self.statusBar().addWidget(self.status_bar_cpu_load)
        self.statusBar().addWidget(QLabel("|"))  # Separator
        self.statusBar().addWidget(self.status_bar_dt)
        self.statusBar().addWidget(QLabel("|"))  # Separator
        self.statusBar().addWidget(self.status_bar_tau_e)
        self.statusBar().addWidget(QLabel("|"))  # Separator
        self.statusBar().addWidget(self.status_bar_tau_m)

    def _create_menu_bar(self):
        """Create application menu bar with File, Option, and Help menus."""
        menubar = self.menuBar()

        # File Menu (mnemonic Alt+F)
        file_menu = menubar.addMenu("&File")

        export_action = file_menu.addAction("&Export Simulation Data")
        export_action.setShortcut("Ctrl+S")
        export_action.triggered.connect(self._export_data)

        file_menu.addSeparator()

        import_motor_action = file_menu.addAction("&Import Motor Parameters...")
        import_motor_action.triggered.connect(self._import_motor_parameters)

        save_motor_action = file_menu.addAction("&Save Motor Parameters...")
        save_motor_action.triggered.connect(self._save_motor_parameters)

        save_sim_params_action = file_menu.addAction("Save &Simulation Parameters...")
        save_sim_params_action.triggered.connect(self._save_simulation_parameters)

        self.builtin_motor_menu = file_menu.addMenu("Load Built-in Motor Profile")
        self._refresh_builtin_motor_menu()

        file_menu.addSeparator()

        params_action = file_menu.addAction("&Simulation Parameters")
        params_action.triggered.connect(self._show_simulation_params)

        file_menu.addSeparator()

        quit_action = file_menu.addAction("&Quit")
        quit_action.setShortcut("Ctrl+Q")
        quit_action.triggered.connect(self.close)

        # Option Menu (mnemonic Alt+O)
        option_menu = menubar.addMenu("&Option")
        reset_action = option_menu.addAction("Reset Simulation (F7)")
        reset_action.setShortcut("F7")
        reset_action.triggered.connect(self._reset_simulation)

        self.audio_assistance_action = option_menu.addAction("Audio Assistance Enabled")
        self.audio_assistance_action.setCheckable(True)
        self.audio_assistance_action.setChecked(is_audio_assistance_enabled())
        self.audio_assistance_action.triggered.connect(self._toggle_audio_assistance)

        # Help Menu (mnemonic Alt+H)
        help_menu = menubar.addMenu("&Help")
        html_help_action = help_menu.addAction("Open HTML Help (Sphinx)")
        html_help_action.triggered.connect(self._open_html_help)

        pdf_manual_action = help_menu.addAction("Open PDF User Manual")
        pdf_manual_action.triggered.connect(self._open_user_manual_pdf)

        help_menu.addSeparator()
        about_action = help_menu.addAction("About")
        about_action.triggered.connect(self._show_about)

    def _refresh_builtin_motor_menu(self):
        """Populate built-in motor profile submenu from data/motor_profiles."""
        self.builtin_motor_menu.clear()
        profile_paths = list_motor_profiles(MOTOR_PROFILES_DIR)
        if not profile_paths:
            empty_action = self.builtin_motor_menu.addAction(
                "No built-in profiles found"
            )
            empty_action.setEnabled(False)
            return

        for profile_path in profile_paths:
            action = self.builtin_motor_menu.addAction(profile_path.stem)
            action.triggered.connect(
                lambda checked=False, p=profile_path: (
                    self._load_motor_profile_from_path(p)
                )
            )

    def _collect_current_motor_parameters(self) -> dict:
        """Collect current motor parameter values from UI widgets."""
        return {
            "nominal_voltage": float(self.param_voltage.value()),
            "phase_resistance": float(self.param_resistance.value()),
            "phase_inductance": float(self.param_inductance.value()),
            "back_emf_constant": float(self.param_emf.value()),
            "torque_constant": float(self.param_kt.value()),
            "rotor_inertia": float(self.param_inertia.value()),
            "friction_coefficient": float(self.param_friction.value()),
            "num_poles": int(self.param_poles.value()),
            "ld": float(self.param_ld.value()),
            "lq": float(self.param_lq.value()),
            "model_type": self.param_model_type.currentText(),
            "emf_shape": self.param_emf_shape.currentText(),
        }

    def _apply_motor_profile(self, profile: dict):
        """Apply a loaded motor profile to the UI controls."""
        params = profile["motor_params"]
        self.param_voltage.setValue(float(params["nominal_voltage"]))
        self.param_resistance.setValue(float(params["phase_resistance"]))
        self.param_inductance.setValue(float(params["phase_inductance"]))
        self.param_emf.setValue(float(params["back_emf_constant"]))
        self.param_kt.setValue(float(params["torque_constant"]))
        self.param_inertia.setValue(float(params["rotor_inertia"]))
        self.param_friction.setValue(float(params["friction_coefficient"]))
        self.param_poles.setValue(int(params["num_poles"]))
        self.param_ld.setValue(float(params.get("ld", params["phase_inductance"])))
        self.param_lq.setValue(float(params.get("lq", params["phase_inductance"])))
        self.param_model_type.setCurrentText(params.get("model_type", "dq"))
        self.param_emf_shape.setCurrentText(params.get("emf_shape", "sinusoidal"))

    def _import_motor_parameters(self):
        """Import motor parameters from a JSON profile file."""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Import Motor Parameters",
            str(MOTOR_PROFILES_DIR),
            "JSON Files (*.json)",
        )
        if not filename:
            return

        try:
            profile = load_motor_profile(Path(filename))
            self._apply_motor_profile(profile)
            self._apply_to_simulation()
            QMessageBox.information(
                self,
                "Motor Parameters Imported",
                f"Loaded profile: {profile['profile_name']}",
            )
            speak("Motor parameters imported successfully.")
        except Exception as exc:
            QMessageBox.critical(
                self, "Import Error", f"Failed to import profile: {exc}"
            )

    def _save_motor_parameters(self):
        """Save current motor parameters to a JSON profile file."""
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Motor Parameters",
            str(MOTOR_PROFILES_DIR / "custom_motor_profile.json"),
            "JSON Files (*.json)",
        )
        if not filename:
            return

        file_path = Path(filename)
        if file_path.suffix.lower() != ".json":
            file_path = file_path.with_suffix(".json")

        profile_name = file_path.stem
        rated_info = {
            "rated_voltage_v": float(self.param_voltage.value()),
            "pole_pairs": int(self.param_poles.value() / 2),
        }
        source_info = {
            "origin": "user_saved_profile",
        }

        try:
            save_motor_profile(
                file_path=file_path,
                motor_params=self._collect_current_motor_parameters(),
                profile_name=profile_name,
                rated_info=rated_info,
                source_info=source_info,
            )
            self._refresh_builtin_motor_menu()
            QMessageBox.information(
                self,
                "Motor Parameters Saved",
                f"Saved profile: {file_path.name}",
            )
            speak("Motor parameters saved successfully.")
        except Exception as exc:
            QMessageBox.critical(self, "Save Error", f"Failed to save profile: {exc}")

    def _load_motor_profile_from_path(self, profile_path: Path):
        """Load a built-in motor profile by path."""
        try:
            profile = load_motor_profile(profile_path)
            self._apply_motor_profile(profile)
            self._apply_to_simulation()
            QMessageBox.information(
                self,
                "Built-in Motor Profile Loaded",
                f"Loaded profile: {profile['profile_name']}",
            )
            speak("Built-in motor profile loaded.")
        except Exception as exc:
            QMessageBox.critical(self, "Load Error", f"Failed to load profile: {exc}")

    def _toggle_audio_assistance(self, enabled: bool):
        """Enable or disable spoken assistance from the option menu."""
        set_audio_assistance_enabled(bool(enabled))
        state = "enabled" if enabled else "disabled"
        QMessageBox.information(
            self,
            "Audio Assistance",
            f"Audio assistance is now {state}.",
        )
        speak(f"Audio assistance {state}.")

    def _open_html_help(self):
        """Open generated Sphinx HTML documentation index."""
        html_index = (
            Path(__file__).resolve().parents[2]
            / "docs"
            / "_build"
            / "html"
            / "index.html"
        )
        if not html_index.exists():
            QMessageBox.warning(
                self,
                "HTML Help Not Found",
                "Sphinx HTML help was not found at docs/_build/html/index.html.\n"
                "Generate it with: sphinx-build -b html docs docs/_build/html",
            )
            speak("HTML help not found. Please build Sphinx documentation first.")
            return

        QDesktopServices.openUrl(QUrl.fromLocalFile(str(html_index)))
        speak("HTML help opened.")

    def _ensure_user_manual_pdf(self) -> Path:
        """Generate a PDF user manual when missing and return its path."""
        project_root = Path(__file__).resolve().parents[2]
        pdf_path = project_root / "docs" / "BLIND_SYSTEMS_User_Manual.pdf"
        if pdf_path.exists():
            return pdf_path

        html = (
            "<h1>BLIND SYSTEMS BLDC Simulator - User Manual</h1>"
            "<p><b>Version:</b> 2.1.0</p>"
            "<p><b>Author:</b> Amine Khettat</p>"
            "<h2>1. Getting Started</h2>"
            "<p>Configure motor, load and controller parameters, then start simulation.</p>"
            "<h2>2. Accessibility</h2>"
            "<p>Use Option -> Audio Assistance Enabled to toggle speech output.</p>"
            "<h2>3. Running and Monitoring</h2>"
            "<p>Start: F5, Stop: F6, Reset: F7. Monitor speed, torque, convergence and CPU load metrics.</p>"
            "<h2>4. Data Export</h2>"
            "<p>Use File -> Export Simulation Data or Ctrl+S.</p>"
            "<h2>5. Help</h2>"
            "<p>Open HTML help from Sphinx or this PDF from the Help menu.</p>"
            "<hr/>"
            "<p>Copyright 2026 BLIND SYSTEMS</p>"
        )

        printer = QPrinter(QPrinter.PrinterMode.HighResolution)
        printer.setOutputFormat(QPrinter.OutputFormat.PdfFormat)
        printer.setOutputFileName(str(pdf_path))
        document = QTextDocument()
        document.setHtml(html)
        document.print(printer)
        return pdf_path

    def _open_user_manual_pdf(self):
        """Open generated PDF user manual, generating it if required."""
        pdf_path = self._ensure_user_manual_pdf()
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(pdf_path)))
        speak("PDF user manual opened.")

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

        pwm_hz = 1.0 / dt_val if dt_val > 0.0 else 0.0
        msg = f"Simulation time step (dt): {dt_val} s\n"
        msg += f"PWM frequency: {pwm_hz:.1f} Hz\n"
        if tau_e is not None:
            msg += f"Electrical time constant (L/R): {tau_e:.6f} s\n"
        else:
            msg += "Electrical time constant (L/R): n/a\n"
        if tau_m is not None:
            msg += f"Mechanical time constant (J/b): {tau_m:.6f} s\n"
        else:
            msg += "Mechanical time constant (J/b): n/a\n"

        return msg

    def _collect_simulation_configuration(self) -> dict:
        """Collect complete simulation configuration from current UI state."""
        now_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        switching_frequency_hz = float(self.inverter_switching_frequency.value())
        dt_s = 1.0 / switching_frequency_hz if switching_frequency_hz > 0.0 else None

        config = {
            "schema": "bldc.simulation_config.v1",
            "created_utc": now_utc,
            "control_mode": self.ctrl_mode.currentText(),
            "simulation": {
                "duration_s": float(self.sim_duration.value()),
                "switching_frequency_hz": switching_frequency_hz,
                "dt_s": dt_s,
                "dc_voltage_v": float(self.supply_constant_voltage.value())
                if hasattr(self, "supply_constant_voltage")
                else float(SIMULATION_PARAMS.get("dc_voltage", 48.0)),
            },
            "motor_params": {
                "nominal_voltage": float(self.param_voltage.value()),
                "phase_resistance": float(self.param_resistance.value()),
                "phase_inductance": float(self.param_inductance.value()),
                "back_emf_constant": float(self.param_emf.value()),
                "torque_constant": float(self.param_kt.value()),
                "rotor_inertia": float(self.param_inertia.value()),
                "friction_coefficient": float(self.param_friction.value()),
                "num_poles": int(self.param_poles.value()),
                "pole_pairs": int(self.param_poles.value() / 2),
                "ld": float(self.param_ld.value()),
                "lq": float(self.param_lq.value()),
                "model_type": self.param_model_type.currentText(),
                "emf_shape": self.param_emf_shape.currentText(),
            },
            "load_profile": {
                "type": self.load_type.currentText(),
                "constant_torque_nm": float(self.load_constant_torque.value()),
                "ramp_initial_torque_nm": float(self.load_initial_torque.value()),
                "ramp_final_torque_nm": float(self.load_final_torque.value()),
                "ramp_duration_s": float(self.load_ramp_duration.value()),
            },
            "supply_profile": {
                "type": self.supply_type.currentText()
                if hasattr(self, "supply_type")
                else "Constant",
                "constant_voltage_v": float(self.supply_constant_voltage.value())
                if hasattr(self, "supply_constant_voltage")
                else float(SIMULATION_PARAMS.get("dc_voltage", 48.0)),
                "ramp_initial_v": float(self.supply_ramp_initial.value())
                if hasattr(self, "supply_ramp_initial")
                else 0.0,
                "ramp_final_v": float(self.supply_ramp_final.value())
                if hasattr(self, "supply_ramp_final")
                else 0.0,
                "ramp_duration_s": float(self.supply_ramp_duration.value())
                if hasattr(self, "supply_ramp_duration")
                else 0.0,
            },
            "vf_controller": {
                "v_nominal": float(self.vf_v_nominal.value()),
                "f_nominal": float(self.vf_f_nominal.value()),
                "speed_ref_rpm": float(self.vf_speed_ref.value()),
                "startup_voltage_v": float(self.vf_startup_voltage.value()),
                "frequency_slew_hz_per_s": float(self.vf_freq_slew.value()),
                "startup_sequence_enabled": self.vf_startup_sequence_mode.currentText()
                == "Enabled",
                "align_duration_s": float(self.vf_align_time.value()),
                "align_voltage_v": float(self.vf_align_voltage.value()),
                "align_angle_deg": float(self.vf_align_angle.value()),
                "ramp_initial_frequency_hz": float(
                    self.vf_ramp_initial_frequency.value()
                ),
            },
            "foc_controller": {
                "transform": self.foc_transform.currentText(),
                "output_mode": self.foc_output_mode.currentText(),
                "speed_loop_mode": self.foc_speed_loop_mode.currentText(),
                "id_ref_a": float(self.foc_id_ref.value()),
                "iq_ref_a": float(self.foc_iq_ref.value()),
                "speed_ref_rpm": float(self.foc_speed_ref.value()),
                "iq_limit_a": float(self.foc_iq_limit.value()),
                "speed_pi": {
                    "kp": float(self.foc_speed_kp.value()),
                    "ki": float(self.foc_speed_ki.value()),
                },
                "current_pi": {
                    "d_kp": float(self.foc_d_kp.value()),
                    "d_ki": float(self.foc_d_ki.value()),
                    "q_kp": float(self.foc_q_kp.value()),
                    "q_ki": float(self.foc_q_ki.value()),
                },
                "decoupling": {
                    "enable_d": self.foc_decouple_d_mode.currentText() == "Enabled",
                    "enable_q": self.foc_decouple_q_mode.currentText() == "Enabled",
                },
                "observer": {
                    "mode": self.foc_angle_observer_mode.currentText(),
                    "pll_kp": float(self.foc_pll_kp.value()),
                    "pll_ki": float(self.foc_pll_ki.value()),
                    "smo_k_slide": float(self.foc_smo_k_slide.value()),
                    "smo_lpf_alpha": float(self.foc_smo_lpf_alpha.value()),
                    "smo_boundary": float(self.foc_smo_boundary.value()),
                },
                "startup_sequence": {
                    "enabled": self.foc_startup_sequence_mode.currentText()
                    == "Enabled",
                    "align_duration_s": float(self.foc_align_time.value()),
                    "align_current_a": float(self.foc_align_current.value()),
                    "align_angle_deg": float(self.foc_align_angle.value()),
                    "open_loop_initial_speed_rpm": float(
                        self.foc_open_loop_initial_speed.value()
                    ),
                    "open_loop_target_speed_rpm": float(
                        self.foc_open_loop_target_speed.value()
                    ),
                    "open_loop_ramp_time_s": float(
                        self.foc_open_loop_ramp_time.value()
                    ),
                    "open_loop_id_ref_a": float(self.foc_open_loop_id_ref.value()),
                    "open_loop_iq_ref_a": float(self.foc_open_loop_iq_ref.value()),
                },
                "startup_transition": {
                    "enabled": self.foc_startup_transition_mode.currentText()
                    == "Enabled",
                    "initial_mode": self.foc_startup_initial_observer.currentText(),
                    "min_speed_rpm": float(self.foc_startup_min_speed.value()),
                    "min_elapsed_s": float(self.foc_startup_min_time.value()),
                    "min_emf_v": float(self.foc_startup_min_emf.value()),
                    "min_confidence": float(self.foc_startup_min_confidence.value()),
                    "confidence_hold_s": float(
                        self.foc_startup_confidence_hold.value()
                    ),
                    "confidence_hysteresis": float(
                        self.foc_startup_confidence_hysteresis.value()
                    ),
                    "fallback_enabled": self.foc_startup_fallback_mode.currentText()
                    == "Enabled",
                    "fallback_hold_s": float(self.foc_startup_fallback_hold.value()),
                },
                "field_weakening": {
                    "enabled": self.foc_field_weakening_mode.currentText() == "Enabled",
                    "start_speed_rpm": float(
                        self.foc_field_weakening_start_speed.value()
                    ),
                    "gain": float(self.foc_field_weakening_gain.value()),
                    "max_negative_id_a": float(self.foc_field_weakening_max_id.value()),
                    "headroom_target_v": float(
                        self.foc_field_weakening_headroom_target.value()
                    ),
                },
            },
            "pfc": {
                "enabled": self.pfc_mode.currentText() == "Enabled",
                "target_pf": float(self.pfc_target_pf.value()),
                "kp": float(self.pfc_kp.value()),
                "ki": float(self.pfc_ki.value()),
                "max_compensation_var": float(self.pfc_max_var.value()),
                "window_samples": int(self.pfc_window_samples.value()),
            },
            "inverter_params": self.svm.get_realism_state() if self.svm else {},
            "communication_params": {
                "enabled": bool(
                    hasattr(self, "hardware_enable_backend")
                    and self.hardware_enable_backend.isChecked()
                ),
                "backend": self.hardware_backend_type.currentText()
                if hasattr(self, "hardware_backend_type")
                else "none",
                "mock_noise_std": float(self.hardware_noise_std.value())
                if hasattr(self, "hardware_noise_std")
                else 0.0,
                "mock_seed": int(self.hardware_seed.value())
                if hasattr(self, "hardware_seed")
                else 0,
            },
        }

        return config

    def _save_simulation_parameters(self):
        """Save full simulation configuration as JSON."""
        default_path = Path("data") / "logs" / "simulation_parameters.json"
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Simulation Parameters",
            str(default_path),
            "JSON Files (*.json)",
        )
        if not filename:
            return

        file_path = Path(filename)
        if file_path.suffix.lower() != ".json":
            file_path = file_path.with_suffix(".json")

        try:
            payload = self._collect_simulation_configuration()
            file_path.parent.mkdir(parents=True, exist_ok=True)
            file_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            QMessageBox.information(
                self,
                "Simulation Parameters Saved",
                f"Saved configuration: {file_path.name}",
            )
            speak("Simulation parameters saved successfully.")
        except Exception as exc:
            QMessageBox.critical(
                self,
                "Save Error",
                f"Failed to save simulation parameters: {exc}",
            )

    def _show_about(self):
        """Show about dialog."""
        about_text = (
            "<h2>BLIND SYSTEMS BLDC Simulator v2.1.0</h2>"
            "<p><b>Advanced BLDC Motor Control Simulator</b></p>"
            "<p><b>Author:</b> Amine Khettat</p>"
            "<p><b>Copyright:</b> 2026 BLIND SYSTEMS</p>"
            "<p>Includes accessible audio assistance and real-time control diagnostics.</p>"
        )

        msg = QMessageBox(self)
        msg.setWindowTitle("About")
        msg.setTextFormat(Qt.TextFormat.RichText)
        msg.setText(about_text)

        project_root = Path(__file__).resolve().parents[2]
        candidate_logos = [
            project_root / "docs" / "logo.png",
            project_root / "data" / "logo.png",
        ]
        logo_path = next((p for p in candidate_logos if p.exists()), None)
        if logo_path is not None:
            pix = QPixmap(str(logo_path))
            if not pix.isNull():
                msg.setIconPixmap(
                    pix.scaled(220, 80, Qt.AspectRatioMode.KeepAspectRatio)
                )
        else:
            # Fallback synthetic logo if no file is present yet.
            pix = QPixmap(240, 70)
            pix.fill(QColor("white"))
            painter = QPainter(pix)
            painter.setPen(QPen(QColor("#0A0A0A"), 2))
            painter.drawRect(1, 1, 238, 68)
            painter.setPen(QPen(QColor("#0A3D91"), 2))
            painter.setFont(QFont("Segoe UI", 14, QFont.Weight.Bold))
            painter.drawText(pix.rect(), Qt.AlignmentFlag.AlignCenter, "BLIND SYSTEMS")
            painter.end()
            msg.setIconPixmap(pix)

        msg.exec()

    def _show_guide(self):
        """Show quick start guide."""
        guide_text = (
            "<h3>Quick Start Guide - BLIND SYSTEMS BLDC Simulator</h3>"
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

        self.vf_startup_sequence_mode = LabeledComboBox(
            "Startup Sequence",
            items=["Disabled", "Enabled"],
            description="Apply a standard V/f startup with rotor alignment, open-loop ramp, then steady V/f operation.",
        )
        self.vf_startup_sequence_mode.setCurrentText(
            "Enabled"
            if VF_CONTROLLER_PARAMS["startup_sequence_enabled"]
            else "Disabled"
        )
        self.vf_group_layout.addWidget(self.vf_startup_sequence_mode)

        self.vf_align_time = LabeledSpinBox(
            "Alignment Time",
            min_val=0.0,
            max_val=2.0,
            initial=VF_CONTROLLER_PARAMS["startup_align_duration_s"],
            step=0.005,
            decimals=3,
            suffix=" s",
            description="Fixed-angle pre-magnetization time before the V/f ramp starts.",
        )
        self.vf_group_layout.addWidget(self.vf_align_time)

        self.vf_align_voltage = LabeledSpinBox(
            "Alignment Voltage",
            min_val=0.0,
            max_val=20.0,
            initial=VF_CONTROLLER_PARAMS["startup_align_voltage_v"],
            step=0.1,
            decimals=2,
            suffix=" V",
            description="Voltage magnitude applied during rotor alignment.",
        )
        self.vf_group_layout.addWidget(self.vf_align_voltage)

        self.vf_align_angle = LabeledSpinBox(
            "Alignment Angle",
            min_val=0.0,
            max_val=360.0,
            initial=VF_CONTROLLER_PARAMS["startup_align_angle_deg"],
            step=1.0,
            decimals=1,
            suffix=" deg",
            description="Electrical angle held during the alignment phase.",
        )
        self.vf_group_layout.addWidget(self.vf_align_angle)

        self.vf_ramp_initial_frequency = LabeledSpinBox(
            "Ramp Initial Frequency",
            min_val=0.0,
            max_val=100.0,
            initial=VF_CONTROLLER_PARAMS["startup_ramp_initial_frequency_hz"],
            step=0.5,
            decimals=2,
            suffix=" Hz",
            description="Initial forced frequency used immediately after alignment.",
        )
        self.vf_group_layout.addWidget(self.vf_ramp_initial_frequency)

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

        self.foc_field_weakening_mode = LabeledComboBox(
            "Field Weakening",
            items=["Disabled", "Enabled"],
            description="Independent field-weakening feature toggle. When enabled, additional negative d-axis current is injected above the configured speed threshold.",
        )
        self.foc_field_weakening_mode.setCurrentText(
            "Enabled" if FOC_FIELD_WEAKENING_PARAMS["enabled"] else "Disabled"
        )
        self.foc_group_layout.addWidget(self.foc_field_weakening_mode)

        self.foc_field_weakening_start_speed = LabeledSpinBox(
            "FW Start Speed",
            min_val=0.0,
            max_val=100000.0,
            initial=FOC_FIELD_WEAKENING_PARAMS["start_speed_rpm"],
            step=50.0,
            decimals=1,
            suffix=" RPM",
            description="Speed threshold above which field weakening begins injecting negative d-axis current.",
        )
        self.foc_group_layout.addWidget(self.foc_field_weakening_start_speed)

        self.foc_field_weakening_gain = LabeledSpinBox(
            "FW Gain",
            min_val=0.0,
            max_val=10.0,
            initial=FOC_FIELD_WEAKENING_PARAMS["gain"],
            step=0.05,
            decimals=3,
            suffix="",
            description="Scaling factor used by the field-weakening scheduler.",
        )
        self.foc_group_layout.addWidget(self.foc_field_weakening_gain)

        self.foc_field_weakening_max_id = LabeledSpinBox(
            "FW Max Negative Id",
            min_val=0.0,
            max_val=100.0,
            initial=FOC_FIELD_WEAKENING_PARAMS["max_negative_id_a"],
            step=0.1,
            decimals=2,
            suffix=" A",
            description="Maximum additional negative d-axis current magnitude injected by field weakening.",
        )
        self.foc_group_layout.addWidget(self.foc_field_weakening_max_id)

        self.foc_field_weakening_headroom_target = LabeledSpinBox(
            "FW Target Headroom",
            min_val=0.0,
            max_val=100.0,
            initial=FOC_FIELD_WEAKENING_PARAMS["headroom_target_v"],
            step=0.05,
            decimals=3,
            suffix=" V",
            description="Desired dq voltage reserve maintained by FW. Lower values maximize speed range; higher values preserve control margin.",
        )
        self.foc_group_layout.addWidget(self.foc_field_weakening_headroom_target)

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

        self.foc_startup_sequence_mode = LabeledComboBox(
            "Startup Sequence",
            items=["Disabled", "Enabled"],
            description="Apply a standard FOC startup: align rotor, use open-loop ramp if sensorless, then hand off to closed-loop observer control.",
        )
        self.foc_startup_sequence_mode.setCurrentText(
            "Enabled" if FOC_STARTUP_PARAMS["startup_sequence_enabled"] else "Disabled"
        )
        self.foc_group_layout.addWidget(self.foc_startup_sequence_mode)

        self.foc_align_time = LabeledSpinBox(
            "Alignment Time",
            min_val=0.0,
            max_val=2.0,
            initial=FOC_STARTUP_PARAMS["startup_align_duration_s"],
            step=0.005,
            decimals=3,
            suffix=" s",
            description="Duration of the initial rotor alignment phase.",
        )
        self.foc_group_layout.addWidget(self.foc_align_time)

        self.foc_align_current = LabeledSpinBox(
            "Alignment Current",
            min_val=0.0,
            max_val=20.0,
            initial=FOC_STARTUP_PARAMS["startup_align_current_a"],
            step=0.1,
            decimals=2,
            suffix=" A",
            description="d-axis current used to lock the rotor before acceleration.",
        )
        self.foc_group_layout.addWidget(self.foc_align_current)

        self.foc_align_angle = LabeledSpinBox(
            "Alignment Angle",
            min_val=0.0,
            max_val=360.0,
            initial=FOC_STARTUP_PARAMS["startup_align_angle_deg"],
            step=1.0,
            decimals=1,
            suffix=" deg",
            description="Electrical angle held during rotor alignment.",
        )
        self.foc_group_layout.addWidget(self.foc_align_angle)

        self.foc_open_loop_initial_speed = LabeledSpinBox(
            "Open-Loop Initial Speed",
            min_val=0.0,
            max_val=5000.0,
            initial=FOC_STARTUP_PARAMS["startup_open_loop_initial_speed_rpm"],
            step=5.0,
            decimals=1,
            suffix=" RPM",
            description="Forced mechanical speed used at the beginning of the sensorless ramp.",
        )
        self.foc_group_layout.addWidget(self.foc_open_loop_initial_speed)

        self.foc_open_loop_target_speed = LabeledSpinBox(
            "Open-Loop Target Speed",
            min_val=0.0,
            max_val=10000.0,
            initial=FOC_STARTUP_PARAMS["startup_open_loop_target_speed_rpm"],
            step=10.0,
            decimals=1,
            suffix=" RPM",
            description="Forced mechanical speed reached before closed-loop observer takeover.",
        )
        self.foc_group_layout.addWidget(self.foc_open_loop_target_speed)

        self.foc_open_loop_ramp_time = LabeledSpinBox(
            "Open-Loop Ramp Time",
            min_val=0.0,
            max_val=5.0,
            initial=FOC_STARTUP_PARAMS["startup_open_loop_ramp_time_s"],
            step=0.01,
            decimals=3,
            suffix=" s",
            description="Duration of the forced-angle acceleration ramp.",
        )
        self.foc_group_layout.addWidget(self.foc_open_loop_ramp_time)

        self.foc_open_loop_id_ref = LabeledSpinBox(
            "Open-Loop d-axis Ref",
            min_val=-20.0,
            max_val=20.0,
            initial=FOC_STARTUP_PARAMS["startup_open_loop_id_ref_a"],
            step=0.1,
            decimals=2,
            suffix=" A",
            description="d-axis current reference used during forced open-loop acceleration.",
        )
        self.foc_group_layout.addWidget(self.foc_open_loop_id_ref)

        self.foc_open_loop_iq_ref = LabeledSpinBox(
            "Open-Loop q-axis Ref",
            min_val=-20.0,
            max_val=20.0,
            initial=FOC_STARTUP_PARAMS["startup_open_loop_iq_ref_a"],
            step=0.1,
            decimals=2,
            suffix=" A",
            description="q-axis current reference used to generate torque during the forced ramp.",
        )
        self.foc_group_layout.addWidget(self.foc_open_loop_iq_ref)

        self.foc_startup_transition_mode = LabeledComboBox(
            "Observer Startup Transition",
            items=["Disabled", "Enabled"],
            description="Automatic observer takeover criteria used after the forced open-loop ramp or for legacy observer-only startup mode.",
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

        # Feature toggles let the user dial the realism level up or down without
        # losing any tuned numerical parameters.
        self.inverter_enable_device_drop = QCheckBox("Enable Device Drop")
        self.inverter_enable_device_drop.setChecked(False)
        inverter_layout.addWidget(self.inverter_enable_device_drop)

        self.inverter_enable_dead_time = QCheckBox("Enable Dead-Time Distortion")
        self.inverter_enable_dead_time.setChecked(False)
        inverter_layout.addWidget(self.inverter_enable_dead_time)

        self.inverter_enable_conduction = QCheckBox("Enable Conduction Loss")
        self.inverter_enable_conduction.setChecked(False)
        inverter_layout.addWidget(self.inverter_enable_conduction)

        self.inverter_enable_switching = QCheckBox("Enable Switching Loss")
        self.inverter_enable_switching.setChecked(False)
        inverter_layout.addWidget(self.inverter_enable_switching)

        self.inverter_enable_diode = QCheckBox("Enable Freewheel Diode Path")
        self.inverter_enable_diode.setChecked(False)
        inverter_layout.addWidget(self.inverter_enable_diode)

        self.inverter_enable_min_pulse = QCheckBox("Enable Minimum Pulse Suppression")
        self.inverter_enable_min_pulse.setChecked(False)
        inverter_layout.addWidget(self.inverter_enable_min_pulse)

        self.inverter_enable_bus_ripple = QCheckBox("Enable DC-Link Ripple")
        self.inverter_enable_bus_ripple.setChecked(False)
        inverter_layout.addWidget(self.inverter_enable_bus_ripple)

        self.inverter_enable_thermal = QCheckBox("Enable Thermal Coupling")
        self.inverter_enable_thermal.setChecked(False)
        inverter_layout.addWidget(self.inverter_enable_thermal)

        self.inverter_enable_phase_asymmetry = QCheckBox("Enable Phase Asymmetry")
        self.inverter_enable_phase_asymmetry.setChecked(False)
        inverter_layout.addWidget(self.inverter_enable_phase_asymmetry)

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
            initial=SIMULATION_PARAMS.get("pwm_frequency_hz", 20000.0),
            step=500.0,
            decimals=1,
            suffix=" Hz",
            description="PWM switching frequency; also sets control update period for FOC and V/f loops.",
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

        self.inverter_diode_drop = LabeledSpinBox(
            "Diode Drop",
            min_val=0.0,
            max_val=10.0,
            initial=0.0,
            step=0.01,
            decimals=3,
            suffix=" V",
            description="Additional freewheel diode drop when current polarity opposes commanded phase voltage.",
        )
        inverter_layout.addWidget(self.inverter_diode_drop)

        self.inverter_diode_resistance = LabeledSpinBox(
            "Diode Resistance",
            min_val=0.0,
            max_val=5.0,
            initial=0.0,
            step=0.001,
            decimals=4,
            suffix=" Ohm",
            description="Series resistance used in the freewheel diode path model.",
        )
        inverter_layout.addWidget(self.inverter_diode_resistance)

        self.inverter_min_pulse_fraction = LabeledSpinBox(
            "Min Pulse Fraction",
            min_val=0.0,
            max_val=0.5,
            initial=0.0,
            step=0.001,
            decimals=4,
            suffix=" pu",
            description="Commands smaller than this fraction of half-bus voltage are suppressed to emulate minimum PWM on-time.",
        )
        inverter_layout.addWidget(self.inverter_min_pulse_fraction)

        self.inverter_dc_link_capacitance = LabeledSpinBox(
            "DC-Link Capacitance",
            min_val=0.0,
            max_val=1.0,
            initial=0.0,
            step=0.0001,
            decimals=5,
            suffix=" F",
            description="Capacitance used by the reduced-order DC-link ripple model.",
        )
        inverter_layout.addWidget(self.inverter_dc_link_capacitance)

        self.inverter_dc_link_source_resistance = LabeledSpinBox(
            "DC-Link Source Resistance",
            min_val=0.0,
            max_val=5.0,
            initial=0.0,
            step=0.001,
            decimals=4,
            suffix=" Ohm",
            description="Source resistance that recharges the DC-link capacitor in the ripple model.",
        )
        inverter_layout.addWidget(self.inverter_dc_link_source_resistance)

        self.inverter_dc_link_esr = LabeledSpinBox(
            "DC-Link ESR",
            min_val=0.0,
            max_val=5.0,
            initial=0.0,
            step=0.001,
            decimals=4,
            suffix=" Ohm",
            description="Equivalent series resistance used to compute instantaneous DC-link sag.",
        )
        inverter_layout.addWidget(self.inverter_dc_link_esr)

        self.inverter_thermal_resistance = LabeledSpinBox(
            "Thermal Resistance",
            min_val=0.0,
            max_val=20.0,
            initial=0.0,
            step=0.01,
            decimals=3,
            suffix=" K/W",
            description="Equivalent junction-to-ambient thermal resistance for inverter devices.",
        )
        inverter_layout.addWidget(self.inverter_thermal_resistance)

        self.inverter_thermal_capacitance = LabeledSpinBox(
            "Thermal Capacitance",
            min_val=0.001,
            max_val=10000.0,
            initial=1.0,
            step=0.1,
            decimals=3,
            suffix=" J/K",
            description="Thermal capacitance used by the reduced-order junction temperature model.",
        )
        inverter_layout.addWidget(self.inverter_thermal_capacitance)

        self.inverter_ambient_temp = LabeledSpinBox(
            "Ambient Temperature",
            min_val=-40.0,
            max_val=200.0,
            initial=25.0,
            step=1.0,
            decimals=1,
            suffix=" C",
            description="Ambient temperature used by the inverter thermal model.",
        )
        inverter_layout.addWidget(self.inverter_ambient_temp)

        self.inverter_temp_coeff_resistance = LabeledSpinBox(
            "Resistance Temp Coeff",
            min_val=0.0,
            max_val=0.05,
            initial=0.0,
            step=0.0005,
            decimals=5,
            suffix=" 1/C",
            description="Relative resistance increase per degree C in the thermal coupling model.",
        )
        inverter_layout.addWidget(self.inverter_temp_coeff_resistance)

        self.inverter_temp_coeff_drop = LabeledSpinBox(
            "Drop Temp Coeff",
            min_val=0.0,
            max_val=0.05,
            initial=0.0,
            step=0.0005,
            decimals=5,
            suffix=" 1/C",
            description="Relative device-drop increase per degree C in the thermal coupling model.",
        )
        inverter_layout.addWidget(self.inverter_temp_coeff_drop)

        self.inverter_phase_voltage_scale_a = LabeledSpinBox(
            "Phase A Voltage Scale",
            min_val=0.5,
            max_val=1.5,
            initial=1.0,
            step=0.001,
            decimals=4,
            suffix="",
            description="Phase A mismatch multiplier applied to commanded phase voltage.",
        )
        inverter_layout.addWidget(self.inverter_phase_voltage_scale_a)

        self.inverter_phase_voltage_scale_b = LabeledSpinBox(
            "Phase B Voltage Scale",
            min_val=0.5,
            max_val=1.5,
            initial=1.0,
            step=0.001,
            decimals=4,
            suffix="",
            description="Phase B mismatch multiplier applied to commanded phase voltage.",
        )
        inverter_layout.addWidget(self.inverter_phase_voltage_scale_b)

        self.inverter_phase_voltage_scale_c = LabeledSpinBox(
            "Phase C Voltage Scale",
            min_val=0.5,
            max_val=1.5,
            initial=1.0,
            step=0.001,
            decimals=4,
            suffix="",
            description="Phase C mismatch multiplier applied to commanded phase voltage.",
        )
        inverter_layout.addWidget(self.inverter_phase_voltage_scale_c)

        self.inverter_phase_drop_scale_a = LabeledSpinBox(
            "Phase A Drop Scale",
            min_val=0.5,
            max_val=1.5,
            initial=1.0,
            step=0.001,
            decimals=4,
            suffix="",
            description="Phase A mismatch multiplier applied to inverter loss/drop terms.",
        )
        inverter_layout.addWidget(self.inverter_phase_drop_scale_a)

        self.inverter_phase_drop_scale_b = LabeledSpinBox(
            "Phase B Drop Scale",
            min_val=0.5,
            max_val=1.5,
            initial=1.0,
            step=0.001,
            decimals=4,
            suffix="",
            description="Phase B mismatch multiplier applied to inverter loss/drop terms.",
        )
        inverter_layout.addWidget(self.inverter_phase_drop_scale_b)

        self.inverter_phase_drop_scale_c = LabeledSpinBox(
            "Phase C Drop Scale",
            min_val=0.5,
            max_val=1.5,
            initial=1.0,
            step=0.001,
            decimals=4,
            suffix="",
            description="Phase C mismatch multiplier applied to inverter loss/drop terms.",
        )
        inverter_layout.addWidget(self.inverter_phase_drop_scale_c)

        self.inverter_group.setLayout(inverter_layout)
        layout.addWidget(self.inverter_group)

        self.current_sense_group = AccessibleGroupBox(
            "Current Measurement and FFT",
            "Topology-aware shunt sensing controls and separate current harmonic FFT window.",
        )
        current_sense_layout = QVBoxLayout()

        self.current_sense_enable = QCheckBox("Enable Current Sensing Model")
        self.current_sense_enable.setChecked(False)
        current_sense_layout.addWidget(self.current_sense_enable)

        self.current_sense_topology = LabeledComboBox(
            "Sensing Topology",
            items=["single", "double", "triple"],
            description="Single, double, or triple-shunt current measurement topology.",
        )
        current_sense_layout.addWidget(self.current_sense_topology)

        self.current_sense_r_shunt = LabeledSpinBox(
            "Shunt Resistance",
            min_val=0.00005,
            max_val=0.05,
            initial=0.001,
            step=0.0001,
            decimals=5,
            suffix=" Ohm",
            description="Per-shunt resistance used in current sensing.",
        )
        current_sense_layout.addWidget(self.current_sense_r_shunt)

        self.current_sense_nominal_gain = LabeledSpinBox(
            "Nominal Gain",
            min_val=1.0,
            max_val=500.0,
            initial=20.0,
            step=0.5,
            decimals=2,
            suffix=" V/V",
            description="Gain used by controller reconstruction path.",
        )
        current_sense_layout.addWidget(self.current_sense_nominal_gain)

        self.current_sense_nominal_offset = LabeledSpinBox(
            "Nominal Offset",
            min_val=0.0,
            max_val=5.0,
            initial=1.65,
            step=0.01,
            decimals=3,
            suffix=" V",
            description="Nominal offset used by controller reconstruction path.",
        )
        current_sense_layout.addWidget(self.current_sense_nominal_offset)

        self.current_sense_cutoff_hz = LabeledSpinBox(
            "Sensing Cutoff",
            min_val=100.0,
            max_val=200000.0,
            initial=20000.0,
            step=100.0,
            decimals=1,
            suffix=" Hz",
            description="First-order analog anti-aliasing cutoff frequency.",
        )
        current_sense_layout.addWidget(self.current_sense_cutoff_hz)

        self.current_sense_vcc = LabeledSpinBox(
            "ADC Vcc",
            min_val=1.0,
            max_val=5.0,
            initial=3.3,
            step=0.01,
            decimals=2,
            suffix=" V",
            description="ADC full-scale clamp voltage for sensing channels.",
        )
        current_sense_layout.addWidget(self.current_sense_vcc)

        self.current_sense_actual_gain_a = LabeledSpinBox(
            "Actual Gain A",
            min_val=1.0,
            max_val=500.0,
            initial=20.0,
            step=0.5,
            decimals=2,
            suffix=" V/V",
            description="Runtime actual gain for shunt channel A.",
        )
        current_sense_layout.addWidget(self.current_sense_actual_gain_a)

        self.current_sense_actual_gain_b = LabeledSpinBox(
            "Actual Gain B",
            min_val=1.0,
            max_val=500.0,
            initial=20.0,
            step=0.5,
            decimals=2,
            suffix=" V/V",
            description="Runtime actual gain for shunt channel B.",
        )
        current_sense_layout.addWidget(self.current_sense_actual_gain_b)

        self.current_sense_actual_gain_c = LabeledSpinBox(
            "Actual Gain C",
            min_val=1.0,
            max_val=500.0,
            initial=20.0,
            step=0.5,
            decimals=2,
            suffix=" V/V",
            description="Runtime actual gain for shunt channel C.",
        )
        current_sense_layout.addWidget(self.current_sense_actual_gain_c)

        self.current_sense_actual_offset_a = LabeledSpinBox(
            "Actual Offset A",
            min_val=0.0,
            max_val=5.0,
            initial=1.65,
            step=0.01,
            decimals=3,
            suffix=" V",
            description="Runtime actual offset for shunt channel A.",
        )
        current_sense_layout.addWidget(self.current_sense_actual_offset_a)

        self.current_sense_actual_offset_b = LabeledSpinBox(
            "Actual Offset B",
            min_val=0.0,
            max_val=5.0,
            initial=1.65,
            step=0.01,
            decimals=3,
            suffix=" V",
            description="Runtime actual offset for shunt channel B.",
        )
        current_sense_layout.addWidget(self.current_sense_actual_offset_b)

        self.current_sense_actual_offset_c = LabeledSpinBox(
            "Actual Offset C",
            min_val=0.0,
            max_val=5.0,
            initial=1.65,
            step=0.01,
            decimals=3,
            suffix=" V",
            description="Runtime actual offset for shunt channel C.",
        )
        current_sense_layout.addWidget(self.current_sense_actual_offset_c)

        self.current_sense_fft_window_samples = LabeledSpinBox(
            "FFT Window Samples",
            min_val=64,
            max_val=16384,
            initial=512,
            step=64,
            decimals=0,
            suffix=" samples",
            description="Sample window length used by the asynchronous FFT window.",
        )
        current_sense_layout.addWidget(self.current_sense_fft_window_samples)

        self.current_sense_status_label = QLabel(
            "Current sensing disabled. Enable to expose measured-vs-true current telemetry."
        )
        self.current_sense_status_label.setWordWrap(True)
        current_sense_layout.addWidget(self.current_sense_status_label)

        btn_fft_window = AccessibleButton(
            "Open Current FFT Window",
            "Open separate real-time FFT analysis window for controller-facing currents.",
        )
        btn_fft_window.clicked.connect(self._open_current_fft_window)
        current_sense_layout.addWidget(btn_fft_window)

        self.current_sense_group.setLayout(current_sense_layout)
        layout.addWidget(self.current_sense_group)

        self.mcu_budget_group = AccessibleGroupBox(
            "MCU Budget Estimator",
            "Estimate target MCU control-loop load from measured calculation duration.",
        )
        mcu_layout = QVBoxLayout()

        self.mcu_perf_ratio = LabeledSpinBox(
            "Host-to-MCU Slowdown",
            min_val=0.1,
            max_val=1000.0,
            initial=40.0,
            step=0.5,
            decimals=2,
            suffix=" x",
            description="Estimated slowdown from host runtime to target MCU runtime.",
        )
        mcu_layout.addWidget(self.mcu_perf_ratio)

        self.mcu_reference_clock_mhz = LabeledSpinBox(
            "Reference Clock",
            min_val=1.0,
            max_val=1000.0,
            initial=120.0,
            step=1.0,
            decimals=1,
            suffix=" MHz",
            description="Reference clock used for slowdown normalization.",
        )
        mcu_layout.addWidget(self.mcu_reference_clock_mhz)

        self.mcu_target_clock_1_mhz = LabeledSpinBox(
            "Target Clock 1",
            min_val=1.0,
            max_val=1000.0,
            initial=48.0,
            step=1.0,
            decimals=1,
            suffix=" MHz",
            description="First target MCU clock for load estimation.",
        )
        mcu_layout.addWidget(self.mcu_target_clock_1_mhz)

        self.mcu_target_clock_2_mhz = LabeledSpinBox(
            "Target Clock 2",
            min_val=1.0,
            max_val=1000.0,
            initial=72.0,
            step=1.0,
            decimals=1,
            suffix=" MHz",
            description="Second target MCU clock for load estimation.",
        )
        mcu_layout.addWidget(self.mcu_target_clock_2_mhz)

        self.mcu_target_clock_3_mhz = LabeledSpinBox(
            "Target Clock 3",
            min_val=1.0,
            max_val=1000.0,
            initial=120.0,
            step=1.0,
            decimals=1,
            suffix=" MHz",
            description="Third target MCU clock for load estimation.",
        )
        mcu_layout.addWidget(self.mcu_target_clock_3_mhz)

        self.mcu_budget_group.setLayout(mcu_layout)
        layout.addWidget(self.mcu_budget_group)

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

        self.hardware_group = AccessibleGroupBox(
            "Communication Interface",
            "Optional communication backend settings (for example CAN/LIN style I/O). Disable to run pure software simulation.",
        )
        hardware_layout = QVBoxLayout()

        self.hardware_enable_backend = QCheckBox("Enable Communication Backend")
        self.hardware_enable_backend.setChecked(False)
        hardware_layout.addWidget(self.hardware_enable_backend)

        self.hardware_backend_type = LabeledComboBox(
            "Communication Backend",
            items=["Mock DAQ"],
            description="Communication backend implementation used for command/feedback I/O.",
        )
        hardware_layout.addWidget(self.hardware_backend_type)

        self.hardware_noise_std = LabeledSpinBox(
            "Mock Noise Std",
            min_val=0.0,
            max_val=10.0,
            initial=0.0,
            step=0.001,
            decimals=4,
            suffix=" V",
            description="Standard deviation of Gaussian feedback voltage noise for the Mock DAQ backend.",
        )
        hardware_layout.addWidget(self.hardware_noise_std)

        self.hardware_seed = LabeledSpinBox(
            "Mock Random Seed",
            min_val=0,
            max_val=1000000,
            initial=0,
            step=1,
            decimals=0,
            suffix="",
            description="Deterministic random seed used by the Mock DAQ noise generator.",
        )
        hardware_layout.addWidget(self.hardware_seed)

        self.hardware_group.setLayout(hardware_layout)
        layout.addWidget(self.hardware_group)

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
            ("startup_sequence_enabled", "Startup Sequence Enabled", "0/1"),
            ("startup_phase_code", "Startup Phase Code", "0/1/2/3"),
            ("startup_sequence_elapsed_s", "Startup Sequence Elapsed", "s"),
            ("startup_phase_elapsed_s", "Startup Phase Elapsed", "s"),
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
            ("effective_dc_voltage", "Effective DC-Link Voltage", "V"),
            ("dc_link_ripple_v", "DC-Link Ripple", "V"),
            ("dc_link_bus_current_a", "DC-Link Bus Current", "A"),
            ("inverter_total_loss_power_w", "Inverter Total Loss", "W"),
            ("junction_temperature_c", "Inverter Junction Temp", "C"),
            ("common_mode_voltage", "Common-Mode Voltage", "V"),
            ("control_calc_duration_us", "Control Calc Duration", "us"),
            ("control_cpu_load_pct", "Control CPU Load", "%"),
            ("control_cpu_load_avg_pct", "Control CPU Load Avg", "%"),
            ("mcu_load_target_1_pct", "MCU Load @ Target 1", "%"),
            ("mcu_load_target_2_pct", "MCU Load @ Target 2", "%"),
            ("mcu_load_target_3_pct", "MCU Load @ Target 3", "%"),
            ("hardware_enabled", "Communication Enabled", "0/1"),
            ("hardware_connected", "Communication Connected", "0/1"),
            ("hardware_backend_code", "Communication Backend Code", "0/1/2"),
            ("hardware_write_count", "Communication Write Count", "count"),
            ("hardware_read_count", "Communication Read Count", "count"),
            ("hardware_io_error_flag", "Communication I/O Error Flag", "0/1"),
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

        btn_plot_inverter = AccessibleButton(
            "Plot Inverter Analysis",
            "Generate bus voltage, inverter loss, temperature, and common-mode trends",
        )
        btn_plot_inverter.clicked.connect(self._plot_inverter_analysis)
        button_layout.addWidget(btn_plot_inverter)

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

    # ------------------------------------------------------------------
    # Calibration tab
    # ------------------------------------------------------------------

    def _create_calibration_tab(self):
        """Create the FW loaded-point calibration tab."""
        widget = QScrollArea()
        inner = QWidget()
        layout = QVBoxLayout()

        # ── Motor Profile & Session selection ────────────────────────
        profile_group = AccessibleGroupBox(
            "Motor Profile & Session",
            "Select the motor profile and auto-tuning session for calibration",
        )
        pg_layout = QVBoxLayout()

        profiles = sorted(
            p for p in MOTOR_PROFILES_DIR.glob("*.json") if not p.name.startswith("_")
        )
        profile_names = [p.name for p in profiles] or ["(no profiles found)"]

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Motor profile:"))
        self.calib_profile_combo = QComboBox()
        self.calib_profile_combo.addItems(profile_names)
        self.calib_profile_combo.setAccessibleName("Motor profile for calibration")
        self.calib_profile_combo.setAccessibleDescription(
            "Select motor profile JSON for field-weakening loaded-point calibration"
        )
        self.calib_profile_combo.currentTextChanged.connect(
            self._on_calib_profile_changed
        )
        row1.addWidget(self.calib_profile_combo, 1)
        pg_layout.addLayout(row1)

        self.calib_session_label = QLabel("Session: (auto-detected)")
        self.calib_session_label.setAccessibleName("Tuning session file")
        pg_layout.addWidget(self.calib_session_label)

        self.calib_output_label = QLabel("Output: (auto)")
        self.calib_output_label.setAccessibleName("Calibration output file path")
        pg_layout.addWidget(self.calib_output_label)

        profile_group.setLayout(pg_layout)
        layout.addWidget(profile_group)

        # ── Control buttons ──────────────────────────────────────────
        btn_row = QHBoxLayout()

        self.btn_start_calib = AccessibleButton(
            "Start Calibration",
            "Begin three-stage field-weakening loaded-point calibration",
        )
        self.btn_start_calib.clicked.connect(self._start_calibration)
        btn_row.addWidget(self.btn_start_calib)

        self.btn_stop_calib = AccessibleButton(
            "Stop Calibration",
            "Terminate the running calibration process",
        )
        self.btn_stop_calib.setEnabled(False)
        self.btn_stop_calib.clicked.connect(self._stop_calibration)
        btn_row.addWidget(self.btn_stop_calib)

        btn_row.addStretch()
        layout.addLayout(btn_row)

        # ── Progress log ─────────────────────────────────────────────
        prog_group = AccessibleGroupBox(
            "Calibration Progress",
            "Live output from the calibration process",
        )
        prog_layout = QVBoxLayout()
        self.calib_log = QTextEdit()
        self.calib_log.setReadOnly(True)
        self.calib_log.setMinimumHeight(220)
        mono_font = QFont("Courier New", 9)
        self.calib_log.setFont(mono_font)
        self.calib_log.setAccessibleName("Calibration progress log")
        self.calib_log.setAccessibleDescription(
            "Live text output from calibration; key milestones are announced via audio"
        )
        prog_layout.addWidget(self.calib_log)
        prog_group.setLayout(prog_layout)
        layout.addWidget(prog_group)

        # ── Results panel ────────────────────────────────────────────
        result_group = AccessibleGroupBox(
            "Calibration Results",
            "Key performance metrics from the last completed calibration run",
        )
        res_layout = QVBoxLayout()
        self.calib_result_status = QLabel("Status: Not run")
        self.calib_result_speed = QLabel("Achieved Speed: --")
        self.calib_result_load = QLabel("Load Torque: --")
        self.calib_result_efficiency = QLabel("Efficiency: --")
        self.calib_result_fw = QLabel("FW Injection: --")
        for lbl in (
            self.calib_result_status,
            self.calib_result_speed,
            self.calib_result_load,
            self.calib_result_efficiency,
            self.calib_result_fw,
        ):
            lbl.setAccessibleName(lbl.text())
            res_layout.addWidget(lbl)
        res_layout.addStretch()
        result_group.setLayout(res_layout)
        layout.addWidget(result_group)

        inner.setLayout(layout)
        widget.setWidget(inner)
        widget.setWidgetResizable(True)
        self.tabs.addTab(widget, "Calibration")

        # Prime session/output labels for the initial selection
        if profile_names[0] != "(no profiles found)":
            self._on_calib_profile_changed(profile_names[0])

    def _on_calib_profile_changed(self, profile_name: str) -> None:
        """Update session and output path labels when profile selection changes."""
        stem = Path(profile_name).stem
        session_dir = MOTOR_PROFILES_DIR.parent / "tuning_sessions" / "until_converged"
        session_path = session_dir / f"{stem}_until_converged.json"
        out_path = (
            MOTOR_PROFILES_DIR.parent
            / "logs"
            / f"calibration_{stem}_fw_loaded_point.json"
        )
        exists_tag = "" if session_path.exists() else " ⚠ not found"
        self.calib_session_label.setText(f"Session: {session_path.name}{exists_tag}")
        self.calib_output_label.setText(f"Output: {out_path.name}")

    def _start_calibration(self) -> None:
        """Launch the FW loaded-point calibration as a managed child process."""
        # Check if another task is running
        if not self._can_start_task("calibration"):
            return

        if (
            self.calib_process is not None
            and self.calib_process.state() != QProcess.ProcessState.NotRunning
        ):
            speak("Calibration is already running.")
            return

        profile_name = self.calib_profile_combo.currentText()
        if not profile_name or profile_name == "(no profiles found)":
            QMessageBox.warning(
                self, "No Profile", "Please select a valid motor profile first."
            )
            return

        profile_path = MOTOR_PROFILES_DIR / profile_name
        stem = profile_path.stem
        session_dir = MOTOR_PROFILES_DIR.parent / "tuning_sessions" / "until_converged"
        session_path = session_dir / f"{stem}_until_converged.json"
        out_path = (
            MOTOR_PROFILES_DIR.parent
            / "logs"
            / f"calibration_{stem}_fw_loaded_point.json"
        )

        if not profile_path.exists():
            QMessageBox.critical(
                self, "Missing Profile", f"Profile not found:\n{profile_path}"
            )
            return

        if not session_path.exists():
            QMessageBox.critical(
                self,
                "Missing Session",
                f"No tuning session found for {profile_name}.\n"
                f"Expected:\n{session_path}\n\n"
                "Run the auto-tuning convergence script first.",
            )
            return

        self.calib_output_path = out_path

        # Locate the calibration script relative to this file
        script_path = (
            Path(__file__).resolve().parents[2]
            / "examples"
            / "calibrate_fw_loaded_point.py"
        )

        self.calib_log.clear()
        self.calib_log.append(f"[Starting calibration for {profile_name}]\n")
        self.calib_result_status.setText("Status: Running\u2026")
        self.btn_start_calib.setEnabled(False)
        self.btn_stop_calib.setEnabled(True)

        self.calib_process = QProcess(self)
        self.calib_process.setProcessChannelMode(
            QProcess.ProcessChannelMode.MergedChannels
        )
        self.calib_process.readyReadStandardOutput.connect(self._on_calib_output)
        self.calib_process.finished.connect(self._on_calib_finished)
        self.calib_process.start(
            sys.executable,
            [
                str(script_path),
                "--profile",
                str(profile_path),
                "--session",
                str(session_path),
                "--output",
                str(out_path),
            ],
        )
        self._mark_task_running("calibration")

        # Update status bar
        self.status_bar_state.setText("State: Running")
        self.status_bar_task.setText("Task: Calibration")
        self.status_bar_time_remaining.setText("Remaining: -- s")

        speak(f"Calibration started for {stem.replace('_', ' ')}.")

    def _stop_calibration(self) -> None:
        """Terminate the running calibration process gracefully with timeout."""
        if (
            self.calib_process is not None
            and self.calib_process.state() != QProcess.ProcessState.NotRunning
        ):
            # Attempt graceful termination first
            self.calib_process.terminate()
            # Give it up to 2 seconds to finish gracefully
            if not self.calib_process.waitForFinished(2000):
                # If it didn't finish, force kill
                self.calib_process.kill()
                self.calib_process.waitForFinished(1000)
            self.calib_log.append("\n[Calibration stopped by user]")
            self.calib_result_status.setText("Status: Stopped")

        # Update status bar
        self.status_bar_state.setText("State: Stopped")
        self.status_bar_task.setText("Task: None")
        self.status_bar_time_remaining.setText("Remaining: -- s")

        self.btn_start_calib.setEnabled(True)
        self.btn_stop_calib.setEnabled(False)
        self._mark_task_finished("calibration")
        speak("Calibration stopped.")

    def _on_calib_output(self) -> None:
        """Append captured stdout from calibration process to the progress log."""
        raw = bytes(self.calib_process.readAllStandardOutput())
        text = raw.decode("utf-8", errors="replace")
        for line in text.splitlines():
            self.calib_log.append(line)
        # Auto-scroll to bottom
        sb = self.calib_log.verticalScrollBar()
        sb.setValue(sb.maximum())
        # Speak key milestones for blind users
        if "STEP1_START" in text:
            speak("Step 1: Rated speed field weakening convergence started.")
        elif "STEP2_START" in text:
            speak("Step 2: Torque ramp search started.")
        elif "STEP3_START" in text:
            speak("Step 3: Final working point tuning started.")
        if "TORQUE_OK" in text:
            for line in text.splitlines():
                if line.startswith("TORQUE_OK"):
                    parts = line.split()
                    nm = parts[1] if len(parts) > 1 else "?"
                    speak(f"Torque {nm} Newton metres achievable.")
        if "TORQUE_FAIL" in text:
            for line in text.splitlines():
                if line.startswith("TORQUE_FAIL"):
                    parts = line.split()
                    nm = parts[1] if len(parts) > 1 else "?"
                    speak(f"Torque {nm} Newton metres not achievable.")
        if "REPORT_SAVED" in text:
            speak("Calibration report saved to disk.")

    def _on_calib_finished(self, exit_code: int, exit_status) -> None:
        """Handle calibration process completion and display results."""
        self.btn_start_calib.setEnabled(True)
        self.btn_stop_calib.setEnabled(False)

        # Update status bar
        self.status_bar_state.setText("State: Stopped")
        self.status_bar_task.setText("Task: None")
        self.status_bar_time_remaining.setText("Remaining: -- s")

        if (
            exit_code == 0
            and self.calib_output_path is not None
            and self.calib_output_path.exists()
        ):
            try:
                report = json.loads(self.calib_output_path.read_text(encoding="utf-8"))
                step3 = report.get("step3_final_working_point_tuning", {})
                hifi = step3.get("result_high_fidelity", {})
                metrics = hifi.get("metrics", {})
                speed = float(metrics.get("mean_speed_rpm_last_1s", 0.0))
                eff = float(metrics.get("efficiency_pct_last_1s", 0.0))
                fw_inj = float(metrics.get("fw_injection_dc_a_last_1s", 0.0))
                target_torque = float(step3.get("target_load_nm", 0.0))
                success = bool(step3.get("success", False))

                status_str = "PASS" if success else "FAIL"
                self.calib_result_status.setText(f"Status: {status_str}")
                self.calib_result_speed.setText(f"Achieved Speed: {speed:.1f} RPM")
                self.calib_result_load.setText(f"Load Torque: {target_torque:.2f} Nm")
                self.calib_result_efficiency.setText(f"Efficiency: {eff:.1f}%")
                self.calib_result_fw.setText(f"FW Injection: {fw_inj:.1f} A")

                msg = (
                    f"Calibration complete. "
                    f"Speed {speed:.0f} RPM. "
                    f"Efficiency {eff:.0f} percent. "
                    f"Field weakening injection {abs(fw_inj):.1f} Amperes. "
                    f"Result: {status_str}."
                )
                speak(msg)
                self.statusBar().showMessage(
                    f"Calibration {status_str}: {speed:.0f} RPM, {eff:.0f}% eff, FW={fw_inj:.1f} A",
                    10000,
                )
            except Exception as exc:
                self.calib_result_status.setText(
                    f"Status: Error reading results \u2014 {exc}"
                )
                speak("Calibration finished but results could not be read.")
        else:
            self.calib_result_status.setText(f"Status: Failed (exit {exit_code})")
            speak("Calibration process failed.")

        self.calib_process = None
        self._mark_task_finished("calibration")

    def closeEvent(self, event) -> None:
        """
        Override closeEvent to ensure all background processes are properly terminated.
        This prevents orphaned processes when the GUI is closed.
        """
        try:
            # Stop running simulation thread
            if hasattr(self, "is_running") and self.is_running:
                if hasattr(self, "sim_thread") and self.sim_thread is not None:
                    self.sim_thread.stop_simulation()
                    self.sim_thread.wait(timeout=5000)  # Wait up to 5 seconds
                self.is_running = False

            # Terminate calibration process
            if hasattr(self, "calib_process") and self.calib_process is not None:
                if self.calib_process.state() != QProcess.ProcessState.NotRunning:
                    self.calib_process.terminate()
                    # Give it a moment to terminate gracefully
                    if not self.calib_process.waitForFinished(2000):
                        # If it didn't finish, force kill
                        self.calib_process.kill()
                        self.calib_process.waitForFinished(1000)
                self.calib_process = None

            # Cancel any pending timers
            if hasattr(self, "update_timer"):
                self.update_timer.stop()
            if hasattr(self, "plot_timer"):
                self.plot_timer.stop()
            if (
                hasattr(self, "current_fft_window")
                and self.current_fft_window is not None
            ):
                self.current_fft_window.close()
                self.current_fft_window = None

        except Exception as exc:
            # Log the error but don't block the window from closing
            print(f"Error during cleanup in closeEvent: {exc}")

        # Call parent closeEvent to complete the window closure
        event.accept()

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
        if hasattr(self, "hardware_enable_backend"):
            self.hardware_enable_backend.setChecked(False)
        self._apply_to_simulation()

    def _apply_to_simulation(self):
        """Apply current UI parameters to simulation."""
        requested_pwm_hz = float(self.inverter_switching_frequency.value())
        if requested_pwm_hz <= 0.0:
            requested_pwm_hz = float(SIMULATION_PARAMS.get("pwm_frequency_hz", 20000.0))
        pwm_period_s = 1.0 / requested_pwm_hz

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

        self.motor = BLDCMotor(params, dt=pwm_period_s)
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

        hardware_interface = None
        if (
            hasattr(self, "hardware_enable_backend")
            and self.hardware_enable_backend.isChecked()
            and getattr(self, "hardware_backend_type", None)
            and self.hardware_backend_type.currentText() == "Mock DAQ"
        ):
            hardware_interface = MockDAQHardware(
                noise_std=self.hardware_noise_std.value(),
                seed=int(self.hardware_seed.value()),
            )

        current_sense = self._build_current_sense_model()

        # Create simulation engine
        self.engine = SimulationEngine(
            self.motor,
            load,
            dt=pwm_period_s,
            supply_profile=supply,
            hardware_interface=hardware_interface,
            current_sense=current_sense,
        )
        self.engine.set_pwm_frequency(requested_pwm_hz)
        self.engine.configure_hardware_interface(
            enabled=bool(
                hasattr(self, "hardware_enable_backend")
                and self.hardware_enable_backend.isChecked()
            )
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
        self.svm.set_sample_time(self.engine.dt)
        self.svm.set_nonidealities(
            device_drop_v=self.inverter_device_drop.value(),
            dead_time_fraction=self.inverter_dead_time_fraction.value(),
            conduction_resistance_ohm=self.inverter_conduction_resistance.value(),
            switching_frequency_hz=requested_pwm_hz,
            switching_loss_coeff_v_per_a_khz=self.inverter_switching_loss_coeff.value(),
            enable_device_drop=self.inverter_enable_device_drop.isChecked(),
            enable_dead_time=self.inverter_enable_dead_time.isChecked(),
            enable_conduction_drop=self.inverter_enable_conduction.isChecked(),
            enable_switching_loss=self.inverter_enable_switching.isChecked(),
            enable_diode_freewheel=self.inverter_enable_diode.isChecked(),
            diode_drop_v=self.inverter_diode_drop.value(),
            diode_resistance_ohm=self.inverter_diode_resistance.value(),
            enable_min_pulse=self.inverter_enable_min_pulse.isChecked(),
            min_pulse_fraction=self.inverter_min_pulse_fraction.value(),
            enable_bus_ripple=self.inverter_enable_bus_ripple.isChecked(),
            dc_link_capacitance_f=self.inverter_dc_link_capacitance.value(),
            dc_link_source_resistance_ohm=self.inverter_dc_link_source_resistance.value(),
            dc_link_esr_ohm=self.inverter_dc_link_esr.value(),
            enable_thermal_coupling=self.inverter_enable_thermal.isChecked(),
            thermal_resistance_k_per_w=self.inverter_thermal_resistance.value(),
            thermal_capacitance_j_per_k=self.inverter_thermal_capacitance.value(),
            ambient_temperature_c=self.inverter_ambient_temp.value(),
            temp_coeff_resistance_per_c=self.inverter_temp_coeff_resistance.value(),
            temp_coeff_drop_per_c=self.inverter_temp_coeff_drop.value(),
            enable_phase_asymmetry=self.inverter_enable_phase_asymmetry.isChecked(),
            phase_voltage_scale_a=self.inverter_phase_voltage_scale_a.value(),
            phase_voltage_scale_b=self.inverter_phase_voltage_scale_b.value(),
            phase_voltage_scale_c=self.inverter_phase_voltage_scale_c.value(),
            phase_drop_scale_a=self.inverter_phase_drop_scale_a.value(),
            phase_drop_scale_b=self.inverter_phase_drop_scale_b.value(),
            phase_drop_scale_c=self.inverter_phase_drop_scale_c.value(),
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
            self.controller.set_startup_sequence(
                enable=self.vf_startup_sequence_mode.currentText() == "Enabled",
                align_duration_s=self.vf_align_time.value(),
                align_voltage_v=self.vf_align_voltage.value(),
                align_angle_deg=self.vf_align_angle.value(),
                ramp_initial_frequency_hz=self.vf_ramp_initial_frequency.value(),
            )
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
            self.controller.set_field_weakening(
                enabled=self.foc_field_weakening_mode.currentText() == "Enabled",
                start_speed_rpm=self.foc_field_weakening_start_speed.value(),
                gain=self.foc_field_weakening_gain.value(),
                max_negative_id_a=self.foc_field_weakening_max_id.value(),
                headroom_target_v=self.foc_field_weakening_headroom_target.value(),
            )
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
            self.controller.set_startup_sequence(
                enabled=self.foc_startup_sequence_mode.currentText() == "Enabled",
                align_duration_s=self.foc_align_time.value(),
                align_current_a=self.foc_align_current.value(),
                align_angle_deg=self.foc_align_angle.value(),
                open_loop_initial_speed_rpm=self.foc_open_loop_initial_speed.value(),
                open_loop_target_speed_rpm=self.foc_open_loop_target_speed.value(),
                open_loop_ramp_time_s=self.foc_open_loop_ramp_time.value(),
                open_loop_id_ref_a=self.foc_open_loop_id_ref.value(),
                open_loop_iq_ref_a=self.foc_open_loop_iq_ref.value(),
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

    def _is_any_task_running(self) -> bool:
        """Check if any long-running task (simulation or calibration) is currently active."""
        with self._task_lock:
            return self._running_task_name is not None

    def _get_running_task_name(self) -> Optional[str]:
        """Return the name of the currently running task, or None."""
        with self._task_lock:
            return self._running_task_name

    def _mark_task_running(self, task_name: str) -> None:
        """Mark a task as running (e.g., 'simulation' or 'calibration')."""
        with self._task_lock:
            self._running_task_name = task_name

    def _mark_task_finished(self, task_name: str) -> None:
        """Mark a task as finished. Only clears if the given task is currently running."""
        with self._task_lock:
            if self._running_task_name == task_name:
                self._running_task_name = None

    def _terminate_process_gracefully(
        self,
        process,
        process_type: str = "subprocess",
        timeout_graceful_ms: int = 2000,
        timeout_kill_ms: int = 1000,
    ) -> bool:
        """
        Terminate a process (QThread or QProcess) gracefully with timeout protection.

        Args:
            process: QThread or QProcess to terminate
            process_type: String describing the process for logging ("thread" or "process")
            timeout_graceful_ms: Time (ms) to wait for graceful termination
            timeout_kill_ms: Time (ms) to wait for forceful kill

        Returns:
            True if terminated successfully, False if forceful kill was required
        """
        if process is None:
            return True

        # Check if it's a QThread or QProcess
        if hasattr(process, "stop_simulation"):  # SimulationThread
            process.stop_simulation()
            if process.wait(timeout=timeout_graceful_ms):
                return True
            else:
                print(
                    f"Warning: {process_type} did not finish gracefully within {timeout_graceful_ms}ms"
                )
                return False

        elif hasattr(process, "state"):  # QProcess
            if process.state() == QProcess.ProcessState.NotRunning:
                return True

            # Try graceful termination first
            process.terminate()
            if process.waitForFinished(timeout_graceful_ms):
                return True

            # If that fails, force kill
            print(f"Warning: {process_type} did not terminate gracefully, forcing kill")
            process.kill()
            return not process.waitForFinished(timeout_kill_ms)

        return True

    def _can_start_task(self, new_task_name: str) -> bool:
        """
        Check if a new task can start. If another task is running, show confirmation dialog.
        Returns True if OK to proceed, False if user cancels or task conflicts.
        """
        running = self._get_running_task_name()
        if running is None:
            return True

        # Prevent same task from running twice
        if running == new_task_name:
            speak(f"{new_task_name.capitalize()} is already running.")
            return False

        # Different task is running; ask user to confirm
        reply = QMessageBox.warning(
            self,
            "Another Process Running",
            f"{running.capitalize()} is currently running.\n\n"
            f"Stop it and start {new_task_name}?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )

        if reply == QMessageBox.StandardButton.Yes:
            # Stop the running task
            if running == "simulation" and self.is_running:
                self._stop_simulation()
            elif running == "calibration" and self.calib_process is not None:
                self._stop_calibration()
            return True

        # User canceled
        speak(f"Cancelled; {running} is still running.")
        return False

    def _start_simulation(self):
        """Start simulation."""
        if self.is_running:
            QMessageBox.warning(self, "Warning", "Simulation already running!")
            return

        self._apply_to_simulation()

        self.is_running = True
        self._mark_task_running("simulation")
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

        # Update status bar
        duration = self.sim_duration.value()
        msg = f"Simulation started. Duration: {'∞ (infinite)' if duration == 0 else f'{duration}s'}"
        self.status_bar_state.setText("State: Running")
        self.status_bar_task.setText("Task: Simulation")
        self.status_bar_time_remaining.setText(
            f"Remaining: {duration:.1f}s" if duration > 0 else "Elapsed: 0.0s"
        )

        # Reset speed curve data
        self.speed_history_time = []
        self.speed_history_rpm = []

        # Create and start simulation thread
        self.sim_thread = SimulationThread()
        self.sim_thread.set_simulation(
            self.engine,
            self.svm,
            self.controller,
            max_duration=duration,
            pwm_frequency_hz=float(1.0 / self.engine.dt),
        )
        self.sim_thread.finished_signal.connect(self._on_simulation_finished)
        self.sim_thread.start_simulation()
        self.update_timer.start(int(self.sim_thread.update_interval * 1000.0))

        speak(msg)

    def _stop_simulation(self):
        """Stop simulation."""
        if not self.is_running:
            return
        speak("Simulation stopped.")

        self.is_running = False
        self._mark_task_finished("simulation")

        # Update status bar
        self.status_bar_state.setText("State: Stopped")
        self.status_bar_task.setText("Task: None")
        self.status_bar_time_remaining.setText("Remaining: -- s")
        self.status_bar_cpu_load.setText("CPU: -- %")

        if self.sim_thread:
            self.sim_thread.stop_simulation()
            # Wait with timeout to prevent indefinite hang
            if not self.sim_thread.wait(timeout=5000):
                # Timeout occurred - log and continue cleanup
                print("Warning: Simulation thread did not finish within 5s timeout")
        self.update_timer.stop()

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
        inverter_state = (
            state.get("inverter", {})
            if isinstance(state.get("inverter", {}), dict)
            else {}
        )
        control_timing_state = (
            state.get("control_timing", {})
            if isinstance(state.get("control_timing", {}), dict)
            else {}
        )
        hardware_state = (
            state.get("hardware", {})
            if isinstance(state.get("hardware", {}), dict)
            else {}
        )
        if not pfc_state and self.engine is not None:
            pfc_state = self.engine.get_power_factor_control_state()
        if not efficiency_state and self.engine is not None:
            efficiency_state = self.engine.get_efficiency_state()
        if not inverter_state and self.engine is not None:
            inverter_state = self.engine.get_inverter_state()
        if not control_timing_state and self.engine is not None:
            control_timing_state = self.engine.get_control_timing_state()

        calc_duration_s = float(control_timing_state.get("calc_duration_s", 0.0))
        control_period_s = max(
            float(
                control_timing_state.get(
                    "control_period_s", self.engine.dt if self.engine else 1e-4
                )
            ),
            1e-12,
        )
        slowdown = max(float(self.mcu_perf_ratio.value()), 1e-9)
        ref_clock = max(float(self.mcu_reference_clock_mhz.value()), 1e-9)
        target_1 = max(float(self.mcu_target_clock_1_mhz.value()), 1e-9)
        target_2 = max(float(self.mcu_target_clock_2_mhz.value()), 1e-9)
        target_3 = max(float(self.mcu_target_clock_3_mhz.value()), 1e-9)

        def _mcu_load_pct(target_clock_mhz: float) -> float:
            scaled_duration = (
                calc_duration_s * slowdown * (ref_clock / target_clock_mhz)
            )
            return 100.0 * scaled_duration / control_period_s

        mcu_load_1 = _mcu_load_pct(target_1)
        mcu_load_2 = _mcu_load_pct(target_2)
        mcu_load_3 = _mcu_load_pct(target_3)
        if not hardware_state and self.engine is not None:
            hardware_state = self.engine.get_hardware_state()
        observer_mode_code = 0.0
        startup_phase_code = 0.0
        if isinstance(self.controller, FOCController):
            ctrl_state = self.controller.get_state()
            observer_mode = str(ctrl_state.get("angle_observer_mode", "Measured"))
            startup_phase_code = float(ctrl_state.get("startup_phase_code", 0.0))
            mode_to_code = {
                "Measured": 0.0,
                "PLL": 1.0,
                "SMO": 2.0,
                "Alignment": 3.0,
                "OpenLoop": 4.0,
            }
            observer_mode_code = mode_to_code.get(observer_mode, -1.0)
        elif self.controller is not None:
            ctrl_state = self.controller.get_state()
            startup_phase_code = float(ctrl_state.get("startup_phase_code", 0.0))

        backend_name = str(hardware_state.get("backend", "none")).lower()
        backend_code_map = {"none": 0.0, "mock-daq": 1.0}
        hardware_backend_code = backend_code_map.get(backend_name, 2.0)
        hardware_io_error_flag = (
            1.0 if str(hardware_state.get("last_io_error", "")) else 0.0
        )

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
            self.status_blocks["id_ref"].update_value(
                ctrl_state.get("id_ref_command", ctrl_state.get("id_ref", 0.0))
            )
            self.status_blocks["iq_ref"].update_value(
                ctrl_state.get("iq_ref_command", ctrl_state.get("iq_ref", 0.0))
            )
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
            self.status_blocks["startup_sequence_enabled"].update_value(
                1.0 if ctrl_state.get("startup_sequence_enabled", False) else 0.0
            )
            self.status_blocks["startup_phase_code"].update_value(startup_phase_code)
            self.status_blocks["startup_sequence_elapsed_s"].update_value(
                ctrl_state.get("startup_sequence_elapsed_s", 0.0)
            )
            self.status_blocks["startup_phase_elapsed_s"].update_value(
                ctrl_state.get("startup_phase_elapsed_s", 0.0)
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
            self.status_blocks["effective_dc_voltage"].update_value(
                float(inverter_state.get("effective_dc_voltage", 0.0))
            )
            self.status_blocks["dc_link_ripple_v"].update_value(
                float(inverter_state.get("dc_link_ripple_v", 0.0))
            )
            self.status_blocks["dc_link_bus_current_a"].update_value(
                float(inverter_state.get("dc_link_bus_current_a", 0.0))
            )
            self.status_blocks["inverter_total_loss_power_w"].update_value(
                float(inverter_state.get("total_inverter_loss_power_w", 0.0))
            )
            self.status_blocks["junction_temperature_c"].update_value(
                float(inverter_state.get("junction_temperature_c", 0.0))
            )
            self.status_blocks["common_mode_voltage"].update_value(
                float(inverter_state.get("common_mode_voltage", 0.0))
            )
            self.status_blocks["control_calc_duration_us"].update_value(
                1e6 * float(control_timing_state.get("calc_duration_s", 0.0))
            )
            self.status_blocks["control_cpu_load_pct"].update_value(
                float(control_timing_state.get("cpu_load_pct", 0.0))
            )
            self.status_blocks["control_cpu_load_avg_pct"].update_value(
                float(control_timing_state.get("cpu_load_avg_pct", 0.0))
            )
            self.status_blocks["mcu_load_target_1_pct"].update_value(mcu_load_1)
            self.status_blocks["mcu_load_target_2_pct"].update_value(mcu_load_2)
            self.status_blocks["mcu_load_target_3_pct"].update_value(mcu_load_3)
            self.status_blocks["hardware_enabled"].update_value(
                1.0 if hardware_state.get("enabled", False) else 0.0
            )
            self.status_blocks["hardware_connected"].update_value(
                1.0 if hardware_state.get("connected", False) else 0.0
            )
            self.status_blocks["hardware_backend_code"].update_value(
                hardware_backend_code
            )
            self.status_blocks["hardware_write_count"].update_value(
                float(hardware_state.get("write_count", 0.0))
            )
            self.status_blocks["hardware_read_count"].update_value(
                float(hardware_state.get("read_count", 0.0))
            )
            self.status_blocks["hardware_io_error_flag"].update_value(
                hardware_io_error_flag
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
            self.status_labels["effective_dc_voltage"].setText(
                f"Effective DC-Link Voltage: {inverter_state.get('effective_dc_voltage', 0.0):.3f} V"
            )
            self.status_labels["dc_link_ripple_v"].setText(
                f"DC-Link Ripple: {inverter_state.get('dc_link_ripple_v', 0.0):.3f} V"
            )
            self.status_labels["dc_link_bus_current_a"].setText(
                f"DC-Link Bus Current: {inverter_state.get('dc_link_bus_current_a', 0.0):.3f} A"
            )
            self.status_labels["inverter_total_loss_power_w"].setText(
                f"Inverter Total Loss: {inverter_state.get('total_inverter_loss_power_w', 0.0):.3f} W"
            )
            self.status_labels["junction_temperature_c"].setText(
                f"Inverter Junction Temp: {inverter_state.get('junction_temperature_c', 0.0):.3f} C"
            )
            self.status_labels["common_mode_voltage"].setText(
                f"Common-Mode Voltage: {inverter_state.get('common_mode_voltage', 0.0):.3f} V"
            )
            self.status_labels["control_calc_duration_us"].setText(
                f"Control Calc Duration: {1e6 * float(control_timing_state.get('calc_duration_s', 0.0)):.3f} us"
            )
            self.status_labels["control_cpu_load_pct"].setText(
                f"Control CPU Load: {float(control_timing_state.get('cpu_load_pct', 0.0)):.3f} %"
            )
            self.status_labels["control_cpu_load_avg_pct"].setText(
                f"Control CPU Load Avg: {float(control_timing_state.get('cpu_load_avg_pct', 0.0)):.3f} %"
            )
            self.status_labels["mcu_load_target_1_pct"].setText(
                f"MCU Load @ Target 1: {mcu_load_1:.3f} %"
            )
            self.status_labels["mcu_load_target_2_pct"].setText(
                f"MCU Load @ Target 2: {mcu_load_2:.3f} %"
            )
            self.status_labels["mcu_load_target_3_pct"].setText(
                f"MCU Load @ Target 3: {mcu_load_3:.3f} %"
            )
            self.status_labels["hardware_enabled"].setText(
                f"Communication Enabled: {1 if hardware_state.get('enabled', False) else 0}"
            )
            self.status_labels["hardware_connected"].setText(
                f"Communication Connected: {1 if hardware_state.get('connected', False) else 0}"
            )
            self.status_labels["hardware_backend_code"].setText(
                f"Communication Backend Code: {hardware_backend_code:.0f}"
            )
            self.status_labels["hardware_write_count"].setText(
                f"Communication Write Count: {int(hardware_state.get('write_count', 0))}"
            )
            self.status_labels["hardware_read_count"].setText(
                f"Communication Read Count: {int(hardware_state.get('read_count', 0))}"
            )
            self.status_labels["hardware_io_error_flag"].setText(
                f"Communication I/O Error Flag: {int(hardware_io_error_flag)}"
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

    def _poll_simulation_state(self):
        """Poll latest simulation snapshot without coupling GUI to control timing."""
        if self.sim_thread is None:
            return
        self._update_runtime_current_sense_actuals()
        snapshot = self.sim_thread.get_latest_state()
        if snapshot:
            self._update_monitoring(snapshot)
            self._update_status_bar(snapshot)
            self._update_current_sense_status(snapshot)
            if self.current_fft_window is not None:
                self.current_fft_window.set_window_size(
                    int(self.current_sense_fft_window_samples.value())
                )
                self.current_fft_window.push_snapshot(snapshot)

    def _build_current_sense_model(self) -> Optional[InverterCurrentSense]:
        """Create topology-aware current sense model from GUI settings."""
        if not hasattr(self, "current_sense_enable"):
            return None
        if not self.current_sense_enable.isChecked():
            return None

        topology = self.current_sense_topology.currentText()
        expected_channels = {"single": 1, "double": 2, "triple": 3}.get(topology, 3)
        actual_gains = [
            self.current_sense_actual_gain_a.value(),
            self.current_sense_actual_gain_b.value(),
            self.current_sense_actual_gain_c.value(),
        ]
        actual_offsets = [
            self.current_sense_actual_offset_a.value(),
            self.current_sense_actual_offset_b.value(),
            self.current_sense_actual_offset_c.value(),
        ]

        channels = []
        for idx in range(expected_channels):
            channels.append(
                ShuntAmplifierChannel(
                    r_shunt_ohm=self.current_sense_r_shunt.value(),
                    nominal_gain=self.current_sense_nominal_gain.value(),
                    nominal_offset_v=self.current_sense_nominal_offset.value(),
                    actual_gain=actual_gains[idx],
                    actual_offset_v=actual_offsets[idx],
                    cutoff_frequency_hz=self.current_sense_cutoff_hz.value(),
                    vcc=self.current_sense_vcc.value(),
                )
            )

        return InverterCurrentSense(topology=topology, channels=channels)

    def _update_runtime_current_sense_actuals(self) -> None:
        """Push runtime gain/offset edits into the active sense model during simulation."""
        if self.engine is None or getattr(self.engine, "current_sense", None) is None:
            return
        sense = self.engine.current_sense
        actual_gains = [
            self.current_sense_actual_gain_a.value(),
            self.current_sense_actual_gain_b.value(),
            self.current_sense_actual_gain_c.value(),
        ]
        actual_offsets = [
            self.current_sense_actual_offset_a.value(),
            self.current_sense_actual_offset_b.value(),
            self.current_sense_actual_offset_c.value(),
        ]
        for idx in range(sense.n_shunts):
            sense.set_actual_channel(
                idx, gain=actual_gains[idx], offset_v=actual_offsets[idx]
            )

    def _update_current_sense_status(self, snapshot: dict) -> None:
        """Update current sensing status text for non-visual monitoring."""
        meas = (
            snapshot.get("current_measurement", {})
            if isinstance(snapshot, dict)
            else {}
        )
        if not isinstance(meas, dict) or not meas.get("enabled", False):
            self.current_sense_status_label.setText(
                "Current sensing disabled. Enable to expose measured-vs-true current telemetry."
            )
            return

        topology = meas.get("topology", "unknown")
        sat = meas.get("adc_saturated", [])
        saturated_count = (
            int(sum(1 for item in sat if bool(item))) if isinstance(sat, list) else 0
        )
        drop = meas.get("phase_voltage_drop_v", [0.0, 0.0, 0.0])
        drop_arr = (
            np.asarray(drop, dtype=np.float64)
            if isinstance(drop, list)
            else np.zeros(3)
        )
        drop_rms = (
            float(np.sqrt(np.mean(np.square(drop_arr)))) if drop_arr.size else 0.0
        )
        self.current_sense_status_label.setText(
            f"Current sensing active. Topology: {topology}, ADC saturation channels: {saturated_count}, phase-drop RMS: {drop_rms:.4f} V"
        )

    def _open_current_fft_window(self) -> None:
        """Open or focus the asynchronous current FFT analysis window."""
        if self.current_fft_window is None:
            self.current_fft_window = CurrentSpectrumWindow(
                window_size_samples=int(self.current_sense_fft_window_samples.value()),
                parent=self,
            )
            self.current_fft_window.closed.connect(self._on_current_fft_window_closed)
        self.current_fft_window.show()
        self.current_fft_window.raise_()
        self.current_fft_window.activateWindow()

    def _on_current_fft_window_closed(self) -> None:
        """Track FFT window closure to avoid stale references."""
        self.current_fft_window = None

    def _update_status_bar(self, snapshot: dict) -> None:
        """Update status bar with current simulation telemetry."""
        try:
            # Update simulation state
            state_text = "State: Running" if self.is_running else "State: Stopped"
            self.status_bar_state.setText(state_text)

            # Update task name
            task_name = self._get_running_task_name() or "None"
            self.status_bar_task.setText(f"Task: {task_name.capitalize()}")

            # Update remaining time estimate
            if self.is_running and self.sim_thread:
                current_time = snapshot.get("time", 0.0)
                duration = self.sim_duration.value()  # Get max duration from UI
                if duration > 0:
                    remaining = max(0.0, duration - current_time)
                    self.status_bar_time_remaining.setText(
                        f"Remaining: {remaining:.1f}s"
                    )
                else:
                    # Infinite duration
                    self.status_bar_time_remaining.setText(
                        f"Elapsed: {current_time:.1f}s"
                    )
            else:
                self.status_bar_time_remaining.setText("Remaining: -- s")

            # Update CPU load estimate
            cpu_load = snapshot.get("cpu_load_pct", 0.0)
            if cpu_load > 0:
                self.status_bar_cpu_load.setText(f"CPU: {cpu_load:.1f}%")
            else:
                self.status_bar_cpu_load.setText("CPU: -- %")

        except Exception as e:
            # Silently fail to avoid disrupting GUI updates
            pass

    def _on_simulation_finished(self):
        """Handle simulation thread completion."""
        self.is_running = False
        self._mark_task_finished("simulation")

        # Update status bar
        self.status_bar_state.setText("State: Stopped")
        self.status_bar_task.setText("Task: None")
        self.status_bar_time_remaining.setText("Remaining: -- s")
        self.status_bar_cpu_load.setText("CPU: -- %")

        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.update_timer.stop()

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
            metadata = self._collect_simulation_configuration()

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

    def _plot_inverter_analysis(self):
        """Generate dedicated inverter-realism telemetry plot."""
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
        figure = SimulationPlotter.create_inverter_analysis_plot(
            history,
            grid_on=grid_on,
            grid_spacing=grid_spacing,
            minor_grid=minor_grid,
            grid_spacing_y=grid_spacing_y,
        )
        figure.show()
        speak("Inverter analysis plot generated.")

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
