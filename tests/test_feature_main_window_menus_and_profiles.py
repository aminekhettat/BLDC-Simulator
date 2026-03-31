"""
Atomic features tested in this module:
- refresh builtin motor menu empty
- refresh builtin motor menu nonempty triggers loader
- import motor parameters cancel
- import motor parameters success
- import motor parameters error
- save motor parameters cancel
- save motor parameters success
- save motor parameters error
- load motor profile from path success and error
- toggle audio assistance
- open html help missing and existing
- get simulation params info with and without engine
- collect and save simulation parameters
- accessible text block keyboard navigation
- simulation thread run early return and normal iteration
- simulation thread cartesian fallback and bad output
- simulation thread start and stop paths
- simulation thread max duration break and missing svm io methods
"""

import sys
from collections.abc import Callable
from pathlib import Path
from typing import cast

import numpy as np
import pytest
from PyQt6.QtCore import QEvent, Qt
from PyQt6.QtGui import QKeyEvent
from PyQt6.QtWidgets import QApplication

from src.control.base_controller import BaseController
from src.control.svm_generator import SVMGenerator
from src.core.simulation_engine import SimulationEngine
from src.ui.main_window import AccessibleTextBlock, BLDCMotorControlGUI, SimulationThread


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    return app


@pytest.fixture
def gui(qapp):
    window = BLDCMotorControlGUI()
    yield window
    window.close()


def test_refresh_builtin_motor_menu_empty(gui, monkeypatch):
    monkeypatch.setattr("src.ui.main_window.list_motor_profiles", lambda _p: [])

    gui._refresh_builtin_motor_menu()

    actions = gui.builtin_motor_menu.actions()
    assert len(actions) == 1
    assert actions[0].text() == "No built-in profiles found"
    assert actions[0].isEnabled() is False


def test_refresh_builtin_motor_menu_nonempty_triggers_loader(gui, monkeypatch, tmp_path):
    profile_path = tmp_path / "m1.json"
    profile_path.write_text("{}", encoding="utf-8")

    loaded = {"path": None}
    monkeypatch.setattr(
        "src.ui.main_window.list_motor_profiles",
        lambda _p: [profile_path],
    )
    monkeypatch.setattr(
        gui,
        "_load_motor_profile_from_path",
        lambda p: loaded.__setitem__("path", p),
    )

    gui._refresh_builtin_motor_menu()
    actions = gui.builtin_motor_menu.actions()
    assert len(actions) == 1
    actions[0].trigger()
    assert loaded["path"] == profile_path


def test_import_motor_parameters_cancel(gui, monkeypatch):
    monkeypatch.setattr(
        "src.ui.main_window.QFileDialog.getOpenFileName",
        lambda *args, **kwargs: ("", ""),
    )

    # Should exit early with no exception.
    gui._import_motor_parameters()


def test_import_motor_parameters_success(gui, monkeypatch, tmp_path):
    selected = tmp_path / "profile.json"
    selected.write_text("{}", encoding="utf-8")

    applied = {"called": False}
    informed = {"called": False}
    spoken = {"called": False}

    monkeypatch.setattr(
        "src.ui.main_window.QFileDialog.getOpenFileName",
        lambda *args, **kwargs: (str(selected), "JSON Files (*.json)"),
    )
    monkeypatch.setattr(
        "src.ui.main_window.load_motor_profile",
        lambda _p: {
            "profile_name": "Test Profile",
            "motor_params": gui._collect_current_motor_parameters(),
        },
    )
    monkeypatch.setattr(gui, "_apply_to_simulation", lambda: applied.__setitem__("called", True))
    monkeypatch.setattr(
        "src.ui.main_window.QMessageBox.information",
        lambda *args, **kwargs: informed.__setitem__("called", True),
    )
    monkeypatch.setattr(
        "src.ui.main_window.speak",
        lambda *args, **kwargs: spoken.__setitem__("called", True),
    )

    gui._import_motor_parameters()

    assert applied["called"] is True
    assert informed["called"] is True
    assert spoken["called"] is True


def test_import_motor_parameters_error(gui, monkeypatch, tmp_path):
    selected = tmp_path / "bad.json"
    selected.write_text("{}", encoding="utf-8")

    critical = {"called": False}
    monkeypatch.setattr(
        "src.ui.main_window.QFileDialog.getOpenFileName",
        lambda *args, **kwargs: (str(selected), "JSON Files (*.json)"),
    )
    monkeypatch.setattr(
        "src.ui.main_window.load_motor_profile",
        lambda _p: (_ for _ in ()).throw(RuntimeError("boom")),
    )
    monkeypatch.setattr(
        "src.ui.main_window.QMessageBox.critical",
        lambda *args, **kwargs: critical.__setitem__("called", True),
    )

    gui._import_motor_parameters()
    assert critical["called"] is True


def test_save_motor_parameters_cancel(gui, monkeypatch):
    monkeypatch.setattr(
        "src.ui.main_window.QFileDialog.getSaveFileName",
        lambda *args, **kwargs: ("", ""),
    )

    gui._save_motor_parameters()


def test_save_motor_parameters_success(gui, monkeypatch, tmp_path):
    out = tmp_path / "custom_profile"
    saved = {"path": None}
    informed = {"called": False}

    monkeypatch.setattr(
        "src.ui.main_window.QFileDialog.getSaveFileName",
        lambda *args, **kwargs: (str(out), "JSON Files (*.json)"),
    )
    monkeypatch.setattr(
        "src.ui.main_window.save_motor_profile",
        lambda **kwargs: saved.__setitem__("path", kwargs["file_path"]),
    )
    monkeypatch.setattr(gui, "_refresh_builtin_motor_menu", lambda: None)
    monkeypatch.setattr(
        "src.ui.main_window.QMessageBox.information",
        lambda *args, **kwargs: informed.__setitem__("called", True),
    )
    monkeypatch.setattr("src.ui.main_window.speak", lambda *args, **kwargs: None)

    gui._save_motor_parameters()

    assert saved["path"] is not None
    assert saved["path"].suffix == ".json"
    assert informed["called"] is True


def test_save_motor_parameters_error(gui, monkeypatch, tmp_path):
    out = tmp_path / "custom_profile.json"
    critical = {"called": False}

    monkeypatch.setattr(
        "src.ui.main_window.QFileDialog.getSaveFileName",
        lambda *args, **kwargs: (str(out), "JSON Files (*.json)"),
    )
    monkeypatch.setattr(
        "src.ui.main_window.save_motor_profile",
        lambda **kwargs: (_ for _ in ()).throw(RuntimeError("cannot save")),
    )
    monkeypatch.setattr(
        "src.ui.main_window.QMessageBox.critical",
        lambda *args, **kwargs: critical.__setitem__("called", True),
    )

    gui._save_motor_parameters()
    assert critical["called"] is True


def test_load_motor_profile_from_path_success_and_error(gui, monkeypatch, tmp_path):
    p = tmp_path / "builtin.json"
    p.write_text("{}", encoding="utf-8")

    informed = {"called": False}
    critical = {"called": False}

    monkeypatch.setattr(
        "src.ui.main_window.load_motor_profile",
        lambda _p: {
            "profile_name": "BuiltIn",
            "motor_params": gui._collect_current_motor_parameters(),
        },
    )
    monkeypatch.setattr(gui, "_apply_to_simulation", lambda: None)
    monkeypatch.setattr(
        "src.ui.main_window.QMessageBox.information",
        lambda *args, **kwargs: informed.__setitem__("called", True),
    )
    monkeypatch.setattr("src.ui.main_window.speak", lambda *args, **kwargs: None)

    gui._load_motor_profile_from_path(p)
    assert informed["called"] is True

    monkeypatch.setattr(
        "src.ui.main_window.load_motor_profile",
        lambda _p: (_ for _ in ()).throw(RuntimeError("bad profile")),
    )
    monkeypatch.setattr(
        "src.ui.main_window.QMessageBox.critical",
        lambda *args, **kwargs: critical.__setitem__("called", True),
    )

    gui._load_motor_profile_from_path(p)
    assert critical["called"] is True


def test_toggle_audio_assistance(gui, monkeypatch):
    calls = {"enabled": None, "informed": False, "spoken": False}

    monkeypatch.setattr(
        "src.ui.main_window.set_audio_assistance_enabled",
        lambda enabled: calls.__setitem__("enabled", enabled),
    )
    monkeypatch.setattr(
        "src.ui.main_window.QMessageBox.information",
        lambda *args, **kwargs: calls.__setitem__("informed", True),
    )
    monkeypatch.setattr(
        "src.ui.main_window.speak",
        lambda *args, **kwargs: calls.__setitem__("spoken", True),
    )

    gui._toggle_audio_assistance(True)
    assert calls["enabled"] is True
    assert calls["informed"] is True
    assert calls["spoken"] is True


def test_open_html_help_missing_and_existing(gui, monkeypatch):
    warned = {"called": False}
    opened = {"called": False}

    monkeypatch.setattr(
        "src.ui.main_window.QMessageBox.warning",
        lambda *args, **kwargs: warned.__setitem__("called", True),
    )
    monkeypatch.setattr(
        "src.ui.main_window.QDesktopServices.openUrl",
        lambda *_args, **_kwargs: opened.__setitem__("called", True),
    )
    monkeypatch.setattr("src.ui.main_window.speak", lambda *args, **kwargs: None)

    docs_build = Path("docs") / "_build" / "html"
    docs_build.mkdir(parents=True, exist_ok=True)
    html_index = docs_build / "index.html"

    if html_index.exists():
        html_index.unlink()
    gui._open_html_help()
    assert warned["called"] is True

    warned["called"] = False
    html_index.write_text("<html><body>help</body></html>", encoding="utf-8")
    gui._open_html_help()
    assert opened["called"] is True


def test_get_simulation_params_info_with_and_without_engine(gui):
    msg_without_engine = gui.get_simulation_params_info()
    assert "Simulation time step" in msg_without_engine

    gui._apply_to_simulation()
    gui.engine.motor.params.friction_coefficient = 0.0
    msg_with_engine = gui.get_simulation_params_info()
    assert "Electrical time constant" in msg_with_engine
    assert "Mechanical time constant" in msg_with_engine


def test_open_user_manual_pdf_uses_generated_path(gui, monkeypatch, tmp_path):
    opened = {"called": False}
    pdf_path = tmp_path / "manual.pdf"
    pdf_path.write_bytes(b"%PDF-1.4")

    monkeypatch.setattr(gui, "_ensure_user_manual_pdf", lambda: pdf_path)
    monkeypatch.setattr(
        "src.ui.main_window.QDesktopServices.openUrl",
        lambda *_args, **_kwargs: opened.__setitem__("called", True),
    )
    monkeypatch.setattr("src.ui.main_window.speak", lambda *args, **kwargs: None)

    gui._open_user_manual_pdf()

    assert opened["called"] is True


def test_show_simulation_params_displays_info(gui, monkeypatch):
    informed = {"called": False}
    monkeypatch.setattr(
        "src.ui.main_window.QMessageBox.information",
        lambda *args, **kwargs: informed.__setitem__("called", True),
    )
    monkeypatch.setattr("src.ui.main_window.speak", lambda *args, **kwargs: None)

    gui._show_simulation_params()

    assert informed["called"] is True


def test_collect_and_save_simulation_parameters(gui, monkeypatch, tmp_path):
    payload = gui._collect_simulation_configuration()
    assert payload["schema"] == "bldc.simulation_config.v1"
    assert "motor_params" in payload
    assert "vf_controller" in payload
    assert "foc_controller" in payload

    out = tmp_path / "sim_params"
    informed = {"called": False}
    critical = {"called": False}

    monkeypatch.setattr(
        "src.ui.main_window.QFileDialog.getSaveFileName",
        lambda *args, **kwargs: (str(out), "JSON Files (*.json)"),
    )
    monkeypatch.setattr(
        "src.ui.main_window.QMessageBox.information",
        lambda *args, **kwargs: informed.__setitem__("called", True),
    )
    monkeypatch.setattr("src.ui.main_window.speak", lambda *args, **kwargs: None)

    gui._save_simulation_parameters()
    saved_file = out.with_suffix(".json")
    assert saved_file.exists()
    assert informed["called"] is True

    monkeypatch.setattr(
        "src.ui.main_window.QFileDialog.getSaveFileName",
        lambda *args, **kwargs: (str(saved_file), "JSON Files (*.json)"),
    )
    monkeypatch.setattr(
        "pathlib.Path.write_text",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("io error")),
    )
    monkeypatch.setattr(
        "src.ui.main_window.QMessageBox.critical",
        lambda *args, **kwargs: critical.__setitem__("called", True),
    )

    gui._save_simulation_parameters()
    assert critical["called"] is True


def test_accessible_text_block_keyboard_navigation(qapp):
    called = {"prev": 0, "next": 0}

    class TrackingTextBlock(AccessibleTextBlock):
        def focusPreviousChild(self) -> bool:
            called["prev"] += 1
            return True

        def focusNextChild(self) -> bool:
            called["next"] += 1
            return True

    block = TrackingTextBlock("Speed", "rpm", 0, 3)

    left = QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_Left, Qt.KeyboardModifier.NoModifier)
    down = QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_Down, Qt.KeyboardModifier.NoModifier)
    other = QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_A, Qt.KeyboardModifier.NoModifier, "a")

    block.keyPressEvent(left)
    block.keyPressEvent(down)
    block.keyPressEvent(other)

    assert called["prev"] == 1
    assert called["next"] == 1


class _DummySupply:
    def get_voltage(self, _time_s):
        return 48.0


class _DummyMotor:
    def __init__(self):
        self.currents = np.array([0.1, -0.05, -0.05], dtype=np.float64)


class _DummyEngine:
    def __init__(self, dt=1e-4):
        self.dt = dt
        self.time = 0.0
        self.supply_profile = _DummySupply()
        self.motor = _DummyMotor()
        self.last_pwm_hz = None
        self.last_timing = None
        self.last_telemetry = None
        self.after_step: Callable[[], None] | None = None

    def get_control_timing_state(self):
        return {"control_period_s": self.dt}

    def set_pwm_frequency(self, pwm_hz):
        self.last_pwm_hz = pwm_hz

    def record_control_timing(self, calc_duration_s, control_period_s):
        self.last_timing = (calc_duration_s, control_period_s)

    def set_inverter_telemetry(self, telemetry):
        self.last_telemetry = telemetry

    def step(self, _voltages, log_data=True):
        assert log_data is True
        self.time += self.dt
        if self.after_step is not None:
            self.after_step()

    def get_current_state(self):
        return {"speed_rpm": 100.0, "time": self.time}

    def get_simulation_info(self):
        return {"hardware": {"enabled": False}}


class _DummySVM:
    def __init__(self):
        self.sample_time = None
        self.dc_voltage = None
        self.phase_currents = None

    def set_sample_time(self, dt):
        self.sample_time = dt

    def set_dc_voltage(self, supply_v):
        self.dc_voltage = supply_v

    def set_phase_currents(self, currents):
        self.phase_currents = currents

    def modulate(self, magnitude, angle):
        return np.array([magnitude, angle, -magnitude - angle], dtype=np.float64)

    def modulate_cartesian(self, valpha, vbeta):
        raise RuntimeError("force fallback")

    def get_last_telemetry(self):
        return {"effective_dc_voltage": 48.0}


class _DummyController(BaseController):
    output_cartesian = False

    def update(self, _period):
        return (5.0, 0.1)

    def reset(self) -> None:
        return None

    def get_state(self) -> dict[str, float]:
        return {}


class _DummyCartesianController(BaseController):
    output_cartesian = True

    def update(self, _period):
        return (1.0, -2.0)

    def reset(self) -> None:
        return None

    def get_state(self) -> dict[str, float]:
        return {}


class _BadController(BaseController):
    def update(self, _period):
        return 1.23

    def reset(self) -> None:
        return None

    def get_state(self) -> dict[str, float]:
        return {}


def test_simulation_thread_run_early_return_and_normal_iteration(monkeypatch):
    thread = SimulationThread()
    thread.run()  # no engine/svm/controller -> early return

    engine = _DummyEngine()
    svm = _DummySVM()
    controller = _DummyController()
    thread.set_simulation(
        cast(SimulationEngine, engine),
        cast(SVMGenerator, svm),
        controller,
        pwm_frequency_hz=20000.0,
    )
    thread.update_interval = 0.0
    thread.running = True
    engine.after_step = lambda: setattr(thread, "running", False)
    thread.run()

    assert svm.sample_time == pytest.approx(engine.dt)
    assert engine.last_pwm_hz == pytest.approx(20000.0)
    assert engine.last_timing is not None
    assert thread.get_latest_state().get("speed_rpm") == pytest.approx(100.0)


def test_simulation_thread_cartesian_fallback_and_bad_output():
    engine = _DummyEngine()
    svm = _DummySVM()

    # Cartesian branch falls back to inverse_clarke when modulate_cartesian fails.
    t_cart = SimulationThread()
    t_cart.set_simulation(
        cast(SimulationEngine, engine),
        cast(SVMGenerator, svm),
        _DummyCartesianController(),
    )
    t_cart.update_interval = 0.0
    t_cart.running = True
    engine.after_step = lambda: setattr(t_cart, "running", False)
    t_cart.run()

    # Unexpected controller output should raise ValueError.
    t_bad = SimulationThread()
    engine_bad = _DummyEngine()
    svm_bad = _DummySVM()
    t_bad.set_simulation(
        cast(SimulationEngine, engine_bad),
        cast(SVMGenerator, svm_bad),
        _BadController(),
    )
    t_bad.running = True

    with pytest.raises(ValueError, match="unexpected output format"):
        t_bad.run()


def test_simulation_thread_start_and_stop_paths(monkeypatch):
    thread = SimulationThread()

    started = {"count": 0}
    monkeypatch.setattr(thread, "start", lambda: started.__setitem__("count", started["count"] + 1))

    # Branch where thread is not already running.
    monkeypatch.setattr(thread, "isRunning", lambda: False)
    thread.start_simulation()
    assert thread.running is True
    assert started["count"] == 1

    # Branch where thread is already running should not call start again.
    monkeypatch.setattr(thread, "isRunning", lambda: True)
    thread.start_simulation()
    assert started["count"] == 1

    thread.stop_simulation()
    assert thread.running is False


def test_simulation_thread_max_duration_break_and_missing_svm_io_methods():
    class _NoIoSVM:
        def __init__(self):
            self.sample_time = None

        def set_sample_time(self, dt):
            self.sample_time = dt

        def modulate(self, magnitude, angle):
            return np.array([magnitude, angle, -magnitude - angle], dtype=np.float64)

        def get_last_telemetry(self):
            return {"effective_dc_voltage": 24.0}

    class _BreakEngine(_DummyEngine):
        def __init__(self, dt=1e-4):
            super().__init__(dt=dt)
            self.step_calls = 0

        def step(self, _voltages, log_data=True):
            self.step_calls += 1
            super().step(_voltages, log_data=log_data)

    engine = _BreakEngine(dt=1e-4)
    svm = _NoIoSVM()
    controller = _DummyController()

    thread = SimulationThread()
    thread.set_simulation(
        cast(SimulationEngine, engine),
        cast(SVMGenerator, svm),
        controller,
        max_duration=engine.dt,
    )
    thread.update_interval = 10.0
    thread.running = True
    thread.run()

    # One control step executes, then loop exits via max-duration break path.
    assert engine.step_calls == 1
    assert svm.sample_time == pytest.approx(engine.dt)
