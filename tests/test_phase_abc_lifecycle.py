"""
Atomic features tested in this module:
- MarkTask
- TerminateProcessGracefully
- CanStartTask
- CloseEvent
- StatusBarWidgets
- OnCalibProfileChanged
- StartCalibration
- StopCalibration
- OnCalibOutput
- OnCalibFinished
"""

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest
from PyQt6.QtCore import QByteArray, QProcess
from PyQt6.QtWidgets import QApplication, QMessageBox

from src.ui.main_window import BLDCMotorControlGUI


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


# ---------------------------------------------------------------------------
# Helpers / fakes
# ---------------------------------------------------------------------------


class _FakeSimThread:
    """Minimal fake SimulationThread (has stop_simulation attr like the real class)."""

    def __init__(self, wait_returns=True):
        self._wait_returns = wait_returns
        self.stopped = False

    def stop_simulation(self):
        self.stopped = True

    def wait(self, timeout=0):
        return self._wait_returns


class _FakeQProcess:
    """Minimal fake QProcess (has state attr like the real class)."""

    def __init__(self, running=True, wait_for_finished=True):
        self._running = running
        self._wait_for_finished = wait_for_finished
        self.terminated = False
        self.killed = False

    def state(self):
        if self._running:
            return QProcess.ProcessState.Running
        return QProcess.ProcessState.NotRunning

    def terminate(self):
        self.terminated = True

    def waitForFinished(self, ms):
        return self._wait_for_finished

    def kill(self):
        self.killed = True

    def readAllStandardOutput(self):
        return QByteArray(b"")


# ===========================================================================
# Phase A: Process lifecycle tests
# ===========================================================================


class TestMarkTask:
    def test_mark_task_running_sets_name(self, gui):
        gui._mark_task_running("simulation")
        assert gui._get_running_task_name() == "simulation"
        gui._mark_task_finished("simulation")

    def test_mark_task_finished_clears_matching(self, gui):
        gui._mark_task_running("calibration")
        gui._mark_task_finished("calibration")
        assert gui._get_running_task_name() is None

    def test_mark_task_finished_ignores_mismatch(self, gui):
        gui._mark_task_running("simulation")
        gui._mark_task_finished("calibration")  # different name â†’ no-op
        assert gui._get_running_task_name() == "simulation"
        gui._mark_task_finished("simulation")  # cleanup


class TestTerminateProcessGracefully:
    def test_none_returns_true(self, gui):
        assert gui._terminate_process_gracefully(None) is True

    def test_qthread_succeeds_gracefully(self, gui):
        fake = _FakeSimThread(wait_returns=True)
        result = gui._terminate_process_gracefully(fake)
        assert result is True
        assert fake.stopped is True

    def test_qthread_timeout_returns_false(self, gui):
        fake = _FakeSimThread(wait_returns=False)
        result = gui._terminate_process_gracefully(fake, timeout_graceful_ms=1)
        assert result is False
        assert fake.stopped is True

    def test_qprocess_not_running_returns_true_no_terminate(self, gui):
        fake = _FakeQProcess(running=False)
        result = gui._terminate_process_gracefully(fake)
        assert result is True
        assert fake.terminated is False

    def test_qprocess_graceful_success(self, gui):
        fake = _FakeQProcess(running=True, wait_for_finished=True)
        result = gui._terminate_process_gracefully(fake)
        assert result is True
        assert fake.terminated is True
        assert fake.killed is False

    def test_qprocess_kill_fallback_when_graceful_fails(self, gui):
        # waitForFinished always returns False â†’ kill must be called
        fake = _FakeQProcess(running=True, wait_for_finished=False)
        gui._terminate_process_gracefully(fake)
        assert fake.terminated is True
        assert fake.killed is True


class TestCanStartTask:
    def test_no_conflict_returns_true(self, gui):
        with gui._task_lock:
            gui._running_task_name = None
        assert gui._can_start_task("simulation") is True

    def test_same_task_blocks_and_speaks(self, gui, monkeypatch):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda msg: spoken.append(msg))
        gui._mark_task_running("simulation")
        result = gui._can_start_task("simulation")
        gui._mark_task_finished("simulation")
        assert result is False
        assert any("simulation" in m.lower() for m in spoken)

    def test_different_task_user_cancels(self, gui, monkeypatch):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda msg: spoken.append(msg))
        monkeypatch.setattr(
            "src.ui.main_window.QMessageBox.warning",
            lambda *a, **kw: QMessageBox.StandardButton.Cancel,
        )
        gui._mark_task_running("simulation")
        result = gui._can_start_task("calibration")
        gui._mark_task_finished("simulation")
        assert result is False
        assert any("cancelled" in m.lower() for m in spoken)

    def test_different_task_user_accepts_stops_simulation(self, gui, monkeypatch):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        monkeypatch.setattr(
            "src.ui.main_window.QMessageBox.warning",
            lambda *a, **kw: QMessageBox.StandardButton.Yes,
        )
        stopped = []
        monkeypatch.setattr(gui, "_stop_simulation", lambda: stopped.append(1))

        gui._mark_task_running("simulation")
        gui.is_running = True
        result = gui._can_start_task("calibration")
        gui.is_running = False
        gui._mark_task_finished("simulation")

        assert result is True
        assert len(stopped) == 1

    def test_different_task_user_accepts_stops_calibration(self, gui, monkeypatch):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        monkeypatch.setattr(
            "src.ui.main_window.QMessageBox.warning",
            lambda *a, **kw: QMessageBox.StandardButton.Yes,
        )
        stopped = []
        monkeypatch.setattr(gui, "_stop_calibration", lambda: stopped.append(1))

        gui._mark_task_running("calibration")
        fake_proc = _FakeQProcess(running=True)
        gui.calib_process = fake_proc
        result = gui._can_start_task("simulation")
        gui._mark_task_finished("calibration")
        gui.calib_process = None

        assert result is True
        assert len(stopped) == 1


class TestCloseEvent:
    def test_close_event_stops_running_simulation(self, qapp):
        window = BLDCMotorControlGUI()
        thread = _FakeSimThread(wait_returns=True)
        window.sim_thread = thread
        window.is_running = True
        window.close()  # triggers closeEvent
        assert thread.stopped is True
        assert window.is_running is False

    def test_close_event_terminates_running_calib_process(self, qapp, monkeypatch):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        window = BLDCMotorControlGUI()
        fake_proc = _FakeQProcess(running=True, wait_for_finished=True)
        window.calib_process = fake_proc
        window.close()
        assert fake_proc.terminated is True

    def test_close_event_no_processes_is_safe(self, qapp):
        window = BLDCMotorControlGUI()
        window.is_running = False
        window.calib_process = None
        window.close()  # should not raise

    def test_close_event_kills_calib_process_if_terminate_times_out(
        self, qapp, monkeypatch
    ):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        window = BLDCMotorControlGUI()
        fake_proc = _FakeQProcess(running=True, wait_for_finished=False)
        window.calib_process = fake_proc
        window.close()
        assert fake_proc.terminated is True
        assert fake_proc.killed is True


# ===========================================================================
# Phase B: Status bar tests
# ===========================================================================


class TestStatusBarWidgets:
    def test_status_bar_widgets_exist(self, gui):
        assert hasattr(gui, "status_bar_state")
        assert hasattr(gui, "status_bar_task")
        assert hasattr(gui, "status_bar_time_remaining")
        assert hasattr(gui, "status_bar_cpu_load")

    def test_update_status_bar_running_finite_duration(self, gui, monkeypatch):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        gui.is_running = True
        gui.sim_thread = MagicMock()
        gui.sim_duration.setValue(10.0)

        snapshot = {"time": 3.0, "cpu_load_pct": 42.5}
        gui._update_status_bar(snapshot)

        assert "Running" in gui.status_bar_state.text()
        # remaining = 10 - 3 = 7.0s
        assert "7.0s" in gui.status_bar_time_remaining.text()
        assert "42.5" in gui.status_bar_cpu_load.text()

        # cleanup
        gui.is_running = False
        gui.sim_thread = None

    def test_update_status_bar_running_infinite_duration(self, gui, monkeypatch):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        gui.is_running = True
        gui.sim_thread = MagicMock()
        gui.sim_duration.setValue(0)  # 0 = infinite

        snapshot = {"time": 5.5, "cpu_load_pct": 0.0}
        gui._update_status_bar(snapshot)

        assert "Elapsed: 5.5s" in gui.status_bar_time_remaining.text()
        assert "CPU: -- %" in gui.status_bar_cpu_load.text()

        gui.is_running = False
        gui.sim_thread = None

    def test_update_status_bar_stopped_state(self, gui):
        gui.is_running = False
        snapshot = {"time": 0.0, "cpu_load_pct": 0.0}
        gui._update_status_bar(snapshot)
        assert "Stopped" in gui.status_bar_state.text()
        assert "Remaining: -- s" in gui.status_bar_time_remaining.text()

    def test_update_status_bar_exception_is_silent(self, gui):
        """Deleting a status bar widget should be swallowed by the try/except."""
        original = gui.status_bar_state
        del gui.status_bar_state
        try:
            gui._update_status_bar({"time": 0.0})  # must not raise
        finally:
            gui.status_bar_state = original

    def test_update_status_bar_task_name_uppercase_capitalized(self, gui, monkeypatch):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        gui.is_running = False
        gui._mark_task_running("simulation")
        gui._update_status_bar({"time": 0.0, "cpu_load_pct": 0.0})
        assert "Simulation" in gui.status_bar_task.text()
        gui._mark_task_finished("simulation")


# ===========================================================================
# Phase C: Calibration GUI tests
# ===========================================================================


class TestOnCalibProfileChanged:
    def test_existing_session_no_warning_tag(self, gui, tmp_path, monkeypatch):
        import src.ui.main_window as mw

        profiles_dir = tmp_path / "motor_profiles"
        profiles_dir.mkdir()
        session_dir = tmp_path / "tuning_sessions" / "until_converged"
        session_dir.mkdir(parents=True)
        (session_dir / "some_stem_until_converged.json").write_text("{}")

        monkeypatch.setattr(mw, "MOTOR_PROFILES_DIR", profiles_dir)
        gui._on_calib_profile_changed("some_stem.json")

        assert "\u26a0" not in gui.calib_session_label.text()

    def test_missing_session_shows_warning_tag(self, gui, tmp_path, monkeypatch):
        import src.ui.main_window as mw

        profiles_dir = tmp_path / "motor_profiles"
        profiles_dir.mkdir()
        # Do NOT create the session file

        monkeypatch.setattr(mw, "MOTOR_PROFILES_DIR", profiles_dir)
        gui._on_calib_profile_changed("missing_stem.json")

        assert "\u26a0" in gui.calib_session_label.text()

    def test_output_label_is_updated(self, gui, tmp_path, monkeypatch):
        import src.ui.main_window as mw

        profiles_dir = tmp_path / "motor_profiles"
        profiles_dir.mkdir()
        monkeypatch.setattr(mw, "MOTOR_PROFILES_DIR", profiles_dir)
        gui._on_calib_profile_changed("my_motor.json")

        assert (
            "calibration_my_motor_fw_loaded_point.json" in gui.calib_output_label.text()
        )


class TestStartCalibration:
    def test_no_profile_selected_shows_warning(self, gui, monkeypatch):
        warned = []
        monkeypatch.setattr(
            "src.ui.main_window.QMessageBox.warning",
            lambda *a, **kw: warned.append(True),
        )
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        with gui._task_lock:
            gui._running_task_name = None

        gui.calib_profile_combo.clear()
        gui.calib_profile_combo.addItem("(no profiles found)")
        gui.calib_profile_combo.setCurrentIndex(0)
        gui._start_calibration()

        assert warned

    def test_missing_profile_file_shows_critical(self, gui, monkeypatch, tmp_path):
        import src.ui.main_window as mw

        criticals = []
        monkeypatch.setattr(
            "src.ui.main_window.QMessageBox.critical",
            lambda *a, **kw: criticals.append(True),
        )
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        with gui._task_lock:
            gui._running_task_name = None

        profiles_dir = tmp_path / "motor_profiles"
        profiles_dir.mkdir()
        # Do NOT create the profile file
        monkeypatch.setattr(mw, "MOTOR_PROFILES_DIR", profiles_dir)

        gui.calib_profile_combo.clear()
        gui.calib_profile_combo.addItem("nonexistent.json")
        gui.calib_profile_combo.setCurrentIndex(0)
        gui._start_calibration()

        assert criticals

    def test_missing_session_file_shows_critical(self, gui, monkeypatch, tmp_path):
        import src.ui.main_window as mw

        criticals = []
        monkeypatch.setattr(
            "src.ui.main_window.QMessageBox.critical",
            lambda *a, **kw: criticals.append(True),
        )
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        with gui._task_lock:
            gui._running_task_name = None

        profiles_dir = tmp_path / "motor_profiles"
        profiles_dir.mkdir()
        (profiles_dir / "test_motor.json").write_text("{}")  # profile exists
        # Do NOT create the session file
        monkeypatch.setattr(mw, "MOTOR_PROFILES_DIR", profiles_dir)

        gui.calib_profile_combo.clear()
        gui.calib_profile_combo.addItem("test_motor.json")
        gui.calib_profile_combo.setCurrentIndex(0)
        gui._start_calibration()

        assert criticals

    def test_success_launches_qprocess(self, gui, monkeypatch, tmp_path):
        import src.ui.main_window as mw

        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        with gui._task_lock:
            gui._running_task_name = None

        profiles_dir = tmp_path / "motor_profiles"
        profiles_dir.mkdir()
        (profiles_dir / "test_motor.json").write_text("{}")

        session_dir = tmp_path / "tuning_sessions" / "until_converged"
        session_dir.mkdir(parents=True)
        (session_dir / "test_motor_until_converged.json").write_text("{}")

        monkeypatch.setattr(mw, "MOTOR_PROFILES_DIR", profiles_dir)

        mock_process = MagicMock()
        monkeypatch.setattr(
            "src.ui.main_window.QProcess", MagicMock(return_value=mock_process)
        )

        gui.calib_process = None
        gui.calib_profile_combo.clear()
        gui.calib_profile_combo.addItem("test_motor.json")
        gui.calib_profile_combo.setCurrentIndex(0)
        gui._start_calibration()

        mock_process.start.assert_called_once()
        assert "Running" in gui.status_bar_state.text()
        assert "Calibration" in gui.status_bar_task.text()

        # cleanup: _start_calibration marked task running
        gui._mark_task_finished("calibration")
        gui.calib_process = None


class TestStopCalibration:
    def test_no_process_does_not_raise(self, gui, monkeypatch):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        gui.calib_process = None
        gui._stop_calibration()  # must not raise

    def test_running_process_is_terminated(self, gui, monkeypatch):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        fake = _FakeQProcess(running=True, wait_for_finished=True)
        gui.calib_process = fake
        gui._stop_calibration()

        assert fake.terminated is True
        assert fake.killed is False
        assert gui.btn_start_calib.isEnabled() is True
        assert gui.btn_stop_calib.isEnabled() is False

    def test_kill_fallback_when_graceful_times_out(self, gui, monkeypatch):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        fake = _FakeQProcess(running=True, wait_for_finished=False)
        gui.calib_process = fake
        gui._stop_calibration()

        assert fake.terminated is True
        assert fake.killed is True

    def test_status_bar_updated_to_stopped(self, gui, monkeypatch):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        gui.calib_process = None
        gui._stop_calibration()

        assert "Stopped" in gui.status_bar_state.text()
        assert "None" in gui.status_bar_task.text()


class TestOnCalibOutput:
    def _inject_output(self, gui, data: bytes):
        class _OutputProcess(_FakeQProcess):
            def __init__(self):
                super().__init__(running=True)
                self._data = data

            def readAllStandardOutput(self):
                return QByteArray(self._data)

        gui.calib_process = _OutputProcess()

    def test_output_appended_to_log(self, gui, monkeypatch):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        self._inject_output(gui, b"Hello calibration line\n")
        gui.calib_log.clear()
        gui._on_calib_output()
        assert "Hello calibration line" in gui.calib_log.toPlainText()

    def test_step1_milestone_spoken(self, gui, monkeypatch):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda m: spoken.append(m))
        self._inject_output(gui, b"STEP1_START\n")
        gui._on_calib_output()
        assert any("step 1" in m.lower() for m in spoken)

    def test_step2_milestone_spoken(self, gui, monkeypatch):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda m: spoken.append(m))
        self._inject_output(gui, b"STEP2_START\n")
        gui._on_calib_output()
        assert any("step 2" in m.lower() for m in spoken)

    def test_step3_milestone_spoken(self, gui, monkeypatch):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda m: spoken.append(m))
        self._inject_output(gui, b"STEP3_START\n")
        gui._on_calib_output()
        assert any("step 3" in m.lower() for m in spoken)

    def test_torque_ok_reports_value(self, gui, monkeypatch):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda m: spoken.append(m))
        self._inject_output(gui, b"TORQUE_OK 12.5\n")
        gui._on_calib_output()
        assert any("12.5" in m for m in spoken)

    def test_torque_fail_reports_value(self, gui, monkeypatch):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda m: spoken.append(m))
        self._inject_output(gui, b"TORQUE_FAIL 8.0\n")
        gui._on_calib_output()
        assert any("8.0" in m for m in spoken)

    def test_report_saved_spoken(self, gui, monkeypatch):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda m: spoken.append(m))
        self._inject_output(gui, b"REPORT_SAVED\n")
        gui._on_calib_output()
        assert any("report" in m.lower() for m in spoken)


class TestOnCalibFinished:
    def _make_report(self, tmp_path: Path) -> Path:
        report = {
            "step3_final_working_point_tuning": {
                "target_load_nm": 5.0,
                "success": True,
                "result_high_fidelity": {
                    "metrics": {
                        "mean_speed_rpm_last_1s": 1000.0,
                        "efficiency_pct_last_1s": 87.5,
                        "fw_injection_dc_a_last_1s": -2.3,
                    }
                },
            }
        }
        out = tmp_path / "calib_output.json"
        out.write_text(json.dumps(report), encoding="utf-8")
        return out

    def test_success_parses_json_and_updates_labels(self, gui, monkeypatch, tmp_path):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda m: spoken.append(m))
        out = self._make_report(tmp_path)
        gui.calib_output_path = out

        gui._on_calib_finished(0, None)

        assert "PASS" in gui.calib_result_status.text()
        assert "1000" in gui.calib_result_speed.text()
        assert "87.5" in gui.calib_result_efficiency.text()
        assert any("1000" in m for m in spoken)

    def test_exit_code_nonzero_shows_failure(self, gui, monkeypatch):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda m: spoken.append(m))
        gui.calib_output_path = None
        gui._on_calib_finished(1, None)
        assert "Failed" in gui.calib_result_status.text()
        assert any("failed" in m.lower() for m in spoken)

    def test_exit_code_zero_but_missing_output_file(self, gui, monkeypatch, tmp_path):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda m: spoken.append(m))
        gui.calib_output_path = tmp_path / "nonexistent.json"  # does not exist
        gui._on_calib_finished(0, None)
        assert "Failed" in gui.calib_result_status.text()

    def test_json_parse_error_shows_error_status(self, gui, monkeypatch, tmp_path):
        spoken = []
        monkeypatch.setattr("src.ui.main_window.speak", lambda m: spoken.append(m))
        bad_json = tmp_path / "bad.json"
        bad_json.write_text("NOT VALID JSON", encoding="utf-8")
        gui.calib_output_path = bad_json
        gui._on_calib_finished(0, None)
        assert "Error" in gui.calib_result_status.text()
        assert any("could not be read" in m.lower() for m in spoken)

    def test_calib_process_cleared_and_buttons_reset(self, gui, monkeypatch, tmp_path):
        monkeypatch.setattr("src.ui.main_window.speak", lambda *a: None)
        gui.calib_output_path = None
        gui._on_calib_finished(1, None)
        assert gui.calib_process is None
        assert gui.btn_start_calib.isEnabled() is True
        assert gui.btn_stop_calib.isEnabled() is False
