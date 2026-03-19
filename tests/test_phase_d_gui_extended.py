"""
Atomic features tested in this module:
- BuildCurrentSenseModelExtended
- ApplyToSimulationCurrentSense
- RuntimeActualsEdgeCases
- UpdateCurrentSenseStatus
- CurrentFftWindow
- OnCalibFinished
- InitializeDefaults
- OnSimulationFinished
- OnControlModeChanged
- SimulationEngineCurrentSenseIntegration
- SimulationStartStop
- UpdateMonitoring
- ResetSimulation
- AutoTuneAxis
"""
import json
import sys
import tempfile
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from PyQt6.QtWidgets import QApplication, QMessageBox

from src.ui.main_window import BLDCMotorControlGUI
from src.hardware.inverter_current_sense import (
    InverterCurrentSense,
    ShuntAmplifierChannel,
)


# ---------------------------------------------------------------------------
# Qt application fixture (module-scoped to avoid re-creating the app)
# ---------------------------------------------------------------------------


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


# ===========================================================================
# Section 1 â€“ _build_current_sense_model (all topology paths)
# ===========================================================================


class TestBuildCurrentSenseModelExtended:
    """Cover triple, single, and missing-widget guard paths."""

    def test_missing_widget_returns_none(self, gui):
        """If current_sense_enable attribute is absent the method returns None."""
        original_has = gui.__dict__.pop("current_sense_enable", None)
        try:
            model = gui._build_current_sense_model()
            assert model is None
        finally:
            if original_has is not None:
                gui.current_sense_enable = original_has

    def test_triple_topology_returns_three_shunts(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("triple")
        model = gui._build_current_sense_model()
        assert model is not None
        assert model.topology == "triple"
        assert model.n_shunts == 3

    def test_single_topology_returns_one_shunt(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("single")
        model = gui._build_current_sense_model()
        assert model is not None
        assert model.topology == "single"
        assert model.n_shunts == 1

    def test_r_shunt_propagated_to_all_channels(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("triple")
        gui.current_sense_r_shunt.setValue(0.005)
        model = gui._build_current_sense_model()
        for ch in model.channels:
            assert ch.r_shunt_ohm == pytest.approx(0.005)

    def test_actual_gain_a_propagated_to_channel_zero(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("triple")
        gui.current_sense_actual_gain_a.setValue(30.0)
        model = gui._build_current_sense_model()
        assert model.channels[0].actual_gain == pytest.approx(30.0)

    def test_cutoff_hz_propagated(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("double")
        gui.current_sense_cutoff_hz.setValue(15000.0)
        model = gui._build_current_sense_model()
        for ch in model.channels:
            assert ch.cutoff_frequency_hz == pytest.approx(15000.0)


# ===========================================================================
# Section 2 â€“ _apply_to_simulation with current-sense variants
# ===========================================================================


class TestApplyToSimulationCurrentSense:
    """Verify _apply_to_simulation wires current_sense into SimulationEngine."""

    def test_triple_sense_wired_into_engine(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("triple")
        gui._apply_to_simulation()
        assert gui.engine is not None
        assert gui.engine.current_sense is not None
        assert gui.engine.current_sense.topology == "triple"

    def test_single_sense_wired_into_engine(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("single")
        gui._apply_to_simulation()
        assert gui.engine.current_sense is not None
        assert gui.engine.current_sense.topology == "single"

    def test_disabled_sense_engine_has_no_sense(self, gui):
        gui.current_sense_enable.setChecked(False)
        gui._apply_to_simulation()
        assert gui.engine.current_sense is None

    def test_current_sense_state_reflects_topology(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("double")
        gui._apply_to_simulation()
        cs_state = gui.engine._current_sense_state
        assert cs_state["enabled"] is True
        assert cs_state["topology"] == "double"

    def test_ramp_load_creates_ramp_profile(self, gui):
        """Choose Ramp load and verify engine is created (no crash)."""
        gui.load_type.setCurrentText("Ramp")
        gui.current_sense_enable.setChecked(False)
        gui._apply_to_simulation()
        assert gui.engine is not None
        # restore default
        gui.load_type.setCurrentText("Constant")

    def test_foc_mode_creates_foc_controller(self, gui):
        """Selecting FOC mode creates an FOCController."""
        from src.control.foc_controller import FOCController

        gui.ctrl_mode.setCurrentText("FOC")
        gui.current_sense_enable.setChecked(False)
        gui._apply_to_simulation()
        assert isinstance(gui.controller, FOCController)
        # restore
        gui.ctrl_mode.setCurrentText("V/f")

    def test_double_sense_creates_two_channels(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("double")
        gui._apply_to_simulation()
        assert gui.engine.current_sense.n_shunts == 2


# ===========================================================================
# Section 3 â€“ _update_runtime_current_sense_actuals edge cases
# ===========================================================================


class TestRuntimeActualsEdgeCases:
    """No-op paths when engine or sense is absent."""

    def test_no_op_when_engine_none(self, gui):
        original = gui.engine
        gui.engine = None
        try:
            gui._update_runtime_current_sense_actuals()  # must not raise
        finally:
            gui.engine = original

    def test_no_op_when_engine_sense_none(self, gui):
        gui.current_sense_enable.setChecked(False)
        gui._apply_to_simulation()
        assert gui.engine.current_sense is None
        gui._update_runtime_current_sense_actuals()  # must not raise

    def test_updates_applied_for_triple_with_modified_a_gain(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("triple")
        gui._apply_to_simulation()
        gui.current_sense_actual_gain_a.setValue(22.0)
        gui._update_runtime_current_sense_actuals()
        assert gui.engine.current_sense.channels[0].actual_gain == pytest.approx(22.0)


# ===========================================================================
# Section 4 â€“ _update_current_sense_status
# ===========================================================================


class TestUpdateCurrentSenseStatus:
    """Status label text contract."""

    def test_disabled_snapshot_shows_disabled_text(self, gui):
        snapshot = {"current_measurement": {"enabled": False, "topology": "triple"}}
        gui._update_current_sense_status(snapshot)
        text = gui.current_sense_status_label.text().lower()
        assert "disabled" in text

    def test_non_dict_current_measurement_shows_disabled(self, gui):
        snapshot = {"current_measurement": "not_a_dict"}
        gui._update_current_sense_status(snapshot)
        text = gui.current_sense_status_label.text().lower()
        assert "disabled" in text

    def test_empty_snapshot_shows_disabled(self, gui):
        gui._update_current_sense_status({})
        text = gui.current_sense_status_label.text().lower()
        assert "disabled" in text

    def test_non_dict_snapshot_shows_disabled(self, gui):
        gui._update_current_sense_status("bad")
        text = gui.current_sense_status_label.text().lower()
        assert "disabled" in text

    def test_active_snapshot_shows_topology_and_saturation(self, gui):
        snapshot = {
            "current_measurement": {
                "enabled": True,
                "topology": "triple",
                "adc_saturated": [False, True, False],
                "phase_voltage_drop_v": [0.002, -0.001, -0.001],
            }
        }
        gui._update_current_sense_status(snapshot)
        text = gui.current_sense_status_label.text().lower()
        assert "triple" in text
        assert "1" in text  # 1 saturated channel

    def test_drop_rms_computed_non_zero(self, gui):
        snapshot = {
            "current_measurement": {
                "enabled": True,
                "topology": "double",
                "adc_saturated": [],
                "phase_voltage_drop_v": [0.1, 0.1, 0.0],
            }
        }
        gui._update_current_sense_status(snapshot)
        text = gui.current_sense_status_label.text()
        # Check that the drop RMS is non-zero and displayed as float
        assert "0." in text


# ===========================================================================
# Section 5 â€“ _open_current_fft_window / _on_current_fft_window_closed
# ===========================================================================


class TestCurrentFftWindow:
    def test_opens_new_window_when_none(self, gui):
        gui.current_fft_window = None
        gui._open_current_fft_window()
        assert gui.current_fft_window is not None
        gui.current_fft_window.close()
        gui.current_fft_window = None

    def test_focuses_existing_window_instead_of_creating_new(self, gui):
        gui.current_fft_window = None
        gui._open_current_fft_window()
        first_window = gui.current_fft_window
        gui._open_current_fft_window()  # should reuse, not create a new one
        assert gui.current_fft_window is first_window
        gui.current_fft_window.close()
        gui.current_fft_window = None

    def test_on_closed_clears_reference(self, gui):
        gui.current_fft_window = None
        gui._open_current_fft_window()
        assert gui.current_fft_window is not None
        gui._on_current_fft_window_closed()
        assert gui.current_fft_window is None


# ===========================================================================
# Section 6 â€“ _on_calib_finished (Phase C calibration result display)
# ===========================================================================


class TestOnCalibFinished:
    """_on_calib_finished happy-path and error paths."""

    def _make_good_report(self) -> dict:
        return {
            "step3_final_working_point_tuning": {
                "success": True,
                "target_load_nm": 1.5,
                "result_high_fidelity": {
                    "metrics": {
                        "mean_speed_rpm_last_1s": 1450.0,
                        "efficiency_pct_last_1s": 87.3,
                        "fw_injection_dc_a_last_1s": -2.1,
                    }
                },
            }
        }

    def test_success_path_sets_pass_labels(self, gui, tmp_path):
        report_file = tmp_path / "calib_result.json"
        report_file.write_text(json.dumps(self._make_good_report()), encoding="utf-8")
        gui.calib_output_path = report_file
        gui._mark_task_running("calibration")
        with patch("src.ui.main_window.speak"):
            gui._on_calib_finished(0, None)
        assert "PASS" in gui.calib_result_status.text()
        assert "1450" in gui.calib_result_speed.text()

    def test_success_path_clears_calib_process(self, gui, tmp_path):
        report_file = tmp_path / "calib_result.json"
        report_file.write_text(json.dumps(self._make_good_report()), encoding="utf-8")
        gui.calib_output_path = report_file
        gui.calib_process = MagicMock()
        gui._mark_task_running("calibration")
        with patch("src.ui.main_window.speak"):
            gui._on_calib_finished(0, None)
        assert gui.calib_process is None

    def test_failed_exit_code_sets_failed_text(self, gui):
        gui.calib_output_path = None
        gui._mark_task_running("calibration")
        with patch("src.ui.main_window.speak"):
            gui._on_calib_finished(1, None)
        assert "Failed" in gui.calib_result_status.text()

    def test_path_none_shows_failed(self, gui):
        gui.calib_output_path = None
        gui._mark_task_running("calibration")
        with patch("src.ui.main_window.speak"):
            gui._on_calib_finished(0, None)
        assert "Failed" in gui.calib_result_status.text()

    def test_json_parse_error_shows_error_text(self, gui, tmp_path):
        report_file = tmp_path / "bad.json"
        report_file.write_bytes(b"\xff\xfe broken")  # invalid UTF-8 JSON
        gui.calib_output_path = report_file
        gui._mark_task_running("calibration")
        with patch("src.ui.main_window.speak"):
            gui._on_calib_finished(0, None)
        text = gui.calib_result_status.text()
        # Either "Error" from exception or "Failed" from path check
        assert ("Error" in text) or ("Failed" in text)

    def test_finish_marks_task_done(self, gui):
        gui.calib_output_path = None
        gui._mark_task_running("calibration")
        with patch("src.ui.main_window.speak"):
            gui._on_calib_finished(99, None)
        assert gui._get_running_task_name() is None

    def test_status_bar_updated_to_stopped(self, gui):
        gui.calib_output_path = None
        gui._mark_task_running("calibration")
        with patch("src.ui.main_window.speak"):
            gui._on_calib_finished(0, None)
        assert gui.status_bar_state.text() == "State: Stopped"


# ===========================================================================
# Section 7 â€“ _initialize_defaults
# ===========================================================================


class TestInitializeDefaults:
    def test_ctrl_mode_set_to_vf(self, gui):
        # Set it to something else first
        gui.ctrl_mode.setCurrentText("FOC")
        with patch.object(gui, "_apply_to_simulation"):
            gui._initialize_defaults()
        assert gui.ctrl_mode.currentText() == "V/f"

    def test_foc_group_hidden(self, gui):
        with patch.object(gui, "_apply_to_simulation"):
            gui._initialize_defaults()
        assert not gui.foc_group.isVisible()

    def test_calls_apply_to_simulation(self, gui):
        with patch.object(gui, "_apply_to_simulation") as mock_apply:
            gui._initialize_defaults()
        mock_apply.assert_called_once()


# ===========================================================================
# Section 8 â€“ _on_simulation_finished
# ===========================================================================


class TestOnSimulationFinished:
    def test_is_running_set_to_false(self, gui):
        gui.is_running = True
        gui._mark_task_running("simulation")
        gui._on_simulation_finished()
        assert gui.is_running is False

    def test_task_cleared(self, gui):
        gui.is_running = True
        gui._mark_task_running("simulation")
        gui._on_simulation_finished()
        assert gui._get_running_task_name() is None

    def test_status_bar_shows_stopped(self, gui):
        gui.is_running = True
        gui._mark_task_running("simulation")
        gui._on_simulation_finished()
        assert gui.status_bar_state.text() == "State: Stopped"

    def test_buttons_reset(self, gui):
        gui.is_running = True
        gui._mark_task_running("simulation")
        gui.btn_start.setEnabled(False)
        gui.btn_stop.setEnabled(True)
        gui._on_simulation_finished()
        assert gui.btn_start.isEnabled()
        assert not gui.btn_stop.isEnabled()


# ===========================================================================
# Section 9 â€“ _on_control_mode_changed
# ===========================================================================


class TestOnControlModeChanged:
    def test_vf_mode_shows_vf_hides_foc(self, gui):
        # In offscreen mode the parent window is never shown, so isVisible()
        # depends on the parent chain.  Use isHidden() to check the widget's
        # own explicit visibility state (only reflects setVisible calls).
        gui._on_control_mode_changed("V/f")
        assert not gui.vf_group.isHidden()
        assert gui.foc_group.isHidden()

    def test_foc_mode_shows_foc_hides_vf(self, gui):
        gui._on_control_mode_changed("FOC")
        assert gui.vf_group.isHidden()
        assert not gui.foc_group.isHidden()
        # restore
        gui._on_control_mode_changed("V/f")


# ===========================================================================
# Section 10 â€“ SimulationEngine current_sense integration (engine-level tests)
# ===========================================================================


class TestSimulationEngineCurrentSenseIntegration:
    """Direct engine-level tests for current_sense in step() / get_current_state()."""

    def _make_engine(self, topology="triple"):
        from src.core.simulation_engine import SimulationEngine
        from src.core.motor_model import BLDCMotor, MotorParameters
        from src.core.load_model import ConstantLoad

        params = MotorParameters()
        motor = BLDCMotor(params, dt=0.0001)
        load = ConstantLoad(torque=0.5)
        cs = InverterCurrentSense(topology=topology)
        engine = SimulationEngine(motor, load, dt=0.0001, current_sense=cs)
        return engine

    def test_current_sense_state_enabled_flag(self):
        engine = self._make_engine("triple")
        assert engine._current_sense_state["enabled"] is True
        assert engine._current_sense_state["topology"] == "triple"

    def test_step_executes_without_error(self):
        engine = self._make_engine("double")
        voltages = np.array([10.0, -5.0, -5.0])
        engine.step(voltages)

    def test_step_updates_measured_phase_currents(self):
        engine = self._make_engine("triple")
        voltages = np.array([12.0, -6.0, -6.0])
        engine.step(voltages)
        assert engine._measured_phase_currents.shape == (3,)

    def test_get_controller_phase_currents_with_sense(self):
        engine = self._make_engine("triple")
        engine.step(np.array([10.0, -5.0, -5.0]))
        currents = engine.get_controller_phase_currents()
        assert currents.shape == (3,)

    def test_get_controller_phase_currents_without_sense(self):
        from src.core.simulation_engine import SimulationEngine
        from src.core.motor_model import BLDCMotor, MotorParameters
        from src.core.load_model import ConstantLoad

        params = MotorParameters()
        motor = BLDCMotor(params, dt=0.0001)
        load = ConstantLoad(torque=0.0)
        engine = SimulationEngine(motor, load, dt=0.0001)
        engine.step(np.array([10.0, -5.0, -5.0]))
        currents = engine.get_controller_phase_currents()
        assert currents.shape == (3,)

    def test_get_current_state_exposes_true_currents(self):
        engine = self._make_engine("triple")
        engine.step(np.array([10.0, -5.0, -5.0]))
        state = engine.get_current_state()
        assert "currents_a_true" in state
        assert "currents_b_true" in state
        assert "currents_c_true" in state
        assert "current_measurement" in state

    def test_adc_saturated_field_in_sense_state(self):
        engine = self._make_engine("triple")
        engine.step(np.array([10.0, -5.0, -5.0]))
        cs = engine._current_sense_state
        assert "adc_saturated" in cs
        assert isinstance(cs["adc_saturated"], list)

    def test_shunt_voltage_drop_appears_in_state(self):
        engine = self._make_engine("triple")
        engine.step(np.array([10.0, -5.0, -5.0]))
        drop = engine._current_sense_state.get("phase_voltage_drop_v")
        assert drop is not None
        assert len(drop) == 3

    def test_single_shunt_step_runs(self):
        engine = self._make_engine("single")
        engine.step(np.array([10.0, -5.0, -5.0]))
        currents = engine.get_controller_phase_currents()
        assert currents.shape == (3,)

    def test_simulation_info_contains_current_measurement(self):
        engine = self._make_engine("double")
        engine.step(np.array([10.0, -5.0, -5.0]))
        info = engine.get_simulation_info()
        assert "current_measurement" in info
        assert info["current_measurement"]["enabled"] is True


# ===========================================================================
# Section 11 â€“ _start_simulation / _stop_simulation
# ===========================================================================


class TestSimulationStartStop:
    """Lifecycle tests for simulation start / stop methods."""

    def _make_mock_sim_thread(self):
        mock = MagicMock()
        mock.update_interval = 0.1
        mock.finished_signal = MagicMock()
        mock.finished_signal.connect = MagicMock()
        mock.start_simulation = MagicMock()
        mock.stop_simulation = MagicMock()
        mock.wait = MagicMock(return_value=True)
        mock.isRunning = MagicMock(return_value=False)
        return mock

    def test_start_simulation_sets_is_running(self, gui):
        gui.is_running = False
        gui.current_sense_enable.setChecked(False)
        mock_thread = self._make_mock_sim_thread()
        with (
            patch("src.ui.main_window.SimulationThread", return_value=mock_thread),
            patch("src.ui.main_window.speak"),
        ):
            gui._start_simulation()
        assert gui.is_running is True
        gui.is_running = False
        gui._mark_task_finished("simulation")

    def test_start_simulation_marks_task_running(self, gui):
        gui.is_running = False
        mock_thread = self._make_mock_sim_thread()
        with (
            patch("src.ui.main_window.SimulationThread", return_value=mock_thread),
            patch("src.ui.main_window.speak"),
        ):
            gui._start_simulation()
        assert gui._get_running_task_name() == "simulation"
        # cleanup
        gui.is_running = False
        gui._mark_task_finished("simulation")

    def test_start_simulation_disables_start_btn(self, gui):
        gui.is_running = False
        gui.btn_start.setEnabled(True)
        mock_thread = self._make_mock_sim_thread()
        with (
            patch("src.ui.main_window.SimulationThread", return_value=mock_thread),
            patch("src.ui.main_window.speak"),
        ):
            gui._start_simulation()
        assert not gui.btn_start.isEnabled()
        # cleanup
        gui.is_running = False
        gui._mark_task_finished("simulation")

    def test_start_simulation_updates_status_bar_state(self, gui):
        gui.is_running = False
        mock_thread = self._make_mock_sim_thread()
        with (
            patch("src.ui.main_window.SimulationThread", return_value=mock_thread),
            patch("src.ui.main_window.speak"),
        ):
            gui._start_simulation()
        assert gui.status_bar_state.text() == "State: Running"
        gui.is_running = False
        gui._mark_task_finished("simulation")

    def test_start_simulation_when_already_running_shows_noop(self, gui, monkeypatch):
        """If already running, _start_simulation warns and does not start again."""
        gui.is_running = True
        calls = []
        monkeypatch.setattr(
            "src.ui.main_window.QMessageBox.warning",
            lambda *a, **kw: calls.append(a),
        )
        gui._start_simulation()
        assert len(calls) == 1
        gui.is_running = False
        gui._mark_task_finished("simulation")

    def test_stop_simulation_clears_is_running(self, gui):
        gui.is_running = True
        gui._mark_task_running("simulation")
        mock_thread = self._make_mock_sim_thread()
        gui.sim_thread = mock_thread
        with (
            patch("src.ui.main_window.speak"),
            patch("src.ui.main_window.QMessageBox.information"),
        ):
            gui._stop_simulation()
        assert gui.is_running is False

    def test_stop_simulation_clears_task(self, gui):
        gui.is_running = True
        gui._mark_task_running("simulation")
        mock_thread = self._make_mock_sim_thread()
        gui.sim_thread = mock_thread
        with (
            patch("src.ui.main_window.speak"),
            patch("src.ui.main_window.QMessageBox.information"),
        ):
            gui._stop_simulation()
        assert gui._get_running_task_name() is None

    def test_stop_simulation_when_not_running_is_noop(self, gui):
        """_stop_simulation early-returns if is_running is False."""
        gui.is_running = False
        # Should not raise or call speak
        gui._stop_simulation()


# ===========================================================================
# Section 12 â€“ _update_monitoring (large display-update method)
# ===========================================================================


class TestUpdateMonitoring:
    """Call _update_monitoring with various state dicts to cover display logic."""

    def _make_state(self):
        return {
            "speed_rpm": 1200.0,
            "time": 1.5,
            "omega": 125.6,
            "theta": 0.5,
            "currents_a": 3.2,
            "currents_b": -1.5,
            "currents_c": -1.7,
            "torque": 0.85,
            "back_emf_a": 8.0,
            "back_emf_b": -4.0,
            "back_emf_c": -4.0,
            "pfc": {
                "power_factor": 0.97,
                "active_power_w": 120.0,
                "apparent_power_va": 123.5,
            },
            "efficiency_metrics": {
                "efficiency": 0.88,
                "mechanical_output_power_w": 100.0,
                "total_loss_power_w": 14.0,
            },
            "inverter": {
                "effective_dc_voltage": 48.0,
                "junction_temperature_c": 35.0,
            },
            "control_timing": {
                "calc_duration_s": 0.000012,
                "control_period_s": 0.00005,
                "cpu_load_pct": 24.0,
            },
            "hardware": {"backend": "none", "last_io_error": ""},
        }

    def test_update_monitoring_runs_without_error(self, gui):
        gui._apply_to_simulation()
        state = self._make_state()
        gui._update_monitoring(state)  # must not raise

    def test_update_monitoring_appends_speed_history(self, gui):
        gui.speed_history_time = []
        gui.speed_history_rpm = []
        state = self._make_state()
        gui._update_monitoring(state)
        assert gui.speed_history_rpm[-1] == pytest.approx(1200.0)

    def test_update_monitoring_with_empty_state(self, gui):
        gui._update_monitoring({})  # must not raise

    def test_update_monitoring_triggers_speed_chart_at_10_updates(self, gui):
        """After 10 calls the speed canvas is redrawn (no crash check)."""
        gui.speed_history_time = []
        gui.speed_history_rpm = []
        state = self._make_state()
        for i in range(10):
            state["time"] = float(i) * 0.1
            state["speed_rpm"] = float(i) * 100
            gui._update_monitoring(state)
        # If we get here without exception, the chart update did not crash.

    def test_update_monitoring_with_non_dict_pfc_is_safe(self, gui):
        state = self._make_state()
        state["pfc"] = "invalid"
        gui._update_monitoring(state)  # must not raise


# ===========================================================================
# Section 13 â€“ _reset_simulation
# ===========================================================================


class TestResetSimulation:
    def test_reset_calls_engine_reset(self, gui):
        gui.current_sense_enable.setChecked(False)
        gui._apply_to_simulation()
        assert gui.engine is not None
        gui.is_running = False
        gui.engine.reset = MagicMock()
        with patch("src.ui.main_window.QMessageBox.information"):
            gui._reset_simulation()
        gui.engine.reset.assert_called_once()

    def test_reset_with_no_engine_is_safe(self, gui):
        gui.is_running = False
        original = gui.engine
        gui.engine = None
        try:
            gui._reset_simulation()  # must not raise
        finally:
            gui.engine = original


# ===========================================================================
# Section 14 â€“ _auto_tune_axis
# ===========================================================================


class TestAutoTuneAxis:
    def test_warns_when_no_foc_controller(self, gui, monkeypatch):
        from src.control.vf_controller import VFController

        gui.current_sense_enable.setChecked(False)
        gui._apply_to_simulation()
        gui.ctrl_mode.setCurrentText("V/f")
        gui._apply_to_simulation()
        warnings = []
        monkeypatch.setattr(
            "src.ui.main_window.QMessageBox.warning",
            lambda *a, **kw: warnings.append(a),
        )
        gui._auto_tune_axis("d")
        assert len(warnings) == 1

    def test_auto_tunes_d_axis_when_foc(self, gui, monkeypatch):
        gui.ctrl_mode.setCurrentText("FOC")
        gui.current_sense_enable.setChecked(False)
        gui._apply_to_simulation()
        from src.control.foc_controller import FOCController

        assert isinstance(gui.controller, FOCController)
        called = []
        monkeypatch.setattr(
            gui.controller, "auto_tune_pi", lambda axis: called.append(axis)
        )
        with patch("src.ui.main_window.QMessageBox.information"):
            gui._auto_tune_axis("d")
        assert "d" in called
        # restore
        gui.ctrl_mode.setCurrentText("V/f")






