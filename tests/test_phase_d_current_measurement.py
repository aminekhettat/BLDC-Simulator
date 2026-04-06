"""
Atomic features tested in this module:
- CurrentSenseModelBuild
- RuntimeDriftUpdate
- CurrentSenseStatus
"""

import sys

import pytest
from PyQt6.QtWidgets import QApplication

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


class TestCurrentSenseModelBuild:
    def test_build_returns_none_when_disabled(self, gui):
        gui.current_sense_enable.setChecked(False)
        model = gui._build_current_sense_model()
        assert model is None

    def test_build_double_topology_returns_two_shunts(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("double")
        model = gui._build_current_sense_model()
        assert model is not None
        assert model.topology == "double"
        assert model.n_shunts == 2


class TestRuntimeDriftUpdate:
    def test_runtime_gain_offset_updates_active_model(self, gui):
        gui.current_sense_enable.setChecked(True)
        gui.current_sense_topology.setCurrentText("triple")
        gui._apply_to_simulation()

        assert gui.engine.current_sense is not None

        gui.current_sense_actual_gain_a.setValue(25.0)
        gui.current_sense_actual_offset_a.setValue(1.8)
        gui._update_runtime_current_sense_actuals()

        channel_a = gui.engine.current_sense.channels[0]
        assert channel_a.actual_gain == pytest.approx(25.0)
        assert channel_a.actual_offset_v == pytest.approx(1.8)


class TestCurrentSenseStatus:
    def test_status_text_reflects_snapshot(self, gui):
        snapshot = {
            "current_measurement": {
                "enabled": True,
                "topology": "single",
                "adc_saturated": [True],
                "phase_voltage_drop_v": [0.1, -0.05, -0.05],
            }
        }
        gui._update_current_sense_status(snapshot)
        text = gui.current_sense_status_label.text().lower()
        assert "topology: single" in text
        assert "adc saturation channels: 1" in text
