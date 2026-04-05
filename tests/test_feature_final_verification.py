"""
Atomic features tested in this module:
- visualization create_3phase_plot has grid parameters
- motor model instantiates with numba or fallback
- main window exposes status bar QLabel widgets
- data logger save_simulation_data accepts use_custom_path
- motor parameters expose ld and lq attributes
"""

import inspect
import sys

import pytest
from PyQt6.QtWidgets import QApplication, QLabel


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    return app


def test_visualization_create_3phase_plot_has_grid_parameters():
    from src.visualization.visualization import SimulationPlotter

    sig = inspect.signature(SimulationPlotter.create_3phase_plot)
    params = list(sig.parameters.keys())
    expected = ["history", "figsize", "grid_on", "grid_spacing", "minor_grid", "grid_spacing_y"]
    assert all(p in params for p in expected)


def test_motor_model_instantiates():
    from src.core.motor_model import BLDCMotor, MotorParameters

    motor = BLDCMotor(MotorParameters())
    assert motor is not None


def test_main_window_exposes_status_bar_widgets(qapp):
    from src.ui.main_window import BLDCMotorControlGUI

    gui = BLDCMotorControlGUI()
    assert hasattr(gui, "status_bar_dt")
    assert isinstance(gui.status_bar_dt, QLabel)


def test_data_logger_accepts_use_custom_path_param():
    from src.utils.data_logger import DataLogger

    sig = inspect.signature(DataLogger.save_simulation_data)
    assert "use_custom_path" in sig.parameters


def test_motor_parameters_expose_ld_lq():
    from src.core.motor_model import MotorParameters

    params = MotorParameters()
    assert hasattr(params, "ld")
    assert hasattr(params, "lq")
