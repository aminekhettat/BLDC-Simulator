"""
Atomic features tested in this module:
- ld lq controls and simulation params info are available
"""
import sys

from PyQt6.QtWidgets import QApplication

from src.ui.main_window import BLDCMotorControlGUI


app = QApplication.instance() or QApplication(sys.argv)


def test_ld_lq_controls_and_simulation_params_info_are_available():
    win = BLDCMotorControlGUI()

    assert hasattr(win, "param_ld")
    assert hasattr(win, "param_lq")

    info = win.get_simulation_params_info()
    assert "dt" in info or "Simulation time step" in info






