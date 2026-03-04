"""Tests for power supply profile and FOC controller behavior.

Ensures that dynamic supply voltages propagate through the engine and that
FOC controller returns correctly formatted outputs.
"""

import numpy as np
import pytest
from PyQt6.QtWidgets import QApplication
import sys

# ensure QApplication instance for GUI tests
app = QApplication.instance() or QApplication(sys.argv)

from src.core.power_model import SupplyProfile, ConstantSupply
from src.core.simulation_engine import SimulationEngine
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.load_model import ConstantLoad
from src.control import FOCController, SVMGenerator, CartesianSVMGenerator


def test_supply_profile_basic():
    # constant profile should always return same voltage
    p = ConstantSupply(24.0)
    assert p.get_voltage(0) == pytest.approx(24.0)
    assert p.get_voltage(10.5) == pytest.approx(24.0)


def test_engine_applies_supply_voltage():
    # supply should influence what the engine logs
    motor = BLDCMotor(MotorParameters())
    load = ConstantLoad(0.0)
    supply = ConstantSupply(12.0)
    engine = SimulationEngine(motor, load, dt=0.001, supply_profile=supply)

    # before stepping, history should be empty
    hist = engine.get_history()
    assert hist["supply_voltage"].size == 0
    engine.step(np.zeros(3), log_data=True)
    hist = engine.get_history()
    assert hist["supply_voltage"][0] == pytest.approx(12.0)


def test_foc_controller_polar_output():
    motor = BLDCMotor(MotorParameters())
    ctrl = FOCController(motor=motor)
    # default output_cartesian should be False
    assert not ctrl.output_cartesian
    # without setting references, update returns numeric tuple (mag, angle)
    out = ctrl.update(0.001)
    assert isinstance(out, tuple) and len(out) == 2
    mag, ang = out
    assert isinstance(mag, float)
    assert isinstance(ang, float)


def test_foc_controller_cartesian_output():
    motor = BLDCMotor(MotorParameters())
    ctrl = FOCController(motor=motor)
    ctrl.output_cartesian = True
    ctrl.set_current_references(id_ref=1.0, iq_ref=0.5)
    out = ctrl.update(0.001)
    assert isinstance(out, tuple) and len(out) == 2
    valpha, vbeta = out
    assert isinstance(valpha, float)
    assert isinstance(vbeta, float)


def test_foc_auto_tune_changes_params():
    motor = BLDCMotor(MotorParameters())
    ctrl = FOCController(motor=motor)
    # record original gains
    orig_kp = ctrl.pi_q["kp"]
    ctrl.auto_tune_pi(axis="q", bandwidth=100.0)
    assert ctrl.pi_q["kp"] != orig_kp
    assert ctrl.pi_q["ki"] != 0


def test_svm_cartesian_conversion():
    svm = CartesianSVMGenerator(dc_voltage=48.0)
    # test simple vector along alpha axis
    valpha, vbeta = 10.0, 0.0
    volts = svm.modulate_cartesian(valpha=valpha, vbeta=vbeta)
    assert volts.shape == (3,)
    # confirm that cartesian modulate is equivalent to converting to polar
    magnitude = np.hypot(valpha, vbeta)
    angle = np.arctan2(vbeta, valpha)
    expected = svm.modulate(magnitude, angle)
    assert np.allclose(volts, expected, atol=1e-6)


def test_gui_supply_profile():
    # verify that the GUI applies correct supply profile settings
    from src.ui.main_window import BLDCMotorControlGUI
    from src.core.power_model import RampSupply, ConstantSupply

    gui = BLDCMotorControlGUI()
    gui.supply_type.setCurrentText("Ramp")
    gui.supply_ramp_initial.setValue(10.0)
    gui.supply_ramp_final.setValue(20.0)
    gui.supply_ramp_duration.setValue(5.0)
    gui._apply_to_simulation()
    assert isinstance(gui.engine.supply_profile, RampSupply)
    assert gui.engine.supply_profile.get_voltage(2.5) == pytest.approx(15.0)

    gui.supply_type.setCurrentText("Constant")
    gui.supply_constant_voltage.setValue(33.0)
    gui._apply_to_simulation()
    assert isinstance(gui.engine.supply_profile, ConstantSupply)
    assert gui.engine.supply_profile.get_voltage(0) == pytest.approx(33.0)
