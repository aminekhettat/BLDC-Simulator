"""Tests for hardware backend integration in simulation engine."""

import numpy as np
import pytest

from src.core.load_model import ConstantLoad
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.simulation_engine import SimulationEngine
from src.hardware import HardwareInterface, MockDAQHardware


class FailingHardware(HardwareInterface):
    """Backend that fails on write to validate safe fallback behavior."""

    def __init__(self) -> None:
        super().__init__(name="failing-backend")
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def write_phase_voltages(self, voltages: np.ndarray, time_s: float) -> None:
        _ = voltages
        _ = time_s
        raise RuntimeError("synthetic write failure")

    def read_feedback(self, time_s: float) -> dict[str, float]:
        _ = time_s
        return {}


def _make_engine(hardware: HardwareInterface | None = None) -> SimulationEngine:
    motor = BLDCMotor(MotorParameters())
    load = ConstantLoad(torque=0.1)
    return SimulationEngine(motor, load, dt=0.0005, hardware_interface=hardware)


def test_mock_hardware_backend_wires_into_simulation_step():
    hardware = MockDAQHardware(noise_std=0.0, seed=7)
    engine = _make_engine(hardware)

    cmd = np.array([5.0, -2.5, -2.5], dtype=np.float64)
    engine.step(cmd, log_data=True)

    hist = engine.get_history()
    assert hist["voltages_a"][0] == pytest.approx(5.0)
    assert hist["voltages_b"][0] == pytest.approx(-2.5)
    assert hist["voltages_c"][0] == pytest.approx(-2.5)

    hw = engine.get_hardware_state()
    assert hw["enabled"] is True
    assert hw["connected"] is True
    assert hw["backend"] == "mock-daq"
    assert hw["write_count"] == 1
    assert hw["read_count"] == 1
    assert hw["last_io_error"] == ""


def test_hardware_io_failure_falls_back_to_simulation_path():
    engine = _make_engine(FailingHardware())
    engine.step(np.array([3.0, -1.5, -1.5]), log_data=True)

    hw = engine.get_hardware_state()
    assert hw["enabled"] is True
    assert hw["connected"] is False
    assert "synthetic write failure" in hw["last_io_error"]

    hist = engine.get_history()
    assert hist["time"].size == 1


def test_runtime_hardware_toggle_connects_and_disconnects_backend():
    hardware = MockDAQHardware(noise_std=0.0)
    engine = _make_engine(hardware)

    assert engine.get_hardware_state()["connected"] is True

    engine.configure_hardware_interface(False)
    assert engine.get_hardware_state()["enabled"] is False
    assert engine.get_hardware_state()["connected"] is False

    engine.configure_hardware_interface(True)
    assert engine.get_hardware_state()["enabled"] is True
    assert engine.get_hardware_state()["connected"] is True
