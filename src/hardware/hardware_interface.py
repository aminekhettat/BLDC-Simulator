"""Hardware interface abstractions for Phase-4 integration.

This module defines a minimal, backend-agnostic contract for hardware I/O and
ships a mock DAQ backend that can be used for dry-run testing.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Optional

import numpy as np


class HardwareInterface(ABC):
    """Abstract interface for real hardware communication backends."""

    def __init__(self, name: str) -> None:
        self.name = name

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Return True when the backend is connected and operational."""

    @abstractmethod
    def connect(self) -> None:
        """Open backend resources (drivers, sockets, DAQ channels, etc.)."""

    @abstractmethod
    def disconnect(self) -> None:
        """Close backend resources and leave a safe disconnected state."""

    @abstractmethod
    def write_phase_voltages(self, voltages: np.ndarray, time_s: float) -> None:
        """Send a 3-phase voltage command to hardware for current control tick."""

    @abstractmethod
    def read_feedback(self, time_s: float) -> dict[str, Any]:
        """Read telemetry from hardware for current control tick."""


class MockDAQHardware(HardwareInterface):
    """Mock hardware backend for integration testing and dry runs.

    The backend echoes back the latest commanded voltages (optionally with
    gaussian noise) so that the simulation engine can exercise hardware paths
    without requiring physical devices.
    """

    def __init__(self, noise_std: float = 0.0, seed: Optional[int] = None) -> None:
        super().__init__(name="mock-daq")
        self.noise_std = max(float(noise_std), 0.0)
        self._rng = np.random.default_rng(seed)
        self._connected = False
        self._last_command = np.zeros(3, dtype=np.float64)

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def write_phase_voltages(self, voltages: np.ndarray, time_s: float) -> None:
        _ = time_s
        if not self._connected:
            raise RuntimeError("MockDAQHardware is not connected")
        cmd = np.asarray(voltages, dtype=np.float64)
        if cmd.shape != (3,):
            raise ValueError("voltages must be a length-3 array")
        self._last_command = cmd.copy()

    def read_feedback(self, time_s: float) -> dict[str, Any]:
        if not self._connected:
            raise RuntimeError("MockDAQHardware is not connected")

        applied = self._last_command.copy()
        if self.noise_std > 0.0:
            applied = applied + self._rng.normal(0.0, self.noise_std, size=3)

        return {
            "timestamp_s": float(time_s),
            "applied_voltages": applied,
            "backend": self.name,
        }
