"""Hardware integration backends and interfaces."""

from .hardware_interface import HardwareInterface, MockDAQHardware

__all__ = ["HardwareInterface", "MockDAQHardware"]
