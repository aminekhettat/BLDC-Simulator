"""
UI Package Module

This package contains all GUI components for the BLDC motor control application.

:author: BLDC Control Team
:version: 1.0.0
"""

__all__ = ["BLDCMotorControlGUI"]


def __getattr__(name: str):
    """Lazily import UI symbols to avoid import-time side effects in tests."""
    if name == "BLDCMotorControlGUI":
        from .main_window import BLDCMotorControlGUI

        return BLDCMotorControlGUI
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
