"""
Main Source Package Module

This is the main source code package for BLDC motor control simulator.

Includes:
- core: Motor model, simulation engine, load profiles
- control: Control blocks (SVM, V/f, FOC ready)
- ui: GUI application with accessibility support
- utils: Configuration and utilities
- visualization: Plotting and data export

:author: BLDC Control Team
:version: 1.0.0
"""

from . import core
from . import control
from . import ui
from . import utils
from . import visualization

__all__ = [
    "core",
    "control",
    "ui",
    "utils",
    "visualization",
]
