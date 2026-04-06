"""
Widgets Package Module

This package contains accessible PySide6 widgets for screen reader support.

:author: BLDC Control Team
:version: 0.8.0
"""

from .accessible_widgets import (
    AccessibleButton,
    AccessibleComboBox,
    AccessibleDoubleSpinBox,
    AccessibleGroupBox,
    AccessibleSpinBox,
    AccessibleTabWidget,
    LabeledComboBox,
    LabeledSpinBox,
)

__all__ = [
    "AccessibleButton",
    "AccessibleComboBox",
    "AccessibleDoubleSpinBox",
    "AccessibleGroupBox",
    "AccessibleSpinBox",
    "AccessibleTabWidget",
    "LabeledComboBox",
    "LabeledSpinBox",
]
