"""
Widgets Package Module

This package contains accessible PyQt6 widgets for screen reader support.

:author: BLDC Control Team
:version: 1.0.0
"""

from .accessible_widgets import (
    AccessibleDoubleSpinBox,
    AccessibleSpinBox,
    AccessibleComboBox,
    AccessibleButton,
    AccessibleGroupBox,
    AccessibleTabWidget,
    LabeledSpinBox,
    LabeledComboBox,
)

__all__ = [
    "AccessibleDoubleSpinBox",
    "AccessibleSpinBox",
    "AccessibleComboBox",
    "AccessibleButton",
    "AccessibleGroupBox",
    "AccessibleTabWidget",
    "LabeledSpinBox",
    "LabeledComboBox",
]
