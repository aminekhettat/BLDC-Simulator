"""
Atomic features tested in this module:
- accessibility related modules import
- accessibility widget instances have accessible names
"""

import sys

from PyQt6.QtWidgets import QApplication


def test_accessibility_related_modules_import():
    """Keep a lightweight integration smoke-test for accessibility wiring."""
    from src.ui.main_window import BLDCMotorControlGUI
    from src.ui.widgets.accessible_widgets import (
        AccessibleListWidget,
        AccessibleTableWidget,
    )

    assert AccessibleTableWidget is not None
    assert AccessibleListWidget is not None
    assert BLDCMotorControlGUI is not None


def test_accessibility_widget_instances_have_accessible_names():
    """Validate basic runtime construction under a real QApplication."""
    _ = QApplication.instance() or QApplication(sys.argv)

    from src.ui.widgets.accessible_widgets import (
        AccessibleListWidget,
        AccessibleTableWidget,
    )

    table = AccessibleTableWidget("Test Table", "A test table widget")
    list_widget = AccessibleListWidget("Test List", "A test list widget")

    assert table.accessibleName() == "Test Table"
    assert list_widget.accessibleName() == "Test List"
