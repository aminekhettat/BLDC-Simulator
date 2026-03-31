"""
Atomic features tested in this module:
- accessible double spin box updates tooltip
- accessible spin box updates tooltip
- accessible combo box selection updates tooltip
- accessible button keyboard activation and fallback
- group and tab widget accessibility defaults
- labeled spin box signal value and setter
- labeled combo box signal and selection
- accessible table widget click selection and keyboard
- accessible table widget branch without headers
- accessible list widget selection and keyboard toggle
- accessible list widget selection changed without current item
"""

import sys

import pytest
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QKeyEvent
from PyQt6.QtWidgets import QApplication, QTableWidgetItem, QWidget

from src.ui.widgets.accessible_widgets import (
    AccessibleButton,
    AccessibleComboBox,
    AccessibleDoubleSpinBox,
    AccessibleGroupBox,
    AccessibleListWidget,
    AccessibleSpinBox,
    AccessibleTableWidget,
    AccessibleTabWidget,
    LabeledComboBox,
    LabeledSpinBox,
)

app = QApplication.instance() or QApplication(sys.argv)


def test_accessible_double_spin_box_updates_tooltip():
    w = AccessibleDoubleSpinBox(
        label="Voltage",
        min_val=0.0,
        max_val=100.0,
        initial=10.0,
        step=0.5,
        decimals=2,
        suffix=" V",
    )
    w.setValue(12.5)
    assert w.accessibleName() == "Voltage"
    assert "range 0.0 to 100.0 V" in w.accessibleDescription()
    assert "Voltage: 12.5 V" in w.toolTip()


def test_accessible_spin_box_updates_tooltip():
    w = AccessibleSpinBox(
        label="Poles",
        min_val=2,
        max_val=20,
        initial=8,
        step=2,
        suffix=" p",
    )
    w.setValue(10)
    assert w.accessibleName() == "Poles"
    assert "range 2 to 20 p" in w.accessibleDescription()
    assert "Poles: 10 p" in w.toolTip()


def test_accessible_combo_box_selection_updates_tooltip():
    w = AccessibleComboBox(label="Mode", items=["FOC", "V/f"])
    w.setCurrentText("V/f")
    assert w.accessibleName() == "Mode"
    assert w.accessibleDescription() == "Select mode"
    assert w.toolTip() == "Mode: V/f"


def test_accessible_button_keyboard_activation_and_fallback():
    w = AccessibleButton("Run", tooltip="Start simulation")
    hits = {"n": 0}
    w.clicked.connect(lambda: hits.__setitem__("n", hits["n"] + 1))

    enter_event = QKeyEvent(
        QKeyEvent.Type.KeyPress, Qt.Key.Key_Return, Qt.KeyboardModifier.NoModifier
    )
    space_event = QKeyEvent(
        QKeyEvent.Type.KeyPress, Qt.Key.Key_Space, Qt.KeyboardModifier.NoModifier
    )
    other_event = QKeyEvent(QKeyEvent.Type.KeyPress, Qt.Key.Key_A, Qt.KeyboardModifier.NoModifier)

    w.keyPressEvent(enter_event)
    w.keyPressEvent(space_event)
    w.keyPressEvent(other_event)

    assert hits["n"] == 2
    assert w.accessibleDescription() == "Start simulation"


def test_group_and_tab_widget_accessibility_defaults():
    g = AccessibleGroupBox("Motor", "Motor settings")
    t = AccessibleTabWidget()

    assert g.accessibleName() == "Motor"
    assert g.accessibleDescription() == "Motor settings"
    assert g.focusPolicy() == Qt.FocusPolicy.StrongFocus

    assert t.accessibleName() == "Tab navigation"
    assert "Ctrl+Tab" in t.accessibleDescription()


def test_labeled_spin_box_signal_value_and_setter():
    w = LabeledSpinBox("Current", min_val=0.0, max_val=20.0, initial=1.0, step=0.5)
    seen = {"v": None}
    w.valueChanged.connect(lambda v: seen.__setitem__("v", v))

    w.setValue(3.5)
    assert w.value() == pytest.approx(3.5)
    assert seen["v"] == pytest.approx(3.5)


def test_labeled_combo_box_signal_and_selection():
    w = LabeledComboBox("Controller", items=["FOC", "V/f"])
    seen = {"v": None}
    w.currentTextChanged.connect(lambda txt: seen.__setitem__("v", txt))

    w.setCurrentText("V/f")
    assert w.currentText() == "V/f"
    assert seen["v"] == "V/f"


def test_accessible_table_widget_click_selection_and_keyboard():
    table = AccessibleTableWidget(label="Results")
    table.setRowCount(1)
    table.setColumnCount(1)
    table.setItem(0, 0, QTableWidgetItem("42"))
    table.setSelectionMode(table.SelectionMode.MultiSelection)
    table.setHorizontalHeaderItem(0, QTableWidgetItem("Value"))
    table.setVerticalHeaderItem(0, QTableWidgetItem("RowA"))

    table._on_cell_clicked(0, 0)
    assert "Row 1: RowA, Column 1: Value, Value: 42" in table.accessibleDescription()

    item = table.item(0, 0)
    table.setCurrentItem(item)
    item.setSelected(False)
    space_event = QKeyEvent(
        QKeyEvent.Type.KeyPress, Qt.Key.Key_Space, Qt.KeyboardModifier.NoModifier
    )
    table.keyPressEvent(space_event)
    assert item.isSelected() is True

    table._on_selection_changed()
    assert "selected" in table.toolTip()

    arrow_event = QKeyEvent(
        QKeyEvent.Type.KeyPress, Qt.Key.Key_Right, Qt.KeyboardModifier.NoModifier
    )
    table.keyPressEvent(arrow_event)


def test_accessible_table_widget_branch_without_headers():
    table = AccessibleTableWidget()
    table.setRowCount(1)
    table.setColumnCount(1)
    table.setItem(0, 0, QTableWidgetItem("x"))
    table._on_cell_clicked(0, 0)
    assert "Row 1, Column 1, Value: x" in table.accessibleDescription()


def test_accessible_list_widget_selection_and_keyboard_toggle():
    lst = AccessibleListWidget(label="Profiles")
    lst.addItems(["A", "B"])

    lst.setCurrentRow(0)
    lst.setSelectionMode(lst.SelectionMode.MultiSelection)
    lst._on_selection_changed()
    assert "Item 1 of 2: A" in lst.toolTip()

    space_event = QKeyEvent(
        QKeyEvent.Type.KeyPress, Qt.Key.Key_Space, Qt.KeyboardModifier.NoModifier
    )
    lst.currentItem().setSelected(False)
    lst.keyPressEvent(space_event)
    assert lst.currentItem().isSelected() is True

    arrow_event = QKeyEvent(
        QKeyEvent.Type.KeyPress, Qt.Key.Key_Down, Qt.KeyboardModifier.NoModifier
    )
    lst.keyPressEvent(arrow_event)


def test_accessible_list_widget_selection_changed_without_current_item():
    lst = AccessibleListWidget()
    # Ensure method does not crash when no current item is selected.
    lst._on_selection_changed()
    assert isinstance(lst, QWidget)
