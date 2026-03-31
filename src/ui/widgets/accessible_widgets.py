"""
Accessible Widgets Module
==========================

Provides accessible PyQt6 widgets with proper ARIA labels and keyboard navigation.

All widgets support:
- Screen reader accessibility
- Tab key navigation
- Clear keyboard shortcuts
- Descriptive labels and help text

:author: BLDC Control Team
:version: 0.8.0
"""

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTabWidget,
    QWidget,
)


class AccessibleDoubleSpinBox(QDoubleSpinBox):
    """
    Accessible double precision spinner with screen reader support.

    Features:
    - Auto-generated ARIA labels
    - Clear status text for screen readers
    - Keyboard-friendly increment/decrement
    """

    def __init__(
        self,
        label: str = "",
        min_val: float = 0.0,
        max_val: float = 100.0,
        initial: float = 0.0,
        step: float = 0.1,
        decimals: int = 3,
        suffix: str = "",
    ):
        """
        Initialize accessible spinner.

        :param label: Descriptive label for screen readers
        :param min_val: Minimum value
        :param max_val: Maximum value
        :param initial: Initial value
        :param step: Step size for increment/decrement
        :param decimals: Decimal places
        :param suffix: Unit suffix (e.g., " V", " A")
        """
        super().__init__()

        self.label_text = label
        self.unit_suffix = suffix

        # Configure value range and formatting
        self.setMinimum(min_val)
        self.setMaximum(max_val)
        self.setValue(initial)
        self.setSingleStep(step)
        self.setDecimals(decimals)
        if suffix:
            self.setSuffix(suffix)

        # Accessibility
        self.setAccessibleName(label)
        help_text = f"{label}, range {min_val} to {max_val}{suffix}"
        self.setAccessibleDescription(help_text)

        # Keyboard
        self.valueChanged.connect(self._on_value_changed)

    def _on_value_changed(self):
        """Update status text for screen readers."""
        status = f"{self.label_text}: {self.value()}{self.unit_suffix}"
        self.setToolTip(status)


class AccessibleSpinBox(QSpinBox):
    """Accessible integer spinner for screen readers."""

    def __init__(
        self,
        label: str = "",
        min_val: int = 0,
        max_val: int = 100,
        initial: int = 0,
        step: int = 1,
        suffix: str = "",
    ):
        """
        Initialize accessible integer spinner.

        :param label: Descriptive label
        :param min_val: Minimum value
        :param max_val: Maximum value
        :param initial: Initial value
        :param step: Step size
        :param suffix: Unit suffix
        """
        super().__init__()

        self.label_text = label
        self.unit_suffix = suffix

        self.setMinimum(min_val)
        self.setMaximum(max_val)
        self.setValue(initial)
        self.setSingleStep(step)
        if suffix:
            self.setSuffix(suffix)

        self.setAccessibleName(label)
        help_text = f"{label}, range {min_val} to {max_val}{suffix}"
        self.setAccessibleDescription(help_text)

        self.valueChanged.connect(self._on_value_changed)

    def _on_value_changed(self):
        """Update status text for screen readers."""
        status = f"{self.label_text}: {self.value()}{self.unit_suffix}"
        self.setToolTip(status)


class AccessibleComboBox(QComboBox):
    """Accessible combo box with screen reader support."""

    def __init__(self, label: str = "", items: list = None):
        """
        Initialize accessible combo box.

        :param label: Field label
        :param items: List of items to add
        """
        super().__init__()

        self.label_text = label

        if items:
            self.addItems(items)

        self.setAccessibleName(label)
        self.setAccessibleDescription(f"Select {label.lower()}")

        self.currentTextChanged.connect(self._on_selection_changed)

    def _on_selection_changed(self):
        """Update status for screen readers."""
        status = f"{self.label_text}: {self.currentText()}"
        self.setToolTip(status)


class AccessibleButton(QPushButton):
    """Accessible button with clear labeling."""

    def __init__(self, text: str, tooltip: str = ""):
        """
        Initialize accessible button.

        :param text: Button text
        :param tooltip: Tooltip text (shown to screen readers)
        """
        super().__init__(text)

        self.setAccessibleName(text)
        if tooltip:
            self.setAccessibleDescription(tooltip)
            self.setToolTip(tooltip)

    def keyPressEvent(self, event):
        """Handle keyboard activation."""
        if event.key() in [Qt.Key.Key_Return, Qt.Key.Key_Space]:
            self.click()
        else:
            super().keyPressEvent(event)


class AccessibleGroupBox(QGroupBox):
    """Accessible group box for organizing related controls."""

    def __init__(self, title: str = "", description: str = ""):
        """
        Initialize accessible group box.

        :param title: Group title
        :param description: Description for screen readers
        """
        super().__init__(title)

        self.setAccessibleName(title)
        if description:
            self.setAccessibleDescription(description)

        # Keyboard navigation support
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)


class AccessibleTabWidget(QTabWidget):
    """Accessible tab widget with keyboard navigation."""

    def __init__(self):
        """Initialize accessible tab widget."""
        super().__init__()

        # Provide accessible names and descriptions
        self.setAccessibleName("Tab navigation")
        self.setAccessibleDescription(
            "Use Ctrl+Tab to switch tabs, or use arrow keys in focused tab bar"
        )


class LabeledSpinBox(QWidget):
    """
    Convenience widget combining a label and spinner.

    Simplifies accessibility by grouping related elements.
    """

    valueChanged = pyqtSignal(float)

    def __init__(
        self,
        label: str,
        min_val: float = 0.0,
        max_val: float = 100.0,
        initial: float = 0.0,
        step: float = 0.1,
        decimals: int = 3,
        suffix: str = "",
        description: str = "",
    ):
        """
        Initialize labeled spinner.

        :param label: Field label
        :param min_val: Minimum value
        :param max_val: Maximum value
        :param initial: Initial value
        :param step: Step size
        :param decimals: Decimal places
        :param suffix: Unit suffix
        :param description: Accessibility description
        """
        super().__init__()

        # Layout
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        # Label
        label_widget = QLabel(label)
        label_widget.setMinimumWidth(150)
        label_widget.setAccessibleName(label)
        layout.addWidget(label_widget)

        # Spinner
        self.spinner = AccessibleDoubleSpinBox(
            label, min_val, max_val, initial, step, decimals, suffix
        )
        if description:
            self.spinner.setAccessibleDescription(description)
        self.spinner.valueChanged.connect(self.valueChanged.emit)
        layout.addWidget(self.spinner, 1)

        self.setLayout(layout)

    def value(self) -> float:
        """Get current value."""
        return float(self.spinner.value())

    def setValue(self, val: float) -> None:
        """Set value."""
        self.spinner.setValue(val)


class LabeledComboBox(QWidget):
    """Convenience widget combining label and combo box."""

    currentTextChanged = pyqtSignal(str)

    def __init__(self, label: str, items: list = None, description: str = ""):
        """
        Initialize labeled combo box.

        :param label: Field label
        :param items: List of items
        :param description: Accessibility description
        """
        super().__init__()

        # Layout
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        # Label
        label_widget = QLabel(label)
        label_widget.setMinimumWidth(150)
        layout.addWidget(label_widget)

        # Combo box
        self.combo = AccessibleComboBox(label, items)
        if description:
            self.combo.setAccessibleDescription(description)
        self.combo.currentTextChanged.connect(self.currentTextChanged.emit)
        layout.addWidget(self.combo, 1)

        self.setLayout(layout)

    def currentText(self) -> str:
        """Get selected text."""
        return str(self.combo.currentText())

    def setCurrentText(self, text: str) -> None:
        """Set selected text."""
        self.combo.setCurrentText(text)


class AccessibleTableWidget(QTableWidget):
    """
    Accessible table widget with screen reader support and keyboard navigation.

    Features:
    - ARIA labels for screen readers
    - Arrow key navigation (Up, Down, Left, Right)
    - Space/Enter for selection
    - Tab key moves to next table
    - Announces current cell to screen readers
    """

    def __init__(self, label: str = "", description: str = ""):
        """
        Initialize accessible table widget.

        :param label: Accessible name for the table
        :param description: Description for screen readers
        """
        super().__init__()

        self.setAccessibleName(label if label else "Table")
        help_text = (
            description
            if description
            else "Use arrow keys to navigate, Space or Enter to select, Tab to move to next widget"
        )
        self.setAccessibleDescription(help_text)

        # Enable focus
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # Store current cell for accessibility feedback
        self.cellClicked.connect(self._on_cell_clicked)
        self.itemSelectionChanged.connect(self._on_selection_changed)

    def _on_cell_clicked(self, row: int, col: int):
        """Announce cell click to accessibility tools."""
        item = self.item(row, col)
        if item:
            cell_text = item.text()
            row_header = self.verticalHeaderItem(row)
            col_header = self.horizontalHeaderItem(col)

            info = f"Row {row + 1}"
            if row_header:
                info += f": {row_header.text()}"
            info += f", Column {col + 1}"
            if col_header:
                info += f": {col_header.text()}"
            info += f", Value: {cell_text}"

            self.setAccessibleDescription(info)

    def _on_selection_changed(self):
        """Announce selection changes."""
        selected_items = self.selectedItems()
        if selected_items:
            count = len(selected_items)
            plural = "item" if count == 1 else "items"
            info = f"{count} {plural} selected"
            self.setToolTip(info)

    def keyPressEvent(self, event):
        """Handle keyboard navigation."""
        if event.key() in [
            Qt.Key.Key_Up,
            Qt.Key.Key_Down,
            Qt.Key.Key_Left,
            Qt.Key.Key_Right,
        ]:
            # Allow arrow key navigation
            super().keyPressEvent(event)
        elif event.key() in [Qt.Key.Key_Return, Qt.Key.Key_Space]:
            # Select item with Space or Enter
            current_item = self.currentItem()
            if current_item:
                current_item.setSelected(not current_item.isSelected())
        else:
            super().keyPressEvent(event)


class AccessibleListWidget(QListWidget):
    """
    Accessible list widget with screen reader support and keyboard navigation.

    Features:
    - ARIA labels for screen readers
    - Arrow key navigation
    - Space/Enter for selection
    - Announces list position to screen readers
    """

    def __init__(self, label: str = "", description: str = ""):
        """
        Initialize accessible list widget.

        :param label: Accessible name for the list
        :param description: Description for screen readers
        """
        super().__init__()

        self.setAccessibleName(label if label else "List")
        help_text = (
            description if description else "Use arrow keys to navigate, Space or Enter to select"
        )
        self.setAccessibleDescription(help_text)

        # Enable focus
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # Track current item for accessibility
        self.itemSelectionChanged.connect(self._on_selection_changed)

    def _on_selection_changed(self):
        """Announce selection to accessibility tools."""
        current = self.currentItem()
        if current:
            row = self.row(current)
            total = self.count()
            info = f"Item {row + 1} of {total}: {current.text()}"
            self.setToolTip(info)

    def keyPressEvent(self, event):
        """Handle keyboard navigation."""
        if event.key() in [Qt.Key.Key_Up, Qt.Key.Key_Down]:
            # Allow arrow key navigation
            super().keyPressEvent(event)
        elif event.key() in [Qt.Key.Key_Return, Qt.Key.Key_Space]:
            # Toggle selection with Space or Enter
            current_item = self.currentItem()
            if current_item:
                current_item.setSelected(not current_item.isSelected())
        else:
            super().keyPressEvent(event)
