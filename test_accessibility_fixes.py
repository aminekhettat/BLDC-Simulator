#!/usr/bin/env python3
"""
Test script to verify accessibility and export fixes.
"""

import sys
from pathlib import Path


def test_imports():
    """Test that all modules import correctly."""
    print("🔍 Testing imports...")
    try:
        # importing symbols only to verify modules load; they are intentionally unused
        from src.ui.widgets.accessible_widgets import (
            AccessibleTableWidget,  # noqa: F401
            AccessibleListWidget,  # noqa: F401
        )

        print("✅ AccessibleTableWidget and AccessibleListWidget imported successfully")

        from src.utils.data_logger import DataLogger  # noqa: F401

        print("✅ DataLogger imported successfully")

        from src.ui.main_window import BLDCMotorControlGUI  # noqa: F401

        print("✅ BLDCMotorControlGUI imported successfully")

        return True
    except Exception as e:
        print(f"❌ Import error: {e}")
        return False


def test_data_logger():
    """Test DataLogger with custom paths."""
    print("\n🔍 Testing DataLogger custom path functionality...")
    try:
        import tempfile
        import numpy as np
        from src.utils.data_logger import DataLogger

        # Create temporary directory
        with tempfile.TemporaryDirectory() as tmpdir:
            logger = DataLogger()

            # Create test data
            test_history = {
                "time": np.array([0.0, 0.1, 0.2]),
                "speed": np.array([0.0, 100.0, 200.0]),
                "currents_a": np.array([0.0, 1.0, 2.0]),
                "currents_b": np.array([0.0, 1.5, 2.5]),
                "currents_c": np.array([0.0, 2.0, 3.0]),
                "omega": np.array([0.0, 10.0, 20.0]),
                "theta": np.array([0.0, 0.1, 0.2]),
                "emf_a": np.array([0.0, 1.0, 2.0]),
                "emf_b": np.array([0.0, 1.0, 2.0]),
                "emf_c": np.array([0.0, 1.0, 2.0]),
                "torque": np.array([0.0, 1.0, 2.0]),
                "load_torque": np.array([0.0, 0.5, 1.0]),
                "voltages_a": np.array([0.0, 10.0, 20.0]),
                "voltages_b": np.array([0.0, 10.0, 20.0]),
                "voltages_c": np.array([0.0, 10.0, 20.0]),
            }

            # Test custom path save
            custom_path = Path(tmpdir) / "my_export.csv"
            result = logger.save_simulation_data(
                test_history,
                metadata={"test": "data"},
                filename=str(custom_path),
                use_custom_path=True,
            )

            # Verify files were created
            assert result.exists(), f"CSV file not created at {result}"
            assert result.name == "my_export.csv", f"Unexpected filename: {result.name}"
            print(f"✅ CSV file created at: {result}")

            # Check metadata file
            metadata_path = custom_path.parent / "my_export_metadata.json"
            assert metadata_path.exists(), (
                f"Metadata file not created at {metadata_path}"
            )
            print(f"✅ Metadata file created at: {metadata_path}")

            # Verify content
            with open(result, "r") as f:
                lines = f.readlines()
                assert len(lines) > 1, "CSV file appears to be empty"
                print(
                    f"✅ CSV contains {len(lines)} lines (1 header + {len(lines) - 1} data)"
                )

        return True
    except Exception as e:
        print(f"❌ DataLogger test error: {e}")
        import traceback

        traceback.print_exc()
        return False


def test_accessible_widgets():
    """Test AccessibleTableWidget and AccessibleListWidget."""
    print("\n🔍 Testing AccessibleTableWidget and AccessibleListWidget...")
    try:
        from PyQt6.QtWidgets import QApplication
        from src.ui.widgets.accessible_widgets import (
            AccessibleTableWidget,  # noqa: F401
            AccessibleListWidget,  # noqa: F401
        )

        # Create QApplication for widget testing
        # variable intentionally unused
        _ = QApplication.instance() or QApplication([])

        # Test AccessibleTableWidget
        table = AccessibleTableWidget("Test Table", "A test table widget")
        assert table.accessibleName() == "Test Table"
        print("✅ AccessibleTableWidget created with correct accessible name")

        # Test AccessibleListWidget
        list_widget = AccessibleListWidget("Test List", "A test list widget")
        assert list_widget.accessibleName() == "Test List"
        print("✅ AccessibleListWidget created with correct accessible name")

        return True
    except Exception as e:
        print(f"❌ AccessibleWidgets test error: {e}")
        import traceback

        traceback.print_exc()
        return False


def test_ld_lq_and_params_display():
    """Test that Ld/Lq UI elements exist and simulation params display works."""
    print("\n🔍 Testing Ld/Lq and simulation params display...")
    try:
        from src.ui.main_window import BLDCMotorControlGUI
        from PyQt6.QtWidgets import QApplication

        _ = QApplication.instance() or QApplication([])
        win = BLDCMotorControlGUI()

        # Check Ld/Lq widgets
        assert hasattr(win, "param_ld") and hasattr(win, "param_lq"), (
            "Ld/Lq controls missing"
        )
        print("✅ Ld and Lq controls present")

        # Get simulation params info string (non-blocking)
        info = win.get_simulation_params_info()
        assert "dt" in info or "Simulation time step" in info
        print("✅ Simulation parameters info available")

        return True
    except Exception as e:
        print(f"❌ Ld/Lq test error: {e}")
        import traceback

        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("=" * 60)
    print("🧪 Accessibility and Export Fixes Test Suite")
    print("=" * 60)

    results = []

    # Run tests
    results.append(("Imports", test_imports()))
    results.append(("DataLogger Custom Paths", test_data_logger()))
    results.append(("Accessible Widgets", test_accessible_widgets()))
    results.append(("Ld/Lq and Params Display", test_ld_lq_and_params_display()))

    # Summary
    print("\n" + "=" * 60)
    print("📊 Test Summary")
    print("=" * 60)
    passed = sum(1 for _, result in results if result)
    total = len(results)
    for name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {name}")

    print(f"\n{'=' * 60}")
    print(f"Result: {passed}/{total} tests passed")
    print("=" * 60)

    return all(result for _, result in results)


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
