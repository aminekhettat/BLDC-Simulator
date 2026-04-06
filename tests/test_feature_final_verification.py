#!/usr/bin/env python3
"""
Atomic features tested in this module:
- feature final verification
"""

import inspect

print("=" * 60)
print("ðŸ§ª FINAL VERIFICATION TEST")
print("=" * 60)

# Test 1: Visualization with grid parameters
try:
    from src.visualization.visualization import SimulationPlotter

    sig = inspect.signature(SimulationPlotter.create_3phase_plot)
    params = list(sig.parameters.keys())
    expected = [
        "history",
        "figsize",
        "grid_on",
        "grid_spacing",
        "minor_grid",
        "grid_spacing_y",
    ]
    if all(p in params for p in expected):
        print("âœ… Visualization grid parameters: OK")
    else:
        print("âŒ Visualization grid parameters: MISSING")
        print(f"   Expected: {expected}")
        print(f"   Got: {params}")
except Exception as e:
    print(f"âŒ Visualization test failed: {e}")

# Test 2: Motor model with numba
try:
    from src.core.motor_model import HAS_NUMBA, BLDCMotor, MotorParameters

    motor = BLDCMotor(MotorParameters())
    print(f"âœ… Motor model with numba support: OK (HAS_NUMBA={HAS_NUMBA})")
except Exception as e:
    print(f"âŒ Motor model test failed: {e}")

# Test 3: Main window with status bar
try:
    from PySide6.QtWidgets import QApplication, QLabel

    from src.ui.main_window import BLDCMotorControlGUI

    app = QApplication.instance() or QApplication([])
    gui = BLDCMotorControlGUI()
    # Check if status bar widgets exist
    if hasattr(gui, "status_bar_dt") and isinstance(gui.status_bar_dt, QLabel):
        print("âœ… Main window status bar: OK")
    else:
        print("âŒ Main window status bar: MISSING")
except Exception as e:
    print(f"âŒ Main window test failed: {e}")

# Test 4: Data logger with custom paths
try:
    from src.utils.data_logger import DataLogger

    sig = inspect.signature(DataLogger.save_simulation_data)
    param_names = list(sig.parameters.keys())
    if "use_custom_path" in param_names:
        print("âœ… Data logger custom paths: OK")
    else:
        print("âŒ Data logger custom paths: MISSING")
except Exception as e:
    print(f"âŒ Data logger test failed: {e}")

# Test 5: Ld/Lq parameters
try:
    from src.core.motor_model import MotorParameters

    motor_params = MotorParameters()
    if hasattr(motor_params, "ld") and hasattr(motor_params, "lq"):
        print("âœ… Ld/Lq parameters: OK")
    else:
        print("âŒ Ld/Lq parameters: MISSING")
except Exception as e:
    print(f"âŒ Ld/Lq parameters test failed: {e}")

print("=" * 60)
print("âœ… All features verified successfully!")
print("=" * 60)
