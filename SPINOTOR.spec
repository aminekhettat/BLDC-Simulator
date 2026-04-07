# -*- mode: python ; coding: utf-8 -*-
# =============================================================================
# SPINOTOR — PyInstaller build specification
# Generated for: Windows x64 (one-dir mode with single-file launcher)
# Run from project root:  pyinstaller SPINOTOR.spec
# =============================================================================

import sys
from pathlib import Path

ROOT = Path(SPECPATH)          # project root (where this .spec lives)
SRC  = ROOT / "src"
DATA = ROOT / "data"

# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
a = Analysis(
    [str(ROOT / "main.py")],
    pathex=[str(ROOT)],
    binaries=[],
    datas=[
        # Motor profile JSON presets shipped with the app
        (str(DATA / "motor_profiles"),      "data/motor_profiles"),
        # Icon used at runtime (splash, about dialog, etc.)
        (str(ROOT / "spinotor.ico"),        "."),
        # LOGO folder (SVG assets used by the UI at runtime if any)
        (str(ROOT / "LOGO" / "spinotor.ico"), "LOGO"),
    ],
    hiddenimports=[
        # --- PySide6 / Qt plug-ins often missed by the hook ---
        "PySide6.QtCore",
        "PySide6.QtGui",
        "PySide6.QtWidgets",
        "PySide6.QtPrintSupport",
        "PySide6.QtSvg",
        # --- matplotlib backends ---
        "matplotlib.backends.backend_qt5agg",
        "matplotlib.backends.backend_agg",
        "matplotlib.backends.backend_svg",
        # --- pyqtgraph ---
        "pyqtgraph",
        "pyqtgraph.graphicsItems",
        # --- numba (JIT — must be collected fully) ---
        "numba",
        "numba.core",
        "numba.typed",
        "numba.np.ufunc",
        # --- pyttsx3 TTS drivers ---
        "pyttsx3",
        "pyttsx3.drivers",
        "pyttsx3.drivers.sapi5",    # Windows SAPI5 driver
        "pyttsx3.drivers.nsss",     # macOS driver (ignored on Windows)
        "pyttsx3.drivers.espeak",   # Linux driver (ignored on Windows)
        # --- project packages ---
        "src",
        "src.control",
        "src.control.adaptive_tuning",
        "src.control.base_controller",
        "src.control.field_weakening_calibrator",
        "src.control.foc_controller",
        "src.control.svm_generator",
        "src.control.transforms",
        "src.control.vf_controller",
        "src.core",
        "src.core.load_model",
        "src.core.motor_model",
        "src.core.power_model",
        "src.core.simulation_engine",
        "src.hardware",
        "src.hardware.current_sensor",
        "src.hardware.hardware_interface",
        "src.hardware.inverter_current_sense",
        "src.ui",
        "src.ui.main_window",
        "src.ui.widgets",
        "src.ui.widgets.accessible_widgets",
        "src.ui.widgets.oscilloscope_widget",
        "src.ui.widgets.plot_customizer_dialog",
        "src.utils",
        "src.utils.compute_backend",
        "src.utils.config",
        "src.utils.data_logger",
        "src.utils.motor_profiles",
        "src.utils.regression_baseline",
        "src.utils.speech",
        "src.visualization",
        "src.visualization.visualization",
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        # Development / test tooling — not needed at runtime
        "pytest",
        "mypy",
        "ruff",
        "bandit",
        "IPython",
        "jupyter",
        "nbformat",
        "sphinx",
        "docutils",
        "bump2version",
    ],
    noarchive=False,
    optimize=1,
)

# ---------------------------------------------------------------------------
# PYZ archive
# ---------------------------------------------------------------------------
pyz = PYZ(a.pure)

# ---------------------------------------------------------------------------
# EXE  (windowed = no console window on Windows)
# ---------------------------------------------------------------------------
exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,      # binaries go into COLLECT (one-dir build)
    name="SPINOTOR",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,                   # compress with UPX if available
    console=False,              # GUI app — suppress the black console window
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=str(ROOT / "spinotor.ico"),
    version_file=None,
)

# ---------------------------------------------------------------------------
# COLLECT  — one-dir distribution folder: dist/SPINOTOR/
# ---------------------------------------------------------------------------
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="SPINOTOR",
)
