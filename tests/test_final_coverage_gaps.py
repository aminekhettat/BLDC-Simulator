"""Tests for final coverage gaps: visualization and src/__init__."""

import pytest
import matplotlib
import builtins

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from unittest.mock import Mock, patch, MagicMock
from pathlib import Path
import importlib.util

from src.visualization.visualization import SimulationPlotter
import src
from src.control.transforms import concordia_transform, inverse_concordia
from src.hardware.hardware_interface import MockDAQHardware
from src.utils import compute_backend, motor_profiles
from src.control.vf_controller import VFController
from src.utils import regression_baseline
from src.control.svm_generator import SVMGenerator, CartesianSVMGenerator
from src.control.foc_controller import (
    FOCController,
    _wrap_angle,
    _blend_angles,
    _pi_update,
    _pi_update_anti_windup,
)
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.simulation_engine import SimulationEngine
from src.core.load_model import ConstantLoad
from src.core.power_model import ConstantSupply, PowerFactorController

# PyQt6 imports only if needed (may not be available in all test environments)
try:
    from PyQt6.QtCore import Qt, QEvent
    from PyQt6.QtWidgets import QApplication
    from PyQt6.QtGui import QKeyEvent

    PYQT6_AVAILABLE = True

    from src.ui.widgets.accessible_widgets import (
        AccessibleListWidget,
        AccessibleTableWidget,
    )
except ImportError:
    PYQT6_AVAILABLE = False


# ============ Test accessible_widgets.py missing lines ============


@pytest.fixture
def qapp():
    """Provide a QApplication instance."""
    if not PYQT6_AVAILABLE:
        pytest.skip("PyQt6 not available")
    try:
        app = QApplication.instance()
        if app is None:
            app = QApplication([])
        return app
    except:
        pytest.skip("Qt application not available")


@pytest.mark.skipif(not PYQT6_AVAILABLE, reason="PyQt6 not available")
def test_accessible_list_widget_keyboard_event_non_arrow_non_select(qapp):
    """
    Test AccessibleListWidget handles non-arrow, non-select keys.
    This covers the else branch in keyPressEvent (untested path).
    """
    widget = AccessibleListWidget()
    widget.addItem("Item 1")
    widget.addItem("Item 2")

    # Simulate a non-special key (e.g., 'A')
    event = QKeyEvent(
        QEvent.Type.KeyPress, Qt.Key.Key_A, Qt.KeyboardModifier.NoModifier, text="a"
    )

    # Should call super().keyPressEvent(event) without error
    result = widget.keyPressEvent(event)

    # The call should succeed (returns None)
    assert result is None


@pytest.mark.skipif(not PYQT6_AVAILABLE, reason="PyQt6 not available")
def test_accessible_table_widget_keyboard_event_non_arrow_non_select(qapp):
    """
    Test AccessibleTableWidget handles non-arrow, non-select keys.
    This covers the else branch in keyPressEvent (untested path).
    """
    widget = AccessibleTableWidget()
    widget.setColumnCount(2)
    widget.setRowCount(2)

    # Simulate a non-special key (e.g., 'F')
    event = QKeyEvent(
        QEvent.Type.KeyPress, Qt.Key.Key_F, Qt.KeyboardModifier.NoModifier, text="f"
    )

    # Should call super().keyPressEvent(event) without error
    result = widget.keyPressEvent(event)

    # The call should succeed (returns None)
    assert result is None


# ============ Test visualization.py missing lines ============


def test_3phase_plot_with_minor_grid_and_grid_spacing_y():
    """
    Test 3-phase plot with minor_grid=True and grid_spacing_y set.
    This covers lines 320-321, 323, 325 (minor_grid and grid_spacing_y branches).
    """
    time = np.linspace(0, 1, 100)
    history = {
        "time": time,
        "currents_a": 10 * np.sin(2 * np.pi * 50 * time),
        "currents_b": 10 * np.sin(2 * np.pi * 50 * time - 2.094),
        "currents_c": 10 * np.sin(2 * np.pi * 50 * time - 4.189),
        "voltages_a": 230 * np.sin(2 * np.pi * 50 * time),
        "voltages_b": 230 * np.sin(2 * np.pi * 50 * time - 2.094),
        "voltages_c": 230 * np.sin(2 * np.pi * 50 * time - 4.189),
        "emf_a": 200 * np.sin(2 * np.pi * 50 * time),
        "emf_b": 200 * np.sin(2 * np.pi * 50 * time - 2.094),
        "emf_c": 200 * np.sin(2 * np.pi * 50 * time - 4.189),
        "speed": 1000 * np.ones(100),
        "torque": 5 * np.ones(100),
        "load_torque": 3 * np.ones(100),
        "rotation_speed_rpm": 1000 * np.ones(100),
        "motor_torque": 5 * np.ones(100),
    }

    fig = SimulationPlotter.create_3phase_plot(
        history=history,
        minor_grid=True,  # Enable minor grid (lines 320-321)
        grid_spacing_y=50,  # Set Y-axis grid spacing (lines 323, 325)
    )

    # Verify the plot was created
    assert fig is not None

    plt.close(fig)


def test_3phase_plot_without_optional_grid_params():
    """
    Test 3-phase plot without minor_grid or grid_spacing_y.
    Baseline test for comparison.
    """
    time = np.linspace(0, 1, 100)
    history = {
        "time": time,
        "currents_a": 10 * np.sin(2 * np.pi * 50 * time),
        "currents_b": 10 * np.sin(2 * np.pi * 50 * time - 2.094),
        "currents_c": 10 * np.sin(2 * np.pi * 50 * time - 4.189),
        "voltages_a": 230 * np.sin(2 * np.pi * 50 * time),
        "voltages_b": 230 * np.sin(2 * np.pi * 50 * time - 2.094),
        "voltages_c": 230 * np.sin(2 * np.pi * 50 * time - 4.189),
        "emf_a": 200 * np.sin(2 * np.pi * 50 * time),
        "emf_b": 200 * np.sin(2 * np.pi * 50 * time - 2.094),
        "emf_c": 200 * np.sin(2 * np.pi * 50 * time - 4.189),
        "speed": 1000 * np.ones(100),
        "torque": 5 * np.ones(100),
        "load_torque": 3 * np.ones(100),
        "rotation_speed_rpm": 1000 * np.ones(100),
        "motor_torque": 5 * np.ones(100),
    }

    fig = SimulationPlotter.create_3phase_plot(
        history=history,
        minor_grid=False,
        grid_spacing_y=None,
    )

    assert fig is not None
    plt.close(fig)


# ============ Test visualization.py missing lines ============ (placeholder for future)
# The remaining missing lines (320-321, 323, 325, 390-391, 393, 395, 482-483, 485, 487)
# are in specialized plotting methods. Will be covered when those methods are called in
# actual simulations or additional plot generation tests.


# ============ Test src/__init__.py missing lines ============


def test_src_init_attribute_error_path():
    """
    Test the AttributeError path in src.__getattr__.
    This covers lines 32-34 of src/__init__.py (the error case).
    """
    # Attempt to access a truly non-existent submodule/attribute
    with pytest.raises(AttributeError) as exc_info:
        _ = src.nonexistent_module_xyz_abc_123

    # Verify the error message contains the module name
    error_msg = str(exc_info.value)
    assert "nonexistent_module_xyz_abc_123" in error_msg or "module" in error_msg


def test_src_init_valid_imports():
    """
    Test that valid submodule imports work through __getattr__.
    """
    # These should not raise
    _ = src.core
    _ = src.control
    _ = src.utils
    _ = src.ui
    _ = src.visualization
    _ = src.hardware

    # Verify they are modules
    assert hasattr(src.core, "__name__")
    assert hasattr(src.control, "__name__")


# ============ Quick coverage wins across utility/control modules ============


def test_inverse_concordia_round_trip_matches_original_abc():
    va, vb, vc = 11.2, -3.7, -7.5
    alpha, beta = concordia_transform(va, vb, vc)

    va2, vb2, vc2 = inverse_concordia(alpha, beta)

    assert va2 == pytest.approx(va)
    assert vb2 == pytest.approx(vb)
    assert vc2 == pytest.approx(vc)


def test_mockdaq_errors_when_not_connected_and_on_bad_shape():
    hw = MockDAQHardware(noise_std=0.0)

    with pytest.raises(RuntimeError):
        hw.write_phase_voltages(np.array([1.0, 2.0, 3.0]), 0.0)
    with pytest.raises(RuntimeError):
        hw.read_feedback(0.0)

    hw.connect()
    with pytest.raises(ValueError):
        hw.write_phase_voltages(np.array([1.0, 2.0]), 0.0)


def test_motor_profiles_rejects_odd_num_poles():
    with pytest.raises(ValueError, match="num_poles must be even"):
        motor_profiles._normalize_motor_params({"num_poles": 7})


def test_compute_backend_windows_cuda_home_and_auto_gpu(monkeypatch, tmp_path):
    cuda_root = tmp_path / "cuda_root"
    (cuda_root / "bin").mkdir(parents=True)
    (cuda_root / "include").mkdir(parents=True)
    (cuda_root / "include" / "cuda.h").write_text("// stub", encoding="utf-8")

    monkeypatch.setattr(compute_backend.platform, "system", lambda: "Windows")
    monkeypatch.delenv("CUDA_PATH", raising=False)
    monkeypatch.setenv("CUDA_HOME", str(cuda_root))
    monkeypatch.setenv("PATH", "")

    toolkit_root = tmp_path / "toolkit_missing"
    original_path = Path

    def fake_path(value=""):
        s = str(value)
        if s == "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA":
            return toolkit_root
        return original_path(value)

    monkeypatch.setattr(compute_backend, "Path", fake_path)
    compute_backend._ensure_cuda_path_windows()
    assert compute_backend.os.environ["CUDA_PATH"] == str(cuda_root)
    assert str(cuda_root / "bin") in compute_backend.os.environ["PATH"]

    monkeypatch.setattr(
        compute_backend, "_probe_cupy", lambda: (True, "cupy_cuda_devices=1")
    )
    state = compute_backend.resolve_compute_backend("auto")
    assert state.selected == "gpu"
    assert state.gpu_available is True


def test_compute_backend_non_windows_is_noop(monkeypatch):
    monkeypatch.setattr(compute_backend.platform, "system", lambda: "Linux")
    monkeypatch.delenv("CUDA_PATH", raising=False)
    compute_backend._ensure_cuda_path_windows()
    assert "CUDA_PATH" not in compute_backend.os.environ


def test_vf_controller_validation_and_state_branches():
    with pytest.raises(ValueError, match="Nominal frequency must be positive"):
        VFController(v_nominal=48.0, f_nominal=0.0)
    with pytest.raises(ValueError, match="Nominal voltage must be positive"):
        VFController(v_nominal=0.0, f_nominal=50.0)

    ctrl = VFController(v_nominal=48.0, f_nominal=100.0, v_startup=1.0)

    with pytest.raises(ValueError, match="align_duration_s must be non-negative"):
        ctrl.set_startup_sequence(enable=True, align_duration_s=-0.1)
    with pytest.raises(ValueError, match="align_voltage_v must be non-negative"):
        ctrl.set_startup_sequence(enable=True, align_voltage_v=-0.1)
    with pytest.raises(
        ValueError, match="ramp_initial_frequency_hz must be non-negative"
    ):
        ctrl.set_startup_sequence(enable=True, ramp_initial_frequency_hz=-0.1)

    with pytest.raises(ValueError, match="Nominal frequency must be positive"):
        ctrl.set_vf_characteristic(v_startup=1.0, v_nominal=48.0, f_nominal=0.0)
    with pytest.raises(ValueError, match="Slew rate must be non-negative"):
        ctrl.set_frequency_slew_rate(-1.0)
    with pytest.raises(ValueError, match="Ramp rate must be non-negative"):
        ctrl.set_voltage_ramp_rate(-1.0)

    ctrl.set_speed_reference(-5.0)
    ctrl.set_startup_sequence(
        enable=True,
        align_duration_s=0.0,
        ramp_initial_frequency_hz=2.0,
    )
    # open-loop branch uses sign of frequency_ref
    ctrl._enter_startup_phase("open_loop")
    assert ctrl.frequency_actual < 0.0

    # unknown phase should map to "run"
    ctrl._enter_startup_phase("anything_else")
    assert ctrl.startup_phase == "run"

    ctrl.enable_startup_boost(enable=False, duration=0.2)
    assert ctrl.use_startup_boost is False
    assert ctrl.startup_boost_time == pytest.approx(0.2)

    ctrl.frequency_ref = 8.0
    ctrl.frequency_actual = 4.0
    ctrl.voltage_actual = 2.0
    ctrl.angle = 1.0
    ctrl.angle_velocity = 3.0
    ctrl.startup_timer = 0.2
    ctrl.reset()
    assert ctrl.frequency_ref == 0.0
    assert ctrl.frequency_actual == pytest.approx(
        ctrl.startup_ramp_initial_frequency_hz
    )
    assert ctrl.voltage_actual == 0.0
    assert ctrl.angle == 0.0
    assert ctrl.angle_velocity == 0.0
    assert ctrl.startup_timer == 0.0


def test_regression_baseline_make_load_supply_and_empty_kpis():
    with pytest.raises(ValueError, match="Unsupported load kind"):
        regression_baseline._make_load("step", {"torque": 1.0})
    with pytest.raises(ValueError, match="Unsupported supply kind"):
        regression_baseline._make_supply("step", {"voltage": 48.0})

    empty_history = {
        "speed": np.array([], dtype=np.float64),
        "currents_a": np.array([], dtype=np.float64),
        "currents_b": np.array([], dtype=np.float64),
        "currents_c": np.array([], dtype=np.float64),
        "torque": np.array([], dtype=np.float64),
        "load_torque": np.array([], dtype=np.float64),
    }
    kpis = regression_baseline._compute_kpis(empty_history)
    assert kpis["final_speed_rpm"] == 0.0
    assert kpis["mean_load_torque_nm"] == 0.0


def test_regression_baseline_save_variants_and_missing_rows(tmp_path, monkeypatch):
    monkeypatch.setattr(
        regression_baseline,
        "run_reference_suite",
        lambda: {"scenario_a": {"k1": 10.0, "k2": 20.0}},
    )
    monkeypatch.setattr(
        regression_baseline,
        "run_foc_reference_suite",
        lambda: {"scenario_foc": {"k1": 12.0}},
    )

    out_std = tmp_path / "baseline.json"
    out_foc = tmp_path / "baseline_foc.json"

    regression_baseline.save_baseline(out_std, tolerances={"scenario_a": {"k1": 0.1}})
    regression_baseline.save_foc_baseline(
        out_foc, tolerances={"scenario_foc": {"k1": 0.2}}
    )

    payload_std = regression_baseline.load_baseline(out_std)
    payload_foc = regression_baseline.load_baseline(out_foc)

    assert payload_std["scenarios"]["scenario_a"]["k1"] == 10.0
    assert payload_foc["scenarios"]["scenario_foc"]["k1"] == 12.0

    baseline = {
        "scenarios": {
            "missing_scenario": {"k1": 1.0},
            "present_scenario": {"missing_kpi": 2.0, "ok_kpi": 3.0},
        },
        "tolerances": {"present_scenario": {"missing_kpi": 0.1, "ok_kpi": 0.1}},
    }
    current = {
        "present_scenario": {"ok_kpi": 3.0},
    }

    rows = regression_baseline.build_drift_report(
        current=current, baseline_payload=baseline
    )
    statuses = {(r["scenario"], r["kpi"]): r["status"] for r in rows}
    assert statuses[("missing_scenario", "*scenario*")] == "missing"
    assert statuses[("present_scenario", "missing_kpi")] == "missing"

    failures = regression_baseline.compare_to_baseline(
        current=current, baseline_payload=baseline
    )
    assert any("Missing scenario result: missing_scenario" in msg for msg in failures)
    assert any("present_scenario: missing KPI missing_kpi" in msg for msg in failures)


def test_regression_baseline_thresholds_warn_fail_and_formatting():
    diagnostics = {
        "kpi_nan": float("nan"),
        "kpi_fail_high": 10.0,
        "kpi_warn_high": 6.5,
        "kpi_pass": 5.0,
    }
    thresholds = {
        "kpi_nan": {"warn_min": 1.0, "fail_min": 0.0},
        "kpi_fail_high": {"warn_max": 8.0, "fail_max": 9.0},
        "kpi_warn_high": {"warn_max": 6.0, "fail_max": 9.0},
        "kpi_pass": {
            "warn_min": 2.0,
            "warn_max": 8.0,
            "fail_min": 1.0,
            "fail_max": 9.0,
        },
    }

    rows = regression_baseline.evaluate_startup_transition_thresholds(
        diagnostics=diagnostics,
        thresholds=thresholds,
    )
    by_kpi = {row["kpi"]: row for row in rows}
    assert by_kpi["kpi_nan"]["status"] == "fail"
    assert by_kpi["kpi_fail_high"]["status"] == "fail"
    assert by_kpi["kpi_warn_high"]["status"] == "warn"
    assert by_kpi["kpi_pass"]["status"] == "pass"

    text = regression_baseline.format_startup_threshold_report(rows)
    assert "kpi | actual | warn_min | warn_max | fail_min | fail_max | status" in text
    assert "kpi_warn_high" in text


def test_motor_model_numba_fallback_and_validation_branches(monkeypatch):
    src_path = Path("src/core/motor_model.py")
    code = src_path.read_text(encoding="utf-8")

    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "numba":
            raise ImportError("forced for coverage")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    module_ns = {"__name__": "_motor_model_no_numba_test"}
    exec(compile(code, str(src_path), "exec"), module_ns)
    assert module_ns["HAS_NUMBA"] is False
    assert callable(module_ns["njit"])

    # Restore normal imports before touching the live numba-decorated class.
    monkeypatch.setattr(builtins, "__import__", real_import)

    with pytest.raises(ValueError, match="flux_weakening_id_coefficient"):
        MotorParameters(flux_weakening_id_coefficient=-0.1)
    with pytest.raises(ValueError, match="flux_weakening_min_ratio"):
        MotorParameters(flux_weakening_min_ratio=1.5)

    with pytest.raises(ValueError, match="Time step dt must be positive"):
        BLDCMotor(MotorParameters(), dt=0.0)
    with pytest.raises(ValueError, match="Phase inductance must be positive"):
        BLDCMotor(MotorParameters(phase_inductance=0.0), dt=1e-4)
    with pytest.raises(ValueError, match=r"d-axis inductance \(Ld\) must be positive"):
        BLDCMotor(MotorParameters(model_type="dq", ld=0.0, lq=0.001), dt=1e-4)
    with pytest.raises(ValueError, match=r"q-axis inductance \(Lq\) must be positive"):
        BLDCMotor(MotorParameters(model_type="dq", ld=0.001, lq=0.0), dt=1e-4)

    # Cover trapezoidal/sinusoidal helper branches directly.
    assert BLDCMotor._trapezoidal_fast(0.0) == pytest.approx(1.0)
    assert BLDCMotor._trapezoidal_fast(np.pi - 0.01) > -1.0
    assert BLDCMotor._trapezoidal_fast(np.pi + 0.01) == pytest.approx(-1.0)
    assert BLDCMotor._sinusoidal_fast(np.pi / 2.0) == pytest.approx(1.0)
    assert BLDCMotor._trapezoidal(0.1) == pytest.approx(1.0)
    assert BLDCMotor._trapezoidal(np.pi - 0.1) < 1.0
    assert BLDCMotor._trapezoidal(4.5) == pytest.approx(-1.0)


def test_simulation_engine_error_and_reset_branches():
    params = MotorParameters()
    motor = BLDCMotor(params, dt=1e-4)
    load = ConstantLoad(torque=0.0)

    with pytest.raises(ValueError, match="Time step dt must be positive"):
        SimulationEngine(motor, load, dt=0.0)

    class BrokenHardware(MockDAQHardware):
        def connect(self):
            raise RuntimeError("connect failed")

    broken_hw = BrokenHardware()
    engine_with_broken_hw = SimulationEngine(
        motor,
        load,
        dt=1e-4,
        hardware_interface=broken_hw,
    )
    hw_state = engine_with_broken_hw.get_hardware_state()
    assert hw_state["enabled"] is False
    assert hw_state["connected"] is False
    assert "connect failed" in hw_state["last_io_error"]

    # No telemetry should be a no-op.
    before = engine_with_broken_hw.get_inverter_state().copy()
    engine_with_broken_hw.set_inverter_telemetry(None)
    assert engine_with_broken_hw.get_inverter_state() == before

    with pytest.raises(ValueError, match="pwm_frequency_hz must be positive"):
        engine_with_broken_hw.set_pwm_frequency(0.0)
    with pytest.raises(ValueError, match="control_period_s must be positive"):
        engine_with_broken_hw.record_control_timing(1e-5, control_period_s=0.0)
    assert "cpu_load_pct" in engine_with_broken_hw.get_control_timing_state()

    # No hardware backend: enable request should return with connected=False.
    plain_engine = SimulationEngine(motor, load, dt=1e-4)
    plain_engine.configure_hardware_interface(True)
    assert plain_engine.get_hardware_state()["connected"] is False


def test_simulation_engine_log_data_step_paths_and_getters():
    params = MotorParameters()
    motor = BLDCMotor(params, dt=1e-4)
    load = ConstantLoad(torque=0.0)
    pfc = PowerFactorController(
        target_pf=0.95, kp=0.1, ki=1.0, max_compensation_var=1000.0
    )

    hw = MockDAQHardware(noise_std=0.0, seed=1)
    hw.connect()

    engine = SimulationEngine(
        motor,
        load,
        dt=1e-4,
        supply_profile=ConstantSupply(voltage=0.0),
        pfc_controller=pfc,
        hardware_interface=hw,
    )

    # Manual log path before any step should use default zero voltages.
    engine.log_data()

    telemetry = {
        "effective_dc_voltage": 0.0,
        "dc_link_ripple_v": 0.0,
        "dc_link_bus_current_a": 0.0,
        "total_inverter_loss_power_w": 0.0,
    }
    engine.set_inverter_telemetry(telemetry)

    engine.step(np.array([1.0, -0.5, -0.5]), log_data=True)
    hist = engine.get_history()
    assert hist["time"].size >= 1

    # Ensure getter branches are exercised.
    state = engine.get_current_state()
    assert "omega" in state
    info = engine.get_simulation_info()
    assert "hardware" in info

    engine.reset(initial_speed=1.0)
    hw_state = engine.get_hardware_state()
    assert hw_state["last_feedback"] == {}
    assert hw_state["last_io_error"] == ""


def test_motor_model_sinusoidal_properties_and_shape_validation():
    params = MotorParameters(
        emf_shape="sinusoidal",
        phase_inductance=0.01,
        ld=None,
        lq=None,
    )
    assert params.ld == pytest.approx(0.01)
    assert params.lq == pytest.approx(0.01)

    motor = BLDCMotor(params, dt=1e-4)
    motor.state = np.array([1.0, -2.0, 1.0, 120.0, 7.0 * np.pi], dtype=np.float64)

    assert np.allclose(motor.currents, np.array([1.0, -2.0, 1.0]))
    assert motor.omega == pytest.approx(120.0)
    assert motor.theta == pytest.approx((7.0 * np.pi) % (2.0 * np.pi))
    assert motor.speed_rpm == pytest.approx(120.0 * 60.0 / (2.0 * np.pi))

    emf = motor._calculate_back_emf(motor.theta)
    assert emf.shape == (3,)

    with pytest.raises(AssertionError, match="Voltages must be 3-phase array"):
        motor._state_derivatives(motor.state.copy(), np.array([1.0, 2.0]))


def test_motor_model_dq_flux_weakening_and_fast_torque_helper():
    params_dq = MotorParameters(
        model_type="dq",
        ld=0.002,
        lq=0.003,
        flux_weakening_id_coefficient=0.2,
        flux_weakening_min_ratio=0.4,
    )
    motor_dq = BLDCMotor(params_dq, dt=1e-4)

    # Strong negative Id should clamp to configured minimum ratio.
    assert motor_dq._flux_weakening_ratio(-10.0) == pytest.approx(0.4)
    assert motor_dq._flux_weakening_ratio(5.0) == pytest.approx(1.0)

    torque_fast = BLDCMotor._calculate_electromagnetic_torque_fast(0.2, 3.0, -1.0, -2.0)
    assert torque_fast == pytest.approx(0.0)

    motor_dq.step(np.array([2.0, -1.0, -1.0]), load_torque=0.1)
    state = motor_dq.get_state_dict()
    assert "torque" in state


def test_simulation_engine_runtime_hardware_feedback_and_reconnect_error():
    class RuntimeHardware:
        name = "runtime-hw"

        def __init__(self):
            self.is_connected = False
            self.connect_should_fail = False

        def connect(self):
            if self.connect_should_fail:
                raise RuntimeError("runtime connect failed")
            self.is_connected = True

        def disconnect(self):
            self.is_connected = False

        def write_phase_voltages(self, _voltages, _time_s):
            return None

        def read_feedback(self, _time_s):
            return {
                "applied_voltages": np.array([0.3, -0.1, -0.2], dtype=np.float64),
                "load_torque": 1.5,
            }

    motor = BLDCMotor(MotorParameters(), dt=1e-4)
    load = ConstantLoad(torque=0.0)
    hw = RuntimeHardware()
    engine = SimulationEngine(motor, load, dt=1e-4, hardware_interface=hw)

    engine.set_compute_backend("cpu")
    assert engine.get_compute_backend_state()["selected"] == "cpu"

    engine.record_control_timing(calc_duration_s=1e-5, control_period_s=2e-4)
    timing = engine.get_control_timing_state()
    assert timing["pwm_frequency_hz"] == pytest.approx(5000.0)

    engine.step(np.array([1.0, -0.5, -0.5]), log_data=True)
    history = engine.get_history()
    assert history["load_torque"][-1] == pytest.approx(1.5)

    engine.log_data(load_torque=None)
    assert engine.get_history()["time"].size >= 2

    # Force runtime reconnect failure branch in configure_hardware_interface.
    hw.is_connected = False
    hw.connect_should_fail = True
    engine.configure_hardware_interface(True)
    hw_state = engine.get_hardware_state()
    assert hw_state["connected"] is False
    assert "runtime connect failed" in hw_state["last_io_error"]


def test_svm_generator_validation_and_runtime_branches():
    svm = SVMGenerator(dc_voltage=48.0)

    with pytest.raises(ValueError, match="sample_time_s must be positive"):
        svm.set_sample_time(0.0)

    invalid_nonideal_kwargs = [
        {"device_drop_v": -0.1},
        {"dead_time_fraction": -0.1},
        {"dead_time_fraction": 0.3},
        {"conduction_resistance_ohm": -0.1},
        {"switching_frequency_hz": -1.0},
        {"switching_loss_coeff_v_per_a_khz": -0.1},
        {"diode_drop_v": -0.1},
        {"min_pulse_fraction": 0.7},
        {"dc_link_capacitance_f": -1.0},
        {"dc_link_source_resistance_ohm": -0.1},
        {"thermal_resistance_k_per_w": -0.1},
        {"thermal_capacitance_j_per_k": 0.0},
        {"temp_coeff_resistance_per_c": -0.1},
        {"enable_phase_asymmetry": True, "phase_voltage_scale_a": 0.0},
    ]
    for kwargs in invalid_nonideal_kwargs:
        with pytest.raises(ValueError):
            svm.set_nonidealities(**kwargs)

    with pytest.raises(ValueError, match="phase_currents must be a 3-element array"):
        svm.set_phase_currents(np.array([1.0, 2.0]))

    # Cover source_r <= 1e-12 branch in bus ripple state update.
    svm.set_nonidealities(
        enable_bus_ripple=True,
        dc_link_capacitance_f=1e-3,
        dc_link_source_resistance_ohm=0.0,
    )
    svm._update_bus_ripple_state(source_current=5.0)
    assert svm._cap_voltage == pytest.approx(svm.source_dc_voltage)

    # Cover thermal coupling guard branch where r_th or c_th is effectively zero.
    svm.set_nonidealities(
        enable_thermal_coupling=True,
        thermal_resistance_k_per_w=0.0,
        thermal_capacitance_j_per_k=1.0,
    )
    svm._update_thermal_state(total_loss_power_w=10.0)
    assert svm._junction_temperature_c == pytest.approx(
        svm.realism.ambient_temperature_c
    )

    # Default no-asymmetry path returns all-ones scales.
    svm.set_nonidealities(enable_phase_asymmetry=False)
    assert np.allclose(svm._phase_voltage_scales(), np.ones(3))

    with pytest.raises(ValueError, match="DC voltage must be positive"):
        svm.set_dc_voltage(0.0)

    cart = CartesianSVMGenerator(dc_voltage=48.0)
    abc = cart.cartesian_to_threephase(2.0, 3.0)
    assert abc.shape == (3,)
    assert abc[0] == pytest.approx(2.0)


def test_foc_helper_functions_and_pi_branches():
    wrapped = _wrap_angle(5 * np.pi)
    assert -np.pi <= wrapped < np.pi

    theta = _blend_angles(0.1, 1.1, 0.5)
    assert 0.0 <= theta < 2.0 * np.pi

    pi_state = {"kp": 1.0, "ki": 2.0, "integral": 0.0}
    out = _pi_update(pi_state, error=1.5, dt=0.1)
    assert out > 0.0
    assert pi_state["integral"] == pytest.approx(0.15)

    aw_state = {"kp": 10.0, "ki": 5.0, "kaw": 0.2, "integral": 0.0}
    sat = _pi_update_anti_windup(aw_state, error=10.0, dt=0.1, limit=1.0)
    assert sat == pytest.approx(1.0)

    motor = BLDCMotor(MotorParameters(), dt=1e-4)
    foc = FOCController(motor=motor)
    with pytest.raises(ValueError, match="Unsupported angle observer mode"):
        foc.set_angle_observer("bad-mode")


def test_vf_controller_positive_setters_and_state_snapshot():
    ctrl = VFController(v_nominal=48.0, f_nominal=120.0, v_startup=2.0)

    ctrl.set_speed_reference(15.0)
    assert ctrl.get_speed_reference() == pytest.approx(15.0)

    ctrl.set_vf_characteristic(v_startup=3.0, v_nominal=60.0, f_nominal=150.0)
    assert ctrl.kv_f == pytest.approx((60.0 - 3.0) / 150.0)

    ctrl.set_frequency_slew_rate(25.0)
    ctrl.set_voltage_ramp_rate(12.5)
    ctrl.enable_startup_boost(enable=True, duration=0.4)

    state = ctrl.get_state()
    assert state["frequency_ref"] == pytest.approx(15.0)
    assert "startup_phase" in state

    ctrl.reset()
    assert ctrl.frequency_ref == pytest.approx(0.0)
    assert ctrl.frequency_actual == pytest.approx(0.0)
    assert ctrl.startup_timer == pytest.approx(0.0)


def test_svm_state_snapshot_and_dc_voltage_update_paths():
    svm = SVMGenerator(dc_voltage=48.0)

    telem0 = svm.get_last_telemetry()
    assert "effective_dc_voltage" in telem0
    assert telem0["effective_dc_voltage"] > 0.0

    state0 = svm.get_realism_state()
    assert "sample_time_s" in state0
    assert state0["source_dc_voltage"] == pytest.approx(48.0)

    svm.set_dc_voltage(52.0)
    assert svm.source_dc_voltage == pytest.approx(52.0)

    svm.set_nonidealities(
        enable_bus_ripple=True,
        dc_link_capacitance_f=2e-3,
        dc_link_source_resistance_ohm=0.2,
    )
    svm.set_dc_voltage(50.0)
    assert svm.source_dc_voltage == pytest.approx(50.0)

    svm.reset_realism_state()
    state1 = svm.get_realism_state()
    assert state1["junction_temperature_c"] == pytest.approx(
        svm.realism.ambient_temperature_c
    )


def test_foc_startup_fallback_branch_behavior():
    motor = BLDCMotor(MotorParameters(), dt=1e-4)
    foc = FOCController(motor=motor)

    foc.startup_fallback_enabled = False
    assert (
        foc._maybe_apply_sensorless_fallback(0.02, emf_mag=0.0, confidence=0.0) is False
    )

    foc.startup_fallback_enabled = True
    foc.startup_min_confidence = 0.7
    foc.startup_confidence_hysteresis = 0.1
    foc.startup_min_speed_rpm = 500.0
    foc.startup_min_emf_v = 1.0
    foc.startup_fallback_hold_s = 0.03
    foc.startup_handoff_count = 1

    # Force degraded conditions: low confidence, low speed, low EMF.
    motor.state[3] = 0.0
    assert (
        foc._maybe_apply_sensorless_fallback(0.01, emf_mag=0.1, confidence=0.2) is False
    )
    assert (
        foc._maybe_apply_sensorless_fallback(0.03, emf_mag=0.1, confidence=0.2) is True
    )
    assert foc.startup_fallback_event_count >= 1
    assert 0.0 <= foc.startup_handoff_stability_ratio <= 1.0
