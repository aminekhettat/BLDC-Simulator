"""Tests for power supply profile and FOC controller behavior.

Ensures that dynamic supply voltages propagate through the engine and that
FOC controller returns correctly formatted outputs.
"""

import numpy as np
import pytest
from PyQt6.QtWidgets import QApplication
import sys

# ensure QApplication instance for GUI tests
app = QApplication.instance() or QApplication(sys.argv)

from src.core.power_model import (
    SupplyProfile,
    ConstantSupply,
    PowerFactorController,
    compute_power_metrics,
    required_reactive_compensation,
)
from src.core.simulation_engine import SimulationEngine
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.load_model import ConstantLoad
from src.control import FOCController, SVMGenerator, CartesianSVMGenerator


def test_supply_profile_basic():
    # constant profile should always return same voltage
    p = ConstantSupply(24.0)
    assert p.get_voltage(0) == pytest.approx(24.0)
    assert p.get_voltage(10.5) == pytest.approx(24.0)


def test_power_metrics_unity_pf_for_in_phase_waveforms():
    t = np.linspace(0.0, 2.0 * np.pi, 2000, endpoint=False)
    v = 230.0 * np.sin(t)
    i = 10.0 * np.sin(t)

    metrics = compute_power_metrics(v, i)
    assert metrics["power_factor"] == pytest.approx(1.0, abs=1e-3)
    assert metrics["reactive_power_var"] == pytest.approx(0.0, abs=1.0)
    assert metrics["active_power_w"] > 0.0


def test_power_metrics_near_zero_pf_for_quadrature_waveforms():
    t = np.linspace(0.0, 2.0 * np.pi, 2000, endpoint=False)
    v = 230.0 * np.sin(t)
    i = 10.0 * np.sin(t + np.pi / 2.0)

    metrics = compute_power_metrics(v, i)
    assert abs(metrics["power_factor"]) <= 0.02
    assert abs(metrics["active_power_w"]) <= 5.0
    assert metrics["reactive_power_var"] > 0.0


def test_required_reactive_compensation_reduces_target_reactive_power():
    res = required_reactive_compensation(
        active_power_w=1000.0,
        current_pf=0.80,
        target_pf=0.95,
    )

    assert res["required_compensation_var"] > 0.0
    assert res["target_reactive_var"] < res["current_reactive_var"]


def test_power_factor_controller_generates_non_negative_command():
    pfc = PowerFactorController(target_pf=0.95, kp=0.2, ki=0.5)
    cmd = pfc.update(current_pf=0.70, active_power_w=1200.0, dt=0.001)
    assert cmd >= 0.0
    assert pfc.last_error > 0.0


def test_engine_pfc_telemetry_hook_updates_metrics():
    motor = BLDCMotor(MotorParameters())
    load = ConstantLoad(0.1)
    engine = SimulationEngine(
        motor, load, dt=0.0005, supply_profile=ConstantSupply(48.0)
    )
    engine.configure_power_factor_control(enabled=True, target_pf=0.97, kp=0.1, ki=0.2)

    for _ in range(80):
        engine.step(np.array([4.0, -2.0, -2.0]), log_data=True)

    hist = engine.get_history()
    assert hist["power_factor"].size == hist["time"].size
    assert hist["pfc_command_var"].size == hist["time"].size

    info = engine.get_simulation_info()
    assert "pfc" in info
    assert info["pfc"]["enabled"] is True
    assert 0.0 <= abs(info["pfc"]["power_factor"]) <= 1.0
    assert info["pfc"]["compensation_command_var"] >= 0.0


def test_engine_applies_supply_voltage():
    # supply should influence what the engine logs
    motor = BLDCMotor(MotorParameters())
    load = ConstantLoad(0.0)
    supply = ConstantSupply(12.0)
    engine = SimulationEngine(motor, load, dt=0.001, supply_profile=supply)

    # before stepping, history should be empty
    hist = engine.get_history()
    assert hist["supply_voltage"].size == 0
    engine.step(np.zeros(3), log_data=True)
    hist = engine.get_history()
    assert hist["supply_voltage"][0] == pytest.approx(12.0)


def test_foc_controller_polar_output():
    motor = BLDCMotor(MotorParameters())
    ctrl = FOCController(motor=motor)
    # default output_cartesian should be False
    assert not ctrl.output_cartesian
    # without setting references, update returns numeric tuple (mag, angle)
    out = ctrl.update(0.001)
    assert isinstance(out, tuple) and len(out) == 2
    mag, ang = out
    assert isinstance(mag, float)
    assert isinstance(ang, float)


def test_foc_controller_cartesian_output():
    motor = BLDCMotor(MotorParameters())
    ctrl = FOCController(motor=motor)
    ctrl.output_cartesian = True
    ctrl.set_current_references(id_ref=1.0, iq_ref=0.5)
    out = ctrl.update(0.001)
    assert isinstance(out, tuple) and len(out) == 2
    valpha, vbeta = out
    assert isinstance(valpha, float)
    assert isinstance(vbeta, float)


def test_foc_auto_tune_changes_params():
    motor = BLDCMotor(MotorParameters())
    ctrl = FOCController(motor=motor)
    # record original gains
    orig_kp = ctrl.pi_q["kp"]
    ctrl.auto_tune_pi(axis="q", bandwidth=100.0)
    assert ctrl.pi_q["kp"] != orig_kp
    assert ctrl.pi_q["ki"] != 0


def test_foc_optional_cascaded_speed_loop_generates_bounded_iq_ref():
    motor = BLDCMotor(MotorParameters())
    ctrl = FOCController(motor=motor, enable_speed_loop=True)
    ctrl.set_cascaded_speed_loop(True, iq_limit_a=2.0)
    ctrl.set_speed_reference(2000.0)

    _ = ctrl.update(0.001)

    assert abs(ctrl.iq_ref) <= 2.0 + 1e-9
    assert ctrl.get_state()["speed_loop_enabled"] is True


def test_foc_current_loop_voltage_is_limited_by_vdq_limit():
    motor = BLDCMotor(MotorParameters())
    ctrl = FOCController(motor=motor)

    ctrl.vdq_limit = 1.0
    ctrl.set_current_references(id_ref=50.0, iq_ref=50.0)
    mag, _ = ctrl.update(0.001)

    assert float(mag) <= 1.0 + 1e-9


def test_foc_set_current_pi_gains_updates_states():
    motor = BLDCMotor(MotorParameters())
    ctrl = FOCController(motor=motor)

    ctrl.set_current_pi_gains(d_kp=2.5, d_ki=0.4, q_kp=3.2, q_ki=0.7, kaw=0.8)

    assert ctrl.pi_d["kp"] == pytest.approx(2.5)
    assert ctrl.pi_d["ki"] == pytest.approx(0.4)
    assert ctrl.pi_d["kaw"] == pytest.approx(0.8)
    assert ctrl.pi_q["kp"] == pytest.approx(3.2)
    assert ctrl.pi_q["ki"] == pytest.approx(0.7)
    assert ctrl.pi_q["kaw"] == pytest.approx(0.8)


def test_foc_decoupling_feedforward_changes_dq_voltage_commands():
    motor = BLDCMotor(MotorParameters())
    motor.state[0:3] = np.array([2.0, -1.0, -1.0])
    motor.state[3] = 120.0
    motor.state[4] = 0.7

    ctrl_no_ff = FOCController(motor=motor)
    ctrl_no_ff.set_current_references(id_ref=0.2, iq_ref=0.3)
    _ = ctrl_no_ff.update(0.001)

    ctrl_ff = FOCController(motor=motor)
    ctrl_ff.set_current_references(id_ref=0.2, iq_ref=0.3)
    ctrl_ff.set_decoupling(enable_d=True, enable_q=True)
    _ = ctrl_ff.update(0.001)

    state_no_ff = ctrl_no_ff.get_state()
    state_ff = ctrl_ff.get_state()
    assert state_ff["decouple_d_enabled"] is True
    assert state_ff["decouple_q_enabled"] is True
    assert abs(state_ff["v_d_ff"]) > 0.0
    assert abs(state_ff["v_q_ff"]) > 0.0
    assert abs(state_ff["v_d_cmd"] - state_no_ff["v_d_cmd"]) > 1e-9
    assert abs(state_ff["v_q_cmd"] - state_no_ff["v_q_cmd"]) > 1e-9


def test_foc_angle_observer_modes_pll_and_smo_update_state():
    motor = BLDCMotor(MotorParameters())
    motor.state[0:3] = np.array([0.5, -0.25, -0.25])
    motor.state[3] = 80.0
    motor.state[4] = 0.6
    motor._last_emf = motor._calculate_back_emf(motor.theta)

    ctrl = FOCController(motor=motor)
    ctrl.set_current_references(id_ref=0.1, iq_ref=0.2)

    ctrl.set_angle_observer("PLL")
    ctrl.set_pll_gains(kp=60.0, ki=1500.0)
    _ = ctrl.update(0.001)
    pll_state = ctrl.get_state()
    assert pll_state["angle_observer_mode"] == "PLL"
    assert np.isfinite(pll_state["theta_electrical"])

    ctrl.set_angle_observer("SMO")
    ctrl.set_smo_gains(k_slide=500.0, lpf_alpha=0.1, boundary=0.05)
    _ = ctrl.update(0.001)
    smo_state = ctrl.get_state()
    assert smo_state["angle_observer_mode"] == "SMO"
    assert np.isfinite(smo_state["theta_electrical"])


def test_foc_observer_startup_transition_handoffs_to_target_mode():
    motor = BLDCMotor(MotorParameters())
    ctrl = FOCController(motor=motor)
    ctrl.set_angle_observer("PLL")
    ctrl.set_startup_transition(
        enabled=True,
        initial_mode="Measured",
        min_speed_rpm=200.0,
        min_elapsed_s=0.01,
        min_emf_v=0.1,
        min_confidence=0.0,
        confidence_hold_s=0.0,
    )

    # Below threshold: should stay in startup initial mode.
    motor.state[3] = 5.0
    motor.state[4] = 0.2
    motor._last_emf = motor._calculate_back_emf(motor.theta)
    _ = ctrl.update(0.001)
    s0 = ctrl.get_state()
    assert s0["angle_observer_mode"] == "Measured"
    assert s0["startup_transition_done"] is False

    # Above thresholds long enough: should hand off to target observer.
    motor.state[3] = 80.0
    motor.state[4] = 0.6
    for _ in range(20):
        motor._last_emf = motor._calculate_back_emf(motor.theta)
        _ = ctrl.update(0.001)

    s1 = ctrl.get_state()
    assert s1["startup_transition_done"] is True
    assert s1["angle_observer_mode"] == "PLL"
    assert s1["startup_handoff_count"] >= 1
    assert s1["startup_last_handoff_time_s"] >= 0.0
    assert 0.0 <= s1["startup_last_handoff_confidence"] <= 1.0
    assert 0.0 <= s1["startup_handoff_confidence_peak"] <= 1.0
    assert 0.0 <= s1["startup_handoff_quality"] <= 1.0
    assert 0.0 <= s1["startup_handoff_stability_ratio"] <= 1.0


def test_foc_observer_confidence_is_bounded_and_reported():
    motor = BLDCMotor(MotorParameters())
    motor.state[0:3] = np.array([1.0, -0.5, -0.5])
    motor.state[3] = 70.0
    motor.state[4] = 0.4
    motor._last_emf = motor._calculate_back_emf(motor.theta)

    ctrl = FOCController(motor=motor)
    ctrl.set_angle_observer("PLL")
    _ = ctrl.update(0.001)
    state = ctrl.get_state()

    assert 0.0 <= state["observer_confidence"] <= 1.0
    assert 0.0 <= state["observer_confidence_emf"] <= 1.0
    assert 0.0 <= state["observer_confidence_speed"] <= 1.0
    assert 0.0 <= state["observer_confidence_coherence"] <= 1.0
    weighted = (
        0.45 * state["observer_confidence_emf"]
        + 0.35 * state["observer_confidence_speed"]
        + 0.20 * state["observer_confidence_coherence"]
    )
    assert state["observer_confidence"] == pytest.approx(weighted, abs=1e-9)
    assert 0.0 <= state["observer_confidence_ema"] <= 1.0
    assert -1.0 <= state["observer_confidence_trend"] <= 1.0
    assert state["observer_confidence_above_threshold_time_s"] >= 0.0
    assert state["observer_confidence_below_threshold_time_s"] >= 0.0
    assert state["observer_confidence_crossings_up"] >= 0
    assert state["observer_confidence_crossings_down"] >= 0


def test_foc_sensorless_blend_weight_low_speed_prefers_measured_angle():
    motor = BLDCMotor(MotorParameters())
    motor.state[0:3] = np.array([0.6, -0.3, -0.3])
    motor.state[3] = 5.0
    motor.state[4] = 0.08
    motor._last_emf = motor._calculate_back_emf(motor.theta)

    ctrl = FOCController(motor=motor)
    ctrl.set_angle_observer("PLL")
    ctrl.set_sensorless_blend(enabled=True, min_speed_rpm=200.0, min_confidence=0.8)
    _ = ctrl.update(0.001)
    state = ctrl.get_state()

    assert state["angle_observer_mode"] == "PLL"
    assert 0.0 <= state["sensorless_blend_weight"] <= 0.25
    theta_measured = (motor.theta * motor.params.poles_pairs) % (2 * np.pi)
    err_blended = abs(
        np.arctan2(
            np.sin(state["theta_electrical"] - theta_measured),
            np.cos(state["theta_electrical"] - theta_measured),
        )
    )
    err_raw = abs(
        np.arctan2(
            np.sin(state["theta_sensorless_raw"] - theta_measured),
            np.cos(state["theta_sensorless_raw"] - theta_measured),
        )
    )
    assert err_blended <= err_raw + 1e-9


def test_foc_sensorless_blend_weight_high_speed_uses_observer_angle():
    motor = BLDCMotor(MotorParameters())
    motor.state[0:3] = np.array([1.2, -0.6, -0.6])
    motor.state[3] = 100.0
    motor.state[4] = 0.8
    motor._last_emf = motor._calculate_back_emf(motor.theta)

    ctrl = FOCController(motor=motor)
    ctrl.set_angle_observer("SMO")
    ctrl.set_sensorless_blend(enabled=True, min_speed_rpm=30.0, min_confidence=0.2)
    _ = ctrl.update(0.001)
    state = ctrl.get_state()

    assert state["angle_observer_mode"] == "SMO"
    assert state["sensorless_blend_weight"] >= 0.95


def test_foc_set_sensorless_blend_validates_inputs():
    ctrl = FOCController(motor=BLDCMotor(MotorParameters()))

    with pytest.raises(ValueError):
        ctrl.set_sensorless_blend(min_speed_rpm=-1.0)
    with pytest.raises(ValueError):
        ctrl.set_sensorless_blend(min_confidence=1.2)


def test_foc_observer_startup_transition_can_fallback_on_degraded_conditions():
    motor = BLDCMotor(MotorParameters())
    ctrl = FOCController(motor=motor)
    ctrl.set_angle_observer("PLL")
    ctrl.set_startup_transition(
        enabled=True,
        initial_mode="Measured",
        min_speed_rpm=100.0,
        min_elapsed_s=0.0,
        min_emf_v=0.05,
        min_confidence=0.2,
        confidence_hold_s=0.0,
        confidence_hysteresis=0.1,
        fallback_enabled=True,
        fallback_hold_s=0.005,
    )

    motor.state[3] = 90.0
    motor.state[4] = 0.5
    for _ in range(20):
        motor._last_emf = motor._calculate_back_emf(motor.theta)
        _ = ctrl.update(0.001)

    switched = ctrl.get_state()
    assert switched["startup_transition_done"] is True
    assert switched["angle_observer_mode"] == "PLL"

    motor.state[3] = 2.0
    motor.state[4] = 0.05
    for _ in range(12):
        motor._last_emf = motor._calculate_back_emf(motor.theta)
        _ = ctrl.update(0.001)

    degraded = ctrl.get_state()
    assert degraded["startup_transition_done"] is False
    assert degraded["angle_observer_mode"] == "Measured"
    assert degraded["startup_fallback_event_count"] >= 1
    assert degraded["startup_handoff_count"] >= 1
    assert 0.0 <= degraded["startup_handoff_stability_ratio"] <= 1.0
    assert degraded["startup_handoff_stability_ratio"] < 1.0


def test_svm_cartesian_conversion():
    svm = CartesianSVMGenerator(dc_voltage=48.0)
    # test simple vector along alpha axis
    valpha, vbeta = 10.0, 0.0
    volts = svm.modulate_cartesian(valpha=valpha, vbeta=vbeta)
    assert volts.shape == (3,)
    # confirm that cartesian modulate is equivalent to converting to polar
    magnitude = np.hypot(valpha, vbeta)
    angle = np.arctan2(vbeta, valpha)
    expected = svm.modulate(magnitude, angle)
    assert np.allclose(volts, expected, atol=1e-6)


def test_svm_nonidealities_reduce_voltage_magnitude():
    ideal = SVMGenerator(dc_voltage=48.0)
    v_ideal = ideal.modulate(magnitude=20.0, angle=0.4)

    nonideal = SVMGenerator(dc_voltage=48.0)
    nonideal.set_nonidealities(device_drop_v=1.5, dead_time_fraction=0.05)
    v_nonideal = nonideal.modulate(magnitude=20.0, angle=0.4)

    assert np.linalg.norm(v_nonideal) < np.linalg.norm(v_ideal)
    assert np.max(np.abs(v_nonideal)) <= 24.0 + 1e-9


def test_svm_current_dependent_conduction_drop_reduces_voltage():
    svm_base = SVMGenerator(dc_voltage=48.0)
    svm_base.set_phase_currents(np.array([8.0, -8.0, 1.0]))
    v_base = svm_base.modulate(magnitude=18.0, angle=0.25)

    svm_cond = SVMGenerator(dc_voltage=48.0)
    svm_cond.set_nonidealities(
        device_drop_v=0.0,
        dead_time_fraction=0.0,
        conduction_resistance_ohm=0.15,
    )
    svm_cond.set_phase_currents(np.array([8.0, -8.0, 1.0]))
    v_cond = svm_cond.modulate(magnitude=18.0, angle=0.25)

    assert np.linalg.norm(v_cond) < np.linalg.norm(v_base)


def test_svm_switching_frequency_loss_reduces_voltage():
    svm_base = SVMGenerator(dc_voltage=48.0)
    svm_base.set_phase_currents(np.array([6.0, -4.0, -2.0]))
    v_base = svm_base.modulate(magnitude=16.0, angle=0.6)

    svm_sw = SVMGenerator(dc_voltage=48.0)
    svm_sw.set_nonidealities(
        switching_frequency_hz=12000.0,
        switching_loss_coeff_v_per_a_khz=0.01,
    )
    svm_sw.set_phase_currents(np.array([6.0, -4.0, -2.0]))
    v_sw = svm_sw.modulate(magnitude=16.0, angle=0.6)

    assert np.linalg.norm(v_sw) < np.linalg.norm(v_base)


def test_gui_supply_profile():
    # verify that the GUI applies correct supply profile settings
    from src.ui.main_window import BLDCMotorControlGUI
    from src.core.power_model import RampSupply, ConstantSupply

    gui = BLDCMotorControlGUI()
    gui.supply_type.setCurrentText("Ramp")
    gui.supply_ramp_initial.setValue(10.0)
    gui.supply_ramp_final.setValue(20.0)
    gui.supply_ramp_duration.setValue(5.0)
    gui._apply_to_simulation()
    assert isinstance(gui.engine.supply_profile, RampSupply)
    assert gui.engine.supply_profile.get_voltage(2.5) == pytest.approx(15.0)

    gui.supply_type.setCurrentText("Constant")
    gui.supply_constant_voltage.setValue(33.0)
    gui._apply_to_simulation()
    assert isinstance(gui.engine.supply_profile, ConstantSupply)
    assert gui.engine.supply_profile.get_voltage(0) == pytest.approx(33.0)


def test_gui_foc_cascaded_speed_loop_wiring():
    from src.ui.main_window import BLDCMotorControlGUI

    gui = BLDCMotorControlGUI()
    gui.ctrl_mode.setCurrentText("FOC")
    gui._on_control_mode_changed("FOC")

    gui.foc_speed_loop_mode.setCurrentText("Cascaded PI")
    gui.foc_iq_limit.setValue(3.5)
    gui.foc_speed_kp.setValue(0.03)
    gui.foc_speed_ki.setValue(2.0)
    gui.foc_d_kp.setValue(1.7)
    gui.foc_d_ki.setValue(0.25)
    gui.foc_q_kp.setValue(1.9)
    gui.foc_q_ki.setValue(0.35)
    gui.foc_decouple_d_mode.setCurrentText("Enabled")
    gui.foc_decouple_q_mode.setCurrentText("Enabled")
    gui._apply_to_simulation()

    assert isinstance(gui.controller, FOCController)
    assert gui.controller.enable_speed_loop is True
    assert gui.controller.iq_limit_a == pytest.approx(3.5)
    assert gui.controller.pi_speed["kp"] == pytest.approx(0.03)
    assert gui.controller.pi_speed["ki"] == pytest.approx(2.0)
    assert gui.controller.pi_d["kp"] == pytest.approx(1.7)
    assert gui.controller.pi_d["ki"] == pytest.approx(0.25)
    assert gui.controller.pi_q["kp"] == pytest.approx(1.9)
    assert gui.controller.pi_q["ki"] == pytest.approx(0.35)
    assert gui.controller.enable_decouple_d is True
    assert gui.controller.enable_decouple_q is True


def test_gui_foc_angle_observer_wiring():
    from src.ui.main_window import BLDCMotorControlGUI

    gui = BLDCMotorControlGUI()
    gui.ctrl_mode.setCurrentText("FOC")
    gui._on_control_mode_changed("FOC")

    gui.foc_angle_observer_mode.setCurrentText("SMO")
    gui.foc_pll_kp.setValue(120.0)
    gui.foc_pll_ki.setValue(3200.0)
    gui.foc_smo_k_slide.setValue(700.0)
    gui.foc_smo_lpf_alpha.setValue(0.12)
    gui.foc_smo_boundary.setValue(0.08)
    gui.foc_startup_transition_mode.setCurrentText("Enabled")
    gui.foc_startup_initial_observer.setCurrentText("Measured")
    gui.foc_startup_min_speed.setValue(250.0)
    gui.foc_startup_min_time.setValue(0.02)
    gui.foc_startup_min_emf.setValue(0.2)
    gui.foc_startup_min_confidence.setValue(0.55)
    gui.foc_startup_confidence_hold.setValue(0.03)
    gui.foc_startup_confidence_hysteresis.setValue(0.12)
    gui.foc_startup_fallback_mode.setCurrentText("Enabled")
    gui.foc_startup_fallback_hold.setValue(0.04)
    gui._apply_to_simulation()

    assert isinstance(gui.controller, FOCController)
    assert gui.controller.observer_target_mode == "SMO"
    assert gui.controller.angle_observer_mode == "Measured"
    assert gui.controller.pll["kp"] == pytest.approx(120.0)
    assert gui.controller.pll["ki"] == pytest.approx(3200.0)
    assert gui.controller.smo["k_slide"] == pytest.approx(700.0)
    assert gui.controller.smo["lpf_alpha"] == pytest.approx(0.12)
    assert gui.controller.smo["boundary"] == pytest.approx(0.08)
    assert gui.controller.startup_transition_enabled is True
    assert gui.controller.startup_initial_mode == "Measured"
    assert gui.controller.startup_min_speed_rpm == pytest.approx(250.0)
    assert gui.controller.startup_min_elapsed_s == pytest.approx(0.02)
    assert gui.controller.startup_min_emf_v == pytest.approx(0.2)
    assert gui.controller.startup_min_confidence == pytest.approx(0.55)
    assert gui.controller.startup_confidence_hold_s == pytest.approx(0.03)
    assert gui.controller.startup_confidence_hysteresis == pytest.approx(0.12)
    assert gui.controller.startup_fallback_enabled is True
    assert gui.controller.startup_fallback_hold_s == pytest.approx(0.04)


def test_gui_inverter_nonidealities_wiring():
    from src.ui.main_window import BLDCMotorControlGUI

    gui = BLDCMotorControlGUI()
    gui.inverter_device_drop.setValue(0.8)
    gui.inverter_dead_time_fraction.setValue(0.015)
    gui.inverter_conduction_resistance.setValue(0.02)
    gui.inverter_switching_frequency.setValue(10000.0)
    gui.inverter_switching_loss_coeff.setValue(0.006)
    gui._apply_to_simulation()

    assert gui.svm.device_drop_v == pytest.approx(0.8)
    assert gui.svm.dead_time_fraction == pytest.approx(0.015)
    assert gui.svm.conduction_resistance_ohm == pytest.approx(0.02)
    assert gui.svm.switching_frequency_hz == pytest.approx(10000.0)
    assert gui.svm.switching_loss_coeff_v_per_a_khz == pytest.approx(0.006)


def test_gui_pfc_wiring_to_engine_configuration():
    from src.ui.main_window import BLDCMotorControlGUI

    gui = BLDCMotorControlGUI()
    gui.pfc_mode.setCurrentText("Enabled")
    gui.pfc_target_pf.setValue(0.98)
    gui.pfc_kp.setValue(0.25)
    gui.pfc_ki.setValue(1.8)
    gui.pfc_max_var.setValue(4500.0)
    gui.pfc_window_samples.setValue(96)
    gui._apply_to_simulation()

    pfc_state = gui.engine.get_power_factor_control_state()
    assert pfc_state["enabled"] is True
    assert pfc_state["target_pf"] == pytest.approx(0.98)
    assert gui.engine.pfc_controller.kp == pytest.approx(0.25)
    assert gui.engine.pfc_controller.ki == pytest.approx(1.8)
    assert gui.engine.pfc_controller.max_compensation_var == pytest.approx(4500.0)
    assert gui.engine.pfc_window_samples == 96


def test_gui_monitoring_updates_pfc_status_blocks():
    from src.ui.main_window import BLDCMotorControlGUI

    gui = BLDCMotorControlGUI()
    gui.pfc_mode.setCurrentText("Enabled")
    gui._apply_to_simulation()

    gui._update_monitoring(
        {
            "speed_rpm": 850.0,
            "omega": 45.0,
            "theta": 0.4,
            "time": 0.2,
            "pfc": {
                "enabled": True,
                "target_pf": 0.97,
                "power_factor": 0.91,
                "active_power_w": 320.0,
                "reactive_power_var": 145.0,
                "compensation_command_var": 120.0,
            },
        }
    )

    assert gui.status_blocks["pfc_enabled"].current_value == pytest.approx(1.0)
    assert gui.status_blocks["pfc_target_pf"].current_value == pytest.approx(0.97)
    assert gui.status_blocks["pfc_power_factor"].current_value == pytest.approx(0.91)
    assert gui.status_blocks["pfc_active_power_w"].current_value == pytest.approx(320.0)
    assert gui.status_blocks["pfc_reactive_power_var"].current_value == pytest.approx(
        145.0
    )
    assert gui.status_blocks["pfc_command_var"].current_value == pytest.approx(120.0)


def test_gui_monitoring_updates_advanced_foc_status_blocks():
    from src.ui.main_window import BLDCMotorControlGUI

    gui = BLDCMotorControlGUI()
    gui.ctrl_mode.setCurrentText("FOC")
    gui._on_control_mode_changed("FOC")

    gui.foc_speed_loop_mode.setCurrentText("Cascaded PI")
    gui.foc_decouple_d_mode.setCurrentText("Enabled")
    gui.foc_decouple_q_mode.setCurrentText("Enabled")
    gui.foc_angle_observer_mode.setCurrentText("SMO")
    gui.foc_speed_ref.setValue(1500.0)
    gui._apply_to_simulation()

    assert isinstance(gui.controller, FOCController)

    gui.engine.motor.state[0:3] = np.array([1.2, -0.6, -0.6])
    gui.engine.motor.state[3] = 90.0
    gui.engine.motor.state[4] = 0.8
    gui.controller.set_current_references(id_ref=0.4, iq_ref=0.9)
    gui.controller.update(0.001)

    gui._update_monitoring(
        {
            "speed_rpm": 1250.0,
            "omega": 90.0,
            "theta": 0.8,
            "i_a": 1.2,
            "i_b": -0.6,
            "i_c": -0.6,
            "torque": 0.2,
            "back_emf_a": 1.0,
            "back_emf_b": -0.5,
            "back_emf_c": -0.5,
            "time": 0.15,
        }
    )

    ctrl_state = gui.controller.get_state()
    assert gui.status_blocks["id_ref"].current_value == pytest.approx(
        ctrl_state["id_ref"]
    )
    assert gui.status_blocks["iq_ref"].current_value == pytest.approx(
        ctrl_state["iq_ref"]
    )
    assert gui.status_blocks["speed_error"].current_value == pytest.approx(
        ctrl_state["speed_error"]
    )
    assert gui.status_blocks["v_d_ff"].current_value == pytest.approx(
        ctrl_state["v_d_ff"]
    )
    assert gui.status_blocks["v_q_ff"].current_value == pytest.approx(
        ctrl_state["v_q_ff"]
    )
    assert gui.status_blocks["speed_loop_enabled"].current_value == pytest.approx(1.0)
    assert gui.status_blocks["decouple_d_enabled"].current_value == pytest.approx(1.0)
    assert gui.status_blocks["decouple_q_enabled"].current_value == pytest.approx(1.0)
    assert gui.status_blocks["observer_mode_code"].current_value == pytest.approx(2.0)
    assert gui.status_blocks["theta_electrical"].current_value == pytest.approx(
        ctrl_state["theta_electrical"]
    )
    assert gui.status_blocks["theta_meas_emf"].current_value == pytest.approx(
        ctrl_state["theta_meas_emf"]
    )
    assert gui.status_blocks["theta_error_pll"].current_value == pytest.approx(
        ctrl_state["theta_error_pll"]
    )
    assert gui.status_blocks["theta_error_smo"].current_value == pytest.approx(
        ctrl_state["theta_error_smo"]
    )
    assert gui.status_blocks["smo_omega_est"].current_value == pytest.approx(
        ctrl_state["smo"]["omega_est"]
    )
    assert gui.status_blocks["observer_confidence"].current_value == pytest.approx(
        ctrl_state["observer_confidence"]
    )
    assert gui.status_blocks["observer_confidence_emf"].current_value == pytest.approx(
        ctrl_state["observer_confidence_emf"]
    )
    assert gui.status_blocks[
        "observer_confidence_speed"
    ].current_value == pytest.approx(ctrl_state["observer_confidence_speed"])
    assert gui.status_blocks[
        "observer_confidence_coherence"
    ].current_value == pytest.approx(ctrl_state["observer_confidence_coherence"])
    assert gui.status_blocks["observer_confidence_ema"].current_value == pytest.approx(
        ctrl_state["observer_confidence_ema"]
    )
    assert gui.status_blocks[
        "observer_confidence_trend"
    ].current_value == pytest.approx(ctrl_state["observer_confidence_trend"])
    assert gui.status_blocks[
        "observer_confidence_above_threshold_time_s"
    ].current_value == pytest.approx(
        ctrl_state["observer_confidence_above_threshold_time_s"]
    )
    assert gui.status_blocks[
        "observer_confidence_below_threshold_time_s"
    ].current_value == pytest.approx(
        ctrl_state["observer_confidence_below_threshold_time_s"]
    )
    assert gui.status_blocks["startup_handoff_quality"].current_value == pytest.approx(
        ctrl_state["startup_handoff_quality"]
    )
    assert gui.status_blocks[
        "startup_handoff_stability_ratio"
    ].current_value == pytest.approx(ctrl_state["startup_handoff_stability_ratio"])
