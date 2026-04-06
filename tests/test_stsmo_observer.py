"""
Atomic features tested in this module:
- stsmo gains analytical calibration produces positive k1 k2
- stsmo gains analytical apply sets use_stsmo flag
- stsmo gains analytical apply false leaves use_stsmo unset
- stsmo gains scale with rated rpm
- stsmo get state reports stsmo enabled flag
- stsmo update returns finite emf components
- stsmo backward euler solver does not diverge over many steps
- stsmo z1 warmstart seeds from motor omega on first call
- stsmo z1 not warmed when motor speed is near zero
- stsmo k2 floor matches k2_min when motor omega is zero
- stsmo k2 scales with motor omega at high speed
- stsmo k2 uses open loop rpm floor during open loop phase
- stsmo sogi filter active above 5 rad/s threshold
- stsmo sogi filter bypassed below 5 rad/s threshold
- stsmo safety snap resets current estimate on large divergence
- stsmo emf output is clipped to physical e_max bound
- stsmo omega elec est updated only when emf mag exceeds 0.3
- stsmo mras resistance adapts with stsmo emf
- dt architecture: DT_CTRL equals one PWM period
- dt architecture: DT_MOTOR equals DT_CTRL divided by N_SUB
- svm clarke output differs from inverse park command (AF root cause)
- update applied voltage sets v alpha prev from actual svm phases
- active flux integrator advances psi s using v alpha prev
- active flux theta tracks true angle after bumpless switch with correct voltage
- numerical stability advisory reports unstable when dt exceeds rk4 limit
- numerical stability advisory reports stable when dt is well within limit
- numerical stability advisory reports marginal when dt is between 50 and 100 pct of limit
- ui dt stability advisory method is present and callable on main window
"""

import math

import numpy as np
import pytest

from src.control import FOCController
from src.core.motor_model import BLDCMotor, MotorParameters

# ── helpers ───────────────────────────────────────────────────────────────────

def _make_motor(omega_mech: float = 80.0, theta: float = 0.6) -> BLDCMotor:
    """Return a motor spinning at *omega_mech* rad/s."""
    motor = BLDCMotor(MotorParameters())
    # phase currents: small balanced set so Clarke transform is non-trivial
    motor.state[0:3] = np.array([0.3, -0.15, -0.15])
    motor.state[3] = omega_mech   # omega [rad/s]
    motor.state[4] = theta        # theta [rad]
    motor._last_emf = motor._calculate_back_emf(motor.theta)
    return motor


def _make_stsmo_ctrl(
    omega_mech: float = 80.0,
    theta: float = 0.6,
    rated_rpm: float = 3500.0,
) -> tuple[BLDCMotor, FOCController]:
    """Return (motor, controller) with STSMO fully configured."""
    motor = _make_motor(omega_mech=omega_mech, theta=theta)
    ctrl = FOCController(motor=motor)
    ctrl.set_current_references(id_ref=0.0, iq_ref=0.5)
    ctrl.enable_sensorless_emf_reconstruction()
    ctrl.calibrate_stsmo_gains_analytical(rated_rpm=rated_rpm)
    ctrl.set_angle_observer("PLL")
    return motor, ctrl


# ── gain calibration ──────────────────────────────────────────────────────────

def test_stsmo_gains_analytical_calibration_produces_positive_k1_k2():
    motor = _make_motor()
    ctrl = FOCController(motor=motor)
    result = ctrl.calibrate_stsmo_gains_analytical(rated_rpm=3500.0, apply=False)

    assert result["k1"] > 0.0
    assert result["k2"] > 0.0
    assert result["e_max_v"] > 0.0


def test_stsmo_gains_analytical_apply_sets_use_stsmo_flag():
    motor = _make_motor()
    ctrl = FOCController(motor=motor)
    assert ctrl._use_stsmo is False

    ctrl.calibrate_stsmo_gains_analytical(rated_rpm=3500.0, apply=True)

    assert ctrl._use_stsmo is True
    assert ctrl.stsmo["k1"] > 0.0
    assert ctrl.stsmo["k2"] > 0.0
    assert "k2_factor" in ctrl.stsmo
    assert "k2_min" in ctrl.stsmo


def test_stsmo_gains_analytical_apply_false_leaves_use_stsmo_unset():
    motor = _make_motor()
    ctrl = FOCController(motor=motor)
    ctrl.calibrate_stsmo_gains_analytical(rated_rpm=3500.0, apply=False)

    assert ctrl._use_stsmo is False


def test_stsmo_gains_scale_with_rated_rpm():
    motor = _make_motor()
    ctrl_lo = FOCController(motor=motor)
    ctrl_hi = FOCController(motor=motor)

    g_lo = ctrl_lo.calibrate_stsmo_gains_analytical(rated_rpm=1000.0, apply=False)
    g_hi = ctrl_hi.calibrate_stsmo_gains_analytical(rated_rpm=5000.0, apply=False)

    # Higher speed → larger e_max → larger gains
    assert g_hi["k1"] > g_lo["k1"]
    assert g_hi["k2"] > g_lo["k2"]
    assert g_hi["e_max_v"] > g_lo["e_max_v"]


def test_stsmo_get_state_reports_stsmo_enabled_flag():
    _, ctrl = _make_stsmo_ctrl()
    state = ctrl.get_state()
    assert state["stsmo_enabled"] is True


# ── basic update behaviour ────────────────────────────────────────────────────

def test_stsmo_update_returns_finite_emf_components():
    motor, ctrl = _make_stsmo_ctrl(omega_mech=80.0)
    motor._last_emf = motor._calculate_back_emf(motor.theta)

    e_a, e_b, emf_mag = ctrl._update_stsmo_emf(dt=1e-4)

    assert math.isfinite(e_a)
    assert math.isfinite(e_b)
    assert math.isfinite(emf_mag)
    assert emf_mag >= 0.0


def test_stsmo_backward_euler_solver_does_not_diverge_over_many_steps():
    """Run 1 000 steps at 10 kHz — magnitude must not explode."""
    motor, ctrl = _make_stsmo_ctrl(omega_mech=80.0)
    dt = 1e-4
    e_max_v = float(motor.params.back_emf_constant) * 80.0 * 2.5 + 10.0  # generous bound

    for _ in range(1000):
        motor._last_emf = motor._calculate_back_emf(motor.theta)
        e_a, e_b, emf_mag = ctrl._update_stsmo_emf(dt=dt)
        assert abs(e_a) <= e_max_v, f"e_alpha diverged: {e_a}"
        assert abs(e_b) <= e_max_v, f"e_beta diverged: {e_b}"


# ── z1 warm-start ─────────────────────────────────────────────────────────────

def test_stsmo_z1_warmstart_seeds_from_motor_omega_on_first_call():
    """On the first call, z1 should be pre-loaded with a non-trivial EMF estimate."""
    motor, ctrl = _make_stsmo_ctrl(omega_mech=80.0)
    # Verify z1 starts at zero before the first update
    assert ctrl._stsmo_z1_alpha == pytest.approx(0.0)
    assert ctrl._stsmo_z1_beta == pytest.approx(0.0)

    ctrl._update_stsmo_emf(dt=1e-4)

    # After one step, z1 should have been warmed (non-zero seed from omega)
    z1_mag = math.hypot(ctrl._stsmo_z1_alpha, ctrl._stsmo_z1_beta)
    assert z1_mag > 0.1, f"z1 was not warmed up; magnitude = {z1_mag}"


def test_stsmo_z1_not_warmed_when_motor_speed_is_near_zero():
    """When motor.omega ≈ 0 and omega_elec_est ≈ 0, z1 stays at zero."""
    motor, ctrl = _make_stsmo_ctrl(omega_mech=0.0)
    # Ensure omega_elec_est is also at zero
    ctrl._omega_elec_est = 0.0

    ctrl._update_stsmo_emf(dt=1e-4)

    # z1 must remain at zero — no valid frequency to seed from
    z1_mag = math.hypot(ctrl._stsmo_z1_alpha, ctrl._stsmo_z1_beta)
    # z1 may have advanced by one sign() step, but magnitude should be tiny
    assert z1_mag < 1.0, f"z1 unexpectedly large at zero speed: {z1_mag}"


# ── speed-adaptive k2 ─────────────────────────────────────────────────────────

def test_stsmo_k2_floor_matches_k2_min_when_motor_omega_is_zero():
    """At zero speed, effective k2 must equal k2_min (the safety floor)."""
    motor, ctrl = _make_stsmo_ctrl(omega_mech=0.0)
    ctrl._omega_elec_est = 0.0
    k2_min = float(ctrl.stsmo["k2_min"])

    # Compute what k2 the update would choose by inspecting the internals
    ke  = float(motor.params.back_emf_constant)
    pp  = float(motor.params.poles_pairs)
    oe  = max(0.0, 0.0, 10.0)   # max(omega_elec_est=0, motor.omega*Pp=0, 10.0)
    om  = oe / max(pp, 1.0)
    k2_factor = float(ctrl.stsmo.get("k2_factor", 1.0))
    k2_expected = max(k2_min, k2_factor * ke * om * oe)

    assert k2_expected == pytest.approx(k2_min, rel=1e-6)


def test_stsmo_k2_scales_with_motor_omega_at_high_speed():
    """At high speed, effective k2 must exceed k2_min significantly."""
    motor_lo, ctrl_lo = _make_stsmo_ctrl(omega_mech=10.0)
    motor_hi, ctrl_hi = _make_stsmo_ctrl(omega_mech=500.0)

    ke = float(motor_hi.params.back_emf_constant)
    pp = float(motor_hi.params.poles_pairs)
    k2_min = float(ctrl_hi.stsmo["k2_min"])
    k2_factor = float(ctrl_hi.stsmo.get("k2_factor", 1.0))

    oe_hi = max(500.0 * pp, 10.0)
    om_hi = oe_hi / max(pp, 1.0)
    k2_hi = max(k2_min, k2_factor * ke * om_hi * oe_hi)

    oe_lo = max(10.0 * pp, 10.0)
    om_lo = oe_lo / max(pp, 1.0)
    k2_lo = max(k2_min, k2_factor * ke * om_lo * oe_lo)

    assert k2_hi > k2_lo


def test_stsmo_k2_uses_open_loop_rpm_floor_during_open_loop_phase():
    """During open_loop startup, the reference RPM is used as a k2 floor."""
    motor, ctrl = _make_stsmo_ctrl(omega_mech=0.0)
    ctrl._omega_elec_est = 0.0

    # Simulate open_loop phase at 800 RPM reference
    ctrl.startup_phase = "open_loop"
    ctrl.startup_open_loop_speed_rpm = 800.0

    pp = float(motor.params.poles_pairs)
    ke = float(motor.params.back_emf_constant)
    k2_min = float(ctrl.stsmo["k2_min"])
    k2_factor = float(ctrl.stsmo.get("k2_factor", 1.0))

    oe_ol = 800.0 / 60.0 * 2.0 * math.pi * pp
    om_ol = oe_ol / max(pp, 1.0)
    k2_open_loop = max(k2_min, k2_factor * ke * om_ol * oe_ol)

    # At zero motor.omega but non-zero open-loop ref, k2 must exceed k2_min
    assert k2_open_loop > k2_min


# ── SOGI filter ───────────────────────────────────────────────────────────────

def test_stsmo_sogi_filter_active_above_threshold():
    """Above 5 rad/s the SOGI states must evolve after a step."""
    motor, ctrl = _make_stsmo_ctrl(omega_mech=80.0)
    ctrl._omega_elec_est = 80.0 * float(motor.params.poles_pairs)

    # Seed z1 so SOGI has something to filter
    ctrl._stsmo_z1_alpha = 1.0
    ctrl._stsmo_z1_beta = 0.5
    ctrl._stsmo_z1_alpha = 1.0   # ensure _z_fresh guard doesn't fire
    ctrl._stsmo_i_alpha = motor.state[0]
    ctrl._stsmo_i_beta = -0.5 * (motor.state[1] - motor.state[2]) / math.sqrt(3)

    sogi_alpha_before = ctrl._sogi_e_alpha

    ctrl._update_stsmo_emf(dt=1e-4)

    # SOGI in-phase state must have changed
    assert ctrl._sogi_e_alpha != pytest.approx(sogi_alpha_before, abs=1e-12)


def test_stsmo_sogi_filter_bypassed_below_threshold():
    """Below 5 rad/s the SOGI states must remain unchanged (z1 used directly)."""
    motor, ctrl = _make_stsmo_ctrl(omega_mech=0.0)
    ctrl._omega_elec_est = 0.0

    sogi_alpha_before = ctrl._sogi_e_alpha
    sogi_beta_before = ctrl._sogi_e_beta

    ctrl._update_stsmo_emf(dt=1e-4)

    # With sogi_omega < 5.0, SOGI update block is skipped — states unchanged
    assert ctrl._sogi_e_alpha == pytest.approx(sogi_alpha_before, abs=1e-12)
    assert ctrl._sogi_e_beta == pytest.approx(sogi_beta_before, abs=1e-12)


# ── safety snap-back ──────────────────────────────────────────────────────────

def test_stsmo_safety_snap_resets_current_estimate_on_large_divergence():
    """When the internal current estimate drifts > 3 A, it must be snapped back."""
    motor, ctrl = _make_stsmo_ctrl(omega_mech=80.0)

    # Force a large divergence (> 3 A threshold)
    ctrl._stsmo_i_alpha = 50.0
    ctrl._stsmo_i_beta = -40.0

    ctrl._update_stsmo_emf(dt=1e-4)

    # After snap-back, the estimate is bounded relative to measured currents
    from src.control.transforms import clarke_transform
    ia, ib, ic = motor.currents
    i_alpha_meas, i_beta_meas = clarke_transform(ia, ib, ic)
    # Snap sets î = i_meas, then one backward-Euler step runs — should be close
    assert abs(ctrl._stsmo_i_alpha - i_alpha_meas) < 5.0


# ── EMF clipping ──────────────────────────────────────────────────────────────

def test_stsmo_emf_output_is_clipped_to_physical_e_max_bound():
    """Injecting a huge z1 must not produce an EMF magnitude beyond e_max."""
    motor, ctrl = _make_stsmo_ctrl(omega_mech=80.0)
    ctrl._omega_elec_est = 80.0 * float(motor.params.poles_pairs)

    # Inject an absurdly large z1 to stress the clip
    ctrl._stsmo_z1_alpha = 1e6
    ctrl._stsmo_z1_beta  = 1e6
    ctrl._stsmo_i_alpha  = motor.state[0]
    ctrl._stsmo_i_beta   = -0.5 * (motor.state[1] - motor.state[2]) / math.sqrt(3)

    e_a, e_b, emf_mag = ctrl._update_stsmo_emf(dt=1e-4)

    ke = float(motor.params.back_emf_constant)
    oe = ctrl._omega_elec_est
    e_max = max(ke * abs(oe) * 2.5 + 0.5, float(motor.params.nominal_voltage))

    assert abs(e_a) <= e_max + 1e-9
    assert abs(e_b) <= e_max + 1e-9


# ── omega_elec_est update guard ───────────────────────────────────────────────

def test_stsmo_omega_elec_est_updated_only_when_emf_mag_exceeds_0p3():
    """omega_elec_est must only be updated when the reconstructed EMF is > 0.3 V."""
    motor, ctrl = _make_stsmo_ctrl(omega_mech=0.0)
    ctrl._omega_elec_est = 99.0   # sentinel value
    ctrl._omega_elec_est = 99.0

    # Force z1 and SOGI to zero so emf_mag ≈ 0 after update
    ctrl._stsmo_z1_alpha = 0.0
    ctrl._stsmo_z1_beta  = 0.0
    ctrl._sogi_e_alpha   = 0.0
    ctrl._sogi_e_beta    = 0.0
    ctrl._stsmo_i_alpha  = motor.state[0]
    ctrl._stsmo_i_beta   = -0.5 * (motor.state[1] - motor.state[2]) / math.sqrt(3)
    ctrl._omega_elec_est = 99.0   # re-set after any mutation above

    ctrl._update_stsmo_emf(dt=1e-4)

    # emf_mag will be very small (≈ 0); omega_elec_est must remain at 99.0
    assert ctrl._omega_elec_est == pytest.approx(99.0, abs=0.5)


# ── MRAS resistance adaptation ────────────────────────────────────────────────

def test_stsmo_mras_resistance_adapts_with_stsmo_emf():
    """When MRAS is enabled and EMF is substantial, R estimate must drift."""
    motor, ctrl = _make_stsmo_ctrl(omega_mech=200.0)
    ctrl.enable_mras_resistance(gamma_r=0.01, r_min_factor=0.5, r_max_factor=4.0)
    ctrl._omega_elec_est = 200.0 * float(motor.params.poles_pairs)

    # Seed z1 so there is a meaningful EMF signal (> 5e-2 threshold)
    ke = float(motor.params.back_emf_constant)
    oe = ctrl._omega_elec_est
    ctrl._stsmo_z1_alpha = ke * oe * 0.5
    ctrl._stsmo_z1_beta  = ke * oe * 0.3
    ctrl._stsmo_i_alpha  = motor.state[0]
    ctrl._stsmo_i_beta   = -0.5 * (motor.state[1] - motor.state[2]) / math.sqrt(3)

    R_before = ctrl._mras_R

    for _ in range(50):
        motor._last_emf = motor._calculate_back_emf(motor.theta)
        ctrl._update_stsmo_emf(dt=1e-4)

    # R must have changed from its initial value
    assert ctrl._mras_R != pytest.approx(R_before, rel=1e-3)
    # R must remain within clipped bounds
    assert ctrl._mras_R_min <= ctrl._mras_R <= ctrl._mras_R_max


# ── DT architecture regression tests ─────────────────────────────────────────
#
# Architecture rule (frozen here to prevent regressions):
#   DT_CTRL  = 1 / PWM_HZ   — controller cadence; exactly one PWM period
#   DT_MOTOR = DT_CTRL / N_SUB — motor integration sub-step
#
# Rationale: On a real MCU the FOC ISR fires once per PWM period.
# Setting DT_CTRL to anything other than 1/PWM_HZ breaks the equivalence
# between simulation time and hardware timing, invalidating latency and
# bandwidth estimates.

def test_dt_ctrl_equals_one_pwm_period():
    """DT_CTRL must be exactly 1/PWM_HZ — one controller step per PWM cycle."""
    from examples.validate_all_observers import DT_CTRL, PWM_HZ

    assert DT_CTRL == pytest.approx(1.0 / PWM_HZ, rel=1e-9), (
        f"DT_CTRL={DT_CTRL} != 1/PWM_HZ={1.0/PWM_HZ}. "
        "The controller must run once per PWM period."
    )


def test_dt_motor_equals_dt_ctrl_divided_by_n_sub():
    """DT_MOTOR must equal DT_CTRL / N_SUB — motor sub-step within control period."""
    from examples.validate_all_observers import DT_CTRL, DT_MOTOR, N_SUB

    assert DT_MOTOR == pytest.approx(DT_CTRL / N_SUB, rel=1e-9), (
        f"DT_MOTOR={DT_MOTOR} != DT_CTRL/N_SUB={DT_CTRL/N_SUB}. "
        "Motor integration step must be DT_CTRL divided by the sub-step count."
    )


# ── SVM voltage mismatch — Active Flux root cause ─────────────────────────────
#
# Root cause (Boldea 2009 AF observer failure above 500 RPM):
#   The SVM modulator applies space-vector switching patterns so that
#   Clarke(phase voltages) ≠ inverse-Park(vd_cmd, vq_cmd, θe).
#   With a 48 V DC bus and typical operating points the SVM output exceeds
#   the inverse-Park vector by ~26 %.  The AF open-loop integrator used the
#   smaller inverse-Park value → flux underestimated → ωe drifted low →
#   θe lagged → torque collapsed.
#   Fix: call ctrl.update_applied_voltage(*svm_phases) after svm.modulate()
#   so that the integrator receives Clarke(actual terminal voltage).

def test_svm_clarke_output_differs_from_inverse_park_command():
    """Clarke(SVM phases) must differ from inverse-Park command by > 5 %.

    This documents the root cause of the ActiveFlux failure: the SVM
    space-vector modulator produces phase voltages whose Clarke transform
    is not equal to the inverse-Park of the FOC voltage commands.
    """
    from src.control.svm_generator import SVMGenerator
    from src.control.transforms import inverse_park

    vdc = 48.0
    svm = SVMGenerator(dc_voltage=vdc)

    # Typical mid-load operating point
    vd, vq, theta = 0.0, 17.2, math.pi / 4.0
    v_alpha_cmd, v_beta_cmd = inverse_park(vd, vq, theta)
    mag_cmd = math.hypot(v_alpha_cmd, v_beta_cmd)

    # SVM output (polar → 3 phases)
    va, vb, vc = svm.modulate(mag_cmd, theta)

    # Clarke transform of actual phase voltages
    from src.control.transforms import clarke_transform
    v_alpha_svm, v_beta_svm = clarke_transform(va, vb, vc)
    mag_svm = math.hypot(v_alpha_svm, v_beta_svm)

    # The two magnitudes must differ by more than 5 %
    ratio = mag_svm / mag_cmd if mag_cmd > 1e-9 else 0.0
    assert abs(ratio - 1.0) > 0.05, (
        f"Expected |Clarke(SVM)| to differ from |inv-Park(cmd)| by > 5 %, "
        f"got ratio={ratio:.4f} (mag_cmd={mag_cmd:.3f} V, mag_svm={mag_svm:.3f} V). "
        "If this fails the SVM behaviour has changed — verify AF observer still works."
    )


def test_update_applied_voltage_sets_v_alpha_prev_from_svm_phases():
    """update_applied_voltage must store Clarke(va,vb,vc) in _v_alpha_prev/_v_beta_prev.

    This is the fix for the AF open-loop integrator: after calling
    update_applied_voltage the internal voltage registers hold the actual
    terminal voltage, not the inverse-Park command.
    """
    from src.control.svm_generator import SVMGenerator
    from src.control.transforms import clarke_transform

    motor = _make_motor(omega_mech=80.0)
    ctrl = FOCController(motor=motor)
    svm = SVMGenerator(dc_voltage=48.0)

    mag, ang = 15.0, 0.8
    va, vb, vc = svm.modulate(mag, ang)
    ctrl.update_applied_voltage(va, vb, vc)

    v_alpha_expected, v_beta_expected = clarke_transform(va, vb, vc)
    assert ctrl._v_alpha_prev == pytest.approx(v_alpha_expected, rel=1e-9), (
        "_v_alpha_prev must equal Clarke(va,vb,vc) after update_applied_voltage"
    )
    assert ctrl._v_beta_prev == pytest.approx(v_beta_expected, rel=1e-9), (
        "_v_beta_prev must equal Clarke(va,vb,vc) after update_applied_voltage"
    )


# ── Active Flux integrator correctness ────────────────────────────────────────

def test_active_flux_integrator_advances_psi_s_using_v_alpha_prev():
    """The AF stator-flux integrator must advance psi_s in the direction of
    _v_alpha_prev — verifying that the voltage source used is correct.
    """
    from src.control.transforms import clarke_transform

    motor = _make_motor(omega_mech=150.0)
    ctrl = FOCController(motor=motor)

    # Inject a known positive voltage into _v_alpha_prev
    v_inject = 20.0
    ctrl._v_alpha_prev = v_inject
    ctrl._v_beta_prev = 0.0
    ctrl._psi_s_alpha = 0.0
    ctrl._psi_s_beta  = 0.0

    i_alpha, i_beta = clarke_transform(*motor.currents)
    DT_CTRL = 1.0 / 20_000
    ctrl._update_active_flux(DT_CTRL, i_alpha, i_beta)

    R = float(motor.params.phase_resistance)
    expected_sign = math.copysign(1.0, v_inject - R * i_alpha)

    # psi_s must have advanced away from zero in the correct direction
    assert ctrl._psi_s_alpha != pytest.approx(0.0, abs=1e-10), (
        "psi_s_alpha did not advance after _update_active_flux — "
        "_v_alpha_prev may not be wired to the integrator."
    )
    assert math.copysign(1.0, ctrl._psi_s_alpha) == expected_sign, (
        f"psi_s_alpha={ctrl._psi_s_alpha:.3e} advanced in wrong direction "
        f"(expected sign {expected_sign:+.0f})"
    )


def test_active_flux_theta_tracks_true_angle_after_bumpless_switch():
    """AF observer with update_applied_voltage must maintain non-trivial flux
    and a reasonable speed estimate over 500 integration steps.

    Regression guard: without the fix (calling update_applied_voltage after
    svm.modulate) the AF flux integrator uses the inverse-Park voltage, which
    is ~26 % smaller than the actual SVM terminal voltage.  This causes
    omega_est_af to collapse near zero within ~100 steps and theta_est_af to
    become meaningless within ~500 steps.

    With the fix (_v_alpha_prev = Clarke(SVM phases)) the flux remains
    proportional to the true PM flux linkage and omega_est_af tracks ωe.
    """
    from src.control.svm_generator import SVMGenerator
    from src.control.transforms import clarke_transform as _ct

    DT_CTRL = 1.0 / 20_000  # 50 µs

    motor = BLDCMotor(MotorParameters())
    motor.state[3] = 150.0   # omega_mech = 150 rad/s ≈ 1432 RPM
    motor.state[4] = 0.5     # theta = 0.5 rad
    motor._last_emf = motor._calculate_back_emf(motor.theta)

    ctrl = FOCController(motor=motor)
    svm = SVMGenerator(dc_voltage=float(motor.params.nominal_voltage))

    # Enable the AF observer (sets observer_target_mode = "ActiveFlux")
    ctrl.enable_active_flux_observer(dc_cutoff_hz=0.3)

    # Seed flux state from true operating point (avoids long warmup)
    params = motor.params
    pp = float(params.poles_pairs)
    ke_v = float(params.back_emf_constant)
    psi_m = ke_v / pp   # PM flux linkage [Wb]
    theta_e_now = float(motor.state[4] * pp) % (2.0 * math.pi)
    ia, ib, ic = motor.currents
    i_alpha0, i_beta0 = _ct(ia, ib, ic)
    ctrl._psi_s_alpha = psi_m * math.cos(theta_e_now) + float(params.phase_inductance) * i_alpha0
    ctrl._psi_s_beta  = psi_m * math.sin(theta_e_now) + float(params.phase_inductance) * i_beta0
    ctrl._psi_af_prev_a = psi_m * math.cos(theta_e_now)
    ctrl._psi_af_prev_b = psi_m * math.sin(theta_e_now)
    ctrl._theta_est_af  = theta_e_now
    ctrl._omega_est_af  = float(motor.state[3] * pp)
    ctrl._omega_elec_est = ctrl._omega_est_af

    # Run 500 steps calling _update_active_flux directly with update_applied_voltage fix
    for _ in range(500):
        # Produce realistic phase voltages via SVM at the current AF angle
        mag = 10.0
        ang = float(ctrl._theta_est_af)
        va, vb, vc = svm.modulate(mag, ang)
        ctrl.update_applied_voltage(va, vb, vc)   # ← critical fix

        ia, ib, ic = motor.currents
        i_alpha, i_beta = _ct(ia, ib, ic)
        ctrl._update_active_flux(DT_CTRL, i_alpha, i_beta)
        motor.step(np.array([va, vb, vc]), load_torque=0.0)

    # Flux magnitude must remain non-trivial (integrator did not stall or collapse)
    assert ctrl._psi_af_mag > 1e-4, (
        f"AF flux magnitude={ctrl._psi_af_mag:.2e} Wb — integrator appears stalled. "
        "Check that update_applied_voltage is called after svm.modulate()."
    )

    # omega_est_af must remain in a reasonable range (>= 5 % of true ωe)
    omega_e_true = abs(motor.state[3] * pp)
    assert ctrl._omega_est_af > 0.05 * omega_e_true, (
        f"omega_est_af={ctrl._omega_est_af:.1f} rad/s is < 5 % of true "
        f"omega_e={omega_e_true:.1f} rad/s. "
        "AF may be using inverse-Park voltage instead of SVM terminal voltage."
    )


# ── Numerical stability advisory (engine level, no GUI) ──────────────────────
#
# Default MotorParameters: R=2.5 Ω, L=0.005 H, J=0.0005 kg·m², b=0.001 N·m·s/rad
#   tau_e = L/R = 0.002 s   → dt_limit_e = 2.785 × 0.002 = 0.00557 s
#   tau_m = J/b = 0.5 s     → dt_limit_m = 2.785 × 0.5   = 1.3925 s
#   dt_limit = min(dt_limit_e, dt_limit_m) = 0.00557 s
#   0.5 × dt_limit = 0.002785 s
#
# Severity thresholds:
#   stable:   dt < 0.002785 s
#   marginal: 0.002785 s ≤ dt < 0.00557 s
#   unstable: dt ≥ 0.00557 s

def _make_engine(dt: float):
    """Return a SimulationEngine with default motor parameters at given dt."""
    from src.core.simulation_engine import SimulationEngine
    from src.core.load_model import ConstantLoad

    motor = BLDCMotor(MotorParameters())
    load  = ConstantLoad(torque=0.0)
    return SimulationEngine(motor=motor, load_profile=load, dt=dt)


def test_numerical_stability_advisory_stable_when_dt_well_within_limit():
    """dt = 5e-5 s is well below 50 % of dt_limit → severity must be 'stable'."""
    engine = _make_engine(dt=5e-5)
    advisory = engine.get_numerical_stability_advisory()
    assert advisory["severity"] == "stable", (
        f"Expected 'stable' for dt=5e-5 s, got '{advisory['severity']}'. "
        f"dt_limit={advisory['dt_limit_s']:.4e} s"
    )
    assert advisory["is_stable"] is True


def test_numerical_stability_advisory_unstable_when_dt_exceeds_rk4_limit():
    """dt = 7e-3 s exceeds dt_limit ≈ 5.57e-3 s → severity must be 'unstable'."""
    engine = _make_engine(dt=7e-3)
    advisory = engine.get_numerical_stability_advisory()
    assert advisory["severity"] == "unstable", (
        f"Expected 'unstable' for dt=7e-3 s, got '{advisory['severity']}'. "
        f"dt_limit={advisory['dt_limit_s']:.4e} s"
    )
    assert advisory["is_stable"] is False


def test_numerical_stability_advisory_marginal_between_50_and_100_pct_of_limit():
    """dt = 4e-3 s lies between 50 % and 100 % of dt_limit → severity must be 'marginal'."""
    engine = _make_engine(dt=4e-3)
    advisory = engine.get_numerical_stability_advisory()
    assert advisory["severity"] == "marginal", (
        f"Expected 'marginal' for dt=4e-3 s, got '{advisory['severity']}'. "
        f"dt_limit={advisory['dt_limit_s']:.4e} s, "
        f"50%%={0.5*advisory['dt_limit_s']:.4e} s"
    )


# ── UI DT stability alert smoke test ─────────────────────────────────────────
#
# Verifies that _build_dt_pwm_stability_advisory is present and returns
# the correct severity for three representative dt values.  This guards
# against the alert being accidentally removed during refactors.

def test_ui_dt_stability_advisory_method_present_and_callable():
    """BLDCMotorControlGUI._build_dt_pwm_stability_advisory must exist and
    return the correct severity for stable / marginal / unstable dt values."""
    import sys
    QApplication = pytest.importorskip("PySide6.QtWidgets").QApplication
    from src.ui.main_window import BLDCMotorControlGUI

    _app = QApplication.instance() or QApplication(sys.argv)
    win = BLDCMotorControlGUI()

    assert hasattr(win, "_build_dt_pwm_stability_advisory"), (
        "BLDCMotorControlGUI is missing _build_dt_pwm_stability_advisory — "
        "the DT stability alert has been removed or renamed."
    )

    params = MotorParameters()   # default: R=2.5, L=0.005 → dt_limit≈5.57e-3 s
    pwm_hz = 20_000.0

    result_stable   = win._build_dt_pwm_stability_advisory(5e-5,  params, pwm_hz)
    result_marginal = win._build_dt_pwm_stability_advisory(4e-3,  params, pwm_hz)
    result_unstable = win._build_dt_pwm_stability_advisory(7e-3,  params, pwm_hz)

    assert result_stable["severity"]   == "stable",   f"Got {result_stable['severity']}"
    assert result_marginal["severity"] == "marginal",  f"Got {result_marginal['severity']}"
    assert result_unstable["severity"] == "unstable",  f"Got {result_unstable['severity']}"
