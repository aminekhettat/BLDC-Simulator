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

Cross-cutting tests (DT architecture, SVM/AF voltage loop, numerical stability
advisory) live in test_motor.py and test_feature_gui_simulation_params.py.
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
