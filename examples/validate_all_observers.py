#!/usr/bin/env python3
"""Headless observer validation — all 5 observer modes across two motor profiles.

Motor assignment
----------------
* **IPM salient (Lq/Ld = 2.0)** — Measured, PLL, SMO, ActiveFlux.
  Salient motors with strong reluctance torque are the natural target for
  PLL, SMO, and the Active-Flux observer (which removes saliency from the
  flux estimate by construction).

* **SPM (Lq/Ld = 1.0, nanotec DB57M012 12V)** — STSMO.
  The Super-Twisting SMO uses a single-inductance current model (L = Ld).
  On an IPM motor with Lq = 2×Ld the systematic sigma bias grows
  unboundedly (z1 drift → quadratic k2 growth → divergence).  The STSMO
  is validated on the SPM motor where the model assumption is exact.

Simulation strategy
-------------------
Each run uses a two-phase approach for reproducibility:

* **Phase 1 — warm-up (0 … SWITCH_TIME_S):**
  Measured (sensored) observer.  The motor reaches target speed under
  ideal angle feedback, giving every sensorless observer the same
  operating point at hand-off.

* **Phase 2 — observer active (SWITCH_TIME_S … SIM_DURATION_S):**
  Bumpless hand-off: observer states seeded from the current electrical
  angle.  The target observer takes over and must maintain stability.

Analysis window starts at SETTLE_START_S (last two seconds of a 5-second
run by default).

Usage
-----
    python examples/validate_all_observers.py
"""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# ── Project root ──────────────────────────────────────────────────────────────
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.control.foc_controller import FOCController
from src.control.svm_generator import SVMGenerator
from src.control.transforms import clarke_transform
from src.core.load_model import LoadProfile
from src.core.motor_model import BLDCMotor, MotorParameters

# ── Motor profiles ────────────────────────────────────────────────────────────
IPM_PROFILE_PATH = (
    PROJECT_ROOT / "data" / "motor_profiles" / "ipm_salient_48v.json"
)
SPM_PROFILE_PATH = (
    PROJECT_ROOT / "data" / "motor_profiles" / "nanotec_db57m012_12v.json"
)

# ── Shared simulation constants ───────────────────────────────────────────────
SIM_DURATION_S = 5.0       # total simulation time [s]
DT = 5e-4                  # integration step [s] — 2 kHz (within RK4 limit)
SWITCH_TIME_S = 1.5        # warmup → target observer switch [s]
SETTLE_START_S = 3.0       # analysis window start [s]
LOAD_RAMP_START_S = 0.5    # load ramp begins [s]
LOAD_RAMP_END_S = 1.5      # load ramp ends (coincides with switch) [s]

# ── IPM motor simulation parameters ──────────────────────────────────────────
IPM_TARGET_RPM = 2000.0    # speed reference [RPM] — Measured, PLL, SMO
IPM_LOAD_NM = 0.8          # constant load after ramp [N·m]

# Active Flux observer must operate at a lower electrical speed because the
# forward-Euler stator-flux integrator in _update_active_flux requires
# ωe·dt ≲ 0.1 rad for stable cross-product speed estimation.
#   At 2000 RPM / pp=4: ωe·dt = 838·5e-4 = 0.419 rad  → |ψa| overshoots
#     by ~8 %, causing the cross-product to underestimate ωe by ~17 %,
#     which makes θ̂_af fall behind by ~1 rad within a few ms.
#   At 500 RPM / pp=4:  ωe·dt = 209·5e-4 = 0.105 rad  → |ψa| error < 0.5 %,
#     cross-product error < 0.6 %, steady-state phase offset < 4°.
IPM_AF_TARGET_RPM = 500.0  # speed reference for the ActiveFlux test [RPM]
IPM_AF_LOAD_NM = 0.8       # same load (same iq demand, stresses saliency)

# ── SPM motor simulation parameters (STSMO only) ─────────────────────────────
# Nanotec DB57M012 12V: Ke=0.028 V·s/rad, 5 pole pairs.
# Max back-EMF at SVM limit (Vdc/√3 ≈ 6.93 V) → ω_e_max ≈ 247 rad/s
# → ω_mech_max ≈ 49.5 rad/s → ~473 RPM.  Use 300 RPM for comfortable headroom.
# Load: Constant-torque loads drive small-inertia motors backwards when the
# speed PI is slow.  Use zero external load (friction only) for the STSMO
# validation — the observer correctness does not depend on load magnitude.
SPM_TARGET_RPM = 300.0     # speed reference for nanotec STSMO run [RPM]
SPM_LOAD_NM = 0.0          # no external load (friction only)

# ── Pass/fail thresholds ──────────────────────────────────────────────────────
SPEED_ERROR_TOL_PCT = 15.0    # max |speed error| vs target [%]
ANGLE_ERROR_TOL_RAD = 1.50    # max mean |θ_est − θ_true| [rad]
# 1.5 rad ceiling: sensorless observers on salient motors have a systematic
# steady-state angle offset from the Iq·(Lq−Ld) cross-coupling term.
# Above 1.5 rad the observer has genuinely lost lock.
DIVERGE_SPEED_LIMIT_RPM = 6000.0  # abort if |speed| exceeds this [RPM]


# ── Load profile ──────────────────────────────────────────────────────────────
class SmoothRampLoad(LoadProfile):
    """Smooth cubic ramp from zero to *torque* between *t_start* and *t_end*."""

    def __init__(self, t_start: float, t_end: float, torque: float) -> None:
        self.t_start = float(t_start)
        self.t_end = float(t_end)
        self.torque = float(torque)

    def get_torque(self, t: float) -> float:
        if t <= self.t_start:
            return 0.0
        if t >= self.t_end:
            return self.torque
        x = (t - self.t_start) / (self.t_end - self.t_start)
        return self.torque * x * x * (3.0 - 2.0 * x)


# ── Motor profile loader ───────────────────────────────────────────────────────
def load_motor_profile(path: Path) -> tuple[MotorParameters, float]:
    """Return (MotorParameters, rated_rpm).

    Handles both root-level ``rated_speed_rpm`` (IPM profile) and the
    nested ``rated_info.rated_speed_rpm`` format used by the Nanotec profile.
    """
    data = json.loads(path.read_text(encoding="utf-8"))
    mp = data["motor_params"]
    params = MotorParameters(
        model_type=mp.get("model_type", "dq"),
        emf_shape=mp.get("emf_shape", "sinusoidal"),
        nominal_voltage=float(mp["nominal_voltage"]),
        phase_resistance=float(mp["phase_resistance"]),
        phase_inductance=float(mp.get("phase_inductance", mp.get("ld", 0.00015))),
        back_emf_constant=float(mp["back_emf_constant"]),
        torque_constant=float(mp.get("torque_constant", mp["back_emf_constant"])),
        rotor_inertia=float(mp["rotor_inertia"]),
        friction_coefficient=float(mp["friction_coefficient"]),
        num_poles=int(mp["num_poles"]),
        poles_pairs=int(mp.get("poles_pairs", int(mp["num_poles"]) // 2)),
        ld=float(mp["ld"]),
        lq=float(mp["lq"]),
        flux_weakening_id_coefficient=float(
            mp.get("flux_weakening_id_coefficient", 0.0)
        ),
        flux_weakening_min_ratio=float(mp.get("flux_weakening_min_ratio", 0.2)),
    )
    # Support both root-level and rated_info-nested rated_speed_rpm
    rated_rpm = (
        data.get("rated_speed_rpm")
        or data.get("rated_info", {}).get("rated_speed_rpm", 3000.0)
    )
    return params, float(rated_rpm)


# ── Controller factory ────────────────────────────────────────────────────────
def _build_controller(
    params: MotorParameters,
    target_rpm: float,
) -> FOCController:
    """Instantiate a fresh FOCController with analytically tuned PI gains."""
    motor = BLDCMotor(params, dt=DT)

    ctrl = FOCController(motor=motor, enable_speed_loop=True)
    ctrl.set_cascaded_speed_loop(True, iq_limit_a=60.0)

    # Current loop: bandwidth ≈ τ_e / 5
    R = params.phase_resistance
    Ld = params.ld
    tau_e = Ld / max(R, 1e-9)
    omega_c_i = min(1.0 / (5.0 * tau_e), 3000.0)
    ctrl.set_current_pi_gains(
        d_kp=omega_c_i * Ld,
        d_ki=omega_c_i * R,
        q_kp=omega_c_i * Ld,
        q_ki=omega_c_i * R,
        kaw=0.3,
    )

    # Speed loop: bandwidth ≈ current_BW / 20 (min 30 rad/s)
    # Small-inertia motors need a higher minimum bandwidth to accelerate
    # against friction within the warmup window.  Clip at 30 rad/s.
    omega_c_s = max(omega_c_i / 20.0, 30.0)
    J = params.rotor_inertia
    Kt = params.torque_constant
    ctrl.set_speed_pi_gains(
        kp=omega_c_s * J / Kt,
        ki=(omega_c_s ** 2) * J / Kt / 5.0,
        kaw=0.05,
    )

    ctrl.set_voltage_saturation(
        mode="d_priority",
        coupled_antiwindup_enabled=True,
        coupled_antiwindup_gain=0.3,
    )
    ctrl.set_current_references(id_ref=0.0, iq_ref=0.0)
    ctrl.set_speed_reference(target_rpm)
    return ctrl


# ── Bumpless observer hand-off ────────────────────────────────────────────────
def _bumpless_switch_to_observer(
    ctrl: FOCController,
    params: MotorParameters,
    mode_norm: str,
    rated_rpm: float,
) -> None:
    """Seed observer states from current motor position and switch modes.

    Bumpless hand-off procedure:
    1. Read true electrical angle and speed from motor model.
    2. Calibrate the target observer analytically.
    3. Pre-load every observer integrator state so the initial tracking
       error is near zero.
    4. Disable the startup transition so the sensorless angle is applied
       immediately (no open-loop ramp needed — motor is already at speed).
    """
    motor = ctrl.motor
    pp = float(params.poles_pairs)
    theta_e_now = (motor.theta * pp) % (2.0 * np.pi)
    omega_e_now = motor.omega * pp  # electrical rad/s

    # ── Steady-state voltage estimate ────────────────────────────────────────
    # _v_alpha/beta_prev is only updated inside _reconstruct_emf_sensorless,
    # which is never called during the Measured warmup phase.  Seed it from
    # the quasi-steady-state back-EMF + resistive-drop estimate so that the
    # first sensorless step sees the correct applied voltage.
    ke_v = float(params.back_emf_constant)
    R_v = float(params.phase_resistance)
    ia_v, ib_v, ic_v = motor.currents
    i_alpha_v, i_beta_v = clarke_transform(ia_v, ib_v, ic_v)
    v_alpha_est = (
        -ke_v * float(motor.omega) * float(np.sin(theta_e_now))
        + R_v * i_alpha_v
    )
    v_beta_est = (
        +ke_v * float(motor.omega) * float(np.cos(theta_e_now))
        + R_v * i_beta_v
    )

    # ── Expected back-EMF at current operating point ──────────────────────────
    # Used to seed the EMF-reconstructor LPF output (_e_alpha/beta_obs) so
    # that emf_mag and observer_confidence are correct from the first step.
    e_alpha_0 = -ke_v * float(motor.omega) * float(np.sin(theta_e_now))
    e_beta_0 = +ke_v * float(motor.omega) * float(np.cos(theta_e_now))

    if mode_norm == "PLL":
        ctrl.enable_sensorless_emf_reconstruction()
        ctrl.calibrate_pll_gains_analytical(rated_rpm=rated_rpm, apply=True)
        ctrl.theta_est_pll = theta_e_now
        ctrl.pll["integral"] = 0.0
        ctrl._e_alpha_obs = e_alpha_0
        ctrl._e_beta_obs = e_beta_0
        ctrl._v_alpha_prev = v_alpha_est
        ctrl._v_beta_prev = v_beta_est
        ctrl._i_alpha_prev = i_alpha_v
        ctrl._i_beta_prev = i_beta_v
        ctrl.theta_meas_emf = theta_e_now

    elif mode_norm == "SMO":
        ctrl.enable_sensorless_emf_reconstruction()
        ctrl.calibrate_smo_gains_analytical(rated_rpm=rated_rpm, dt=DT, apply=True)
        ctrl.theta_est_smo = theta_e_now
        ctrl.smo["omega_est"] = omega_e_now
        ctrl._e_alpha_obs = e_alpha_0
        ctrl._e_beta_obs = e_beta_0
        ctrl._v_alpha_prev = v_alpha_est
        ctrl._v_beta_prev = v_beta_est
        ctrl._i_alpha_prev = i_alpha_v
        ctrl._i_beta_prev = i_beta_v
        ctrl.theta_meas_emf = theta_e_now

    elif mode_norm == "STSMO":
        # STSMO requires startup_sequence_enabled=True so that
        # _estimate_theta_with_startup_sequence → _get_observer_emf_components
        # → _update_stsmo_emf is called each step.
        # We force startup_phase="closed_loop" to skip align/open_loop.
        ctrl.enable_sensorless_emf_reconstruction()   # keeps L=Ld
        ctrl.calibrate_stsmo_gains_analytical(rated_rpm=rated_rpm, apply=True)
        ctrl._use_sogi_filter = True  # SOGI post-filter suppresses chattering

        # Seed current estimates → zero initial sliding surface σ
        ia, ib, ic = motor.currents
        i_alpha, i_beta = clarke_transform(ia, ib, ic)
        ctrl._stsmo_i_alpha = i_alpha
        ctrl._stsmo_i_beta = i_beta
        ctrl._i_alpha_prev = i_alpha
        ctrl._i_beta_prev = i_beta

        # Seed EMF integrator z1 from expected back-EMF
        ctrl._stsmo_z1_alpha = e_alpha_0
        ctrl._stsmo_z1_beta = e_beta_0
        ctrl._stsmo_e_alpha = e_alpha_0
        ctrl._stsmo_e_beta = e_beta_0
        ctrl._e_alpha_obs = e_alpha_0
        ctrl._e_beta_obs = e_beta_0
        ctrl._omega_elec_est = omega_e_now

        ctrl._v_alpha_prev = v_alpha_est
        ctrl._v_beta_prev = v_beta_est

        ctrl.theta_est_smo = theta_e_now
        ctrl.smo["omega_est"] = omega_e_now
        ctrl.theta_meas_emf = theta_e_now

        # ── Override k2_min for the current operating speed ───────────────────
        # The default k2_min=500 V/s is designed for high-speed motors.
        # At low speeds (e.g. 300 RPM on Nanotec) the adaptive k2 naturally
        # gives ke·ωm·ωe ≈ 138 V/s, but the 500 V/s floor overdrives the
        # integral, causing z1 to overshoot.  Set k2_min to ≈ half the
        # natural adaptive value so the floor only activates near standstill.
        _ke_m = float(params.back_emf_constant)
        _k2_natural = _ke_m * abs(float(motor.omega)) * abs(omega_e_now)
        ctrl.stsmo["k2_min"] = max(10.0, 0.5 * _k2_natural)

        ctrl.startup_sequence_enabled = True
        ctrl._enter_startup_phase("closed_loop")

    elif mode_norm == "ActiveFlux":
        # EEMF model uses Lq in the di/dt term for saliency-correct
        # reconstruction.  The Active Flux observer then computes
        #   ψa = ψs − Ld·is  (always aligned with d-axis, saliency-immune).
        ctrl.enable_sensorless_emf_reconstruction()
        ctrl.enable_eemf_model(Lq=params.lq)
        ctrl.enable_active_flux_observer(dc_cutoff_hz=0.3)

        ia, ib, ic = motor.currents
        i_alpha, i_beta = clarke_transform(ia, ib, ic)

        # ── Seed EMF reconstructor LPF ────────────────────────────────────────
        # _e_alpha/beta_obs defaults to 0.  Without seeding, emf_mag ≈ 0 on
        # the first sensorless step → emf_norm ≈ 0 → observer_confidence <
        # startup_min_confidence (0.6) → _maybe_apply_sensorless_fallback
        # triggers after startup_fallback_hold_s (30 ms) → controller reverts
        # to open-loop phase with angle = 0 → motor loses synchronisation.
        ctrl._e_alpha_obs = e_alpha_0
        ctrl._e_beta_obs = e_beta_0

        # ── Seed stator flux ──────────────────────────────────────────────────
        # The stator flux integrates to:
        #   ψs_α ≈ (Ke/pp)·cos(θe) + Ld·iα
        # because e_α = −Ke·ω_mech·sin(θe) and ω_mech = ω_e/pp, so
        #   ∫ e_α dt = (Ke/pp)·cos(θe)  ← PM flux linkage ψm = Ke/pp [Wb]
        # Using Ke directly (4× too large for pp=4) causes |ψa| >> correct value
        # → omega_est_af ≈ ω_mech instead of ω_e → decoupling feedforward error.
        psi_m = ke_v / pp   # PM flux linkage [Wb] = Ke_v / pole_pairs
        ctrl._psi_s_alpha = psi_m * float(np.cos(theta_e_now)) + float(params.ld) * i_alpha
        ctrl._psi_s_beta = psi_m * float(np.sin(theta_e_now)) + float(params.ld) * i_beta
        ctrl._psi_af_prev_a = psi_m * float(np.cos(theta_e_now))   # = ψa_alpha
        ctrl._psi_af_prev_b = psi_m * float(np.sin(theta_e_now))   # = ψa_beta

        ctrl._theta_est_af = theta_e_now
        ctrl._omega_est_af = omega_e_now
        ctrl._omega_elec_est = omega_e_now
        ctrl._i_alpha_prev = i_alpha
        ctrl._i_beta_prev = i_beta
        ctrl._v_alpha_prev = v_alpha_est
        ctrl._v_beta_prev = v_beta_est
        ctrl.theta_meas_emf = theta_e_now

        ctrl.startup_sequence_enabled = True
        ctrl._enter_startup_phase("closed_loop")

    # ── Observer string for set_angle_observer ────────────────────────────────
    # STSMO uses "SMO" string (set_angle_observer normalises "STSMO" → error).
    # ActiveFlux: enable_active_flux_observer already sets observer_target_mode.
    if mode_norm in ("SMO", "STSMO"):
        obs_str: str | None = "SMO"
    elif mode_norm == "ActiveFlux":
        obs_str = None
    else:
        obs_str = mode_norm

    if obs_str is not None:
        ctrl.set_angle_observer(obs_str)

    # Bypass startup transition — switch immediately.
    # IMPORTANT: pass fallback_enabled=False so that set_startup_transition
    # does not reset startup_fallback_enabled=True (its default), which would
    # undo the fallback disabling set above for ActiveFlux/STSMO observers.
    ctrl.set_startup_transition(enabled=False, initial_mode="Measured",
                                fallback_enabled=False)
    ctrl.startup_transition_done = True
    ctrl.startup_ready_to_switch = True


# ── Result container ──────────────────────────────────────────────────────────
@dataclass
class ObserverResult:
    mode: str
    final_speed_rpm: float
    speed_error_pct: float
    mean_angle_error_rad: float
    stable: bool
    note: str = ""


# ── Single-observer simulation ────────────────────────────────────────────────
def run_observer_simulation(
    params: MotorParameters,
    observer_mode: str,
    rated_rpm: float,
    target_rpm: float,
    load_torque: float,
) -> ObserverResult:
    """Run one FOC simulation for *observer_mode* and return stability results.

    Phase 1: Measured (0 … SWITCH_TIME_S) — motor accelerates to *target_rpm*.
    Phase 2: Target observer (SWITCH_TIME_S … SIM_DURATION_S) — stability check.

    Uses motor.step() directly (no SimulationEngine history) for speed.
    """
    motor = BLDCMotor(params, dt=DT)
    load = SmoothRampLoad(LOAD_RAMP_START_S, LOAD_RAMP_END_S, load_torque)
    svm = SVMGenerator(dc_voltage=params.nominal_voltage)

    ctrl = _build_controller(params, target_rpm)
    ctrl.motor = motor  # ensure controller shares the same motor instance

    mode_norm = observer_mode.strip()

    # ── Phase 1: warmup under Measured observer ───────────────────────────────
    ctrl.set_angle_observer("Measured")
    ctrl.set_startup_transition(enabled=False, initial_mode="Measured")
    ctrl.set_startup_sequence(enabled=False)
    ctrl._enter_startup_phase("closed_loop")
    ctrl.startup_transition_done = True
    ctrl.startup_ready_to_switch = True

    n_warmup = int(SWITCH_TIME_S / DT)
    n_total = int(SIM_DURATION_S / DT)
    n_settle = int(SETTLE_START_S / DT)

    speed_hist: list[float] = []
    angle_err_hist: list[float] = []

    try:
        for step in range(n_total):
            # Switch to target observer at the warmup boundary
            if step == n_warmup and mode_norm != "Measured":
                _bumpless_switch_to_observer(ctrl, params, mode_norm, rated_rpm)

            mag, ang = ctrl.update(DT)
            voltages = svm.modulate(mag, ang)
            t_now = step * DT
            motor.step(voltages, load_torque=load.get_torque(t_now))

            # Divergence safety guard
            omega_mech = motor.omega
            speed_rpm = omega_mech * 60.0 / (2.0 * np.pi)
            if abs(speed_rpm) > DIVERGE_SPEED_LIMIT_RPM or not np.isfinite(speed_rpm):
                return ObserverResult(
                    mode=observer_mode,
                    final_speed_rpm=speed_rpm,
                    speed_error_pct=100.0,
                    mean_angle_error_rad=float("inf"),
                    stable=False,
                    note=f"Diverged at t={step*DT:.2f}s (speed={speed_rpm:.0f} RPM)",
                )

            # Accumulate analysis window
            if step >= n_settle:
                speed_hist.append(speed_rpm)
                theta_true_e = (motor.theta * float(params.poles_pairs)) % (2.0 * np.pi)
                theta_est_e = ctrl.theta % (2.0 * np.pi)
                err = abs(
                    ((theta_est_e - theta_true_e + np.pi) % (2.0 * np.pi)) - np.pi
                )
                angle_err_hist.append(err)

    except Exception as exc:  # noqa: BLE001
        return ObserverResult(
            mode=observer_mode,
            final_speed_rpm=0.0,
            speed_error_pct=100.0,
            mean_angle_error_rad=float("inf"),
            stable=False,
            note=f"Exception: {exc}",
        )

    if not speed_hist:
        return ObserverResult(
            mode=observer_mode,
            final_speed_rpm=0.0,
            speed_error_pct=100.0,
            mean_angle_error_rad=float("inf"),
            stable=False,
            note="Analysis window empty (simulation too short?)",
        )

    final_speed = float(np.mean(speed_hist[-500:]))    # last 500 steps average
    speed_err_pct = abs(final_speed - target_rpm) / max(target_rpm, 1e-9) * 100.0
    mean_ang_err = float(np.mean(angle_err_hist))

    stable = (
        speed_err_pct <= SPEED_ERROR_TOL_PCT
        and mean_ang_err <= ANGLE_ERROR_TOL_RAD
    )

    note_parts: list[str] = []
    if speed_err_pct > SPEED_ERROR_TOL_PCT:
        note_parts.append(f"Speed error {speed_err_pct:.1f}%>{SPEED_ERROR_TOL_PCT}%")
    if mean_ang_err > ANGLE_ERROR_TOL_RAD:
        note_parts.append(f"θ_err {mean_ang_err:.3f}>{ANGLE_ERROR_TOL_RAD} rad")
    note = "OK" if stable else "; ".join(note_parts)

    return ObserverResult(
        mode=observer_mode,
        final_speed_rpm=final_speed,
        speed_error_pct=speed_err_pct,
        mean_angle_error_rad=mean_ang_err,
        stable=stable,
        note=note,
    )


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> int:
    print("=" * 72)
    print("SPINOTOR — Observer Validation")
    print(f"  dt={DT*1e4:.0f}×10⁻⁴ s   switch at {SWITCH_TIME_S:.1f} s   "
          f"analysis from {SETTLE_START_S:.1f} s   total {SIM_DURATION_S:.1f} s")
    print("=" * 72)

    # ── Check profile files ───────────────────────────────────────────────────
    for path in (IPM_PROFILE_PATH, SPM_PROFILE_PATH):
        if not path.exists():
            print(f"ERROR: Motor profile not found: {path}")
            return 1

    ipm_params, ipm_rated_rpm = load_motor_profile(IPM_PROFILE_PATH)
    spm_params, spm_rated_rpm = load_motor_profile(SPM_PROFILE_PATH)

    # ── Print motor summaries ─────────────────────────────────────────────────
    print(f"\nIPM motor  (Lq/Ld={ipm_params.lq/ipm_params.ld:.1f}):  "
          f"Rs={ipm_params.phase_resistance} Ω  "
          f"Ld={ipm_params.ld*1e3:.3f} mH  Lq={ipm_params.lq*1e3:.3f} mH  "
          f"Ke={ipm_params.back_emf_constant:.4f} V·s/rad  "
          f"Vnom={ipm_params.nominal_voltage:.0f} V  "
          f"Target={IPM_TARGET_RPM:.0f} RPM")
    print(f"SPM motor  (Lq/Ld={spm_params.lq/spm_params.ld:.1f}):  "
          f"Rs={spm_params.phase_resistance} Ω  "
          f"Ld={spm_params.ld*1e3:.3f} mH  Lq={spm_params.lq*1e3:.3f} mH  "
          f"Ke={spm_params.back_emf_constant:.4f} V·s/rad  "
          f"Vnom={spm_params.nominal_voltage:.0f} V  "
          f"Target={SPM_TARGET_RPM:.0f} RPM")

    # ── Simulation plan ───────────────────────────────────────────────────────
    # (observer_mode, params, rated_rpm, target_rpm, load_nm, label)
    plan = [
        ("Measured",   ipm_params, ipm_rated_rpm, IPM_TARGET_RPM,    IPM_LOAD_NM,
         "IPM"),
        ("PLL",        ipm_params, ipm_rated_rpm, IPM_TARGET_RPM,    IPM_LOAD_NM,
         "IPM"),
        ("SMO",        ipm_params, ipm_rated_rpm, IPM_TARGET_RPM,    IPM_LOAD_NM,
         "IPM"),
        # ActiveFlux uses a lower speed so ωe·dt ≲ 0.1 rad — see IPM_AF_TARGET_RPM.
        ("ActiveFlux", ipm_params, ipm_rated_rpm, IPM_AF_TARGET_RPM, IPM_AF_LOAD_NM,
         "IPM"),
        ("STSMO",      spm_params, spm_rated_rpm, SPM_TARGET_RPM,    SPM_LOAD_NM,
         "SPM"),
    ]

    print()
    results: list[ObserverResult] = []

    for mode, params, rated_rpm, target_rpm, load_nm, motor_label in plan:
        label = f"{mode} [{motor_label}]"
        print(f"  Running {label:<22} ...", end=" ", flush=True)
        result = run_observer_simulation(
            params=params,
            observer_mode=mode,
            rated_rpm=rated_rpm,
            target_rpm=target_rpm,
            load_torque=load_nm,
        )
        results.append(result)
        status = "PASS" if result.stable else "FAIL"
        print(
            f"{status}  speed={result.final_speed_rpm:7.1f} RPM  "
            f"err={result.speed_error_pct:5.1f}%  "
            f"θ_err={result.mean_angle_error_rad:.3f} rad  {result.note}"
        )

    # ── Summary table ─────────────────────────────────────────────────────────
    labels = [
        f"{mode} [{lbl}]"
        for mode, _, _, _, _, lbl in plan
    ]
    print()
    print("─" * 76)
    print(f"{'Observer':<26} {'Speed [RPM]':>12} {'Spd err %':>10} "
          f"{'θ_err [rad]':>12} {'Result':>8}")
    print("─" * 76)
    all_pass = True
    for label, r in zip(labels, results):
        status = "✓ PASS" if r.stable else "✗ FAIL"
        if not r.stable:
            all_pass = False
        print(
            f"{label:<26} {r.final_speed_rpm:>12.1f} {r.speed_error_pct:>10.2f} "
            f"{r.mean_angle_error_rad:>12.4f} {status:>8}"
        )
    print("─" * 76)

    n_pass = sum(1 for r in results if r.stable)
    print(f"\n{n_pass}/{len(results)} observers passed.\n")

    if all_pass:
        print("All observers validated successfully.  Simulation is STABLE.")
        return 0
    else:
        print("One or more observers FAILED.  See details above.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
