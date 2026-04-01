"""Observer validation — truly sensorless PLL and SMO.

Runs 4 seconds of simulation on the Nanotec DB57M012 12V motor.
Validates that both PLL and SMO observers (in sensorless EMF reconstruction
mode) track the true motor electrical angle, speed, and back-EMF.

Speed profile:
  0 → 1.0 s  : ramp 0 → 1500 RPM
  1.0 → 2.0 s: hold 1500 RPM
  2.0 → 3.0 s: ramp 1500 → 3000 RPM (enters FW zone at ~2363 RPM)
  3.0 → 4.0 s: hold 3000 RPM
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── Paths ─────────────────────────────────────────────────────────────────────
ROOT = Path("/sessions/vibrant-kind-gates/mnt/BLDC-Simulator")
PROFILE_PATH = ROOT / "data/motor_profiles/nanotec_db57m012_12v.json"
OUT_DIR = Path("/sessions/vibrant-kind-gates/sim_results_observer")
OUT_DIR.mkdir(parents=True, exist_ok=True)

sys.path.insert(0, str(ROOT))

from src.control import FOCController, SVMGenerator
from src.control.field_weakening_calibrator import FieldWeakeningCalibrator
from src.core import BLDCMotor, ConstantLoad, MotorParameters, SimulationEngine

# ── Motor profile ──────────────────────────────────────────────────────────────
profile = json.loads(PROFILE_PATH.read_text())
mp = profile["motor_params"]
ri = profile["rated_info"]

R       = float(mp["phase_resistance"])
L_val   = float(mp["phase_inductance"])
Ke      = float(mp["back_emf_constant"])
Kt      = float(mp["torque_constant"])
Pp      = int(mp["poles_pairs"])
J       = float(mp["rotor_inertia"])
B_fric  = float(mp["friction_coefficient"])
V_NOM   = float(mp["nominal_voltage"])

rated_rpm = float(ri["rated_speed_rpm"])
V_LIM = V_NOM / math.sqrt(3.0)
tau_e = L_val / R

print(f"Motor: {profile.get('profile_name','Nanotec')}")
print(f"  R={R} Ω  L={L_val*1e3:.3f} mH  Ke={Ke} V·s/rad  Pp={Pp}")
print(f"  V_dc={V_NOM} V  V_lim={V_LIM:.3f} V  τ_e={tau_e*1e3:.3f} ms")
print(f"  FW base speed ≈ {V_LIM/Ke*30/math.pi:.0f} RPM  rated={rated_rpm:.0f} RPM")

# ── Analytical FW calibration ──────────────────────────────────────────────────
print("\n[1] Analytical FW calibration...")
cal = FieldWeakeningCalibrator(
    motor_params_dict=mp, rated_info=ri,
    fw_current_fraction=0.25, safety_factor=1.15,
    sim_duration=2.0, verbose=False,
)
physics = cal.compute_physics_params()
k_fw      = physics.flux_weakening_id_coefficient
min_ratio = physics.flux_weakening_min_ratio
fw_gain   = physics.fw_gain
fw_id_max = 0.25 * float(ri["rated_current_a"])
fw_start  = min(physics.fw_start_rpm, 1200.0)
headroom  = physics.fw_headroom_target_v
print(f"  FW start={fw_start:.0f} RPM  gain={fw_gain:.4f}  id_max={fw_id_max:.2f}A")

# ── Simulation parameters ──────────────────────────────────────────────────────
DT      = 1e-4       # 100 µs — one PWM period
T_SIM   = 4.0        # 4 s simulation
N_STEPS = int(T_SIM / DT)

params = MotorParameters(
    nominal_voltage       = V_NOM,
    phase_resistance      = R,
    phase_inductance      = L_val,
    back_emf_constant     = Ke,
    torque_constant       = Kt,
    rotor_inertia         = J,
    friction_coefficient  = B_fric,
    num_poles             = int(mp["num_poles"]),
    poles_pairs           = Pp,
    ld                    = float(mp["ld"]),
    lq                    = float(mp["lq"]),
    model_type            = mp["model_type"],
    emf_shape             = mp["emf_shape"],
    flux_weakening_id_coefficient = k_fw,
    flux_weakening_min_ratio      = min_ratio,
)

# Speed profile array
t_vec = np.arange(N_STEPS, dtype=np.float64) * DT
speed_ref = np.where(
    t_vec < 1.0,  t_vec / 1.0 * 1500.0,
    np.where(
        t_vec < 2.0, 1500.0,
        np.where(
            t_vec < 3.0, 1500.0 + (t_vec - 2.0) / 1.0 * 1500.0,
            3000.0
        )
    )
)

# ── Helper: build motor+engine+controller for one observer mode ───────────────
def build_sim(observer_mode: str):
    motor  = BLDCMotor(params, dt=DT)
    load   = ConstantLoad(torque=0.0)
    engine = SimulationEngine(motor, load, dt=DT, max_history=200)
    svm    = SVMGenerator(dc_voltage=V_NOM)
    ctrl   = FOCController(motor=motor, enable_speed_loop=True)

    # Current PI: analytically tuned
    i_kp = R * 1.5
    i_ki = R / L_val * 0.3
    ctrl.set_current_pi_gains(d_kp=i_kp, d_ki=i_ki, q_kp=i_kp, q_ki=i_ki)
    ctrl.configure_mcu_timing(pwm_freq_hz=1.0 / DT, speed_loop_hz=100.0)
    ctrl.set_field_weakening(
        enabled=True, start_speed_rpm=fw_start, gain=fw_gain,
        max_negative_id_a=fw_id_max, headroom_target_v=headroom,
    )

    # Observer mode
    ctrl.observer_target_mode = observer_mode
    ctrl.angle_observer_mode  = observer_mode

    # ── Analytical calibration ──────────────────────────────────────────────
    if observer_mode == "PLL":
        # Compute omega_e_max at rated speed
        omega_e_max = rated_rpm * math.pi / 30.0 * Pp
        # Use omega_n = omega_e_max / 10 for stability at low transition speed.
        # At transition (800 RPM): omega_elec = 419 rad/s >> omega_n = 183 rad/s.
        # The /5 rule is the *upper bound* at rated speed; /10 is safer for
        # wide-speed-range operation and prevents instability at low speed.
        omega_n = omega_e_max / 10.0
        zeta    = 0.9
        kp_pll  = 2.0 * zeta * omega_n
        ki_pll  = omega_n ** 2
        ctrl.set_pll_gains(kp=kp_pll, ki=ki_pll)
        print(f"  PLL: kp={kp_pll:.3f}  ki={ki_pll:.2f}  "
              f"ωn={omega_n:.2f} rad/s  (ωe_max={omega_e_max:.0f} rad/s, /10)")
    elif observer_mode == "SMO":
        res = ctrl.calibrate_smo_gains_analytical(rated_rpm=rated_rpm, dt=DT, apply=True)
        print(f"  SMO: k_slide={res['k_slide']:.1f}  "
              f"lpf_alpha={res['lpf_alpha']:.5f}  boundary={res['boundary']:.3f}  "
              f"τe={res['tau_e_s']*1e3:.2f} ms")

    # ── Enable sensorless EMF reconstruction ────────────────────────────────
    # LPF tau must be much shorter than the electrical period at rated speed.
    # tau_e = L/R = 1.25 ms;  electrical period at rated speed = 1/292 Hz = 3.4 ms.
    # Use tau_lpf = DT (alpha=0.5): ~10° phase lag at rated speed, no amplitude attenuation.
    ctrl.enable_sensorless_emf_reconstruction(
        R=R, L=L_val, lpf_tau_s=DT, use_estimated_speed_ff=True,
    )
    ctrl.vdq_limit = V_LIM

    # Speed PI: same gains used in validated 1500 RPM auto-tune session
    ctrl.set_speed_pi_gains(kp=0.045, ki=0.0025)

    # ── Startup sequence: align → open-loop ramp → sensorless observer ──────
    # Sensorless can't start from zero (no EMF at rest).  The startup sequence
    # aligns the rotor to a known angle, then ramps open-loop to ~400 RPM where
    # the back-EMF (≈1.2V) is large enough for the observer to lock on.
    ctrl.set_startup_sequence(
        enabled=True,
        align_duration_s=0.08,              # 80 ms rotor alignment
        align_current_a=2.0,                # 2 A alignment current (< rated 5 A)
        align_angle_deg=0.0,                # align rotor to phase-A axis
        open_loop_initial_speed_rpm=30.0,
        open_loop_target_speed_rpm=800.0,   # ramp to 800 RPM before handoff
        open_loop_ramp_time_s=0.50,         # 500 ms ramp → smoother acceleration
        open_loop_id_ref_a=0.0,
        open_loop_iq_ref_a=0.50,            # 0.50 A: enough to overcome friction at 800 RPM
        # Friction at 800 RPM = B·ω = 1e-4×83.8 ≈ 8.4 mNm → iq_fric≈0.30 A.
        # 0.50 A gives net acceleration torque even at 1300 RPM (safety margin).
        # With update_applied_voltage() wiring the actual SVM output to the EMF
        # reconstructor, motor angle overshoot no longer corrupts the voltage
        # reference — the α-β voltage stored is always in the stator frame.
    )
    # Transition to closed-loop observer when speed and EMF are sufficient.
    # At 700 RPM: omega_elec = 367 rad/s >> PLL omega_n = 183 rad/s (stable).
    # EMF at 700 RPM ≈ 1.7V (strong enough for reliable reconstruction).
    ctrl.set_startup_transition(
        enabled=True,
        initial_mode="Measured",            # fallback mode if observer loses lock
        min_speed_rpm=700.0,                # ω_elec = 367 rad/s >> PLL BW = 183 rad/s
        min_elapsed_s=0.15,
        min_emf_v=1.0,                      # |E| ≥ 1.0 V (well above noise floor)
        min_confidence=0.60,
        confidence_hold_s=0.04,
        fallback_enabled=True,
        fallback_hold_s=0.08,
    )

    return motor, engine, svm, ctrl


# ── Run a full simulation for one observer mode, return logs ──────────────────
def run_sim(observer_mode: str):
    print(f"\n[2] Running {observer_mode} sensorless simulation ({T_SIM:.1f} s)...")
    motor, engine, svm, ctrl = build_sim(observer_mode)

    # Log arrays
    t_log         = np.empty(N_STEPS, np.float32)
    speed_true    = np.empty(N_STEPS, np.float32)
    theta_true    = np.empty(N_STEPS, np.float32)
    emf_true_mag  = np.empty(N_STEPS, np.float32)
    theta_obs     = np.empty(N_STEPS, np.float32)
    emf_recon_mag = np.empty(N_STEPS, np.float32)
    speed_est     = np.empty(N_STEPS, np.float32)
    confidence    = np.empty(N_STEPS, np.float32)

    # Pre-compute FW correction constants
    lambda_pm = Ke / Pp        # permanent-magnet flux linkage [Wb_e / rad_e] ≈ Ke
    ld_val    = float(mp["ld"])

    t0 = time.perf_counter()
    for i in range(N_STEPS):
        ctrl.set_speed_reference(float(speed_ref[i]))
        mag, ang  = ctrl.update(DT)
        voltages  = svm.modulate(mag, ang)
        # Feed actual SVM-applied phase voltages back to the controller so that
        # EMF reconstruction uses the real terminal voltage, not the FOC-commanded
        # inverse-Park output (which can differ by ±15 % due to SVM sector gain).
        ctrl.update_applied_voltage(float(voltages[0]), float(voltages[1]), float(voltages[2]))
        engine.step(voltages, log_data=False)

        # True motor state
        t_log[i]        = t_vec[i]
        speed_true[i]   = motor.speed_rpm
        theta_e_t       = (motor.theta * Pp) % (2 * math.pi)
        theta_true[i]   = theta_e_t

        # True back-EMF magnitude — FW-corrected:
        # motor.back_emf reflects only PM flux (= Ke·ωe) and ignores id injection.
        # In FW zone id < 0 → effective flux λ_eff = λpm + Ld·id < λpm.
        # The EMF reconstructor sees the ACTUAL reduced flux, so we must use the
        # same formula: |E_true_fw| = ωe · |λ_eff| = ωe · |Ke/Pp + Ld·id|·Pp.
        ia, ib, ic = motor.currents
        i_alpha_m = ia                             # Clarke amplitude-invariant
        i_beta_m  = (ib - ic) / math.sqrt(3.0)
        cos_th = math.cos(theta_e_t)
        sin_th = math.sin(theta_e_t)
        id_actual  = i_alpha_m * cos_th + i_beta_m * sin_th
        iq_actual  = -i_alpha_m * sin_th + i_beta_m * cos_th
        omega_e_actual = abs(motor.omega) * Pp     # [rad_e / s]
        lambda_eff = lambda_pm + ld_val * id_actual
        lambda_eff = max(lambda_eff, 0.3 * lambda_pm)   # floor: avoid div/0
        # Full magnitude: sqrt((ωe·Ld·iq)² + (ωe·λ_eff)²)  — cross-coupling term
        emf_true_mag[i] = omega_e_actual * math.sqrt(
            (ld_val * iq_actual) ** 2 + lambda_eff ** 2
        )

        # Observer state
        st = ctrl.get_state()
        if observer_mode == "PLL":
            theta_obs[i] = ctrl.theta_est_pll
        else:
            theta_obs[i] = ctrl.theta_est_smo
        emf_recon_mag[i] = float(st.get("emf_reconstructed_mag", 0.0))
        omega_est        = float(st.get("omega_elec_est_rad_s", 0.0))
        speed_est[i]     = omega_est / max(float(Pp), 1.0) * 30.0 / math.pi
        confidence[i]    = float(st.get("observer_confidence", 0.0))

        if (i + 1) % 10000 == 0:
            print(f"  t={t_vec[i]:.2f}s  ω={motor.speed_rpm:.0f}/{speed_ref[i]:.0f} RPM  "
                  f"|E|_true={emf_true_mag[i]:.3f}V  |E|_est={emf_recon_mag[i]:.3f}V  "
                  f"conf={confidence[i]:.3f}")

    elapsed = time.perf_counter() - t0
    print(f"  Done: {elapsed:.1f}s  ({N_STEPS/elapsed:.0f} steps/s)")

    return {
        "t": t_log, "speed_true": speed_true, "theta_true": theta_true,
        "emf_true": emf_true_mag, "theta_obs": theta_obs,
        "emf_recon": emf_recon_mag, "speed_est": speed_est,
        "confidence": confidence,
    }


# Run both
logs_pll = run_sim("PLL")
logs_smo = run_sim("SMO")

# ── Angle error ────────────────────────────────────────────────────────────────
def wrap_arr(arr):
    return (arr + math.pi) % (2 * math.pi) - math.pi

err_pll = wrap_arr(logs_pll["theta_obs"] - logs_pll["theta_true"])
err_smo = wrap_arr(logs_smo["theta_obs"] - logs_smo["theta_true"])
err_pll_deg = np.degrees(err_pll)
err_smo_deg = np.degrees(err_smo)

# ── Metrics ───────────────────────────────────────────────────────────────────
# Two windows:
#   pre_fw : t=1.5–2.0 s  (steady 1500 RPM, no field-weakening)
#            → primary pass/fail: angle, speed, EMF are all clean here
#   fw_ss  : t=3.0–4.0 s  (steady 3000 RPM, deep FW)
#            → informational: FW-corrected EMF comparison
#
# NOTE: emf_true is already FW-corrected in this script (computed from actual
# motor currents and true dq currents), so the comparison is fair everywhere.
t_log     = logs_pll["t"]
pre_fw_lo = int(1.5 / DT)
pre_fw_hi = int(2.0 / DT)
fw_lo     = int(3.0 / DT)
fw_hi     = N_STEPS

def rms(x):  return float(np.sqrt(np.mean(x ** 2)))
def peak(x): return float(np.max(np.abs(x)))

def window_metrics(err_deg_pll, err_deg_smo, logs_p, logs_s, lo, hi, label):
    ep = err_deg_pll[lo:hi]
    es = err_deg_smo[lo:hi]
    e_true_p = logs_p["emf_true"][lo:hi]
    e_true_s = logs_s["emf_true"][lo:hi]
    e_rec_p  = logs_p["emf_recon"][lo:hi]
    e_rec_s  = logs_s["emf_recon"][lo:hi]
    s_true_p = logs_p["speed_true"][lo:hi]
    s_est_p  = logs_p["speed_est"][lo:hi]
    s_true_s = logs_s["speed_true"][lo:hi]
    s_est_s  = logs_s["speed_est"][lo:hi]
    emf_mean_p = max(float(np.mean(e_true_p)), 1e-3)
    emf_mean_s = max(float(np.mean(e_true_s)), 1e-3)
    print(f"\n── {label} ─────────────────────────────────")
    print(f"  PLL: θ_err RMS={rms(ep):.2f}°  peak={peak(ep):.2f}°  "
          f"|E| err={rms(e_rec_p-e_true_p):.4f}V ({rms(e_rec_p-e_true_p)/emf_mean_p*100:.1f}%)  "
          f"ω err={rms(s_est_p-s_true_p):.1f} RPM")
    print(f"  SMO: θ_err RMS={rms(es):.2f}°  peak={peak(es):.2f}°  "
          f"|E| err={rms(e_rec_s-e_true_s):.4f}V ({rms(e_rec_s-e_true_s)/emf_mean_s*100:.1f}%)  "
          f"ω err={rms(s_est_s-s_true_s):.1f} RPM")
    return {
        "pll_angle_rms_deg":     rms(ep),
        "pll_angle_peak_deg":    peak(ep),
        "smo_angle_rms_deg":     rms(es),
        "smo_angle_peak_deg":    peak(es),
        "pll_emf_rms_v":         rms(e_rec_p - e_true_p),
        "smo_emf_rms_v":         rms(e_rec_s - e_true_s),
        "pll_emf_rel":           rms(e_rec_p - e_true_p) / emf_mean_p,
        "smo_emf_rel":           rms(e_rec_s - e_true_s) / emf_mean_s,
        "pll_speed_err_rms_rpm": rms(s_est_p - s_true_p),
        "smo_speed_err_rms_rpm": rms(s_est_s - s_true_s),
        "pll_conf_mean":         float(np.mean(logs_p["confidence"][lo:hi])),
        "smo_conf_mean":         float(np.mean(logs_s["confidence"][lo:hi])),
        "emf_mean_pll_v":        emf_mean_p,
        "emf_mean_smo_v":        emf_mean_s,
    }

metrics_prefw = window_metrics(
    err_pll_deg, err_smo_deg, logs_pll, logs_smo,
    pre_fw_lo, pre_fw_hi,
    "Pre-FW steady-state (t=1.5–2.0 s, 1500 RPM)",
)
metrics_fw = window_metrics(
    err_pll_deg, err_smo_deg, logs_pll, logs_smo,
    fw_lo, fw_hi,
    "FW steady-state (t=3.0–4.0 s, 3000 RPM)  [informational]",
)

(OUT_DIR / "observer_metrics.json").write_text(
    json.dumps({"pre_fw": metrics_prefw, "fw_ss": metrics_fw}, indent=2)
)

# ── Plots ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(5, 1, figsize=(14, 22), sharex=True)
fig.suptitle(
    "Sensorless Observer Validation — Nanotec DB57M012 12V\n"
    "PLL vs SMO (EMF reconstructed from v_αβ and i_αβ)",
    fontsize=13,
)

# 1. Speed
ax = axes[0]
ax.plot(t_log, logs_pll["speed_true"], "k-",  lw=1.5, label="True speed")
ax.plot(t_log, speed_ref,              "b--",  lw=1,   alpha=0.4, label="Speed ref")
ax.plot(t_log, logs_pll["speed_est"], "r-",   lw=1,   alpha=0.9, label="ω̂_PLL")
ax.plot(t_log, logs_smo["speed_est"], "g-",   lw=1,   alpha=0.9, label="ω̂_SMO")
ax.set_ylabel("Speed [RPM]")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
ax.set_title("Motor Speed — True vs Observer Estimates")

# 2. Electrical angle (show first 0.5 s)
ax = axes[1]
m = t_log < 0.5
ax.plot(t_log[m], np.degrees(logs_pll["theta_true"][m]), "k-",  lw=1.5, label="θ_true")
ax.plot(t_log[m], np.degrees(logs_pll["theta_obs"][m]),  "r--", lw=1,   label="θ_PLL")
ax.plot(t_log[m], np.degrees(logs_smo["theta_obs"][m]),  "g:",  lw=1,   label="θ_SMO")
ax.set_ylabel("θ_e [deg]")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
ax.set_title("Electrical Angle Tracking — first 0.5 s")

# 3. Back-EMF magnitude
ax = axes[2]
ax.plot(t_log, logs_pll["emf_true"],  "k-",  lw=1.5, label="|E|_true")
ax.plot(t_log, logs_pll["emf_recon"], "r-",  lw=1,   alpha=0.9, label="|E|_PLL (recon)")
ax.plot(t_log, logs_smo["emf_recon"], "g--", lw=1,   alpha=0.9, label="|E|_SMO (recon)")
ax.set_ylabel("|E| [V]")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
ax.set_title("Back-EMF Magnitude — True vs Reconstructed")

# 4. Angle error
ax = axes[3]
ax.plot(t_log, err_pll_deg, "r-", lw=0.8, alpha=0.9, label="PLL error")
ax.plot(t_log, err_smo_deg, "g-", lw=0.8, alpha=0.9, label="SMO error")
ax.axhline( 5, color="gray", lw=0.5, ls="--", alpha=0.5, label="±5° band")
ax.axhline(-5, color="gray", lw=0.5, ls="--", alpha=0.5)
ax.axhline( 0, color="k",   lw=0.5)
ax.set_ylabel("Error [deg]")
ax.set_ylim(-30, 30)
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
ax.set_title("Angle Tracking Error (θ_obs − θ_true)")

# 5. Confidence
ax = axes[4]
ax.plot(t_log, logs_pll["confidence"], "r-", lw=1, label="PLL confidence")
ax.plot(t_log, logs_smo["confidence"], "g-", lw=1, label="SMO confidence")
ax.axhline(0.6, color="k", lw=0.5, ls="--", alpha=0.5, label="threshold (0.6)")
ax.set_ylabel("Confidence [0–1]")
ax.set_xlabel("Time [s]")
ax.set_ylim(0, 1.1)
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
ax.set_title("Observer Confidence Score")

plt.tight_layout()
fig_path = str(OUT_DIR / "observer_validation.png")
fig.savefig(fig_path, dpi=150, bbox_inches="tight")
plt.close()
print(f"\nPlot saved: {fig_path}")

# ── Pass / Fail (based on pre-FW steady-state window) ─────────────────────────
print("\n── PASS / FAIL  [pre-FW window: t=1.5–2.0 s, 1500 RPM] ───────────────")
THRESH_ANGLE_RMS = 10.0   # ° RMS
THRESH_EMF_REL   = 0.20   # 20 % relative RMS error vs FW-corrected true EMF
THRESH_SPEED_RPM = 300.0  # RPM RMS error

results = {}
for obs in ["PLL", "SMO"]:
    k = obs.lower()
    ae_rms  = metrics_prefw[f"{k}_angle_rms_deg"]
    ae_peak = metrics_prefw[f"{k}_angle_peak_deg"]
    emf_err = metrics_prefw[f"{k}_emf_rms_v"]
    emf_rel = metrics_prefw[f"{k}_emf_rel"]
    spd_err = metrics_prefw[f"{k}_speed_err_rms_rpm"]
    conf    = metrics_prefw[f"{k}_conf_mean"]

    angle_ok = ae_rms  < THRESH_ANGLE_RMS
    emf_ok   = emf_rel < THRESH_EMF_REL
    speed_ok = spd_err < THRESH_SPEED_RPM
    passed   = angle_ok and emf_ok and speed_ok
    tag      = "✅ PASS" if passed else "❌ FAIL"

    print(f"  {obs}: {tag}")
    print(f"    θ error   RMS={ae_rms:.2f}°  peak={ae_peak:.2f}°  "
          f"[{'OK' if angle_ok else 'FAIL'}]")
    print(f"    |E| error RMS={emf_err:.4f}V ({emf_rel*100:.1f}%)  "
          f"[{'OK' if emf_ok else 'FAIL'}]")
    print(f"    ω error   RMS={spd_err:.1f} RPM  [{'OK' if speed_ok else 'FAIL'}]")
    print(f"    confidence mean={conf:.3f}")
    results[obs] = {"passed": passed, "angle_rms_deg": ae_rms,
                    "emf_rel_err": emf_rel, "speed_rms_rpm": spd_err, "conf_mean": conf}

all_pass = all(r["passed"] for r in results.values())
print(f"\n{'✅ ALL OBSERVERS PASS' if all_pass else '❌ SOME OBSERVERS FAILED'}")

(OUT_DIR / "observer_results.json").write_text(
    json.dumps({
        "metrics_prefw": metrics_prefw,
        "metrics_fw_ss": metrics_fw,
        "results": results,
        "all_pass": all_pass,
    }, indent=2)
)
print(f"Results saved: {OUT_DIR}/")
