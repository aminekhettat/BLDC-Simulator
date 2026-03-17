"""Diagnose voltage headroom and torque capability for Innotec 255-EZS48-160."""

import numpy as np

Ke = 0.152  # back-EMF constant, V·s/rad (mechanical)
Kt = 0.165  # torque constant, Nm/A
R = 0.005  # phase resistance, Ohm
L = 0.00004  # phase inductance H (Lq=Ld)
J = 0.006761  # rotor inertia, kg·m²
B = 0.001  # friction coefficient, Nm·s/rad
Vdc = 48.0  # DC bus voltage, V
pp = 5  # pole pairs
rated_i = 160.0  # rated current, A

# Phase voltage limits
V_ph_svm = (Vdc / np.sqrt(3.0)) * 0.95  # SVM with 5% reserve
V_ph_full = Vdc / np.sqrt(3.0)  # SVM full modulation

print("=" * 70)
print(f"Innotec 255-EZS48-160  Vdc={Vdc}V  Ke={Ke} V·s/rad  Kt={Kt} Nm/A")
print(f"V_phase_limit (SVM 0.95) = {V_ph_svm:.3f} V")
print(f"V_phase_limit (SVM full) = {V_ph_full:.3f} V")
print("=" * 70)

print("\n--- Back-EMF vs. speed vs. voltage headroom ---")
print(f"{'RPM':>6}  {'v_emf':>7}  {'headroom(0.95)':>14}  {'headroom(full)':>14}")
for rpm in [200, 400, 600, 800, 1000, 1500, 2000, 3000]:
    w = rpm * 2 * np.pi / 60
    ve = Ke * w
    print(f"  {rpm:5d}  {ve:7.2f}V  {V_ph_svm - ve:14.2f}V  {V_ph_full - ve:14.2f}V")

max_rpm_095 = (V_ph_svm / Ke) * 60 / (2 * np.pi)
max_rpm_full = (V_ph_full / Ke) * 60 / (2 * np.pi)
print(f"\n*** Max speed (back-EMF = V_phase, SVM 0.95) = {max_rpm_095:.0f} RPM ***")
print(f"*** Max speed (back-EMF = V_phase, SVM full) = {max_rpm_full:.0f} RPM ***")
print(f"*** Rated speed = 3000 RPM  →  REQUIRES FIELD WEAKENING! ***")

print("\n--- Torque and acceleration to 1000 RPM (no load, friction only) ---")
w_1k = 1000 * 2 * np.pi / 60
print(
    f"{'Iq (A)':>10}  {'T_drive':>9}  {'T_friction':>11}  {'T_net':>7}  {'alpha (rad/s²)':>14}  {'t_to_1kRPM (s)':>14}"
)
for pct in [0.05, 0.10, 0.20, 0.35, 0.50, 0.65]:
    iq = rated_i * pct
    Td = Kt * iq
    Tf = B * w_1k
    Tn = Td - Tf
    alpha = Tn / J
    t = w_1k / alpha if alpha > 0 else float("inf")
    print(
        f"  {iq:7.1f}A  {Td:9.3f}Nm  {Tf:11.4f}Nm  {Tn:7.3f}Nm  {alpha:14.4f}  {t:14.4f}s"
    )

print("\n--- V/f voltage command at 1000 RPM (open-loop startup) ---")
w = 1000 * 2 * np.pi / 60
we = w * pp
ve = Ke * w
for pct in [0.35, 0.50, 0.65]:
    iq = rated_i * pct
    vres = R * iq
    vind = we * L * iq
    vreq = np.sqrt(ve**2 + (vres + vind) ** 2)
    vdq_base = Vdc / np.sqrt(3)  # what the controller base_vdq_limit typically is
    vf_ratio = np.clip(vreq / vdq_base, 0.9, 1.0)
    iq_delivered = max(0.90 * iq, iq * vf_ratio)
    print(
        f"  iq_base={iq:.1f}A  v_emf={ve:.2f}V  v_req={vreq:.3f}V  "
        f"vdq_base={vdq_base:.2f}V  vf_ratio={vf_ratio:.3f}  "
        f"iq_delivered={iq_delivered:.1f}A   -> OK? {vreq <= V_ph_svm}"
    )

print("\n--- What voltage IS needed to sustain 400 RPM (stall zone)? ---")
w_s = 400 * 2 * np.pi / 60
we_s = w_s * pp
ve_s = Ke * w_s
iq_stall = B * w_s / Kt
vreq_s = np.sqrt(ve_s**2 + (R * iq_stall + we_s * L * iq_stall) ** 2)
print(f"  v_emf @400 RPM = {ve_s:.3f} V")
print(f"  Iq needed (friction only) = {iq_stall:.5f} A")
print(f"  v_required = {vreq_s:.4f} V   (well within {V_ph_svm:.2f}V limit)")
print()
print("CONCLUSION: voltage headroom is NOT the limiting factor for 1000 RPM.")
print("The issue must be in CONTROL LAW or OPEN-LOOP IQ DELIVERY.")

print("\n--- Check: does open-loop iq_ref produce enough torque to overcome stall? ---")
print("  The controller uses startup_open_loop_iq_ref_a during open-loop phase.")
print("  This is the q-axis REFERENCE, not the actual delivered torque.")
print("  Key question: is the current ACTUALLY flowing (angle tracking correct)?")
print()
print("  In V/f open-loop, the voltage vector angle is externally commanded.")
print("  If angle is wrong (e.g., lagging the rotor), torque is reduced.")
print()
print("  Also check: controller.vdq_limit sets the voltage magnitude cap")
print(
    f"  If vdq_limit = Vdc/sqrt(3) = {Vdc / np.sqrt(3):.2f}V, that IS enough for 1000 RPM."
)
print(
    f"  v_emf@1000 RPM = {Ke * (1000 * 2 * np.pi / 60):.2f}V  << {Vdc / np.sqrt(3):.2f}V  -> voltage OK"
)
print()
print("  The pre-tune ramp 'pre_tune_ramp_time_s' is clipped to [3, 30]s.")
print("  With J=0.006761 and any IQ > 1A, the motor reaches 1000 RPM in <1s.")
print("  So the ramp clipping is NOT the problem either.")
print()
print("  >>> HYPOTHESIS: The open-loop speed command ramps to 1000 RPM but the")
print("      motor's ACTUAL speed doesn't follow because the open-loop IQ is")
print("      being reduced excessively or the voltage vector is being misapplied.")
