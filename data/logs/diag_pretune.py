"""
Step-by-step diagnostic for Innotec pre-tune phase.
Runs a single trial and prints speed trajectory every 0.1s.
"""

import sys
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.control import FOCController, SVMGenerator
from src.core.load_model import ConstantLoad
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.simulation_engine import SimulationEngine
from src.utils.motor_profiles import load_motor_profile

profile_path = (
    PROJECT_ROOT / "data/motor_profiles/_tmp_innotec/innotec_255_ezs48_160.json"
)
profile = load_motor_profile(profile_path)
mp = profile["motor_params"]

# Build motor params
params = MotorParameters(
    model_type=mp.get("model_type", "dq"),
    emf_shape=mp.get("emf_shape", "sinusoidal"),
    nominal_voltage=float(mp["nominal_voltage"]),
    phase_resistance=float(mp["phase_resistance"]),
    phase_inductance=float(mp["phase_inductance"]),
    back_emf_constant=float(mp["back_emf_constant"]),
    torque_constant=float(mp["torque_constant"]),
    rotor_inertia=float(mp["rotor_inertia"]),
    friction_coefficient=float(mp["friction_coefficient"]),
    num_poles=int(float(mp["num_poles"])),
    poles_pairs=int(float(mp["num_poles"]) / 2),
    ld=float(mp.get("ld", mp["phase_inductance"])),
    lq=float(mp.get("lq", mp["phase_inductance"])),
)

dt = 1e-4
sim_time_s = 10.0  # enough to see the full pre-tune phase
rated_speed_rpm = 1522.0  # effective after feasibility fallback
pre_tune_speed_rpm = 1000.0
rated_current = 160.0

# Anchor candidate (good representative)
iq_limit_a = rated_current * 2.4  # 384 A
open_loop_iq_ref_a = rated_current * 0.45  # 72 A

motor = BLDCMotor(params, dt=dt)
engine = SimulationEngine(
    motor, ConstantLoad(0.0), dt=dt, compute_backend="cpu", max_history=100
)

controller = FOCController(motor=motor, enable_speed_loop=True)
controller.set_cascaded_speed_loop(True, iq_limit_a=iq_limit_a)

# Theory seed gains at 1000 RPM
lq = max(float(params.lq), 1e-7)
r = max(float(params.phase_resistance), 1e-7)
tau_e = lq / r
wc_i = float(np.clip(4.0 / max(tau_e, 1e-6), 200.0, 4000.0))
current_kp = float(np.clip(lq * wc_i, 1e-4, 10.0))
current_ki = float(np.clip(r * wc_i, 1e-4, 5000.0))

j = float(params.rotor_inertia)
b = float(params.friction_coefficient)
kt = float(params.torque_constant)
wn_s = float(np.clip(0.08 * pre_tune_speed_rpm * 2 * np.pi / 60, 8.0, 120.0))
zeta = 0.90
speed_kp = max(1e-4, (2 * zeta * wn_s * j - b) / kt)
speed_ki = max(1e-4, (wn_s**2) * j / kt)

print(f"Seed gains: speed_kp={speed_kp:.4f} speed_ki={speed_ki:.4f}")
print(f"Current gains: current_kp={current_kp:.4f} current_ki={current_ki:.4f}")
pre_tune_speed_kp = speed_kp * 2.0
pre_tune_speed_ki = speed_ki * 2.0
pre_tune_current_kp = current_kp * 1.5
pre_tune_current_ki = current_ki * 1.5
print(f"Pre-tune speed gains: kp={pre_tune_speed_kp:.4f} ki={pre_tune_speed_ki:.4f}")
print(
    f"Pre-tune current gains: kp={pre_tune_current_kp:.4f} ki={pre_tune_current_ki:.4f}"
)

controller.set_speed_pi_gains(kp=pre_tune_speed_kp, ki=pre_tune_speed_ki, kaw=0.05)
controller.set_current_pi_gains(
    d_kp=pre_tune_current_kp,
    d_ki=pre_tune_current_ki,
    q_kp=pre_tune_current_kp,
    q_ki=pre_tune_current_ki,
    kaw=0.2,
)
controller.set_current_references(id_ref=0.0, iq_ref=0.0)

pre_tune_ramp_time_s = 3.0  # clipped minimum
open_loop_initial_speed = max(20.0, pre_tune_speed_rpm * 0.05)  # 50 RPM

controller.set_angle_observer("Measured")
controller.set_startup_transition(
    enabled=False,
    initial_mode="Measured",
    min_speed_rpm=max(20.0, 0.85 * pre_tune_speed_rpm),
    min_elapsed_s=max(0.05, pre_tune_ramp_time_s + 0.5),
    min_emf_v=0.1,
    min_confidence=0.3,
    confidence_hold_s=0.02,
    confidence_hysteresis=0.05,
    fallback_enabled=False,
    fallback_hold_s=0.1,
)

controller.set_startup_sequence(
    enabled=True,
    align_duration_s=0.0,
    align_current_a=rated_current * 0.08,
    align_angle_deg=0.0,
    open_loop_initial_speed_rpm=open_loop_initial_speed,
    open_loop_target_speed_rpm=pre_tune_speed_rpm,
    open_loop_ramp_time_s=pre_tune_ramp_time_s,
    open_loop_id_ref_a=0.0,
    open_loop_iq_ref_a=open_loop_iq_ref_a,
)

base_vdq_limit = float(controller.vdq_limit)
svm = SVMGenerator(dc_voltage=params.nominal_voltage)
svm.set_sample_time(dt)

controller.set_speed_reference(float(pre_tune_speed_rpm))
controller.set_field_weakening(
    enabled=False, start_speed_rpm=1e9, gain=0.0, max_negative_id_a=0.0
)
controller._enter_startup_phase("open_loop")

print(f"\nvdq_limit = {base_vdq_limit:.4f} V")
print(
    f"open_loop_iq_ref_a = {open_loop_iq_ref_a:.1f} A  (reduced to {open_loop_iq_ref_a * 0.9:.1f} with vf_ratio=0.9)"
)
print(f"pre_tune_ramp_time_s = {pre_tune_ramp_time_s:.1f} s")
print(
    f"\n{'t(s)':>6}  {'speed(RPM)':>10}  {'phase':>12}  {'iq_ref(A)':>10}  {'cmd_spd':>8}  {'ema_rpm':>8}"
)
print("-" * 75)

speed_ema = 0.0
pre_tune_entry = pre_tune_speed_rpm
pre_tune_band = max(15.0, 0.05 * pre_tune_entry)
tuning_started = False
print_interval = int(0.1 / dt)
last_print_t = -1.0

steps = int(sim_time_s / dt)
for k in range(steps):
    sim_t = float((k + 1) * dt)
    ramp_ratio = min(sim_t / max(pre_tune_ramp_time_s, dt), 1.0)

    if not tuning_started:
        if sim_t <= pre_tune_ramp_time_s:
            controller.set_speed_reference(pre_tune_speed_rpm * ramp_ratio)
            if controller.startup_phase != "open_loop":
                controller._enter_startup_phase("open_loop")
        else:
            controller.set_speed_reference(float(pre_tune_entry))
            if controller.startup_phase != "closed_loop":
                controller._enter_startup_phase("closed_loop")
                controller.startup_transition_done = True
                controller.startup_ready_to_switch = True

        ema_alpha = dt / (0.25 + dt)
        speed_ema = (1.0 - ema_alpha) * speed_ema + ema_alpha * abs(motor.speed_rpm)

        if controller.startup_phase == "open_loop":
            # Compute V/f ratio and set iq_ref
            w_cmd = controller.startup_open_loop_speed_rpm
            w_mech = w_cmd * 2 * np.pi / 60
            w_elec = w_mech * params.poles_pairs
            Ke = float(params.back_emf_constant)
            R = float(params.phase_resistance)
            Lq = float(params.lq)
            iq_b = open_loop_iq_ref_a
            v_emf = Ke * w_mech
            v_req = float(np.sqrt(v_emf**2 + (R * iq_b + w_elec * Lq * iq_b) ** 2))
            vf_ratio = float(np.clip(v_req / max(base_vdq_limit, 1e-9), 0.9, 1.0))
            controller.startup_open_loop_iq_ref_a = max(0.90 * iq_b, iq_b * vf_ratio)
            controller.vdq_limit = base_vdq_limit
        else:
            controller.vdq_limit = base_vdq_limit

        if abs(speed_ema - pre_tune_entry) <= pre_tune_band:
            tuning_started = True

    svm.set_phase_currents(motor.currents)
    mag, ang = controller.update(dt)
    if not np.isfinite(mag) or not np.isfinite(ang):
        print(f"  t={sim_t:.3f}  NON-FINITE controller output!")
        break
    phase_voltages = svm.modulate(mag, ang)
    if not np.all(np.isfinite(np.array(phase_voltages, dtype=np.float64))):
        print(f"  t={sim_t:.3f}  NON-FINITE modulation!")
        break
    engine.step(phase_voltages, log_data=False)

    if not np.isfinite(motor.omega) or abs(motor.speed_rpm) > 1e6:
        print(f"  t={sim_t:.3f}  DIVERGED! speed={motor.speed_rpm:.0f} RPM")
        break

    # Print every 0.1s or at key events
    if sim_t >= last_print_t + 0.1 - dt * 0.5:
        last_print_t = sim_t
        iq_ref_now = (
            controller.startup_open_loop_iq_ref_a
            if controller.startup_phase == "open_loop"
            else "CL"
        )
        cmd_speed = (
            getattr(controller, "startup_open_loop_speed_rpm", 0)
            if controller.startup_phase == "open_loop"
            else controller.speed_ref
        )
        print(
            f"  {sim_t:5.2f}  {motor.speed_rpm:10.2f}  {controller.startup_phase:>12}  "
            f"{str(iq_ref_now):>10}  {cmd_speed:8.1f}  {speed_ema:8.2f}  "
            f"{'<<GATE' if tuning_started else ''}"
        )

print(f"\nFinal speed: {motor.speed_rpm:.2f} RPM")
print(f"Pre-tune target: {pre_tune_entry:.0f} RPM  band: ±{pre_tune_band:.1f} RPM")
print(f"Gate fired: {tuning_started}")
