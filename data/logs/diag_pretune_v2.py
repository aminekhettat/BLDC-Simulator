"""
Step-by-step diagnostic for Innotec pre-tune phase (FIXED: closed-loop from t=0).
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
sim_time_s = 5.0
pre_tune_speed_rpm = 1000.0
rated_current = 160.0
iq_limit_a = rated_current * 2.4  # 384 A

motor = BLDCMotor(params, dt=dt)
engine = SimulationEngine(
    motor, ConstantLoad(0.0), dt=dt, compute_backend="cpu", max_history=100
)

controller = FOCController(motor=motor, enable_speed_loop=True)
controller.set_cascaded_speed_loop(True, iq_limit_a=iq_limit_a)

# Theory seed gains at pre_tune_speed_rpm
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
pre_tune_speed_kp = speed_kp * 2.0
pre_tune_speed_ki = speed_ki * 2.0
pre_tune_current_kp = current_kp * 1.5
pre_tune_current_ki = current_ki * 1.5

controller.set_speed_pi_gains(kp=pre_tune_speed_kp, ki=pre_tune_speed_ki, kaw=0.05)
controller.set_current_pi_gains(
    d_kp=pre_tune_current_kp,
    d_ki=pre_tune_current_ki,
    q_kp=pre_tune_current_kp,
    q_ki=pre_tune_current_ki,
    kaw=0.2,
)
controller.set_current_references(id_ref=0.0, iq_ref=0.0)

# FIXED: sensored motor -> closed-loop from t=0, no open-loop ramp
controller.set_angle_observer("Measured")
controller.set_startup_transition(
    enabled=False,
    initial_mode="Measured",
    min_speed_rpm=20.0,
    min_elapsed_s=0.05,
    min_emf_v=0.1,
    min_confidence=0.3,
    confidence_hold_s=0.02,
    confidence_hysteresis=0.05,
    fallback_enabled=False,
    fallback_hold_s=0.1,
)
controller.set_startup_sequence(enabled=False)
controller._enter_startup_phase("closed_loop")
controller.startup_transition_done = True
controller.startup_ready_to_switch = True
controller.set_speed_reference(float(pre_tune_speed_rpm))
controller.set_field_weakening(
    enabled=False, start_speed_rpm=1e9, gain=0.0, max_negative_id_a=0.0
)

base_vdq_limit = float(controller.vdq_limit)
svm = SVMGenerator(dc_voltage=params.nominal_voltage)
svm.set_sample_time(dt)

print(f"vdq_limit = {base_vdq_limit:.4f} V")
print(f"pre_tune_speed_kp = {pre_tune_speed_kp:.4f}  ki = {pre_tune_speed_ki:.4f}")
pre_tune_entry = pre_tune_speed_rpm
pre_tune_band = max(15.0, 0.05 * pre_tune_entry)
print(f"Gate target = {pre_tune_entry:.0f} RPM ± {pre_tune_band:.1f} RPM")
print(f"\n{'t(s)':>6}  {'speed(RPM)':>10}  {'phase':>12}  {'ema_rpm':>8}  {'note':>15}")
print("-" * 70)

speed_ema = 0.0
tuning_started = False
last_print_t = -1.0

steps = int(sim_time_s / dt)
for k in range(steps):
    sim_t = float((k + 1) * dt)

    if not tuning_started:
        ema_alpha = dt / (0.25 + dt)
        speed_ema = (1.0 - ema_alpha) * speed_ema + ema_alpha * abs(motor.speed_rpm)
        controller.vdq_limit = base_vdq_limit

        if abs(speed_ema - pre_tune_entry) <= pre_tune_band:
            tuning_started = True
            print(
                f"  {sim_t:5.3f}  {motor.speed_rpm:10.2f}  {controller.startup_phase:>12}  "
                f"{speed_ema:8.2f}   *** GATE FIRED ***"
            )

    svm.set_phase_currents(motor.currents)
    mag, ang = controller.update(dt)
    if not np.isfinite(mag) or not np.isfinite(ang):
        print(f"  t={sim_t:.3f}  NON-FINITE!")
        break
    phase_voltages = svm.modulate(mag, ang)
    engine.step(phase_voltages, log_data=False)

    if not np.isfinite(motor.omega) or abs(motor.speed_rpm) > 1e6:
        print(f"  t={sim_t:.3f}  DIVERGED! speed={motor.speed_rpm:.0f}")
        break

    if sim_t >= last_print_t + 0.1 - dt * 0.5:
        last_print_t = sim_t
        note = "<<GATE" if tuning_started else ""
        print(
            f"  {sim_t:5.2f}  {motor.speed_rpm:10.2f}  {controller.startup_phase:>12}  "
            f"{speed_ema:8.2f}  {note:>15}"
        )

print(f"\nFinal speed: {motor.speed_rpm:.2f} RPM")
print(f"Gate fired: {tuning_started}")
