"""Run long FOC-only auto-tuning sessions until rated-speed convergence.

This script tunes controller-side parameters only (speed/current PI gains,
IQ limit, and observer settings) while keeping motor parameters immutable.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.control import AdaptiveFOCTuner, FOCController, SVMGenerator  # noqa: E402
from src.control.transforms import clarke_transform, park_transform  # noqa: E402
from src.core.load_model import ConstantLoad  # noqa: E402
from src.core.motor_model import BLDCMotor, MotorParameters  # noqa: E402
from src.core.simulation_engine import SimulationEngine  # noqa: E402
from src.utils.motor_profiles import (  # noqa: E402
    list_motor_profiles,
    load_motor_profile,
)


@dataclass
class Candidate:
    speed_pi: tuple[float, float]
    current_pi: tuple[float, float]
    iq_limit_a: float
    startup: dict


def _to_motor_params(m: dict) -> MotorParameters:
    num_poles_raw = float(m["num_poles"])
    if not np.isfinite(num_poles_raw):
        raise ValueError(f"Motor parameter num_poles is non-finite: {num_poles_raw}")
    num_poles = int(num_poles_raw)
    return MotorParameters(
        model_type=m.get("model_type", "dq"),
        emf_shape=m.get("emf_shape", "sinusoidal"),
        nominal_voltage=float(m["nominal_voltage"]),
        phase_resistance=float(m["phase_resistance"]),
        phase_inductance=float(m["phase_inductance"]),
        back_emf_constant=float(m["back_emf_constant"]),
        torque_constant=float(m["torque_constant"]),
        rotor_inertia=float(m["rotor_inertia"]),
        friction_coefficient=float(m["friction_coefficient"]),
        num_poles=num_poles,
        poles_pairs=int(num_poles / 2),
        ld=float(m.get("ld", m["phase_inductance"])),
        lq=float(m.get("lq", m["phase_inductance"])),
    )


def _motor_params_fingerprint(m: dict) -> str:
    return json.dumps(m, sort_keys=True, separators=(",", ":"))


def _compute_theory_seed_gains(
    params: MotorParameters,
    rated_speed_rpm: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    # Current-loop PI from RL plant G(s)=1/(L s + R): Kp=L*wc, Ki=R*wc.
    lq = max(float(params.lq), 1.0e-7)
    r = max(float(params.phase_resistance), 1.0e-7)
    tau_e = lq / r
    wc_i = float(np.clip(4.0 / max(tau_e, 1.0e-6), 200.0, 4000.0))
    current_kp = float(np.clip(lq * wc_i, 1.0e-4, 10.0))
    current_ki = float(np.clip(r * wc_i, 1.0e-4, 5000.0))

    # Speed-loop PI from J*w_dot + B*w = Kt*i_q with 2nd-order target dynamics.
    j = max(float(params.rotor_inertia), 1.0e-8)
    b = max(float(params.friction_coefficient), 0.0)
    kt = max(float(params.torque_constant), 1.0e-7)
    rated_omega = abs(rated_speed_rpm) * (2.0 * np.pi / 60.0)
    wn_s = float(np.clip(0.08 * rated_omega, 8.0, 120.0))
    zeta = 0.90
    speed_kp = max(1.0e-4, (2.0 * zeta * wn_s * j - b) / kt)
    speed_ki = max(1.0e-4, (wn_s**2) * j / kt)

    return (float(speed_kp), float(speed_ki)), (float(current_kp), float(current_ki))


def _speed_pi_design_terms(
    params: MotorParameters,
    target_speed_rpm: float,
) -> dict:
    """Return speed-loop design terms used by the analytic PI model."""
    inertia = max(float(params.rotor_inertia), 1.0e-8)
    friction = max(float(params.friction_coefficient), 0.0)
    torque_constant = max(float(params.torque_constant), 1.0e-7)
    rated_omega = abs(float(target_speed_rpm)) * (2.0 * np.pi / 60.0)
    wn = float(np.clip(0.08 * rated_omega, 8.0, 120.0))
    zeta = 0.90
    speed_kp_seed = max(
        1.0e-4,
        (2.0 * zeta * wn * inertia - friction) / torque_constant,
    )
    speed_ki_seed = max(1.0e-4, (wn**2) * inertia / torque_constant)
    return {
        "equations": {
            "plant": "J*w_dot + B*w = Kt*iq",
            "speed_kp": "(2*zeta*wn*J - B)/Kt",
            "speed_ki": "(wn^2*J)/Kt",
            "wn_rule": "clip(0.08*omega_target, 8, 120)",
            "zeta": "0.90",
        },
        "inputs": {
            "target_speed_rpm": float(target_speed_rpm),
            "inertia_J": float(inertia),
            "friction_B": float(friction),
            "torque_constant_Kt": float(torque_constant),
        },
        "derived": {
            "omega_target_rad_s": float(rated_omega),
            "wn_rad_s": float(wn),
            "zeta": float(zeta),
        },
        "seed_speed_pi": {
            "kp": float(speed_kp_seed),
            "ki": float(speed_ki_seed),
        },
    }


def _synthesize_current_pi(
    params: MotorParameters,
    bandwidth_rad_s: float,
) -> tuple[float, float]:
    resistance = max(float(params.phase_resistance), 1.0e-9)
    inductance = max(float(params.lq), 1.0e-9)
    bandwidth = max(float(bandwidth_rad_s), 1.0)
    return inductance * bandwidth, resistance * bandwidth


def _synthesize_speed_pi(
    params: MotorParameters,
    natural_frequency_rad_s: float,
    damping_ratio: float,
) -> tuple[float, float]:
    inertia = max(float(params.rotor_inertia), 1.0e-9)
    friction = max(float(params.friction_coefficient), 0.0)
    torque_constant = max(float(params.torque_constant), 1.0e-9)
    wn = max(float(natural_frequency_rad_s), 1.0)
    zeta = max(float(damping_ratio), 0.1)
    kp = max(1.0e-4, ((2.0 * zeta * wn * inertia) - friction) / torque_constant)
    ki = max(1.0e-4, ((wn**2) * inertia) / torque_constant)
    return kp, ki


def _build_two_stage_analytic_candidates(
    params: MotorParameters,
    target_speed_rpm: float,
    rated_current_a: float,
    max_candidates: int,
) -> tuple[list[Candidate], tuple[float, float], tuple[float, float]]:
    tuner = AdaptiveFOCTuner(params=params)

    mech_omega_target = abs(float(target_speed_rpm)) * (2.0 * np.pi / 60.0)
    elec_omega_target = mech_omega_target * max(float(params.poles_pairs), 1.0)

    current_bw_base = float(
        np.clip(max(1500.0, 6.0 * elec_omega_target), 1500.0, 12000.0)
    )
    speed_wn_base = float(np.clip(max(10.0, 0.10 * mech_omega_target), 10.0, 140.0))

    current_designs: list[tuple[float, float, float]] = []
    for bw_mult in [0.7, 1.0, 1.4]:
        current_kp, current_ki = _synthesize_current_pi(
            params=params,
            bandwidth_rad_s=current_bw_base * bw_mult,
        )
        analysis = tuner.analyze_current_loop(current_kp, current_ki)
        margin = analysis["margin"]
        if bool(analysis["controllable"]) and bool(analysis["observable"]):
            score = float(margin.phase_margin_deg + min(margin.gain_margin_db, 40.0))
            current_designs.append((current_kp, current_ki, score))
    if not current_designs:
        current_kp, current_ki = _synthesize_current_pi(params, current_bw_base)
        current_designs.append((current_kp, current_ki, 0.0))
    current_designs.sort(key=lambda item: item[2], reverse=True)

    speed_designs: list[tuple[float, float, float]] = []
    for wn_mult in [0.7, 1.0, 1.3]:
        for damping_ratio in [0.9, 1.0, 1.1]:
            speed_kp, speed_ki = _synthesize_speed_pi(
                params=params,
                natural_frequency_rad_s=speed_wn_base * wn_mult,
                damping_ratio=damping_ratio,
            )
            analysis = tuner.analyze_speed_loop(speed_kp, speed_ki)
            margin = analysis["margin"]
            if bool(analysis["controllable"]) and bool(analysis["observable"]):
                score = float(
                    margin.phase_margin_deg + min(margin.gain_margin_db, 40.0)
                )
                speed_designs.append((speed_kp, speed_ki, score))
    if not speed_designs:
        speed_kp, speed_ki = _synthesize_speed_pi(params, speed_wn_base, 1.0)
        speed_designs.append((speed_kp, speed_ki, 0.0))
    speed_designs.sort(key=lambda item: item[2], reverse=True)

    seed_speed_pi = (float(speed_designs[0][0]), float(speed_designs[0][1]))
    seed_current_pi = (float(current_designs[0][0]), float(current_designs[0][1]))

    startup_pref_speed_rpm = _startup_pref_speed_rpm(target_speed_rpm)

    out: list[Candidate] = []
    for speed_kp, speed_ki, _ in speed_designs[:3]:
        for current_kp, current_ki, _ in current_designs[:3]:
            for iq_mult in [1.8, 2.4, 3.2]:
                for startup_speed_mult in [0.40, 0.55, 0.70]:
                    for startup_iq_mult in [0.35, 0.50, 0.65]:
                        startup_target = max(
                            startup_pref_speed_rpm,
                            target_speed_rpm * startup_speed_mult,
                        )
                        out.append(
                            Candidate(
                                speed_pi=(float(speed_kp), float(speed_ki)),
                                current_pi=(float(current_kp), float(current_ki)),
                                iq_limit_a=max(
                                    5.0,
                                    min(
                                        rated_current_a * iq_mult, 2.0 * rated_current_a
                                    ),
                                ),
                                startup={
                                    "align_duration_s": 0.0,
                                    "align_current_a": rated_current_a * 0.08,
                                    "open_loop_initial_speed_rpm": max(
                                        50.0,
                                        startup_target * 0.18,
                                    ),
                                    "open_loop_target_speed_rpm": float(startup_target),
                                    "open_loop_ramp_time_s": 0.20,
                                    "open_loop_iq_ref_a": rated_current_a
                                    * startup_iq_mult,
                                },
                            )
                        )
                        if len(out) >= max_candidates:
                            return out, seed_speed_pi, seed_current_pi

    return out, seed_speed_pi, seed_current_pi


def _build_current_stage_candidates(
    params: MotorParameters,
    target_speed_rpm: float,
    rated_current_a: float,
    max_candidates: int,
) -> tuple[list[Candidate], tuple[float, float], tuple[float, float]]:
    tuner = AdaptiveFOCTuner(params=params)

    mech_omega_target = abs(float(target_speed_rpm)) * (2.0 * np.pi / 60.0)
    elec_omega_target = mech_omega_target * max(float(params.poles_pairs), 1.0)

    current_bw_base = float(
        np.clip(max(1200.0, 5.0 * elec_omega_target), 1200.0, 10000.0)
    )
    speed_wn_base = float(np.clip(max(8.0, 0.08 * mech_omega_target), 8.0, 100.0))

    current_designs: list[tuple[float, float, float]] = []
    for bw_mult in [0.6, 0.9, 1.2, 1.6]:
        current_kp, current_ki = _synthesize_current_pi(
            params=params,
            bandwidth_rad_s=current_bw_base * bw_mult,
        )
        analysis = tuner.analyze_current_loop(current_kp, current_ki)
        margin = analysis["margin"]
        if bool(analysis["controllable"]) and bool(analysis["observable"]):
            score = float(margin.phase_margin_deg + min(margin.gain_margin_db, 40.0))
            current_designs.append((current_kp, current_ki, score))
    if not current_designs:
        current_kp, current_ki = _synthesize_current_pi(params, current_bw_base)
        current_designs.append((current_kp, current_ki, 0.0))
    current_designs.sort(key=lambda item: item[2], reverse=True)

    speed_kp_seed, speed_ki_seed = _synthesize_speed_pi(params, speed_wn_base, 1.0)
    seed_speed_pi = (float(speed_kp_seed), float(speed_ki_seed))
    seed_current_pi = (float(current_designs[0][0]), float(current_designs[0][1]))

    startup_pref_speed_rpm = _startup_pref_speed_rpm(target_speed_rpm)
    out: list[Candidate] = []
    for current_kp, current_ki, _ in current_designs[:4]:
        for iq_mult in [1.8, 2.4, 3.2]:
            for startup_iq_mult in [0.35, 0.50, 0.65]:
                out.append(
                    Candidate(
                        speed_pi=(
                            max(1.0e-4, seed_speed_pi[0] * 1.2),
                            max(1.0e-4, seed_speed_pi[1] * 1.2),
                        ),
                        current_pi=(float(current_kp), float(current_ki)),
                        iq_limit_a=max(
                            5.0, min(rated_current_a * iq_mult, 2.0 * rated_current_a)
                        ),
                        startup={
                            "align_duration_s": 0.0,
                            "align_current_a": rated_current_a * 0.08,
                            "open_loop_initial_speed_rpm": max(
                                40.0,
                                startup_pref_speed_rpm * 0.18,
                            ),
                            "open_loop_target_speed_rpm": float(startup_pref_speed_rpm),
                            "open_loop_ramp_time_s": 0.20,
                            "open_loop_iq_ref_a": rated_current_a * startup_iq_mult,
                        },
                    )
                )
                if len(out) >= max_candidates:
                    return out, seed_speed_pi, seed_current_pi
    return out, seed_speed_pi, seed_current_pi


def _build_speed_stage_candidates(
    params: MotorParameters,
    target_speed_rpm: float,
    rated_current_a: float,
    fixed_current_pi: tuple[float, float],
    iq_limit_anchor_a: float,
    max_candidates: int,
) -> tuple[list[Candidate], tuple[float, float]]:
    tuner = AdaptiveFOCTuner(params=params)

    mech_omega_target = abs(float(target_speed_rpm)) * (2.0 * np.pi / 60.0)
    speed_wn_base = float(np.clip(max(8.0, 0.10 * mech_omega_target), 8.0, 140.0))

    speed_designs: list[tuple[float, float, float]] = []
    for wn_mult in [0.65, 0.9, 1.15, 1.4]:
        for damping_ratio in [0.85, 1.0, 1.15]:
            speed_kp, speed_ki = _synthesize_speed_pi(
                params=params,
                natural_frequency_rad_s=speed_wn_base * wn_mult,
                damping_ratio=damping_ratio,
            )
            analysis = tuner.analyze_speed_loop(speed_kp, speed_ki)
            margin = analysis["margin"]
            if bool(analysis["controllable"]) and bool(analysis["observable"]):
                score = float(
                    margin.phase_margin_deg + min(margin.gain_margin_db, 40.0)
                )
                speed_designs.append((speed_kp, speed_ki, score))
    if not speed_designs:
        speed_kp, speed_ki = _synthesize_speed_pi(params, speed_wn_base, 1.0)
        speed_designs.append((speed_kp, speed_ki, 0.0))
    speed_designs.sort(key=lambda item: item[2], reverse=True)

    seed_speed_pi = (float(speed_designs[0][0]), float(speed_designs[0][1]))
    startup_pref_speed_rpm = _startup_pref_speed_rpm(target_speed_rpm)

    out: list[Candidate] = []
    for speed_kp, speed_ki, _ in speed_designs[:5]:
        for iq_mult in [0.9, 1.0, 1.2, 1.4]:
            for startup_iq_mult in [0.40, 0.55, 0.70]:
                out.append(
                    Candidate(
                        speed_pi=(float(speed_kp), float(speed_ki)),
                        current_pi=(
                            float(fixed_current_pi[0]),
                            float(fixed_current_pi[1]),
                        ),
                        iq_limit_a=max(
                            5.0, min(iq_limit_anchor_a * iq_mult, 3.0 * rated_current_a)
                        ),
                        startup={
                            "align_duration_s": 0.0,
                            "align_current_a": rated_current_a * 0.08,
                            "open_loop_initial_speed_rpm": max(
                                40.0,
                                startup_pref_speed_rpm * 0.18,
                            ),
                            "open_loop_target_speed_rpm": float(startup_pref_speed_rpm),
                            "open_loop_ramp_time_s": 0.20,
                            "open_loop_iq_ref_a": rated_current_a * startup_iq_mult,
                        },
                    )
                )
                if len(out) >= max_candidates:
                    return out, seed_speed_pi
    return out, seed_speed_pi


def _build_speed_neighbors(base: Candidate, rated_current_a: float) -> list[Candidate]:
    out: list[Candidate] = []
    for speed_kp_mult in [0.8, 1.0, 1.25, 1.5, 1.8]:
        for speed_ki_mult in [0.8, 1.0, 1.25, 1.5]:
            out.append(
                Candidate(
                    speed_pi=(
                        max(1.0e-4, base.speed_pi[0] * speed_kp_mult),
                        max(1.0e-4, base.speed_pi[1] * speed_ki_mult),
                    ),
                    current_pi=(float(base.current_pi[0]), float(base.current_pi[1])),
                    iq_limit_a=float(base.iq_limit_a),
                    startup=dict(base.startup),
                )
            )
    for iq_mult in [0.85, 1.0, 1.2, 1.4]:
        out.append(
            Candidate(
                speed_pi=(float(base.speed_pi[0]), float(base.speed_pi[1])),
                current_pi=(float(base.current_pi[0]), float(base.current_pi[1])),
                iq_limit_a=max(
                    5.0,
                    min(float(base.iq_limit_a) * iq_mult, 3.0 * float(rated_current_a)),
                ),
                startup=dict(base.startup),
            )
        )
    return out


def _evaluate_no_fw_feasibility(
    params: MotorParameters,
    rated_speed_rpm: float,
    rated_current_a: float,
    rated_voltage_v: float,
) -> dict:
    # Conservative dq-voltage estimate at id=0 for no-FW operation.
    omega_mech = abs(rated_speed_rpm) * (2.0 * np.pi / 60.0)
    omega_elec = omega_mech * max(float(params.poles_pairs), 1.0)

    r = max(float(params.phase_resistance), 1.0e-9)
    lq = max(float(params.lq), 1.0e-9)
    ke = max(float(params.back_emf_constant), 0.0)
    vdc = max(float(params.nominal_voltage), 1.0)

    # SVM linear modulation limit approximation in phase-voltage domain.
    v_phase_limit = (vdc / np.sqrt(3.0)) * 0.95
    v_emf = ke * omega_mech
    v_res = r * rated_current_a
    v_ind = omega_elec * lq * rated_current_a
    v_required = float(np.sqrt(max(v_emf, 0.0) ** 2 + (v_res + v_ind) ** 2))

    margin_v = float(v_phase_limit - v_required)
    feasible = bool(margin_v >= 0.0)
    hardware_voltage_ok = bool(
        float(params.nominal_voltage) >= max(rated_voltage_v, 1.0)
    )
    omega_mech_max = float(v_phase_limit / max(ke, 1.0e-9))
    max_no_fw_speed_rpm = float(omega_mech_max * (60.0 / (2.0 * np.pi)))

    # Respect hardware rating declaration: if rated voltage is supported,
    # keep tuning enabled and report conservative feasibility as diagnostics.
    effective_feasible = bool(feasible or hardware_voltage_ok)

    return {
        "no_fw_voltage_feasible": effective_feasible,
        "no_fw_voltage_feasible_conservative_model": feasible,
        "hardware_voltage_ok": hardware_voltage_ok,
        "rated_voltage_v": float(rated_voltage_v),
        "rated_speed_rpm": float(rated_speed_rpm),
        "rated_current_a": float(rated_current_a),
        "v_phase_limit_v": float(v_phase_limit),
        "v_required_v": float(v_required),
        "v_emf_v": float(v_emf),
        "v_resistive_v": float(v_res),
        "v_inductive_v": float(v_ind),
        "margin_v": margin_v,
        "max_no_fw_speed_rpm_est": max_no_fw_speed_rpm,
    }


def _startup_pref_speed_rpm(target_speed_rpm: float) -> float:
    return float(max(200.0, min(1000.0, abs(float(target_speed_rpm)))))


def _vf_open_loop_vdq_limit(
    params: MotorParameters,
    speed_rpm: float,
    iq_ref_a: float,
) -> float:
    speed_abs_rpm = abs(float(speed_rpm))
    omega_mech = speed_abs_rpm * (2.0 * np.pi / 60.0)
    omega_elec = omega_mech * max(float(params.poles_pairs), 1.0)

    ke = max(float(params.back_emf_constant), 0.0)
    r = max(float(params.phase_resistance), 1.0e-9)
    lq = max(float(params.lq), 1.0e-9)
    iq_abs = abs(float(iq_ref_a))

    # V/f-style open-loop voltage request with basic RL drop compensation.
    v_emf = ke * omega_mech
    v_res = r * iq_abs
    v_ind = omega_elec * lq * iq_abs
    v_required = float(np.sqrt((v_emf**2) + (v_res + v_ind) ** 2))

    v_phase_limit = float((float(params.nominal_voltage) / np.sqrt(3.0)) * 0.95)
    v_min = 0.40 * v_phase_limit
    return float(np.clip(v_required, v_min, v_phase_limit))


def _observer_pool() -> list[dict]:
    return [
        {"mode": "Measured"},
        {"mode": "PLL", "pll_kp": 25.0, "pll_ki": 300.0},
        {"mode": "PLL", "pll_kp": 120.0, "pll_ki": 2500.0},
        {"mode": "PLL", "pll_kp": 250.0, "pll_ki": 6000.0},
        {
            "mode": "SMO",
            "smo_k_slide": 50.0,
            "smo_lpf_alpha": 0.2,
            "smo_boundary": 0.06,
        },
        {
            "mode": "SMO",
            "smo_k_slide": 140.0,
            "smo_lpf_alpha": 0.4,
            "smo_boundary": 0.12,
        },
    ]


def _build_theory_candidates(
    seed_speed_pi: tuple[float, float],
    seed_current_pi: tuple[float, float],
    rated_current_a: float,
    rated_speed_rpm: float,
    max_candidates: int,
) -> list[Candidate]:
    speed_kp_mult = [0.7, 1.0, 1.4, 2.0]
    speed_ki_mult = [0.7, 1.0, 1.4, 2.0]
    current_kp_mult = [0.8, 1.0, 1.3, 1.7, 2.2]
    current_ki_mult = [0.8, 1.0, 1.4, 1.9, 2.4]
    iq_mult = [1.4, 1.8, 2.3, 2.8, 3.2]
    startup_speed_mult = [0.30, 0.45, 0.60, 0.75]
    startup_iq_mult = [0.35, 0.50, 0.65]
    ramp_s = [0.12, 0.20, 0.30]

    startup_pref_speed_rpm = _startup_pref_speed_rpm(rated_speed_rpm)

    out: list[Candidate] = []
    # Evaluate exact theory seed first.
    out.append(
        Candidate(
            speed_pi=seed_speed_pi,
            current_pi=seed_current_pi,
            iq_limit_a=max(5.0, rated_current_a * 2.3),
            startup={
                "align_duration_s": 0.0,
                "align_current_a": rated_current_a * 0.08,
                "open_loop_initial_speed_rpm": max(50.0, rated_speed_rpm * 0.10),
                "open_loop_target_speed_rpm": max(
                    startup_pref_speed_rpm,
                    min(2200.0, rated_speed_rpm * 0.45),
                ),
                "open_loop_ramp_time_s": 0.20,
                "open_loop_iq_ref_a": rated_current_a * 0.50,
            },
        )
    )
    if len(out) >= max_candidates:
        return out

    for skp_m in speed_kp_mult:
        for ski_m in speed_ki_mult:
            for ckp_m in current_kp_mult:
                for cki_m in current_ki_mult:
                    for iq_m in iq_mult:
                        for start_spd_m in startup_speed_mult:
                            for start_iq_m in startup_iq_mult:
                                for ramp in ramp_s:
                                    startup_target = max(
                                        startup_pref_speed_rpm,
                                        min(
                                            2200.0,
                                            rated_speed_rpm * start_spd_m,
                                        ),
                                    )
                                    startup = {
                                        "align_duration_s": 0.0,
                                        "align_current_a": rated_current_a * 0.08,
                                        "open_loop_initial_speed_rpm": max(
                                            40.0, startup_target * 0.18
                                        ),
                                        "open_loop_target_speed_rpm": startup_target,
                                        "open_loop_ramp_time_s": float(ramp),
                                        "open_loop_iq_ref_a": rated_current_a
                                        * start_iq_m,
                                    }
                                    out.append(
                                        Candidate(
                                            speed_pi=(
                                                max(1.0e-4, seed_speed_pi[0] * skp_m),
                                                max(1.0e-4, seed_speed_pi[1] * ski_m),
                                            ),
                                            current_pi=(
                                                max(
                                                    1.0e-4,
                                                    seed_current_pi[0] * ckp_m,
                                                ),
                                                max(
                                                    1.0e-4,
                                                    seed_current_pi[1] * cki_m,
                                                ),
                                            ),
                                            iq_limit_a=max(5.0, rated_current_a * iq_m),
                                            startup=startup,
                                        )
                                    )
                                    if len(out) >= max_candidates:
                                        return out
    return out


def _run_trial(
    base_motor_params: dict,
    rated_speed_rpm: float,
    candidate: Candidate,
    sim_time_s: float,
    dt: float,
    tolerance_ratio: float,
    static_error_rpm_limit: float,
    max_sim_time_s: float | None = None,
    overcurrent_limit_a: float | None = None,
) -> dict:
    base_fp = _motor_params_fingerprint(base_motor_params)
    params = _to_motor_params(dict(base_motor_params))

    min_steps = max(1, int(sim_time_s / dt))
    effective_max_sim_time_s = (
        float(max_sim_time_s) if max_sim_time_s is not None else float(sim_time_s)
    )
    effective_max_sim_time_s = max(float(sim_time_s), effective_max_sim_time_s)
    max_steps = max(min_steps, int(effective_max_sim_time_s / dt))

    motor = BLDCMotor(params, dt=dt)
    engine = SimulationEngine(
        motor,
        ConstantLoad(0.0),
        dt=dt,
        compute_backend="cpu",
        max_history=5000,
    )

    controller = FOCController(motor=motor, enable_speed_loop=True)
    controller.set_cascaded_speed_loop(True, iq_limit_a=candidate.iq_limit_a)
    controller.set_speed_pi_gains(
        kp=candidate.speed_pi[0], ki=candidate.speed_pi[1], kaw=0.05
    )
    controller.set_current_pi_gains(
        d_kp=candidate.current_pi[0],
        d_ki=candidate.current_pi[1],
        q_kp=candidate.current_pi[0],
        q_ki=candidate.current_pi[1],
        kaw=0.2,
    )
    controller.set_current_references(id_ref=0.0, iq_ref=0.0)

    pre_tune_speed_rpm = _startup_pref_speed_rpm(rated_speed_rpm)

    # Use stronger temporary gains to make pre-tune speed acquisition robust.
    candidate_speed_pi = (float(candidate.speed_pi[0]), float(candidate.speed_pi[1]))
    candidate_current_pi = (
        float(candidate.current_pi[0]),
        float(candidate.current_pi[1]),
    )
    seed_speed_pi, seed_current_pi = _compute_theory_seed_gains(
        params=params,
        rated_speed_rpm=pre_tune_speed_rpm,
    )
    pre_tune_speed_pi = (
        max(1.0e-4, seed_speed_pi[0] * 2.0),
        max(1.0e-4, seed_speed_pi[1] * 2.0),
    )
    pre_tune_current_pi = (
        max(1.0e-4, seed_current_pi[0] * 1.5),
        max(1.0e-4, seed_current_pi[1] * 1.5),
    )
    controller.set_speed_pi_gains(
        kp=pre_tune_speed_pi[0], ki=pre_tune_speed_pi[1], kaw=0.05
    )
    controller.set_current_pi_gains(
        d_kp=pre_tune_current_pi[0],
        d_ki=pre_tune_current_pi[1],
        q_kp=pre_tune_current_pi[0],
        q_ki=pre_tune_current_pi[1],
        kaw=0.2,
    )

    pre_tune_entry_speed_rpm = float(pre_tune_speed_rpm)
    pre_tune_steady_band_rpm = max(
        40.0, 0.20 * pre_tune_entry_speed_rpm
    )  # 20% of target → fires on transient rising edge
    pre_tune_ref_ramp_s = float(
        np.clip(0.35 * pre_tune_entry_speed_rpm / 1000.0, 0.20, 1.20)
    )
    pre_tune_iq_limit_a = float(
        np.clip(
            max(20.0, 1.25 * float(candidate.startup["open_loop_iq_ref_a"])),
            20.0,
            max(20.0, float(candidate.iq_limit_a)),
        )
    )
    max_abs_speed_seen_rpm = 0.0
    speed_ema_abs_rpm = 0.0
    tuning_started = False
    tuning_gains_restored = False
    tuning_start_time_s = None

    # PI calibration in sensored mode: measured electrical angle and measured currents.
    # Open-loop V/f is NOT used here: this motor has a position sensor, so closed-loop
    # speed control works from t=0. Open-loop would cause pole slip because the motor's
    # natural acceleration time (~0.07s) is far shorter than any safe minimum ramp.
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
    controller.set_cascaded_speed_loop(True, iq_limit_a=pre_tune_iq_limit_a)
    controller.set_speed_reference(0.0)

    controller.set_field_weakening(
        enabled=False,
        start_speed_rpm=1.0e9,
        gain=0.0,
        max_negative_id_a=0.0,
    )
    # Enable d/q feed-forward decoupling to reduce cross-coupling at speed,
    # which prevents id ≠ 0 drift that inflates orthogonality error.
    controller.set_decoupling(enable_d=True, enable_q=True)

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(dt)
    base_vdq_limit = float(controller.vdq_limit)

    stride = max(1, max_steps // 2200)
    speed_samples = []
    t_samples = []
    orthogonality_error_deg_samples = []
    id_abs_samples = []
    stable = True

    fail_reason = None

    for k in range(max_steps):
        try:
            sim_t = float((k + 1) * dt)

            if not tuning_started:
                pre_tune_ref_signed_rpm = float(
                    np.sign(rated_speed_rpm) * pre_tune_entry_speed_rpm
                )
                pre_tune_ref_cmd_rpm = float(
                    np.clip(
                        pre_tune_ref_signed_rpm
                        * (sim_t / max(pre_tune_ref_ramp_s, dt)),
                        -abs(pre_tune_ref_signed_rpm),
                        abs(pre_tune_ref_signed_rpm),
                    )
                )
                controller.set_speed_reference(pre_tune_ref_cmd_rpm)
                ema_alpha = dt / (
                    0.10 + dt
                )  # τ=0.10 s: gate fires at ~0.45 s for 400 RPM target
                speed_ema_abs_rpm = (
                    1.0 - ema_alpha
                ) * speed_ema_abs_rpm + ema_alpha * abs(motor.speed_rpm)
                max_abs_speed_seen_rpm = max(
                    max_abs_speed_seen_rpm, abs(motor.speed_rpm)
                )
                controller.vdq_limit = base_vdq_limit

                if (
                    abs(speed_ema_abs_rpm - pre_tune_entry_speed_rpm)
                    <= pre_tune_steady_band_rpm
                ):
                    tuning_started = True
                    tuning_start_time_s = sim_t
                    controller._enter_startup_phase("closed_loop")
                    controller.startup_transition_done = True
                    controller.startup_ready_to_switch = True
                    controller.set_speed_pi_gains(
                        kp=candidate_speed_pi[0],
                        ki=candidate_speed_pi[1],
                        kaw=0.05,
                    )
                    controller.set_current_pi_gains(
                        d_kp=candidate_current_pi[0],
                        d_ki=candidate_current_pi[1],
                        q_kp=candidate_current_pi[0],
                        q_ki=candidate_current_pi[1],
                        kaw=0.2,
                    )
                    tuning_gains_restored = True
                    controller.set_cascaded_speed_loop(
                        True, iq_limit_a=candidate.iq_limit_a
                    )
                    controller.set_speed_reference(rated_speed_rpm)
                    controller.vdq_limit = base_vdq_limit
                    controller.pi_speed["integral"] = 0.0
            else:
                if not tuning_gains_restored:
                    controller.set_speed_pi_gains(
                        kp=candidate_speed_pi[0], ki=candidate_speed_pi[1], kaw=0.05
                    )
                    controller.set_current_pi_gains(
                        d_kp=candidate_current_pi[0],
                        d_ki=candidate_current_pi[1],
                        q_kp=candidate_current_pi[0],
                        q_ki=candidate_current_pi[1],
                        kaw=0.2,
                    )
                    controller.set_cascaded_speed_loop(
                        True, iq_limit_a=candidate.iq_limit_a
                    )
                    tuning_gains_restored = True
                controller.set_speed_reference(rated_speed_rpm)
                controller.vdq_limit = base_vdq_limit

            svm.set_phase_currents(motor.currents)
            magnitude, angle = controller.update(dt)
            if not np.isfinite(magnitude) or not np.isfinite(angle):
                stable = False
                fail_reason = "non_finite_controller_output"
                break

            phase_voltages = svm.modulate(magnitude, angle)
            if not np.all(np.isfinite(np.array(phase_voltages, dtype=np.float64))):
                stable = False
                fail_reason = "non_finite_modulation_output"
                break

            engine.step(phase_voltages, log_data=False)
        except Exception:
            stable = False
            fail_reason = "simulation_exception"
            break

        # Divergence check BEFORE sample append so NaN never enters speed_samples.
        if not np.isfinite(motor.omega) or abs(motor.speed_rpm) > 1.0e6:
            stable = False
            fail_reason = "non_finite_or_divergent_speed"
            break

        if overcurrent_limit_a is not None and overcurrent_limit_a > 0.0:
            peak_phase_current = float(np.max(np.abs(motor.currents)))
            if peak_phase_current > float(overcurrent_limit_a):
                stable = False
                fail_reason = "overcurrent_abort"
                break

        if tuning_started and ((k % stride) == 0 or k == max_steps - 1):
            speed_samples.append(float(motor.speed_rpm))
            t_samples.append(float((k + 1) * dt))
            theta_meas = float((motor.theta * motor.params.poles_pairs) % (2.0 * np.pi))
            ia, ib, ic = motor.currents
            i_alpha, i_beta = clarke_transform(ia, ib, ic)
            i_d, i_q = park_transform(i_alpha, i_beta, theta_meas)
            # Use signed id/iq for DC-component tracking; orthogonality is computed
            # from the MEAN (DC) id vs mean iq at the end, not from instantaneous amplitude.
            orthogonality_error_deg_samples.append(float(i_d))  # store signed id
            id_abs_samples.append(float(i_q))  # store signed iq (renamed, see below)

    if _motor_params_fingerprint(base_motor_params) != base_fp:
        raise RuntimeError("Motor parameters mutated during tuning trial")

    if not tuning_started:
        return {
            "stable": False,
            "converged": False,
            "full_converged": False,
            "score": 1.0e9,
            "final_speed_rpm": float(motor.speed_rpm),
            "final_speed_ratio": 0.0,
            "reason": "pre_tuning_target_not_reached",
            "pre_tune_target_speed_rpm": float(pre_tune_entry_speed_rpm),
            "pre_tune_steady_band_rpm": float(pre_tune_steady_band_rpm),
        }

    if len(speed_samples) < 8:
        return {
            "stable": False,
            "converged": False,
            "full_converged": False,
            "score": 1.0e9,
            "final_speed_rpm": 0.0,
            "final_speed_ratio": 0.0,
            "reason": fail_reason or "insufficient_samples",
            "tuning_start_time_s": None
            if tuning_start_time_s is None
            else float(tuning_start_time_s),
            "pre_tune_target_speed_rpm": float(pre_tune_entry_speed_rpm),
        }

    speed = np.array(speed_samples, dtype=np.float64)
    t = np.array(t_samples, dtype=np.float64)

    if t[-1] < float(sim_time_s):
        return {
            "stable": False,
            "converged": False,
            "full_converged": False,
            "score": 1.0e9,
            "final_speed_rpm": float(motor.speed_rpm),
            "final_speed_ratio": float(
                motor.speed_rpm / max(abs(float(rated_speed_rpm)), 1.0e-9)
            ),
            "reason": "stopped_before_min_observation_window",
        }

    err = rated_speed_rpm - speed
    band = max(tolerance_ratio * rated_speed_rpm, 10.0)
    tail = max(20, len(speed) // 8)

    final_speed = float(speed[-1])
    if not np.isfinite(final_speed):
        return {
            "stable": False,
            "converged": False,
            "full_converged": False,
            "score": 1.0e9,
            "final_speed_rpm": 0.0,
            "final_speed_ratio": 0.0,
            "reason": "non_finite_final_speed",
        }
    final_ratio = float(final_speed / max(rated_speed_rpm, 1e-9))
    final_error = float(np.nan_to_num(err[-1], nan=rated_speed_rpm))
    tail_mean = float(np.nan_to_num(np.mean(np.abs(err[-tail:])), nan=1.0e6))
    tail_max = float(np.nan_to_num(np.max(np.abs(err[-tail:])), nan=1.0e6))
    # orthogonality_error_deg_samples stores signed id; id_abs_samples stores signed iq.
    # Use DC-component (mean of signed values) to eliminate AC electrical-frequency ripple
    # before computing FOC angle quality — mean|id| would count harmonic ripple as flux error.
    tail_id_dc = float(
        np.nan_to_num(
            np.mean(
                np.array(orthogonality_error_deg_samples[-tail:], dtype=np.float64)
            ),
            nan=0.0,
        )
    )
    tail_iq_dc = float(
        np.nan_to_num(
            np.mean(np.array(id_abs_samples[-tail:], dtype=np.float64)),
            nan=1.0,
        )
    )
    tail_id_abs_mean_a = abs(tail_id_dc)
    _flux_angle = float(
        np.degrees(np.arctan2(abs(tail_iq_dc), abs(tail_id_dc) + 1e-12))
    )
    tail_orthogonality_error_deg = float(abs(90.0 - _flux_angle))

    # At very low load, iq approaches zero and id/iq angle becomes numerically fragile.
    # Enforce strict orthogonality only when torque-producing current is above a floor.
    iq_quality_floor_a = max(2.0, 0.02 * float(candidate.iq_limit_a))
    orthogonality_gate_pass = bool(
        abs(tail_iq_dc) < iq_quality_floor_a or tail_orthogonality_error_deg <= 2.0
    )

    within = np.abs(err) <= band
    settle_idx = None
    for i in range(len(within)):
        if np.all(within[i:]):
            settle_idx = i
            break

    converged = bool(
        stable
        and abs(final_error) <= band
        and abs(final_error) <= static_error_rpm_limit
        and tail_mean <= band
        and tail_max <= 2.0 * band
        and orthogonality_gate_pass
        and settle_idx is not None
    )

    full_converged = bool(converged and final_ratio >= (1.0 - tolerance_ratio))

    orthogonality_norm = min(
        0.0
        if abs(tail_iq_dc) < iq_quality_floor_a
        else tail_orthogonality_error_deg / 2.0,
        5.0,
    )
    id_efficiency_norm = min(
        tail_id_abs_mean_a / max(0.20 * candidate.iq_limit_a, 1.0),
        5.0,
    )

    # Lower is better; enforce speed tracking, stability, and MTPA-like orthogonality.
    score = (
        (0.78 * (1.0 - np.clip(final_ratio, 0.0, 2.0)))
        + (0.14 * (tail_mean / max(rated_speed_rpm, 1.0)))
        + (0.06 * (tail_max / max(rated_speed_rpm, 1.0)))
        + (0.06 * orthogonality_norm)
        + (0.03 * id_efficiency_norm)
    )
    if final_ratio < 0.60:
        score += 2.0 * (0.60 - final_ratio)
    if not stable:
        score += 5.0
    if converged:
        score -= 1.0
    if full_converged:
        score -= 2.0
    score = float(np.nan_to_num(score, nan=1.0e9, posinf=1.0e9, neginf=-1.0e9))

    return {
        "stable": stable,
        "converged": converged,
        "full_converged": full_converged,
        "score": float(score),
        "rated_speed_rpm": float(rated_speed_rpm),
        "final_speed_rpm": final_speed,
        "final_speed_ratio": final_ratio,
        "final_error_rpm": final_error,
        "tail_abs_mean_error_rpm": tail_mean,
        "tail_abs_max_error_rpm": tail_max,
        "tail_orthogonality_error_deg": tail_orthogonality_error_deg,
        "tail_orthogonality_gate_pass": orthogonality_gate_pass,
        "tail_iq_dc_a": float(tail_iq_dc),
        "iq_quality_floor_a": float(iq_quality_floor_a),
        "tail_id_abs_mean_a": tail_id_abs_mean_a,
        "settling_time_s": None if settle_idx is None else float(t[settle_idx]),
        "sim_time_s": float(t[-1]),
        "requested_min_sim_time_s": float(sim_time_s),
        "requested_max_sim_time_s": float(effective_max_sim_time_s),
        "band_rpm": float(band),
        "tuning_start_time_s": None
        if tuning_start_time_s is None
        else float(tuning_start_time_s),
        "pre_tune_target_speed_rpm": float(pre_tune_entry_speed_rpm),
        "pre_tune_steady_band_rpm": float(pre_tune_steady_band_rpm),
    }


def _build_neighbors(base: Candidate) -> list[Candidate]:
    out: list[Candidate] = []
    for speed_kp_mult in [0.8, 1.0, 1.4, 1.8, 2.2]:
        for speed_ki_mult in [0.8, 1.0, 1.3, 1.7]:
            out.append(
                Candidate(
                    speed_pi=(
                        base.speed_pi[0] * speed_kp_mult,
                        base.speed_pi[1] * speed_ki_mult,
                    ),
                    current_pi=base.current_pi,
                    iq_limit_a=base.iq_limit_a,
                    startup=dict(base.startup),
                )
            )
    for current_kp_mult in [0.8, 1.0, 1.3, 1.7, 2.1]:
        for current_ki_mult in [0.8, 1.0, 1.3, 1.7, 2.1]:
            out.append(
                Candidate(
                    speed_pi=base.speed_pi,
                    current_pi=(
                        base.current_pi[0] * current_kp_mult,
                        base.current_pi[1] * current_ki_mult,
                    ),
                    iq_limit_a=base.iq_limit_a,
                    startup=dict(base.startup),
                )
            )
    for iq_mult in [0.8, 1.0, 1.3, 1.6]:
        out.append(
            Candidate(
                speed_pi=base.speed_pi,
                current_pi=base.current_pi,
                iq_limit_a=max(5.0, base.iq_limit_a * iq_mult),
                startup=dict(base.startup),
            )
        )
    startup_mult = [0.7, 1.0, 1.3, 1.6]
    for mf in startup_mult:
        out.append(
            Candidate(
                speed_pi=base.speed_pi,
                current_pi=base.current_pi,
                iq_limit_a=base.iq_limit_a,
                startup={
                    "align_duration_s": float(base.startup["align_duration_s"]),
                    "align_current_a": float(base.startup["align_current_a"] * mf),
                    "open_loop_initial_speed_rpm": float(
                        max(20.0, base.startup["open_loop_initial_speed_rpm"] * mf)
                    ),
                    "open_loop_target_speed_rpm": float(
                        base.startup["open_loop_target_speed_rpm"] * mf
                    ),
                    "open_loop_ramp_time_s": float(
                        max(0.01, base.startup["open_loop_ramp_time_s"] * mf)
                    ),
                    "open_loop_iq_ref_a": float(
                        base.startup["open_loop_iq_ref_a"] * mf
                    ),
                },
            )
        )
    return out


def tune_one_profile_until_converged(
    profile_path: Path,
    max_trials: int,
    search_time_s: float,
    verify_time_s: float,
    tolerance_ratio: float,
    static_error_rpm_limit: float,
    target_speed_rpm: float,
    seed: int,
    search_max_time_s: float,
    verify_max_time_s: float,
    overcurrent_limit_a: float,
) -> dict:
    profile = load_motor_profile(profile_path)
    rated = profile.get("rated_info", {})
    rated_speed = float(rated.get("rated_speed_rpm", 0.0))
    if rated_speed <= 0.0:
        raise ValueError(f"Missing rated_speed_rpm in {profile_path.name}")
    rated_current = float(
        rated.get("rated_current_a", rated.get("rated_current_a_rms", 20.0))
    )
    rated_voltage = float(
        rated.get(
            "rated_voltage_v", profile["motor_params"].get("nominal_voltage", 0.0)
        )
    )

    base_motor_params = dict(profile["motor_params"])
    params = _to_motor_params(base_motor_params)
    feasibility = _evaluate_no_fw_feasibility(
        params=params,
        rated_speed_rpm=rated_speed,
        rated_current_a=rated_current,
        rated_voltage_v=rated_voltage,
    )

    effective_target_speed_rpm = float(rated_speed)
    target_reason = "rated_speed"
    requested_target_rpm = float(target_speed_rpm)
    if requested_target_rpm > 0.0:
        effective_target_speed_rpm = min(float(rated_speed), abs(requested_target_rpm))
        target_reason = "user_target_override"

    if not bool(feasibility["no_fw_voltage_feasible_conservative_model"]):
        no_fw_cap_rpm = 0.92 * float(
            feasibility.get("max_no_fw_speed_rpm_est", effective_target_speed_rpm)
        )
        if effective_target_speed_rpm > no_fw_cap_rpm:
            effective_target_speed_rpm = max(
                100.0,
                min(
                    effective_target_speed_rpm,
                    no_fw_cap_rpm,
                ),
            )
            target_reason = "no_fw_reachable_speed_fallback"

    startup_pref_speed_rpm = _startup_pref_speed_rpm(effective_target_speed_rpm)
    effective_overcurrent_limit_a = (
        float(overcurrent_limit_a) if overcurrent_limit_a > 0.0 else None
    )

    trial_limit: int | None = int(max_trials) if int(max_trials) > 0 else None

    def _can_run_more_trials(tested_trials: int) -> bool:
        return trial_limit is None or tested_trials < trial_limit

    def _trial_cap_value() -> int:
        # Session payload expects an int; -1 denotes unbounded search mode.
        return -1 if trial_limit is None else int(trial_limit)

    def _remaining_trials(tested_trials: int) -> int:
        if trial_limit is None:
            return 10**9
        return max(0, trial_limit - tested_trials)

    if not bool(feasibility["no_fw_voltage_feasible"]):
        return {
            "schema": "bldc.auto_tune_until_convergence.v2",
            "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "profile": {
                "name": profile.get("profile_name", profile_path.stem),
                "file": profile_path.name,
            },
            "search_setup": {
                "max_trials": int(max_trials),
                "search_time_s": float(search_time_s),
                "verify_time_s": float(verify_time_s),
                "search_max_time_s": float(search_max_time_s),
                "verify_max_time_s": float(verify_max_time_s),
                "overcurrent_limit_a": None
                if effective_overcurrent_limit_a is None
                else float(effective_overcurrent_limit_a),
                "tolerance_ratio": float(tolerance_ratio),
                "static_error_rpm_limit": float(static_error_rpm_limit),
                "field_weakening_enabled": False,
                "sensored_pi_tuning": True,
                "startup_open_loop_calibration": True,
                "candidate_strategy": "sequential_current_then_speed",
                "feasibility_gate": "no_fw_voltage_headroom",
                "requested_target_speed_rpm": float(requested_target_rpm),
                "effective_target_speed_rpm": float(effective_target_speed_rpm),
                "effective_target_reason": target_reason,
                "pre_tuning_speed_ramp": {
                    "enabled": True,
                    "steady_state_entry_speed_rpm": float(startup_pref_speed_rpm),
                    "vf_ratio_applied_in_open_loop": True,
                },
            },
            "best_candidate": None,
            "motor_params": base_motor_params,
            "feasibility": feasibility,
            "results": {
                "best_search_trial": None,
                "verification": {
                    "stable": False,
                    "converged": False,
                    "full_converged": False,
                    "final_speed_rpm": 0.0,
                    "final_speed_ratio": 0.0,
                    "reason": "no_fw_voltage_infeasible_at_rated_speed",
                },
                "tested_trials": 0,
            },
            "acceptance": {
                "accepted": False,
                "reason": "infeasible_target_no_fw_voltage_limit",
            },
        }

    if trial_limit is None:
        current_stage_budget = 120
        speed_stage_budget = 120
    else:
        current_stage_budget = max(1, int(trial_limit * 0.45))
        speed_stage_budget = max(1, trial_limit - current_stage_budget)

    current_stage_budget = max(1, min(current_stage_budget, _remaining_trials(0)))
    speed_stage_budget = max(
        1,
        min(speed_stage_budget, max(1, _remaining_trials(current_stage_budget))),
    )

    current_candidates, seed_speed_pi, seed_current_pi = (
        _build_current_stage_candidates(
            params=params,
            target_speed_rpm=effective_target_speed_rpm,
            rated_current_a=rated_current,
            max_candidates=max(current_stage_budget, 1),
        )
    )

    best_trial = None
    best_candidate = None
    tested = 0
    speed_pi_reference = _speed_pi_design_terms(
        params=params,
        target_speed_rpm=effective_target_speed_rpm,
    )
    trial_debug: list[dict] = []

    def _append_trial_debug(
        phase: str,
        trial_idx: int,
        cand: Candidate,
        trial_result: dict,
        trial_time_s: float,
        trial_dt: float,
    ) -> None:
        trial_debug.append(
            {
                "trial_index": int(trial_idx),
                "phase": phase,
                "sim_time_s": float(trial_time_s),
                "dt_s": float(trial_dt),
                "speed_pi": {
                    "kp": float(cand.speed_pi[0]),
                    "ki": float(cand.speed_pi[1]),
                    "kaw": 0.05,
                },
                "current_pi": {
                    "kp": float(cand.current_pi[0]),
                    "ki": float(cand.current_pi[1]),
                    "kaw": 0.2,
                },
                "iq_limit_a": float(cand.iq_limit_a),
                "startup": {
                    "open_loop_iq_ref_a": float(
                        cand.startup.get("open_loop_iq_ref_a", 0.0)
                    ),
                    "open_loop_target_speed_rpm": float(
                        cand.startup.get("open_loop_target_speed_rpm", 0.0)
                    ),
                },
                "result": {
                    "stable": bool(trial_result.get("stable", False)),
                    "converged": bool(trial_result.get("converged", False)),
                    "full_converged": bool(trial_result.get("full_converged", False)),
                    "score": float(trial_result.get("score", 1.0e9)),
                    "final_speed_rpm": float(trial_result.get("final_speed_rpm", 0.0)),
                    "final_speed_ratio": float(
                        trial_result.get("final_speed_ratio", 0.0)
                    ),
                    "reason": trial_result.get("reason"),
                    "tuning_start_time_s": trial_result.get("tuning_start_time_s"),
                    "pre_tune_target_speed_rpm": trial_result.get(
                        "pre_tune_target_speed_rpm"
                    ),
                },
            }
        )

    # Stage 1: tune current PI with conservative speed PI.
    current_anchor = Candidate(
        speed_pi=(seed_speed_pi[0] * 1.2, seed_speed_pi[1] * 1.2),
        current_pi=seed_current_pi,
        iq_limit_a=max(5.0, rated_current * 2.4),
        startup={
            "align_duration_s": 0.0,
            "align_current_a": rated_current * 0.06,
            "open_loop_initial_speed_rpm": 80.0,
            "open_loop_target_speed_rpm": max(
                startup_pref_speed_rpm,
                min(1800.0, rated_speed * 0.45),
            ),
            "open_loop_ramp_time_s": 0.20,
            "open_loop_iq_ref_a": rated_current * 0.45,
        },
    )
    current_anchor_trial = _run_trial(
        base_motor_params=base_motor_params,
        rated_speed_rpm=effective_target_speed_rpm,
        candidate=current_anchor,
        sim_time_s=search_time_s,
        dt=1.0 / 3500.0,
        tolerance_ratio=tolerance_ratio,
        static_error_rpm_limit=static_error_rpm_limit,
        max_sim_time_s=search_max_time_s,
        overcurrent_limit_a=effective_overcurrent_limit_a,
    )
    best_trial = current_anchor_trial
    best_candidate = current_anchor
    tested = 1
    _append_trial_debug(
        phase="current_stage_anchor",
        trial_idx=tested,
        cand=current_anchor,
        trial_result=current_anchor_trial,
        trial_time_s=float(search_time_s),
        trial_dt=float(1.0 / 3500.0),
    )

    for cand in current_candidates:
        if tested >= current_stage_budget or not _can_run_more_trials(tested):
            break
        trial = _run_trial(
            base_motor_params=base_motor_params,
            rated_speed_rpm=effective_target_speed_rpm,
            candidate=cand,
            sim_time_s=search_time_s,
            dt=1.0 / 3500.0,
            tolerance_ratio=tolerance_ratio,
            static_error_rpm_limit=static_error_rpm_limit,
            max_sim_time_s=search_max_time_s,
            overcurrent_limit_a=effective_overcurrent_limit_a,
        )
        tested += 1
        _append_trial_debug(
            phase="current_stage_search",
            trial_idx=tested,
            cand=cand,
            trial_result=trial,
            trial_time_s=float(search_time_s),
            trial_dt=float(1.0 / 3500.0),
        )

        if best_trial is None or trial["score"] < best_trial["score"]:
            best_trial = trial
            best_candidate = cand

    if best_trial is None or best_candidate is None:
        raise RuntimeError("No current-stage trial executed")

    # Stage 2: tune speed PI while locking the best current PI from stage 1.
    speed_candidates, speed_seed_pi = _build_speed_stage_candidates(
        params=params,
        target_speed_rpm=effective_target_speed_rpm,
        rated_current_a=rated_current,
        fixed_current_pi=best_candidate.current_pi,
        iq_limit_anchor_a=best_candidate.iq_limit_a,
        max_candidates=max(speed_stage_budget, 1),
    )

    speed_anchor = Candidate(
        speed_pi=(float(speed_seed_pi[0]), float(speed_seed_pi[1])),
        current_pi=(
            float(best_candidate.current_pi[0]),
            float(best_candidate.current_pi[1]),
        ),
        iq_limit_a=float(best_candidate.iq_limit_a),
        startup=dict(best_candidate.startup),
    )
    if _can_run_more_trials(tested):
        speed_anchor_trial = _run_trial(
            base_motor_params=base_motor_params,
            rated_speed_rpm=effective_target_speed_rpm,
            candidate=speed_anchor,
            sim_time_s=search_time_s,
            dt=1.0 / 3500.0,
            tolerance_ratio=tolerance_ratio,
            static_error_rpm_limit=static_error_rpm_limit,
            max_sim_time_s=search_max_time_s,
            overcurrent_limit_a=effective_overcurrent_limit_a,
        )
        tested += 1
        _append_trial_debug(
            phase="speed_stage_anchor",
            trial_idx=tested,
            cand=speed_anchor,
            trial_result=speed_anchor_trial,
            trial_time_s=float(search_time_s),
            trial_dt=float(1.0 / 3500.0),
        )
        if speed_anchor_trial["score"] < best_trial["score"]:
            best_trial = speed_anchor_trial
            best_candidate = speed_anchor

    speed_stage_end = current_stage_budget + speed_stage_budget
    for cand in speed_candidates:
        if tested >= speed_stage_end or not _can_run_more_trials(tested):
            break
        trial = _run_trial(
            base_motor_params=base_motor_params,
            rated_speed_rpm=effective_target_speed_rpm,
            candidate=cand,
            sim_time_s=search_time_s,
            dt=1.0 / 3500.0,
            tolerance_ratio=tolerance_ratio,
            static_error_rpm_limit=static_error_rpm_limit,
            max_sim_time_s=search_max_time_s,
            overcurrent_limit_a=effective_overcurrent_limit_a,
        )
        tested += 1
        _append_trial_debug(
            phase="speed_stage_search",
            trial_idx=tested,
            cand=cand,
            trial_result=trial,
            trial_time_s=float(search_time_s),
            trial_dt=float(1.0 / 3500.0),
        )
        if trial["score"] < best_trial["score"]:
            best_trial = trial
            best_candidate = cand

    # Local refinement around the best speed candidate while keeping current PI fixed.
    if best_candidate is not None and not bool(best_trial.get("full_converged", False)):
        for cand in _build_speed_neighbors(
            best_candidate, rated_current_a=rated_current
        ):
            if not _can_run_more_trials(tested):
                break
            trial = _run_trial(
                base_motor_params=base_motor_params,
                rated_speed_rpm=effective_target_speed_rpm,
                candidate=cand,
                sim_time_s=search_time_s,
                dt=1.0 / 5000.0,
                tolerance_ratio=tolerance_ratio,
                static_error_rpm_limit=static_error_rpm_limit,
                max_sim_time_s=search_max_time_s,
                overcurrent_limit_a=effective_overcurrent_limit_a,
            )
            tested += 1
            _append_trial_debug(
                phase="speed_stage_refine",
                trial_idx=tested,
                cand=cand,
                trial_result=trial,
                trial_time_s=float(search_time_s),
                trial_dt=float(1.0 / 5000.0),
            )
            if trial["score"] < best_trial["score"]:
                best_trial = trial
                best_candidate = cand
            if trial["full_converged"]:
                break

    # Unbounded mode: keep expanding neighborhood around the best candidate
    # until full convergence is achieved or trial limit is reached.
    expansion_round = 0
    while (
        trial_limit is None
        and best_candidate is not None
        and not bool(best_trial.get("full_converged", False))
        and _can_run_more_trials(tested)
    ):
        expansion_round += 1
        expansion_scale = 1.0 + min(0.15 * expansion_round, 1.5)
        expanded_seed = Candidate(
            speed_pi=(
                max(1.0e-4, float(best_candidate.speed_pi[0]) * expansion_scale),
                max(1.0e-4, float(best_candidate.speed_pi[1]) * expansion_scale),
            ),
            current_pi=(
                max(1.0e-4, float(best_candidate.current_pi[0]) * expansion_scale),
                max(1.0e-4, float(best_candidate.current_pi[1]) * expansion_scale),
            ),
            iq_limit_a=max(
                float(best_candidate.iq_limit_a),
                min(4.0 * float(rated_current), float(best_candidate.iq_limit_a) * 1.2),
            ),
            startup=dict(best_candidate.startup),
        )

        for cand in _build_neighbors(expanded_seed):
            if not _can_run_more_trials(tested):
                break
            trial = _run_trial(
                base_motor_params=base_motor_params,
                rated_speed_rpm=effective_target_speed_rpm,
                candidate=cand,
                sim_time_s=search_time_s,
                dt=1.0 / 6000.0,
                tolerance_ratio=tolerance_ratio,
                static_error_rpm_limit=static_error_rpm_limit,
                max_sim_time_s=search_max_time_s,
                overcurrent_limit_a=effective_overcurrent_limit_a,
            )
            tested += 1
            _append_trial_debug(
                phase=f"unbounded_refine_round_{expansion_round}",
                trial_idx=tested,
                cand=cand,
                trial_result=trial,
                trial_time_s=float(search_time_s),
                trial_dt=float(1.0 / 6000.0),
            )

            if trial["score"] < best_trial["score"]:
                best_trial = trial
                best_candidate = cand
            if trial["full_converged"]:
                break

    if best_trial is None or best_candidate is None:
        raise RuntimeError("No trial executed")

    # Long verification of best candidate.
    verification = _run_trial(
        base_motor_params=base_motor_params,
        rated_speed_rpm=effective_target_speed_rpm,
        candidate=best_candidate,
        sim_time_s=verify_time_s,
        dt=1.0 / 8000.0,
        tolerance_ratio=tolerance_ratio,
        static_error_rpm_limit=static_error_rpm_limit,
        max_sim_time_s=verify_max_time_s,
        overcurrent_limit_a=effective_overcurrent_limit_a,
    )
    _append_trial_debug(
        phase="verification",
        trial_idx=tested + 1,
        cand=best_candidate,
        trial_result=verification,
        trial_time_s=float(verify_time_s),
        trial_dt=float(1.0 / 8000.0),
    )

    accepted = bool(verification["full_converged"])

    return {
        "schema": "bldc.auto_tune_until_convergence.v2",
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "profile": {
            "name": profile.get("profile_name", profile_path.stem),
            "file": profile_path.name,
        },
        "search_setup": {
            "max_trials": int(max_trials),
            "search_time_s": float(search_time_s),
            "verify_time_s": float(verify_time_s),
            "search_max_time_s": float(search_max_time_s),
            "verify_max_time_s": float(verify_max_time_s),
            "overcurrent_limit_a": None
            if effective_overcurrent_limit_a is None
            else float(effective_overcurrent_limit_a),
            "trial_limit_mode": "unbounded" if trial_limit is None else "bounded",
            "tolerance_ratio": float(tolerance_ratio),
            "static_error_rpm_limit": float(static_error_rpm_limit),
            "field_weakening_enabled": False,
            "sensored_pi_tuning": True,
            "startup_open_loop_calibration": True,
            "candidate_strategy": "sequential_current_then_speed",
            "feasibility_gate": "no_fw_voltage_headroom",
            "requested_target_speed_rpm": float(requested_target_rpm),
            "effective_target_speed_rpm": float(effective_target_speed_rpm),
            "effective_target_reason": target_reason,
            "pre_tuning_speed_ramp": {
                "enabled": True,
                "steady_state_entry_speed_rpm": float(startup_pref_speed_rpm),
                "vf_ratio_applied_in_open_loop": True,
            },
            "candidate_pool_sizes": {
                "current_stage_budget": int(current_stage_budget),
                "speed_stage_budget": int(speed_stage_budget),
                "max_trials_effective": int(_trial_cap_value()),
            },
            "speed_pi_design_reference": speed_pi_reference,
        },
        "best_candidate": {
            "speed_pi": {
                "kp": float(best_candidate.speed_pi[0]),
                "ki": float(best_candidate.speed_pi[1]),
                "kaw": 0.05,
            },
            "current_pi": {
                "d_kp": float(best_candidate.current_pi[0]),
                "d_ki": float(best_candidate.current_pi[1]),
                "q_kp": float(best_candidate.current_pi[0]),
                "q_ki": float(best_candidate.current_pi[1]),
                "kaw": 0.2,
            },
            "iq_limit_a": float(best_candidate.iq_limit_a),
            "observer": {"mode": "Measured"},
            "startup_sequence": best_candidate.startup,
        },
        "motor_params": base_motor_params,
        "feasibility": feasibility,
        "results": {
            "best_search_trial": best_trial,
            "verification": verification,
            "tested_trials": int(tested),
            "trial_limit": int(_trial_cap_value()),
            "trial_debug": trial_debug,
        },
        "acceptance": {
            "accepted": accepted,
            "reason": "full convergence achieved"
            if accepted
            else "full convergence not achieved",
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Auto-tune until rated-speed convergence without field weakening"
    )
    parser.add_argument(
        "--profiles-dir",
        default=str(PROJECT_ROOT / "data" / "motor_profiles"),
        help="Directory containing motor profile JSON files",
    )
    parser.add_argument(
        "--max-trials",
        type=int,
        default=700,
        help="Maximum trials per profile; use <=0 for unbounded iterative search",
    )
    parser.add_argument(
        "--search-time",
        type=float,
        default=0.5,
        help="Simulation time in seconds for each search trial",
    )
    parser.add_argument(
        "--verify-time",
        type=float,
        default=5.0,
        help="Long simulation time in seconds for final verification",
    )
    parser.add_argument(
        "--search-max-time",
        type=float,
        default=20.0,
        help="Maximum search-trial runtime while stable (safety cap)",
    )
    parser.add_argument(
        "--verify-max-time",
        type=float,
        default=40.0,
        help="Maximum verification runtime while stable (safety cap)",
    )
    parser.add_argument(
        "--overcurrent-limit-a",
        type=float,
        default=0.0,
        help="Abort/restart threshold for absolute phase current (A). <=0 disables overcurrent abort",
    )
    parser.add_argument(
        "--tolerance-ratio",
        type=float,
        default=0.02,
        help="Speed convergence tolerance ratio (e.g. 0.02 => 2%)",
    )
    parser.add_argument(
        "--static-error-rpm-limit",
        type=float,
        default=10.0,
        help="Maximum allowed absolute steady-state speed error in RPM",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducible search",
    )
    parser.add_argument(
        "--target-speed-rpm",
        type=float,
        default=400.0,
        help="Target speed for current tuning campaign (RPM). Use <=0 to keep rated speed.",
    )
    args = parser.parse_args()

    profiles_dir = Path(args.profiles_dir)
    profile_paths = list_motor_profiles(profiles_dir)
    if not profile_paths:
        raise FileNotFoundError(f"No profile found in {profiles_dir}")

    sessions_dir = PROJECT_ROOT / "data" / "tuning_sessions" / "until_converged"
    sessions_dir.mkdir(parents=True, exist_ok=True)

    summary = {
        "schema": "bldc.auto_tune_until_convergence_summary.v2",
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "search_setup": {
            "max_trials": int(args.max_trials),
            "search_time_s": float(args.search_time),
            "verify_time_s": float(args.verify_time),
            "search_max_time_s": float(args.search_max_time),
            "verify_max_time_s": float(args.verify_max_time),
            "overcurrent_limit_a": float(args.overcurrent_limit_a),
            "tolerance_ratio": float(args.tolerance_ratio),
            "target_speed_rpm": float(args.target_speed_rpm),
            "seed": int(args.seed),
        },
        "results": [],
        "errors": [],
    }

    for idx, profile_path in enumerate(profile_paths):
        try:
            session = tune_one_profile_until_converged(
                profile_path=profile_path,
                max_trials=int(args.max_trials),
                search_time_s=float(args.search_time),
                verify_time_s=float(args.verify_time),
                tolerance_ratio=float(args.tolerance_ratio),
                static_error_rpm_limit=float(args.static_error_rpm_limit),
                target_speed_rpm=float(args.target_speed_rpm),
                seed=int(args.seed) + idx,
                search_max_time_s=float(args.search_max_time),
                verify_max_time_s=float(args.verify_max_time),
                overcurrent_limit_a=float(args.overcurrent_limit_a),
            )

            session_path = sessions_dir / f"{profile_path.stem}_until_converged.json"
            session_path.write_text(json.dumps(session, indent=2), encoding="utf-8")

            v = session["results"]["verification"]
            summary["results"].append(
                {
                    "profile": session["profile"]["name"],
                    "file": session["profile"]["file"],
                    "session_file": session_path.name,
                    "accepted": bool(
                        session.get("acceptance", {}).get("accepted", False)
                    ),
                    "full_converged": bool(v.get("full_converged", False)),
                    "final_speed_rpm": float(v.get("final_speed_rpm", 0.0)),
                    "final_speed_ratio": float(v.get("final_speed_ratio", 0.0)),
                    "tested_trials": int(session["results"]["tested_trials"]),
                }
            )

            print(
                json.dumps(
                    {
                        "profile": session["profile"]["name"],
                        "accepted": bool(
                            session.get("acceptance", {}).get("accepted", False)
                        ),
                        "full_converged": bool(v.get("full_converged", False)),
                        "final_speed_ratio": float(v["final_speed_ratio"]),
                        "tested_trials": int(session["results"]["tested_trials"]),
                    }
                )
            )

        except Exception as exc:
            summary["errors"].append(
                {
                    "file": profile_path.name,
                    "error": str(exc),
                    "traceback": traceback.format_exc(),
                }
            )
            print(
                json.dumps(
                    {
                        "file": profile_path.name,
                        "error": str(exc),
                        "traceback": traceback.format_exc(),
                    }
                )
            )

    summary_path = (
        PROJECT_ROOT / "data" / "logs" / "motor_profiles_until_converged_summary.json"
    )
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"SUMMARY_SAVED {summary_path}")


if __name__ == "__main__":
    main()
