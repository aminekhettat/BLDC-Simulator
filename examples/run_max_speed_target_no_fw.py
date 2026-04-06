"""Target model-limited maximum speed with field weakening disabled.

This script computes an estimated no-field-weakening speed ceiling from motor
constants and DC bus voltage, then searches controller gains (with adaptive
seeding) to converge near that target.

Field weakening is intentionally disabled to measure baseline capability.
"""

from __future__ import annotations

import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.control import AdaptiveFOCTuner, FOCController, SVMGenerator
from src.core.load_model import ConstantLoad
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.simulation_engine import SimulationEngine


@dataclass
class Candidate:
    dt: float
    speed_kp: float
    speed_ki: float
    current_kp: float
    current_ki: float
    iq_limit_a: float


@dataclass
class Metrics:
    stable: bool
    converged: bool
    final_speed_rpm: float
    final_err_rpm: float
    tail_abs_mean_err_rpm: float
    tail_abs_max_err_rpm: float
    first_half_abs_mean_err_rpm: float
    second_half_abs_mean_err_rpm: float


def build_12v_params() -> MotorParameters:
    kt = 3.51e-3
    rotor_inertia = 0.08e-7
    omega_no_load = 31500.0 * 2.0 * np.pi / 60.0
    friction_coeff = (kt * 0.047) / max(omega_no_load, 1e-12)
    return MotorParameters(
        model_type="dq",
        emf_shape="sinusoidal",
        nominal_voltage=12.0,
        phase_resistance=12.0,
        phase_inductance=132e-6,
        back_emf_constant=kt,
        torque_constant=kt,
        rotor_inertia=rotor_inertia,
        friction_coefficient=friction_coeff,
        num_poles=2,
        poles_pairs=1,
    )


def estimate_max_speed_no_fw_rpm(
    params: MotorParameters,
    dc_voltage: float,
    utilization_margin: float = 0.95,
) -> float:
    """Estimate model speed ceiling without field weakening.

    Uses a conservative linear SVM voltage limit: Vmax ~= Vdc / sqrt(3).
    """
    ke = max(float(params.back_emf_constant), 1e-12)
    v_max = (float(dc_voltage) / np.sqrt(3.0)) * float(np.clip(utilization_margin, 0.1, 1.0))
    omega_max = v_max / ke
    return float(omega_max * 60.0 / (2.0 * np.pi))


def run_candidate(
    params: MotorParameters,
    candidate: Candidate,
    speed_ref_rpm: float,
    load_torque_nm: float,
    sim_time_s: float,
) -> Metrics:
    steps = max(300, int(sim_time_s / candidate.dt))
    sample_stride = max(1, steps // 2500)

    motor = BLDCMotor(params, dt=candidate.dt)
    engine = SimulationEngine(
        motor,
        ConstantLoad(load_torque_nm),
        dt=candidate.dt,
        compute_backend="cpu",
        max_history=6000,
    )

    ctrl = FOCController(motor=motor, enable_speed_loop=True)
    ctrl.set_cascaded_speed_loop(True, iq_limit_a=candidate.iq_limit_a)
    ctrl.set_speed_pi_gains(candidate.speed_kp, candidate.speed_ki, kaw=0.05)
    ctrl.set_current_pi_gains(
        d_kp=candidate.current_kp,
        d_ki=candidate.current_ki,
        q_kp=candidate.current_kp,
        q_ki=candidate.current_ki,
        kaw=0.2,
    )
    ctrl.set_speed_reference(speed_ref_rpm)
    ctrl.set_angle_observer("Measured")
    ctrl.set_field_weakening(
        enabled=False,
        start_speed_rpm=0.0,
        gain=0.0,
        max_negative_id_a=0.0,
    )

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(candidate.dt)

    sampled_speed: list[float] = []
    stable = True

    for k in range(steps):
        svm.set_phase_currents(motor.currents)
        mag, ang = ctrl.update(candidate.dt)
        phase_v = svm.modulate(mag, ang)
        engine.step(phase_v, log_data=False)

        if (k % sample_stride) == 0 or k == steps - 1:
            sampled_speed.append(float(motor.speed_rpm))

        if (not np.isfinite(motor.omega)) or abs(motor.speed_rpm) > 1e6:
            stable = False
            break

    speed = np.asarray(sampled_speed, dtype=np.float64)
    if speed.size < 50:
        return Metrics(
            False,
            False,
            float(speed[-1]) if speed.size else 0.0,
            float("inf"),
            float("inf"),
            float("inf"),
            float("inf"),
            float("inf"),
        )

    err = speed_ref_rpm - speed
    half = len(err) // 2
    tail = max(120, len(err) // 10)

    first_half = float(np.mean(np.abs(err[:half])))
    second_half = float(np.mean(np.abs(err[half:])))
    tail_mean = float(np.mean(np.abs(err[-tail:])))
    tail_max = float(np.max(np.abs(err[-tail:])))
    final_speed = float(speed[-1])
    final_err = float(speed_ref_rpm - final_speed)

    converged = bool(
        stable
        and final_speed > 0.0
        and abs(final_err) <= 0.06 * speed_ref_rpm
        and tail_mean <= 0.06 * speed_ref_rpm
        and tail_max <= 0.12 * speed_ref_rpm
        and second_half < first_half
    )

    return Metrics(
        stable=stable,
        converged=converged,
        final_speed_rpm=final_speed,
        final_err_rpm=final_err,
        tail_abs_mean_err_rpm=tail_mean,
        tail_abs_max_err_rpm=tail_max,
        first_half_abs_mean_err_rpm=first_half,
        second_half_abs_mean_err_rpm=second_half,
    )


def score(metrics: Metrics) -> float:
    if not metrics.stable:
        return 1e18
    trend_penalty = 0.0
    if metrics.second_half_abs_mean_err_rpm >= metrics.first_half_abs_mean_err_rpm:
        trend_penalty = 2e5
    return (
        metrics.tail_abs_mean_err_rpm
        + 0.25 * metrics.tail_abs_max_err_rpm
        + 0.1 * abs(metrics.final_err_rpm)
        + trend_penalty
    )


def main() -> None:
    params = build_12v_params()
    target_rpm = estimate_max_speed_no_fw_rpm(params, dc_voltage=params.nominal_voltage)
    load_torque_nm = 0.0

    tuner = AdaptiveFOCTuner(params)
    tuned = tuner.tune(grid_size=8)

    seed = Candidate(
        dt=1.0 / 20000.0,
        speed_kp=tuned.speed_kp,
        speed_ki=tuned.speed_ki,
        current_kp=tuned.current_kp,
        current_ki=tuned.current_ki,
        iq_limit_a=8.0,
    )

    candidates: list[Candidate] = []
    for dt in [1.0 / 20000.0, 1.0 / 40000.0]:
        for speed_scale in [0.5, 1.0, 2.0, 4.0]:
            for current_scale in [0.5, 1.0, 2.0]:
                for iq_limit in [4.0, 8.0, 12.0, 20.0]:
                    candidates.append(
                        Candidate(
                            dt=dt,
                            speed_kp=seed.speed_kp * speed_scale,
                            speed_ki=max(seed.speed_ki * speed_scale, 1e-6),
                            current_kp=seed.current_kp * current_scale,
                            current_ki=max(seed.current_ki * current_scale, 1e-6),
                            iq_limit_a=iq_limit,
                        )
                    )

    best_cand = None
    best_metrics = None
    best_score = float("inf")

    print("Starting max-speed no-field-weakening search", flush=True)
    print(
        json.dumps({"target_rpm": target_rpm, "candidates": len(candidates)}),
        flush=True,
    )

    for i, cand in enumerate(candidates, start=1):
        metrics = run_candidate(
            params=params,
            candidate=cand,
            speed_ref_rpm=target_rpm,
            load_torque_nm=load_torque_nm,
            sim_time_s=0.25,
        )
        s = score(metrics)
        if s < best_score:
            best_score = s
            best_cand = cand
            best_metrics = metrics

        if i == 1 or i % 5 == 0 or metrics.converged or i == len(candidates):
            print(
                f"SEARCH_PROGRESS {i}/{len(candidates)} ({100.0 * i / len(candidates):.2f}%) "
                f"conv={metrics.converged} final={metrics.final_speed_rpm:.1f} best_score={best_score:.2f}",
                flush=True,
            )

        if metrics.converged:
            break

    if best_cand is None or best_metrics is None:
        raise RuntimeError("No candidate evaluated")

    strict_metrics = run_candidate(
        params=params,
        candidate=best_cand,
        speed_ref_rpm=target_rpm,
        load_torque_nm=load_torque_nm,
        sim_time_s=0.6,
    )

    result = {
        "target": {
            "mode": "max_speed_no_field_weakening",
            "estimated_max_speed_rpm": target_rpm,
            "field_weakening_enabled": False,
        },
        "adaptive_seed": {
            "current_kp": tuned.current_kp,
            "current_ki": tuned.current_ki,
            "speed_kp": tuned.speed_kp,
            "speed_ki": tuned.speed_ki,
            "current_margin_phase_deg": tuned.current_margin.phase_margin_deg,
            "speed_margin_phase_deg": tuned.speed_margin.phase_margin_deg,
        },
        "best_candidate": asdict(best_cand),
        "coarse_metrics": asdict(best_metrics),
        "strict_metrics": asdict(strict_metrics),
    }

    out_path = PROJECT_ROOT / "data" / "logs" / "max_speed_target_no_fw_result.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if strict_metrics.converged:
        print("CONVERGENCE_FOUND", flush=True)
    else:
        print("NO_CONVERGENCE_FOUND", flush=True)
    print(json.dumps(result, indent=2), flush=True)
    print(f"Result saved to: {out_path}", flush=True)


if __name__ == "__main__":
    main()
