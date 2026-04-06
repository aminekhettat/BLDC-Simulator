"""Automatically tune simulation and control parameters for 1000 RPM convergence.

This script searches over:
- Simulation step dt
- Observer mode and observer gains
- PI gains for current and speed loops
- iq current limit

It prints progress in percent so long runs can be monitored.
"""

from __future__ import annotations

import json
import math
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
    observer: str
    speed_kp: float
    speed_ki: float
    current_kp: float
    current_ki: float
    iq_limit_a: float
    pll_kp: float = 80.0
    pll_ki: float = 2000.0
    smo_k_slide: float = 600.0
    smo_lpf_alpha: float = 0.25
    smo_boundary: float = 0.03


@dataclass
class Result:
    stable: bool
    ok: bool
    reason: str
    sim_time_s: float
    final_speed_rpm: float
    final_err_rpm: float
    tail_abs_mean_err_rpm: float
    tail_abs_max_err_rpm: float
    first_half_abs_mean_err_rpm: float
    second_half_abs_mean_err_rpm: float
    settle_time_5pct_s: float | None


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


def run_case(
    params: MotorParameters,
    candidate: Candidate,
    speed_ref_rpm: float,
    load_torque_nm: float,
    sim_time_s: float,
) -> Result:
    steps = max(50, int(sim_time_s / candidate.dt))
    motor = BLDCMotor(params, dt=candidate.dt)
    engine = SimulationEngine(
        motor,
        ConstantLoad(load_torque_nm),
        dt=candidate.dt,
        max_history=max(steps + 100, 10000),
    )

    controller = FOCController(motor=motor, enable_speed_loop=True)
    controller.set_cascaded_speed_loop(True, iq_limit_a=candidate.iq_limit_a)
    controller.set_speed_pi_gains(
        kp=candidate.speed_kp,
        ki=candidate.speed_ki,
        kaw=0.05,
    )
    controller.set_current_pi_gains(
        d_kp=candidate.current_kp,
        d_ki=candidate.current_ki,
        q_kp=candidate.current_kp,
        q_ki=candidate.current_ki,
        kaw=0.2,
    )
    controller.set_speed_reference(speed_ref_rpm)
    controller.set_current_references(id_ref=0.0, iq_ref=0.0)
    controller.set_angle_observer(candidate.observer)

    if candidate.observer == "PLL":
        controller.set_pll_gains(candidate.pll_kp, candidate.pll_ki)
    elif candidate.observer == "SMO":
        controller.set_smo_gains(
            candidate.smo_k_slide,
            candidate.smo_lpf_alpha,
            candidate.smo_boundary,
        )

    # Keep startup transition deterministic for sensorless modes.
    controller.set_startup_transition(
        enabled=(candidate.observer != "Measured"),
        initial_mode="Measured",
        min_speed_rpm=120.0,
        min_elapsed_s=0.02,
        min_emf_v=0.02,
        min_confidence=0.2,
        confidence_hold_s=0.005,
        confidence_hysteresis=0.05,
        fallback_enabled=True,
        fallback_hold_s=0.01,
    )

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(candidate.dt)

    stable = True
    for _ in range(steps):
        svm.set_phase_currents(motor.currents)
        magnitude, angle = controller.update(candidate.dt)
        phase_voltages = svm.modulate(magnitude, angle)
        engine.step(phase_voltages, log_data=True)

        if not np.isfinite(motor.omega) or abs(motor.speed_rpm) > 1.0e6:
            stable = False
            break

    history = engine.get_history()
    speed = history["speed"]
    t = history["time"]

    if speed.size < 40:
        return Result(
            stable=False,
            ok=False,
            reason="insufficient_history",
            sim_time_s=float(t[-1]) if t.size else 0.0,
            final_speed_rpm=float(speed[-1]) if speed.size else 0.0,
            final_err_rpm=float(speed_ref_rpm),
            tail_abs_mean_err_rpm=float("inf"),
            tail_abs_max_err_rpm=float("inf"),
            first_half_abs_mean_err_rpm=float("inf"),
            second_half_abs_mean_err_rpm=float("inf"),
            settle_time_5pct_s=None,
        )

    err = speed_ref_rpm - speed
    half = len(err) // 2
    tail = max(300, len(err) // 8)

    first_half_abs_mean = float(np.mean(np.abs(err[:half])))
    second_half_abs_mean = float(np.mean(np.abs(err[half:])))
    tail_abs_mean = float(np.mean(np.abs(err[-tail:])))
    tail_abs_max = float(np.max(np.abs(err[-tail:])))

    band = 0.05 * speed_ref_rpm
    within = np.abs(err) <= band
    settle_idx = None
    false_indices = np.where(~within)[0]
    if false_indices.size == 0:
        settle_idx = 0
    else:
        candidate_idx = int(false_indices[-1] + 1)
        if candidate_idx < len(within):
            settle_idx = candidate_idx

    settle_time = float(t[settle_idx]) if settle_idx is not None else None
    final_speed = float(speed[-1])
    final_err = float(speed_ref_rpm - final_speed)

    # Strict convergence checks for the final acceptance.
    ok = bool(
        stable
        and (tail_abs_mean <= 0.04 * speed_ref_rpm)
        and (tail_abs_max <= 0.12 * speed_ref_rpm)
        and (abs(final_err) <= 0.05 * speed_ref_rpm)
        and (second_half_abs_mean < first_half_abs_mean)
    )

    return Result(
        stable=bool(stable),
        ok=ok,
        reason="ok" if ok else "not_converged",
        sim_time_s=float(t[-1]),
        final_speed_rpm=final_speed,
        final_err_rpm=final_err,
        tail_abs_mean_err_rpm=tail_abs_mean,
        tail_abs_max_err_rpm=tail_abs_max,
        first_half_abs_mean_err_rpm=first_half_abs_mean,
        second_half_abs_mean_err_rpm=second_half_abs_mean,
        settle_time_5pct_s=settle_time,
    )


def score(result: Result, speed_ref_rpm: float) -> float:
    if not result.stable:
        return 1.0e18
    penalty = 0.0
    if result.settle_time_5pct_s is None:
        penalty += 500.0
    return (
        result.tail_abs_mean_err_rpm
        + 0.25 * result.tail_abs_max_err_rpm
        + 0.1 * abs(result.final_err_rpm)
        + penalty
        + (
            0.0
            if result.second_half_abs_mean_err_rpm < result.first_half_abs_mean_err_rpm
            else 200.0
        )
    )


def dt_is_numerically_stable(
    params: MotorParameters,
    dt: float,
    load_torque_nm: float,
    steps: int = 200,
) -> bool:
    """Quick zero-voltage sanity check for integration stability at a given dt."""
    motor = BLDCMotor(params, dt=dt)
    zero_v = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    for _ in range(steps):
        motor.step(zero_v, load_torque=load_torque_nm)
        if not np.isfinite(motor.omega) or abs(motor.speed_rpm) > 1.0e6:
            return False
    return True


def make_candidates(params: MotorParameters) -> list[Candidate]:
    base = AdaptiveFOCTuner(params).tune(grid_size=12)

    dts = [5.0e-5, 2.0e-5, 1.0e-5]
    stable_dts = [dt for dt in dts if dt_is_numerically_stable(params, dt, 2.0e-4)]
    speed_scales = [0.05, 0.5, 5.0]
    current_scales = [0.01, 0.1, 1.0]
    iq_limits = [0.2, 0.8]

    pll_grid = [
        (40.0, 500.0),
        (80.0, 1200.0),
    ]
    smo_grid = [
        (200.0, 0.12, 0.05),
        (300.0, 0.15, 0.03),
    ]

    candidates: list[Candidate] = []
    for dt in stable_dts:
        for s in speed_scales:
            for c in current_scales:
                for iq_limit in iq_limits:
                    candidates.append(
                        Candidate(
                            dt=dt,
                            observer="Measured",
                            speed_kp=base.speed_kp * s,
                            speed_ki=base.speed_ki * s,
                            current_kp=base.current_kp * c,
                            current_ki=base.current_ki * c,
                            iq_limit_a=iq_limit,
                        )
                    )
                    for pll_kp, pll_ki in pll_grid:
                        candidates.append(
                            Candidate(
                                dt=dt,
                                observer="PLL",
                                speed_kp=base.speed_kp * s,
                                speed_ki=base.speed_ki * s,
                                current_kp=base.current_kp * c,
                                current_ki=base.current_ki * c,
                                iq_limit_a=iq_limit,
                                pll_kp=pll_kp,
                                pll_ki=pll_ki,
                            )
                        )
                    for k_slide, lpf_alpha, boundary in smo_grid:
                        candidates.append(
                            Candidate(
                                dt=dt,
                                observer="SMO",
                                speed_kp=base.speed_kp * s,
                                speed_ki=base.speed_ki * s,
                                current_kp=base.current_kp * c,
                                current_ki=base.current_ki * c,
                                iq_limit_a=iq_limit,
                                smo_k_slide=k_slide,
                                smo_lpf_alpha=lpf_alpha,
                                smo_boundary=boundary,
                            )
                        )
    return candidates


def print_progress(done: int, total: int, best_score: float) -> None:
    percent = 100.0 * done / max(total, 1)
    printable_score = best_score if math.isfinite(best_score) else float("inf")
    print(
        f"PROGRESS {done}/{total} ({percent:6.2f}%) best_score={printable_score:.3f}",
        flush=True,
    )


def main() -> None:
    speed_ref_rpm = 1000.0
    load_torque_nm = 2.0e-4

    params = build_12v_params()
    candidates = make_candidates(params)
    total = len(candidates)

    print(f"Starting automatic tuning for {speed_ref_rpm:.1f} RPM")
    print(f"Total candidates: {total}")

    best_candidate: Candidate | None = None
    best_result: Result | None = None
    best_score = float("inf")

    coarse_sim_time = 0.08
    strict_result: Result | None = None
    for i, cand in enumerate(candidates, start=1):
        res = run_case(
            params=params,
            candidate=cand,
            speed_ref_rpm=speed_ref_rpm,
            load_torque_nm=load_torque_nm,
            sim_time_s=coarse_sim_time,
        )

        sc = score(res, speed_ref_rpm)
        if sc < best_score:
            best_score = sc
            best_candidate = cand
            best_result = res

            # Early strict validation when coarse metrics become promising.
            if (
                res.stable
                and abs(res.final_err_rpm) <= 100.0
                and res.tail_abs_mean_err_rpm <= 120.0
                and res.second_half_abs_mean_err_rpm < res.first_half_abs_mean_err_rpm
            ):
                strict_result = run_case(
                    params=params,
                    candidate=best_candidate,
                    speed_ref_rpm=speed_ref_rpm,
                    load_torque_nm=load_torque_nm,
                    sim_time_s=0.8,
                )
                if strict_result.ok:
                    print("EARLY_STRICT_CONVERGENCE_FOUND", flush=True)
                    print_progress(i, total, best_score)
                    break

        if i == 1 or i % 5 == 0 or i == total:
            print_progress(i, total, best_score)

    if best_candidate is None or best_result is None:
        print("No valid candidate found.")
        return

    if strict_result is None:
        print("Best coarse candidate selected. Running strict validation...")
        strict_result = run_case(
            params=params,
            candidate=best_candidate,
            speed_ref_rpm=speed_ref_rpm,
            load_torque_nm=load_torque_nm,
            sim_time_s=0.8,
        )

    out = {
        "target_speed_rpm": speed_ref_rpm,
        "load_torque_nm": load_torque_nm,
        "best_candidate": asdict(best_candidate),
        "coarse_result": asdict(best_result),
        "strict_result": asdict(strict_result),
    }

    out_path = Path("data/logs/auto_tune_1000rpm_result.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("TUNING_COMPLETE")
    print(json.dumps(out, indent=2))
    print(f"Result saved to: {out_path}")


if __name__ == "__main__":
    main()
