"""Search for a convergent configuration at motor rated speed.

The search includes phase sequence permutations to catch wiring/order issues
(e.g., UVW vs UWV) that can cause negative speed.
"""

from __future__ import annotations

import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.control import FOCController, SVMGenerator
from src.core.load_model import ConstantLoad
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.simulation_engine import SimulationEngine


@dataclass
class Candidate:
    phase_sequence: str
    dt: float
    speed_kp: float
    speed_ki: float
    current_kp: float
    current_ki: float
    iq_limit_a: float


@dataclass
class Metrics:
    stable: bool
    convergent: bool
    final_speed_rpm: float
    final_err_rpm: float
    tail_abs_mean_err_rpm: float
    tail_abs_max_err_rpm: float
    first_half_abs_mean_err_rpm: float
    second_half_abs_mean_err_rpm: float
    negative_speed_detected: bool


def _build_12v_params() -> MotorParameters:
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


def _apply_phase_sequence(vabc: np.ndarray, sequence: str) -> np.ndarray:
    # Base convention from controller/SVM is UVW -> [A, B, C].
    # Permutations emulate alternate physical wiring orders.
    mapping = {
        "UVW": (0, 1, 2),
        "UWV": (0, 2, 1),
        "VUW": (1, 0, 2),
        "VWU": (1, 2, 0),
        "WUV": (2, 0, 1),
        "WVU": (2, 1, 0),
    }
    idx = mapping[sequence]
    return np.array([vabc[idx[0]], vabc[idx[1]], vabc[idx[2]]], dtype=np.float64)


def run_candidate(
    params: MotorParameters,
    cand: Candidate,
    speed_ref_rpm: float,
    load_torque_nm: float,
    sim_time_s: float,
) -> Metrics:
    steps = max(200, int(sim_time_s / cand.dt))
    sample_stride = max(1, steps // 2000)

    motor = BLDCMotor(params, dt=cand.dt)
    engine = SimulationEngine(
        motor,
        ConstantLoad(load_torque_nm),
        dt=cand.dt,
        max_history=max(steps + 100, 10000),
        compute_backend="cpu",
    )

    ctrl = FOCController(motor=motor, enable_speed_loop=True)
    ctrl.set_cascaded_speed_loop(True, iq_limit_a=cand.iq_limit_a)
    ctrl.set_speed_pi_gains(cand.speed_kp, cand.speed_ki, kaw=0.05)
    ctrl.set_current_pi_gains(
        d_kp=cand.current_kp,
        d_ki=cand.current_ki,
        q_kp=cand.current_kp,
        q_ki=cand.current_ki,
        kaw=0.2,
    )
    ctrl.set_current_references(id_ref=0.0, iq_ref=0.0)
    ctrl.set_speed_reference(speed_ref_rpm)
    ctrl.set_angle_observer("Measured")

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(engine.dt)

    stable = True
    neg_detected = False
    sampled_speed: list[float] = []
    for k in range(steps):
        svm.set_phase_currents(motor.currents)
        magnitude, angle = ctrl.update(engine.dt)
        raw_vabc = svm.modulate(magnitude, angle)
        phase_v = _apply_phase_sequence(raw_vabc, cand.phase_sequence)
        engine.step(phase_v, log_data=False)

        if motor.speed_rpm < -10.0:
            neg_detected = True

        if (k % sample_stride) == 0 or k == (steps - 1):
            sampled_speed.append(float(motor.speed_rpm))

        if (not np.isfinite(motor.omega)) or abs(motor.speed_rpm) > 1.0e6:
            stable = False
            break

    speed = np.array(sampled_speed, dtype=np.float64)

    if speed.size < 40:
        return Metrics(
            stable=False,
            convergent=False,
            final_speed_rpm=float(speed[-1]) if speed.size else 0.0,
            final_err_rpm=float("inf"),
            tail_abs_mean_err_rpm=float("inf"),
            tail_abs_max_err_rpm=float("inf"),
            first_half_abs_mean_err_rpm=float("inf"),
            second_half_abs_mean_err_rpm=float("inf"),
            negative_speed_detected=neg_detected,
        )

    err = speed_ref_rpm - speed
    half = len(err) // 2
    tail = max(200, len(err) // 10)

    first_half_abs_mean = float(np.mean(np.abs(err[:half])))
    second_half_abs_mean = float(np.mean(np.abs(err[half:])))
    tail_abs_mean = float(np.mean(np.abs(err[-tail:])))
    tail_abs_max = float(np.max(np.abs(err[-tail:])))
    final_speed = float(speed[-1])
    final_err = float(speed_ref_rpm - final_speed)

    convergent = bool(
        stable
        and (not neg_detected)
        and (final_speed > 0.0)
        and (abs(final_err) <= 0.05 * speed_ref_rpm)
        and (tail_abs_mean <= 0.05 * speed_ref_rpm)
        and (tail_abs_max <= 0.10 * speed_ref_rpm)
        and (second_half_abs_mean < first_half_abs_mean)
    )

    return Metrics(
        stable=stable,
        convergent=convergent,
        final_speed_rpm=final_speed,
        final_err_rpm=final_err,
        tail_abs_mean_err_rpm=tail_abs_mean,
        tail_abs_max_err_rpm=tail_abs_max,
        first_half_abs_mean_err_rpm=first_half_abs_mean,
        second_half_abs_mean_err_rpm=second_half_abs_mean,
        negative_speed_detected=neg_detected,
    )


def score(metrics: Metrics, speed_ref_rpm: float) -> float:
    if not metrics.stable:
        return 1e18
    penalty = 0.0
    if metrics.negative_speed_detected:
        penalty += 1e6
    if metrics.final_speed_rpm <= 0.0:
        penalty += 1e6
    trend_penalty = 0.0
    if metrics.second_half_abs_mean_err_rpm >= metrics.first_half_abs_mean_err_rpm:
        trend_penalty = 2e5
    return (
        metrics.tail_abs_mean_err_rpm
        + 0.2 * metrics.tail_abs_max_err_rpm
        + 0.1 * abs(metrics.final_err_rpm)
        + penalty
        + trend_penalty
    )


def main() -> None:
    params = _build_12v_params()

    # Rated/no-load speed from provided motor data.
    speed_ref_rpm = 31500.0
    load_torque_nm = 0.0
    # Stage 1 (phase sequence): quick sweep to detect wiring/order issues.
    sim_time_s = 0.12
    seqs = ["UVW", "UWV", "VUW", "VWU", "WUV", "WVU"]
    phase_cands = [
        Candidate(
            phase_sequence=seq,
            dt=2e-5,
            speed_kp=5e-4,
            speed_ki=1e-2,
            current_kp=1.0,
            current_ki=10.0,
            iq_limit_a=2.0,
        )
        for seq in seqs
    ]

    # Stage 2 (gains): only for best sequence.
    speed_kp_vals = [5e-4, 1e-3, 2e-3]
    speed_ki_vals = [1e-2, 5e-2]
    cur_kp_vals = [1.0, 5.0, 10.0]
    cur_ki_vals = [10.0, 50.0, 200.0]
    iq_limits = [3.0, 6.0, 10.0]
    dts = [1e-5]

    print("Starting rated-speed convergence search")
    print(
        json.dumps(
            {
                "speed_ref_rpm": speed_ref_rpm,
                "load_torque_nm": load_torque_nm,
                "sim_time_s": sim_time_s,
                "phase_candidates": len(phase_cands),
            }
        ),
        flush=True,
    )

    # ---- Stage 1: phase sequence ----
    best_phase_cand: Candidate | None = None
    best_phase_metrics: Metrics | None = None
    best_phase_score = float("inf")

    phase_total = len(phase_cands)
    for i, cand in enumerate(phase_cands, start=1):
        m = run_candidate(
            params=params,
            cand=cand,
            speed_ref_rpm=speed_ref_rpm,
            load_torque_nm=load_torque_nm,
            sim_time_s=sim_time_s,
        )
        s = score(m, speed_ref_rpm)
        if s < best_phase_score:
            best_phase_score = s
            best_phase_cand = cand
            best_phase_metrics = m

        pct = 100.0 * i / phase_total
        print(
            f"PHASE_PROGRESS {i}/{phase_total} ({pct:.2f}%) "
            f"seq={cand.phase_sequence} stable={m.stable} neg={m.negative_speed_detected} "
            f"final={m.final_speed_rpm:.1f} best_seq={best_phase_cand.phase_sequence if best_phase_cand else 'NA'}",
            flush=True,
        )

    if best_phase_cand is None or best_phase_metrics is None:
        print("NO_CONVERGENCE_FOUND", flush=True)
        return

    print(
        f"PHASE_SELECTION_DONE selected={best_phase_cand.phase_sequence} "
        f"final_speed={best_phase_metrics.final_speed_rpm:.1f}",
        flush=True,
    )

    # ---- Stage 2: gain tuning on selected sequence ----
    tuning_cands: list[Candidate] = []
    tuning_sim_time_s = 0.35
    for dt in dts:
        for skp in speed_kp_vals:
            for ski in speed_ki_vals:
                for ckp in cur_kp_vals:
                    for cki in cur_ki_vals:
                        for iql in iq_limits:
                            tuning_cands.append(
                                Candidate(
                                    phase_sequence=best_phase_cand.phase_sequence,
                                    dt=dt,
                                    speed_kp=skp,
                                    speed_ki=ski,
                                    current_kp=ckp,
                                    current_ki=cki,
                                    iq_limit_a=iql,
                                )
                            )

    print(
        json.dumps(
            {
                "stage": "gain_tuning",
                "selected_phase_sequence": best_phase_cand.phase_sequence,
                "candidates": len(tuning_cands),
            }
        ),
        flush=True,
    )

    best_cand: Candidate | None = None
    best_metrics: Metrics | None = None
    best_score = float("inf")

    total = len(tuning_cands)
    for i, cand in enumerate(tuning_cands, start=1):
        m = run_candidate(
            params=params,
            cand=cand,
            speed_ref_rpm=speed_ref_rpm,
            load_torque_nm=load_torque_nm,
            sim_time_s=tuning_sim_time_s,
        )
        s = score(m, speed_ref_rpm)

        if s < best_score:
            best_score = s
            best_cand = cand
            best_metrics = m

        pct = 100.0 * i / total
        if i == 1 or i % 3 == 0 or m.convergent or i == total:
            print(
                f"TUNING_PROGRESS {i}/{total} ({pct:.2f}%) "
                f"seq={cand.phase_sequence} stable={m.stable} neg={m.negative_speed_detected} "
                f"conv={m.convergent} final={m.final_speed_rpm:.1f} best_score={best_score:.2f}",
                flush=True,
            )

        if m.convergent:
            print("CONVERGENCE_FOUND", flush=True)
            print(
                json.dumps({"candidate": asdict(cand), "metrics": asdict(m)}, indent=2),
                flush=True,
            )
            out_path = PROJECT_ROOT / "data" / "logs" / "rated_speed_convergence_result.json"
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(
                json.dumps({"candidate": asdict(cand), "metrics": asdict(m)}, indent=2),
                encoding="utf-8",
            )
            print(f"Result saved to: {out_path}", flush=True)
            return

    print("NO_CONVERGENCE_FOUND", flush=True)
    if best_cand is not None and best_metrics is not None:
        print(
            json.dumps(
                {
                    "best_candidate": asdict(best_cand),
                    "best_metrics": asdict(best_metrics),
                    "best_score": best_score,
                },
                indent=2,
            ),
            flush=True,
        )


if __name__ == "__main__":
    main()
