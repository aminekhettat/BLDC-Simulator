"""Rated-speed convergence search with phase-order check and field weakening.

This script:
1) finds a positive-rotation phase sequence,
2) tunes PI + field-weakening parameters,
3) reports progress percentages continuously.
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
    sequence: str
    dt: float
    speed_kp: float
    speed_ki: float
    current_kp: float
    current_ki: float
    iq_limit_a: float
    field_weakening_enabled: bool
    fw_start_rpm: float
    fw_gain: float
    fw_id_max_a: float


@dataclass
class Result:
    stable: bool
    convergent: bool
    final_speed_rpm: float
    final_err_rpm: float
    tail_abs_mean_err_rpm: float
    tail_abs_max_err_rpm: float
    first_half_abs_mean_err_rpm: float
    second_half_abs_mean_err_rpm: float
    neg_speed: bool


def build_params() -> MotorParameters:
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


def apply_sequence(vabc: np.ndarray, sequence: str) -> np.ndarray:
    perm = {
        "UVW": (0, 1, 2),
        "UWV": (0, 2, 1),
        "VUW": (1, 0, 2),
        "VWU": (1, 2, 0),
        "WUV": (2, 0, 1),
        "WVU": (2, 1, 0),
    }[sequence]
    return np.array([vabc[perm[0]], vabc[perm[1]], vabc[perm[2]]], dtype=np.float64)


def run_candidate(
    params: MotorParameters,
    cand: Candidate,
    speed_ref_rpm: float,
    sim_time_s: float,
    load_torque_nm: float,
) -> Result:
    steps = max(200, int(sim_time_s / cand.dt))
    sample_stride = max(1, steps // 2500)

    motor = BLDCMotor(params, dt=cand.dt)
    engine = SimulationEngine(
        motor,
        ConstantLoad(load_torque_nm),
        dt=cand.dt,
        compute_backend="cpu",
        max_history=5000,
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
    ctrl.set_speed_reference(speed_ref_rpm)
    ctrl.set_angle_observer("Measured")
    ctrl.set_field_weakening(
        enabled=cand.field_weakening_enabled,
        start_speed_rpm=cand.fw_start_rpm,
        gain=cand.fw_gain,
        max_negative_id_a=cand.fw_id_max_a,
    )

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(cand.dt)

    sampled_speed: list[float] = []
    stable = True
    neg_speed = False

    for k in range(steps):
        svm.set_phase_currents(motor.currents)
        mag, ang = ctrl.update(cand.dt)
        vabc = svm.modulate(mag, ang)
        phase_v = apply_sequence(vabc, cand.sequence)
        engine.step(phase_v, log_data=False)

        if motor.speed_rpm < -10.0:
            neg_speed = True

        if (k % sample_stride) == 0 or k == steps - 1:
            sampled_speed.append(float(motor.speed_rpm))

        if (not np.isfinite(motor.omega)) or abs(motor.speed_rpm) > 1.0e6:
            stable = False
            break

    speed = np.array(sampled_speed, dtype=np.float64)
    if speed.size < 40:
        return Result(
            False,
            False,
            float(speed[-1]) if speed.size else 0.0,
            float("inf"),
            float("inf"),
            float("inf"),
            float("inf"),
            float("inf"),
            neg_speed,
        )

    err = speed_ref_rpm - speed
    half = len(err) // 2
    tail = max(200, len(err) // 10)

    first = float(np.mean(np.abs(err[:half])))
    second = float(np.mean(np.abs(err[half:])))
    tail_mean = float(np.mean(np.abs(err[-tail:])))
    tail_max = float(np.max(np.abs(err[-tail:])))
    final = float(speed[-1])
    ferr = float(speed_ref_rpm - final)

    convergent = bool(
        stable
        and (not neg_speed)
        and final > 0.0
        and abs(ferr) <= 0.05 * speed_ref_rpm
        and tail_mean <= 0.05 * speed_ref_rpm
        and tail_max <= 0.10 * speed_ref_rpm
        and second < first
    )

    return Result(stable, convergent, final, ferr, tail_mean, tail_max, first, second, neg_speed)


def score(res: Result) -> float:
    if not res.stable:
        return 1e18
    pen = 0.0
    if res.neg_speed:
        pen += 1e6
    if res.final_speed_rpm <= 0.0:
        pen += 1e6
    if res.second_half_abs_mean_err_rpm >= res.first_half_abs_mean_err_rpm:
        pen += 2e5
    return (
        res.tail_abs_mean_err_rpm
        + 0.2 * res.tail_abs_max_err_rpm
        + 0.1 * abs(res.final_err_rpm)
        + pen
    )


def main() -> None:
    params = build_params()
    speed_ref_rpm = 31500.0
    load_torque_nm = 0.0
    enable_field_weakening = True

    # Stage 1: phase order check
    phase_cands = [
        Candidate(
            seq,
            2e-5,
            8e-4,
            2e-2,
            5.0,
            50.0,
            8.0,
            False,
            14000.0,
            1.0,
            12.0,
        )
        for seq in ["UVW", "UWV", "VUW", "VWU", "WUV", "WVU"]
    ]

    print("Starting rated-speed field-weakening search", flush=True)
    print(
        json.dumps(
            {
                "target_rpm": speed_ref_rpm,
                "phase_candidates": len(phase_cands),
                "field_weakening_enabled": enable_field_weakening,
            }
        ),
        flush=True,
    )

    best_phase = None
    best_phase_score = float("inf")

    for i, c in enumerate(phase_cands, start=1):
        r = run_candidate(params, c, speed_ref_rpm, sim_time_s=0.15, load_torque_nm=load_torque_nm)
        s = score(r)
        if s < best_phase_score:
            best_phase = c
            best_phase_score = s
        print(
            f"PHASE_PROGRESS {i}/{len(phase_cands)} ({100 * i / len(phase_cands):.2f}%) seq={c.sequence} final={r.final_speed_rpm:.1f} neg={r.neg_speed} stable={r.stable}",
            flush=True,
        )

    if best_phase is None:
        print("NO_CONVERGENCE_FOUND", flush=True)
        return

    print(f"PHASE_SELECTION_DONE selected={best_phase.sequence}", flush=True)

    # Stage 2: tuning with field weakening on selected sequence
    tuning: list[Candidate] = []
    for dt in [1e-5]:
        for skp in [8e-4, 1.5e-3, 3e-3]:
            for ski in [2e-2, 5e-2, 1e-1]:
                for ckp in [5.0, 10.0, 20.0]:
                    for cki in [50.0, 200.0, 500.0]:
                        for iql in [8.0, 12.0, 20.0]:
                            for fw_start in [12000.0, 15000.0, 18000.0]:
                                for fw_gain in [0.7, 1.0, 1.3]:
                                    for fw_idmax in [8.0, 12.0, 18.0]:
                                        tuning.append(
                                            Candidate(
                                                best_phase.sequence,
                                                dt,
                                                skp,
                                                ski,
                                                ckp,
                                                cki,
                                                iql,
                                                enable_field_weakening,
                                                fw_start,
                                                fw_gain,
                                                fw_idmax,
                                            )
                                        )

    # Keep runtime practical: evaluate deterministic subset.
    tuning = tuning[:540]
    total = len(tuning)
    print(json.dumps({"stage": "field_weakening_tuning", "candidates": total}), flush=True)

    best_cand = None
    best_res = None
    best_s = float("inf")

    for i, c in enumerate(tuning, start=1):
        r = run_candidate(params, c, speed_ref_rpm, sim_time_s=0.45, load_torque_nm=load_torque_nm)
        s = score(r)
        if s < best_s:
            best_s = s
            best_cand = c
            best_res = r

        if i == 1 or i % 3 == 0 or r.convergent or i == total:
            print(
                f"TUNING_PROGRESS {i}/{total} ({100 * i / total:.2f}%) "
                f"final={r.final_speed_rpm:.1f} err={r.final_err_rpm:.1f} conv={r.convergent} best_score={best_s:.2f}",
                flush=True,
            )

        if r.convergent:
            out = {"candidate": asdict(c), "metrics": asdict(r)}
            out_path = PROJECT_ROOT / "data" / "logs" / "rated_speed_field_weakening_result.json"
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(json.dumps(out, indent=2), encoding="utf-8")
            print("CONVERGENCE_FOUND", flush=True)
            print(json.dumps(out, indent=2), flush=True)
            print(f"Result saved to: {out_path}", flush=True)
            return

    print("NO_CONVERGENCE_FOUND", flush=True)
    if best_cand and best_res:
        print(
            json.dumps(
                {
                    "best_candidate": asdict(best_cand),
                    "best_metrics": asdict(best_res),
                    "best_score": best_s,
                },
                indent=2,
            ),
            flush=True,
        )


if __name__ == "__main__":
    main()
