"""Auto-tune FOC regulators and observer for saved motor profiles.

This script performs an optimization session for each profile in data/motor_profiles
and saves one result JSON per motor with the best found regulator and observer
parameters, plus a global summary report.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.control import AdaptiveFOCTuner, FOCController, SVMGenerator
from src.core.load_model import ConstantLoad
from src.core.motor_model import BLDCMotor, MotorParameters
from src.core.simulation_engine import SimulationEngine
from src.utils.motor_profiles import list_motor_profiles, load_motor_profile


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


def _build_search_candidates(
    base, deep: bool = False
) -> tuple[list[tuple[float, float]], list[tuple[float, float]], list[dict]]:
    speed_candidates = [
        (base.speed_kp * 0.4, base.speed_ki * 0.4),
        (base.speed_kp, base.speed_ki),
        (base.speed_kp * 2.0, base.speed_ki * 1.8),
    ]
    current_candidates = [
        (base.current_kp * 0.5, base.current_ki * 0.5),
        (base.current_kp, base.current_ki),
        (base.current_kp * 1.8, base.current_ki * 1.6),
    ]

    observer_candidates = [
        {"mode": "PLL", "pll_kp": 25.0, "pll_ki": 300.0},
        {"mode": "PLL", "pll_kp": 150.0, "pll_ki": 2800.0},
        {
            "mode": "SMO",
            "smo_k_slide": 40.0,
            "smo_lpf_alpha": 0.15,
            "smo_boundary": 0.05,
        },
        {
            "mode": "SMO",
            "smo_k_slide": 85.0,
            "smo_lpf_alpha": 0.25,
            "smo_boundary": 0.07,
        },
        {
            "mode": "SMO",
            "smo_k_slide": 120.0,
            "smo_lpf_alpha": 0.35,
            "smo_boundary": 0.10,
        },
    ]

    if deep:
        speed_candidates.extend(
            [
                (base.speed_kp * 0.2, base.speed_ki * 0.2),
                (base.speed_kp * 3.0, base.speed_ki * 2.8),
            ]
        )
        current_candidates.extend(
            [
                (base.current_kp * 0.3, base.current_ki * 0.3),
                (base.current_kp * 2.6, base.current_ki * 2.2),
            ]
        )
        observer_candidates.extend(
            [
                {"mode": "PLL", "pll_kp": 220.0, "pll_ki": 5000.0},
                {
                    "mode": "SMO",
                    "smo_k_slide": 180.0,
                    "smo_lpf_alpha": 0.45,
                    "smo_boundary": 0.12,
                },
            ]
        )

    return speed_candidates, current_candidates, observer_candidates


def _run_trial(
    params: MotorParameters,
    rated_speed_rpm: float,
    rated_current_a: float,
    speed_pi: tuple[float, float],
    current_pi: tuple[float, float],
    observer: dict,
    sim_time_s: float,
    dt: float,
) -> dict:
    steps = int(sim_time_s / dt)
    motor = BLDCMotor(params, dt=dt)
    engine = SimulationEngine(
        motor,
        ConstantLoad(0.0),
        dt=dt,
        compute_backend="cpu",
        max_history=4000,
    )

    controller = FOCController(motor=motor, enable_speed_loop=True)
    controller.set_cascaded_speed_loop(True, iq_limit_a=max(5.0, 0.9 * rated_current_a))
    controller.set_speed_pi_gains(kp=speed_pi[0], ki=speed_pi[1], kaw=0.05)
    controller.set_current_pi_gains(
        d_kp=current_pi[0],
        d_ki=current_pi[1],
        q_kp=current_pi[0],
        q_ki=current_pi[1],
        kaw=0.2,
    )
    controller.set_current_references(id_ref=0.0, iq_ref=0.0)
    controller.set_speed_reference(rated_speed_rpm)

    controller.set_angle_observer(observer["mode"])
    if observer["mode"] == "PLL":
        controller.set_pll_gains(observer["pll_kp"], observer["pll_ki"])
    else:
        controller.set_smo_gains(
            k_slide=observer["smo_k_slide"],
            lpf_alpha=observer["smo_lpf_alpha"],
            boundary=observer["smo_boundary"],
        )

    controller.set_startup_transition(
        enabled=False,
        initial_mode="Measured",
        min_speed_rpm=200.0,
        min_elapsed_s=0.1,
        min_emf_v=0.5,
        min_confidence=0.6,
        confidence_hold_s=0.05,
        confidence_hysteresis=0.1,
        fallback_enabled=False,
        fallback_hold_s=0.15,
    )
    controller.set_field_weakening(
        enabled=False,
        start_speed_rpm=1.0e9,
        gain=0.0,
        max_negative_id_a=0.0,
    )

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(dt)

    stride = max(1, steps // 1600)
    speed_samples = []
    conf_samples = []
    t_samples = []
    stable = True

    for k in range(steps):
        svm.set_phase_currents(motor.currents)
        magnitude, angle = controller.update(dt)
        phase_voltages = svm.modulate(magnitude, angle)
        engine.step(phase_voltages, log_data=False)

        if (k % stride) == 0 or k == steps - 1:
            speed_samples.append(float(motor.speed_rpm))
            conf_samples.append(float(controller.observer_confidence))
            t_samples.append(float((k + 1) * dt))

        if not np.isfinite(motor.omega) or abs(motor.speed_rpm) > 1.0e6:
            stable = False
            break

    if not speed_samples:
        return {
            "stable": False,
            "converged": False,
            "score": 1.0e12,
            "reason": "empty_samples",
        }

    speed = np.array(speed_samples, dtype=np.float64)
    conf = np.array(conf_samples, dtype=np.float64)
    t = np.array(t_samples, dtype=np.float64)

    err = rated_speed_rpm - speed
    band = max(0.05 * rated_speed_rpm, 25.0)

    tail = max(30, len(speed) // 10)
    final_speed = float(speed[-1])
    final_error = float(err[-1])
    tail_abs_mean_error = float(np.mean(np.abs(err[-tail:])))
    tail_abs_max_error = float(np.max(np.abs(err[-tail:])))
    mean_confidence = float(np.mean(conf[-tail:]))

    within = np.abs(err) <= band
    settle_idx = None
    for i in range(len(within)):
        if np.all(within[i:]):
            settle_idx = i
            break

    converged = bool(
        stable
        and abs(final_error) <= band
        and tail_abs_mean_error <= band
        and tail_abs_max_error <= 2.0 * band
        and settle_idx is not None
    )

    # Lower score is better.
    norm = max(rated_speed_rpm, 1.0)
    score = (
        0.68 * (abs(final_error) / norm)
        + 0.25 * (tail_abs_mean_error / norm)
        + 0.07 * (1.0 - float(np.clip(mean_confidence, 0.0, 1.0)))
    )
    if not stable:
        score += 10.0

    return {
        "stable": stable,
        "converged": converged,
        "score": float(score),
        "rated_speed_rpm": float(rated_speed_rpm),
        "final_speed_rpm": final_speed,
        "final_error_rpm": final_error,
        "tail_abs_mean_error_rpm": tail_abs_mean_error,
        "tail_abs_max_error_rpm": tail_abs_max_error,
        "settling_time_5pct_s": None if settle_idx is None else float(t[settle_idx]),
        "sim_time_s": float(t[-1]),
        "mean_observer_confidence": mean_confidence,
    }


def _optimize_one_profile(
    profile_path: Path,
    search_time_s: float,
    verify_time_s: float,
    deep: bool = False,
    min_final_speed_ratio: float = 0.95,
) -> dict:
    profile = load_motor_profile(profile_path)
    rated = profile.get("rated_info", {})
    rated_speed = float(rated.get("rated_speed_rpm", 0.0))
    if rated_speed <= 0.0:
        raise ValueError(f"Profile {profile_path.name} has no rated_speed_rpm")
    rated_current = float(rated.get("rated_current_a", rated.get("rated_current_a_rms", 20.0)))

    base_motor_params = dict(profile["motor_params"])
    base_fp = _motor_params_fingerprint(base_motor_params)
    dt = 1.0 / 8000.0

    params = _to_motor_params(base_motor_params)

    tuner = AdaptiveFOCTuner(params=params)
    base = tuner.tune(grid_size=(12 if deep else 8))

    speed_candidates, current_candidates, observer_candidates = _build_search_candidates(
        base, deep=deep
    )

    best_speed = speed_candidates[2]
    best_current = current_candidates[2]

    # Stage 1: optimize speed PI with a nominal observer and current PI baseline.
    anchor_observer = observer_candidates[1]
    best_stage = None
    for cand in speed_candidates:
        trial = _run_trial(
            params=params,
            rated_speed_rpm=rated_speed,
            rated_current_a=rated_current,
            speed_pi=cand,
            current_pi=best_current,
            observer=anchor_observer,
            sim_time_s=search_time_s,
            dt=dt,
        )
        if best_stage is None or trial["score"] < best_stage["score"]:
            best_stage = trial
            best_speed = cand

    # Stage 2: optimize current PI with the selected speed PI.
    best_stage = None
    for cand in current_candidates:
        trial = _run_trial(
            params=params,
            rated_speed_rpm=rated_speed,
            rated_current_a=rated_current,
            speed_pi=best_speed,
            current_pi=cand,
            observer=anchor_observer,
            sim_time_s=search_time_s,
            dt=dt,
        )
        if best_stage is None or trial["score"] < best_stage["score"]:
            best_stage = trial
            best_current = cand

    # Stage 3: observer sweep with optimized regulators.
    best_observer = observer_candidates[0]
    best_observer_result = None
    for cand in observer_candidates:
        trial = _run_trial(
            params=params,
            rated_speed_rpm=rated_speed,
            rated_current_a=rated_current,
            speed_pi=best_speed,
            current_pi=best_current,
            observer=cand,
            sim_time_s=search_time_s,
            dt=dt,
        )
        if best_observer_result is None or trial["score"] < best_observer_result["score"]:
            best_observer_result = trial
            best_observer = cand

    final_result = _run_trial(
        params=params,
        rated_speed_rpm=rated_speed,
        rated_current_a=rated_current,
        speed_pi=best_speed,
        current_pi=best_current,
        observer=best_observer,
        sim_time_s=verify_time_s,
        dt=dt,
    )
    min_required_speed_rpm = rated_speed * max(0.0, min(1.0, min_final_speed_ratio))
    accepted = bool(
        final_result["converged"] and final_result["final_speed_rpm"] >= min_required_speed_rpm
    )

    session_payload = {
        "schema": "bldc.auto_tuning_session.v2",
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "profile": {
            "name": profile.get("profile_name", profile_path.stem),
            "file": profile_path.name,
        },
        "search_setup": {
            "search_sim_time_s": float(search_time_s),
            "verify_sim_time_s": float(verify_time_s),
            "dt_s": float(dt),
            "load_profile": "ConstantLoad(0.0)",
            "field_weakening_enabled": False,
            "deep_search_enabled": bool(deep),
            "observer_candidates": observer_candidates,
            "speed_candidate_count": len(speed_candidates),
            "current_candidate_count": len(current_candidates),
            "strict_acceptance": {
                "enabled": True,
                "min_final_speed_ratio": float(min_final_speed_ratio),
                "min_required_speed_rpm": float(min_required_speed_rpm),
            },
        },
        "motor_params": base_motor_params,
        "rated_info": profile.get("rated_info", {}),
        "best_parameters": {
            "regulators": {
                "speed_pi": {
                    "kp": float(best_speed[0]),
                    "ki": float(best_speed[1]),
                    "kaw": 0.05,
                },
                "current_pi": {
                    "d_kp": float(best_current[0]),
                    "d_ki": float(best_current[1]),
                    "q_kp": float(best_current[0]),
                    "q_ki": float(best_current[1]),
                    "kaw": 0.2,
                },
                "iq_limit_a": float(max(5.0, 0.9 * rated_current)),
            },
            "observer": best_observer,
        },
        "results": {
            "verification": final_result,
            "observer_stage_best": best_observer_result,
        },
        "acceptance": {
            "accepted": accepted,
            "reason": ("accepted" if accepted else "strict convergence rule not satisfied"),
        },
    }

    if _motor_params_fingerprint(base_motor_params) != base_fp:
        raise RuntimeError("Motor parameters mutated during tuning session")

    return session_payload


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Auto-tune regulators and observer for motor profiles"
    )
    parser.add_argument(
        "--profiles-dir",
        default=str(PROJECT_ROOT / "data" / "motor_profiles"),
        help="Directory containing motor profile JSON files",
    )
    parser.add_argument(
        "--search-time",
        type=float,
        default=0.5,
        help="Simulation time in seconds for search trials",
    )
    parser.add_argument(
        "--verify-time",
        type=float,
        default=1.2,
        help="Simulation time in seconds for final verification trial",
    )
    parser.add_argument(
        "--deep-search",
        action="store_true",
        help="Enable larger controller candidate sets and finer analytical pre-tuning.",
    )
    parser.add_argument(
        "--min-final-speed-ratio",
        type=float,
        default=0.95,
        help="Strict acceptance threshold: minimum final_speed/rated_speed ratio.",
    )
    args = parser.parse_args()

    profiles_dir = Path(args.profiles_dir)
    profile_paths = list_motor_profiles(profiles_dir)
    if not profile_paths:
        raise FileNotFoundError(f"No profile found in {profiles_dir}")

    out_dir = PROJECT_ROOT / "data" / "tuning_sessions"
    out_dir.mkdir(parents=True, exist_ok=True)

    summary = {
        "schema": "bldc.auto_tuning_summary.v2",
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "profiles_dir": str(profiles_dir),
        "search_time_s": float(args.search_time),
        "verify_time_s": float(args.verify_time),
        "results": [],
        "errors": [],
    }

    for profile_path in profile_paths:
        try:
            session = _optimize_one_profile(
                profile_path=profile_path,
                search_time_s=float(args.search_time),
                verify_time_s=float(args.verify_time),
                deep=bool(args.deep_search),
                min_final_speed_ratio=float(args.min_final_speed_ratio),
            )

            out_path = out_dir / f"{profile_path.stem}_auto_tuned.json"
            out_path.write_text(json.dumps(session, indent=2), encoding="utf-8")

            verification = session["results"]["verification"]
            summary["results"].append(
                {
                    "profile": session["profile"]["name"],
                    "file": session["profile"]["file"],
                    "session_file": out_path.name,
                    "accepted": bool(session.get("acceptance", {}).get("accepted", False)),
                    "stable": verification["stable"],
                    "converged": verification["converged"],
                    "final_speed_rpm": verification["final_speed_rpm"],
                    "final_error_rpm": verification["final_error_rpm"],
                    "score": verification["score"],
                }
            )
            print(
                json.dumps(
                    {
                        "profile": session["profile"]["name"],
                        "session_file": out_path.name,
                        "converged": verification["converged"],
                        "final_speed_rpm": verification["final_speed_rpm"],
                        "score": verification["score"],
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

    summary_path = PROJECT_ROOT / "data" / "logs" / "motor_profiles_auto_tuning_summary.json"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"SUMMARY_SAVED {summary_path}")


if __name__ == "__main__":
    main()
