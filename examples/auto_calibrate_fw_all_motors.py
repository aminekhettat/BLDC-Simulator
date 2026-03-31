"""
Auto Field-Weakening Calibration — All Motor Profiles
======================================================

This script automatically calibrates field-weakening (FW) parameters for
every motor profile found in ``data/motor_profiles/``, then verifies that
each motor can reach its rated speed and deliver its rated torque.

Algorithm
---------
For each motor profile the calibrator:

1. **Analytical step** – derives k_fw, fw_id_max, fw_start_rpm, gain and
   headroom_target from motor nameplate data (Ke, rated speed, rated current,
   supply voltage).

2. **Sweep step** – tries a small grid of gain × fw_id_max combinations in
   closed-loop RK4 simulation and picks the best one by speed-tracking score.

3. **Validation step** – runs four operating points (pre-FW, rated-speed
   no-load, rated-speed 30 % load, rated-speed 60 % load) with the winner.

Results are saved to
``data/tuning_sessions/fw_calibrated_{motor_name}.json``.

Usage
-----
::

    # Full calibration (all motors)
    python examples/auto_calibrate_fw_all_motors.py

    # Single motor, use existing session file for PI gains
    python examples/auto_calibrate_fw_all_motors.py \\
        --profile data/motor_profiles/motenergy_me1718_48v.json \\
        --session data/tuning_sessions/until_converged/motenergy_me1718_48v_until_converged.json

    # Quick scan (3 gain values instead of 5)
    python examples/auto_calibrate_fw_all_motors.py --quick

Notes
-----
* Uses a numerically stable time-step derived from the current-loop bandwidth
  (≈ π / (5 × 30 × R/L)).  This avoids the instability of the dt = 1 ms
  used in some older scripts.
* Matched session files are auto-detected if they follow the naming pattern
  ``data/tuning_sessions/until_converged/{motor_stem}_until_converged.json``.
"""

# ruff: noqa: E402
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.control.field_weakening_calibrator import (  # noqa: E402
    FieldWeakeningCalibrator,
    FWCalibrationResult,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _find_session(profile_path: Path) -> Path | None:
    """Heuristically locate a matching tuning-session JSON for a profile."""
    stem = profile_path.stem  # e.g. motenergy_me1718_48v
    roots = [
        PROJECT_ROOT / "data" / "tuning_sessions" / "until_converged",
        PROJECT_ROOT / "data" / "tuning_sessions",
    ]
    candidates = [
        f"{stem}_until_converged.json",
        f"{stem}_auto_tuned.json",
        f"auto_calibrated_{stem}.json",
    ]
    for root in roots:
        for cname in candidates:
            p = root / cname
            if p.exists():
                return p
    return None


def _session_gains(session_path: Path) -> dict:
    """Extract best PI gains from a session JSON (graceful fallback)."""
    try:
        d = json.loads(session_path.read_text())
        bc = d.get("best_candidate", {})
        return {
            "speed_kp": float(bc.get("speed_pi", {}).get("kp", 0.5)),
            "speed_ki": float(bc.get("speed_pi", {}).get("ki", 2.0)),
            "current_kp": float(
                bc.get("current_pi", {}).get("d_kp") or bc.get("current_pi", {}).get("kp", 0.15)
            ),
            "current_ki": float(
                bc.get("current_pi", {}).get("d_ki") or bc.get("current_pi", {}).get("ki", 20.0)
            ),
            "iq_limit_a": float(bc.get("iq_limit_a", 200.0)),
        }
    except Exception:
        return {}


def _print_summary_table(results: list[tuple[str, FWCalibrationResult]]) -> None:
    """Print a concise ASCII table of all calibration results."""
    header = (
        f"{'Motor':<32} {'Rated RPM':>10} {'Best RPM':>10} "
        f"{'k_fw':>8} {'gain':>6} {'id_max A':>9} {'Status':>8}"
    )
    print(f"\n{'=' * len(header)}")
    print(header)
    print(f"{'=' * len(header)}")
    for name, r in results:
        p = r.physics_params
        ok = "✅ PASS" if r.all_passed else "⚠  PART"
        if not r.operating_points:
            ok = "❌ FAIL"
        print(
            f"{name:<32} {p.rated_speed_rpm:>10.0f} {r.best_achieved_rpm:>10.0f} "
            f"{p.flux_weakening_id_coefficient:>8.5f} {r.selected_gain:>6.1f} "
            f"{r.selected_fw_id_max_a:>9.1f} {ok:>8}"
        )
    print(f"{'=' * len(header)}\n")


# ---------------------------------------------------------------------------
# Per-motor calibration
# ---------------------------------------------------------------------------


def calibrate_one(
    profile_path: Path,
    session_path: Path | None,
    quick: bool,
    sim_duration: float,
) -> FWCalibrationResult:
    """Run FW calibration for a single motor profile."""
    # Build calibrator
    cal = FieldWeakeningCalibrator.from_profile_file(
        profile_path,
        fw_current_fraction=0.25,
        safety_factor=1.15,
        sim_duration=sim_duration,
        verbose=True,
    )

    # Extract PI gains from session if available
    gains = {}
    if session_path is not None:
        gains = _session_gains(session_path)
        print(f"  Using PI gains from: {session_path.name}")
    else:
        print("  No session file found — using analytical PI gains.")

    # Gain sweep grid
    # quick=True: use the calibrator's auto-derived gain candidates centred on
    # the analytically designed value (computed in compute_physics_params).
    # Passing gain_candidates=None causes the calibrator to build its own grid.
    if quick:
        gain_cands = None  # let calibrator auto-generate around analytic gain
        id_fracs = [0.25]
    else:
        gain_cands = None  # full sweep; calibrator uses its own centred grid
        id_fracs = [0.20, 0.25, 0.30]

    result = cal.calibrate(
        speed_kp=gains.get("speed_kp"),
        speed_ki=gains.get("speed_ki"),
        current_kp=gains.get("current_kp"),
        current_ki=gains.get("current_ki"),
        iq_limit_a=gains.get("iq_limit_a"),
        gain_candidates=gain_cands,
        fw_id_frac_candidates=id_fracs,
    )
    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Auto field-weakening calibration for all BLDC/PMSM motor profiles."
    )
    parser.add_argument(
        "--profiles-dir",
        default=str(PROJECT_ROOT / "data" / "motor_profiles"),
        help="Directory containing motor profile JSON files.",
    )
    parser.add_argument(
        "--profile",
        default=None,
        help="Calibrate a single profile (overrides --profiles-dir).",
    )
    parser.add_argument(
        "--session",
        default=None,
        help="Tuning session JSON to supply PI gains (used with --profile).",
    )
    parser.add_argument(
        "--out-dir",
        default=str(PROJECT_ROOT / "data" / "tuning_sessions"),
        help="Output directory for calibration JSON results.",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Narrow sweep (3 gain values, 1 id_frac) for faster turnaround.",
    )
    parser.add_argument(
        "--sim-duration",
        type=float,
        default=10.0,
        help="Simulation duration per operating point in seconds (default 10).",
    )
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Collect profiles to calibrate
    if args.profile:
        profiles = [Path(args.profile)]
    else:
        profiles = sorted(
            p for p in Path(args.profiles_dir).glob("*.json") if not p.stem.startswith("_")
        )

    if not profiles:
        print(f"No motor profiles found in {args.profiles_dir}")
        return 1

    print(f"\n{'=' * 64}")
    print(f"  FW Auto-Calibration — {len(profiles)} motor(s)")
    print(f"  quick={args.quick},  sim_duration={args.sim_duration}s")
    print(f"{'=' * 64}")

    all_results: list[tuple[str, FWCalibrationResult]] = []

    for profile_path in profiles:
        print(f"\n>>> {profile_path.name}")

        # Locate session file
        if args.session:
            session_path = Path(args.session)
        else:
            session_path = _find_session(profile_path)

        try:
            result = calibrate_one(
                profile_path=profile_path,
                session_path=session_path,
                quick=args.quick,
                sim_duration=args.sim_duration,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"  ERROR during calibration: {exc}")
            import traceback

            traceback.print_exc()
            continue

        # Save result
        out_name = f"fw_calibrated_{profile_path.stem}.json"
        out_path = out_dir / out_name
        out_path.write_text(json.dumps(result.to_dict(), indent=2))
        try:
            display_path = out_path.resolve().relative_to(PROJECT_ROOT.resolve())
        except ValueError:
            display_path = out_path.resolve()
        print(f"  Saved → {display_path}")

        all_results.append((profile_path.stem, result))

    # Summary table
    if all_results:
        _print_summary_table(all_results)

    passed = sum(1 for _, r in all_results if r.all_passed)
    total = len(all_results)
    print(f"Overall: {passed}/{total} motors fully passed FW calibration.")

    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
