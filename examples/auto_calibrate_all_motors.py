#!/usr/bin/env python3
"""Auto-calibrate all motor profiles using enhanced adaptive tuning.

This script:
1. Discovers all motor profile JSON files in data/motor_profiles/
2. Calls calibrate_motor() for each motor
3. Generates comprehensive calibration report
4. Saves results to data/tuning_sessions/auto_calibrated_{motor_name}.json

The calibration includes:
- Analytical initial guess from motor parameters (L/R, J/B/Kt)
- Multi-resolution frequency-domain search (coarse then fine grid)
- Simulation validation at rated operating point
- Robustness verification at ±20% of rated speed
- Full margin and state-space analysis

Usage:
    python examples/auto_calibrate_all_motors.py [--quick] [--no-sim]

    --quick:   Skip simulation validation for speed (test mode)
    --no-sim:  Disable simulation validation entirely (frequency-domain only)
"""

import json
import sys
import time
from pathlib import Path

# Ensure project root is in path
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.control.adaptive_tuning import calibrate_motor


def discover_motor_profiles() -> list[Path]:
    """Find all motor profile JSON files."""
    profiles_dir = PROJECT_ROOT / "data" / "motor_profiles"
    profiles = sorted(profiles_dir.glob("*.json"))

    # Filter out temporary directories
    profiles = [p for p in profiles if not p.name.startswith("_tmp_")]

    return profiles


def save_calibration_report(report, motor_name: str) -> Path:
    """Save calibration report to JSON file."""
    output_dir = PROJECT_ROOT / "data" / "tuning_sessions"
    output_dir.mkdir(parents=True, exist_ok=True)

    output_file = output_dir / f"auto_calibrated_{motor_name}.json"
    output_file.write_text(json.dumps(report.to_dict(), indent=2), encoding="utf-8")

    return output_file


def print_calibration_summary(report, elapsed_s: float) -> None:
    """Print concise summary of calibration results."""
    result = report.tuning_result
    print()
    print("=" * 70)
    print(f"MOTOR: {report.motor_profile_name}")
    print("=" * 70)

    print("\nAnalytical Initial Guess:")
    print(
        f"  Current Loop: Kp={report.analytical_initial_guess['current_kp']:.6e}, "
        f"Ki={report.analytical_initial_guess['current_ki']:.6e}"
    )
    print(
        f"  Speed Loop:   Kp={report.analytical_initial_guess['speed_kp']:.6e}, "
        f"Ki={report.analytical_initial_guess['speed_ki']:.6e}"
    )

    print("\nOptimized Gains:")
    print(f"  Current Loop: Kp={result.current_kp:.6e}, Ki={result.current_ki:.6e}")
    print(f"  Speed Loop:   Kp={result.speed_kp:.6e}, Ki={result.speed_ki:.6e}")

    print("\nFrequency-Domain Margins:")
    print(
        f"  Current Loop: GM={result.current_margin.gain_margin_db:.2f} dB, "
        f"PM={result.current_margin.phase_margin_deg:.2f}°"
    )
    print(
        f"  Speed Loop:   GM={result.speed_margin.gain_margin_db:.2f} dB, "
        f"PM={result.speed_margin.phase_margin_deg:.2f}°"
    )

    print("\nState-Space Properties:")
    print(f"  Current: Ctrl={result.current_controllable}, Obs={result.current_observable}")
    print(f"  Speed:   Ctrl={result.speed_controllable}, Obs={result.speed_observable}")

    if report.simulation_validation:
        print("\nSimulation Validation:")
        for key, val in report.simulation_validation.items():
            if val.get("stable"):
                print(
                    f"  {key}: STABLE, "
                    f"Speed Error={val.get('speed_error_pct', 0):.2f}%, "
                    f"Orth Error={val.get('orthogonality_error_deg', 0):.2f}°"
                )
            else:
                print(f"  {key}: UNSTABLE")

    print(f"\nCalibration completed in {elapsed_s:.2f} seconds")
    print()


def main() -> int:
    """Main entry point."""
    quick_mode = "--quick" in sys.argv
    no_sim = "--no-sim" in sys.argv
    enable_sim_validation = not no_sim

    print("\n" + "=" * 70)
    print("BLDC Motor Auto-Calibration")
    print("=" * 70)
    print(f"Mode: {'Quick (no simulation)' if quick_mode else 'Full'}")
    print(f"Simulation Validation: {'Enabled' if enable_sim_validation else 'Disabled'}")

    profiles = discover_motor_profiles()
    if not profiles:
        print("ERROR: No motor profiles found in data/motor_profiles/")
        return 1

    print(f"\nDiscovered {len(profiles)} motor profiles:")
    for p in profiles:
        print(f"  - {p.name}")

    print()
    start_time = time.monotonic()
    results = []

    for idx, profile_path in enumerate(profiles, 1):
        print(f"\n[{idx}/{len(profiles)}] Calibrating {profile_path.name}...")
        try:
            motor_start = time.monotonic()
            report = calibrate_motor(
                str(profile_path),
                quick_mode=quick_mode,
                enable_simulation_validation=enable_sim_validation,
            )
            motor_elapsed = time.monotonic() - motor_start

            output_file = save_calibration_report(report, profile_path.stem)
            print_calibration_summary(report, motor_elapsed)
            print(f"Report saved to: {output_file}")

            results.append(
                {
                    "profile": profile_path.name,
                    "status": "SUCCESS",
                    "output_file": str(output_file),
                    "elapsed_s": motor_elapsed,
                }
            )

        except Exception as e:
            print(f"ERROR: Calibration failed: {e}")
            results.append(
                {
                    "profile": profile_path.name,
                    "status": "FAILED",
                    "error": str(e),
                }
            )

    elapsed_total = time.monotonic() - start_time

    # Summary
    print("\n" + "=" * 70)
    print("CALIBRATION SUMMARY")
    print("=" * 70)
    success = sum(1 for r in results if r["status"] == "SUCCESS")
    failed = sum(1 for r in results if r["status"] == "FAILED")
    print(f"Completed:  {success}/{len(results)} motors")
    print(f"Failed:     {failed}/{len(results)} motors")
    print(f"Total Time: {elapsed_total:.2f} seconds")
    print("=" * 70)

    for result in results:
        status_str = (
            "SUCCESS"
            if result["status"] == "SUCCESS"
            else f"FAILED: {result.get('error', 'Unknown error')}"
        )
        print(f"  {result['profile']:40s} {status_str}")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
