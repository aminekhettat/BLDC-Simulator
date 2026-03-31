"""Generate human-readable KPI drift reports for V/f and FOC baselines.

Run:
    python examples/report_regression_drift.py
"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))


def _print_report(title: str, baseline_path: Path, current: dict) -> None:
    from src.utils.regression_baseline import (
        build_drift_report,
        format_drift_report,
        load_baseline,
    )

    print("=" * 80)
    print(title)
    print("=" * 80)

    if not baseline_path.exists():
        print(f"Baseline file not found: {baseline_path}")
        return

    baseline = load_baseline(baseline_path)
    rows = build_drift_report(current=current, baseline_payload=baseline)

    # Show full table first for trend review
    print(format_drift_report(rows, failed_only=False))

    # Then summarize failing rows only
    failed_rows = [r for r in rows if r["status"] in ("fail", "missing")]
    print("\nFailing rows:")
    print(format_drift_report(failed_rows, failed_only=False))


def main() -> None:
    from src.utils.regression_baseline import (
        run_foc_reference_suite,
        run_foc_startup_transition_diagnostics,
        run_reference_suite,
    )

    vf_baseline = PROJECT_ROOT / "tests" / "baselines" / "reference_baseline.json"
    foc_baseline = PROJECT_ROOT / "tests" / "baselines" / "foc_reference_baseline.json"

    vf_current = run_reference_suite()
    foc_current = run_foc_reference_suite()
    transition_diag = run_foc_startup_transition_diagnostics()

    _print_report("V/f Regression Drift Report", vf_baseline, vf_current)
    _print_report("FOC Regression Drift Report", foc_baseline, foc_current)
    print("=" * 80)
    print("FOC Startup Transition Diagnostics")
    print("=" * 80)
    for key in sorted(transition_diag.keys()):
        val = transition_diag[key]
        if key.endswith("_count") or key.endswith("_up") or key.endswith("_down"):
            print(f"{key}: {int(val)}")
        else:
            print(f"{key}: {val:.6f}")


if __name__ == "__main__":
    main()
