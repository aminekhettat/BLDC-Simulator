"""
Atomic features tested in this module:
- reference regression baseline
"""

import sys
from pathlib import Path

import pytest

from src.utils.regression_baseline import (
    build_drift_report,
    compare_to_baseline,
    format_drift_report,
    load_baseline,
    run_reference_suite,
)

BASELINE_FILE = Path(__file__).parent / "baselines" / "reference_baseline.json"


def test_reference_regression_baseline():
    if sys.platform != "win32":
        pytest.skip("Frozen V/f baseline assertions are validated on Windows runners.")

    if not BASELINE_FILE.exists():
        pytest.skip(
            "No frozen baseline found. Generate one by calling "
            "src.utils.regression_baseline.save_baseline(...)."
        )

    baseline = load_baseline(BASELINE_FILE)
    current = run_reference_suite()

    failures = compare_to_baseline(current, baseline)
    if failures:
        rows = build_drift_report(current=current, baseline_payload=baseline)
        report = format_drift_report(rows, failed_only=True)
        assert False, "\n".join(failures) + "\n\n" + report
