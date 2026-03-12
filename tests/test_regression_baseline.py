"""Regression test for deterministic reference scenarios.

This test is intentionally strict once a baseline is frozen.
If no baseline file exists, the test is skipped and a baseline should be generated.
"""

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
