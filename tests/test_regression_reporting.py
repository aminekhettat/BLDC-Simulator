"""Unit tests for regression drift reporting helpers."""

from src.utils.regression_baseline import build_drift_report, format_drift_report


def test_build_drift_report_detects_fail_and_pass():
    baseline = {
        "scenarios": {
            "s1": {"k1": 100.0, "k2": 10.0},
        },
        "tolerances": {
            "s1": {"k1": 0.10, "k2": 0.05},
        },
    }
    current = {
        "s1": {"k1": 108.0, "k2": 11.0},
    }

    rows = build_drift_report(current=current, baseline_payload=baseline)
    by_kpi = {r["kpi"]: r for r in rows}

    assert by_kpi["k1"]["status"] == "pass"  # 8% <= 10%
    assert by_kpi["k2"]["status"] == "fail"  # 10% > 5%


def test_format_drift_report_contains_columns():
    rows = [
        {
            "scenario": "s1",
            "kpi": "k1",
            "expected": 100.0,
            "actual": 110.0,
            "delta_abs": 10.0,
            "delta_pct": 10.0,
            "allowed_abs": 5.0,
            "tolerance_pct": 0.05,
            "status": "fail",
        }
    ]

    text = format_drift_report(rows)
    assert "scenario | kpi | expected | actual | delta | delta% | tol% | status" in text
    assert "s1 | k1" in text
    assert "fail" in text
