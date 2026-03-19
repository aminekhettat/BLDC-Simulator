"""
Atomic features tested in this module:
- build drift report detects fail and pass
- format drift report contains columns
- startup transition diagnostics returns expected kpis
- startup transition thresholds support pass warn fail
- startup transition threshold report format contains columns
"""
from src.utils.regression_baseline import (
    build_drift_report,
    evaluate_startup_transition_thresholds,
    format_startup_threshold_report,
    format_drift_report,
    run_foc_startup_transition_diagnostics,
)


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


def test_startup_transition_diagnostics_returns_expected_kpis():
    diag = run_foc_startup_transition_diagnostics()

    required = {
        "startup_handoff_count",
        "startup_fallback_event_count",
        "startup_last_handoff_time_s",
        "startup_last_handoff_confidence",
        "startup_handoff_confidence_peak",
        "startup_handoff_quality",
        "startup_handoff_stability_ratio",
        "observer_confidence_above_threshold_time_s",
        "observer_confidence_below_threshold_time_s",
        "observer_confidence_crossings_up",
        "observer_confidence_crossings_down",
    }
    assert required.issubset(diag.keys())
    assert diag["startup_handoff_count"] >= 1.0
    assert diag["startup_fallback_event_count"] >= 1.0
    assert 0.0 <= diag["startup_handoff_quality"] <= 1.0
    assert 0.0 <= diag["startup_handoff_stability_ratio"] <= 1.0


def test_startup_transition_thresholds_support_pass_warn_fail():
    diagnostics = {
        "kpi_pass": 0.80,
        "kpi_warn": 0.35,
        "kpi_fail": 0.10,
    }
    thresholds = {
        "kpi_pass": {"warn_min": 0.50, "fail_min": 0.20},
        "kpi_warn": {"warn_min": 0.50, "fail_min": 0.20},
        "kpi_fail": {"warn_min": 0.50, "fail_min": 0.20},
    }

    rows = evaluate_startup_transition_thresholds(
        diagnostics=diagnostics,
        thresholds=thresholds,
    )
    by_kpi = {row["kpi"]: row for row in rows}

    assert by_kpi["kpi_pass"]["status"] == "pass"
    assert by_kpi["kpi_warn"]["status"] == "warn"
    assert by_kpi["kpi_fail"]["status"] == "fail"


def test_startup_transition_threshold_report_format_contains_columns():
    rows = [
        {
            "kpi": "kpi_example",
            "actual": 0.42,
            "warn_min": 0.50,
            "warn_max": None,
            "fail_min": 0.20,
            "fail_max": None,
            "status": "warn",
        }
    ]

    text = format_startup_threshold_report(rows)
    assert "kpi | actual | warn_min | warn_max | fail_min | fail_max | status" in text
    assert "kpi_example" in text
    assert "warn" in text






