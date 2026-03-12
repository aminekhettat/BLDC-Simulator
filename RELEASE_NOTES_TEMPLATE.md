# Release Notes Template

## Summary

- High-level purpose of this release.

## Key Changes

- Feature and fix highlights.

## Regression Gates

- Regression Gates / regression status: PASS/FAIL
- Drift report reference: regression_drift_report.txt (CI artifact)

## Baseline Changes

- Baseline files changed: Yes/No
- Files:
  - tests/baselines/reference_baseline.json
  - tests/baselines/foc_reference_baseline.json
- Baseline rationale:
- Drift evidence summary:
- Impact assessment:

## Validation

- Commands executed:
  - pytest tests/test_baseline_integrity.py tests/test_regression_baseline.py tests/test_regression_baseline_foc.py tests/test_regression_reporting.py -v
- Result summary:

## Risks and Follow-ups

- Known limitations and next actions.
