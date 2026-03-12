## Summary

Describe what changed and why.

## Regression Gate Checklist

- [ ] I verified the `Regression Gates / regression` check is green on this PR.
- [ ] I ran local regression checks when applicable:
  - `pytest tests/test_regression_baseline.py tests/test_regression_baseline_foc.py -v`

## Baseline Change Policy

- [ ] No baseline files changed in this PR.
- [ ] Baseline files changed intentionally, and I documented why:
  - tests/baselines/reference_baseline.json
  - tests/baselines/foc_reference_baseline.json
  - Baseline rationale:
  - Drift evidence:

## Validation

List commands and results:

- `pytest ...`
- `python examples/report_regression_drift.py`

## Notes for Reviewers

Anything specific reviewers should focus on.
