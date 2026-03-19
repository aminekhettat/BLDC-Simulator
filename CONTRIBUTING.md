# Contributing Guide
> **License Reminder:** This project is distributed under the MIT License. See [LICENSE](LICENSE).
> **Disclaimer:** This application is provided as-is for simulation and research use. Users assume all risks.
> The author disclaims liability for any direct or indirect damage, data loss, hardware issues, injury,
> or regulatory non-compliance resulting from use or misuse.


## Scope

This project uses deterministic regression baselines as merge gates.
If you change physics, control, solver settings, or baseline scenarios, you must follow the policy below.

## Regression Baseline Policy

Baseline files:

- tests/baselines/reference_baseline.json
- tests/baselines/foc_reference_baseline.json

Rules:

1. Do not modify baseline files unless behavior change is intentional.
2. If baseline files change, include a clear rationale in the PR body.
3. Attach or reference drift evidence from:
   - python examples/report_regression_drift.py
4. Keep simulation step size stable when comparing against existing baselines:
   - SIMULATION_PARAMS['dt']

## Required Checks Before Merge

The following must pass:

1. Regression Gates / regression
2. Baseline integrity tests

Recommended local command:

```bash
pytest tests/test_baseline_integrity.py tests/test_regression_baseline.py tests/test_regression_baseline_foc.py tests/test_regression_reporting.py -v
```

## Regenerating Baselines

Use only for intentional changes:

```bash
python examples/generate_regression_baseline.py
python examples/generate_foc_regression_baseline.py
python examples/report_regression_drift.py
```

Then document in PR:

1. Why baseline update is needed
2. What changed physically/control-wise
3. Why drift is acceptable

## Pull Request Checklist

Use and complete:

- .github/PULL_REQUEST_TEMPLATE.md

## Release Notes Checklist

Use this template when preparing release notes:

- RELEASE_NOTES_TEMPLATE.md
