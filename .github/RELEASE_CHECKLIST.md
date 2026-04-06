# Release Checklist

Use this checklist before creating a release tag.

## 1. Versioning

- Confirm semantic version target (major/minor/patch)
- Confirm CHANGELOG/release notes scope matches version impact
- Ensure version bump is committed and traceable

## 2. Quality Gates

- ruff check passes
- mypy src passes
- bandit scan passes
- pytest passes with --cov-fail-under=100
- pip-audit reports no blocking vulnerabilities
- Sphinx docs build passes

## 3. Regression Governance

- Baseline files unchanged, or
- Baseline changes include rationale + drift evidence + reviewer approval
- examples/report_regression_drift.py reviewed for unexpected drift

## 4. Product Readiness

- Critical known issues reviewed
- Accessibility-critical paths smoke-tested
- Key example flows run at least once:
  - examples/example_foc_control.py
  - examples/example_vf_control.py
  - examples/report_regression_drift.py

## 5. Release Artifacts

- Tag created from intended commit
- Release notes include behavior-impacting changes
- Security-impacting changes highlighted (if any)
- Upgrade/rollback guidance included

## 6. Post-Release

- Verify CI status on release tag
- Confirm issue templates and docs reference current version
- Monitor first 24h for regressions and triage quickly
