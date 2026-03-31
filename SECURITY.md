# Security Policy

## Supported Scope

This repository is maintained as a simulation and validation tool for BLDC/PMSM control software.
Security support applies to:

- Source code under src/
- Example scripts under examples/
- CI/CD workflows under .github/workflows/
- Runtime and development dependencies in requirements.txt and requirements-dev.txt

## Reporting a Vulnerability

Please report security issues privately by opening a GitHub Security Advisory draft for this repository.
If advisories are unavailable, open a private maintainer communication channel and include:

- Affected file/module
- Reproduction steps
- Impact assessment
- Suggested mitigation if known

Do not publish proof-of-concept exploit details in public issues before triage.

## Triage Targets

- Critical severity: first response within 24 hours
- High severity: first response within 3 business days
- Medium/Low severity: first response within 7 business days

## Remediation Process

1. Reproduce and validate the report.
2. Define fix scope and affected versions.
3. Add regression tests when applicable.
4. Pass mandatory gates: ruff, mypy, bandit, pytest coverage, pip-audit.
5. Publish coordinated fix notes after release.

## Dependency Security

- pip-audit is required in local pre-commit and CI workflows.
- Known vulnerable dependency updates are treated as P0 when exploitable in repository scope.

## Disclosure

After a fix is released, include:

- CVE/advisory reference (if assigned)
- Impacted versions
- Upgrade path and mitigation notes
