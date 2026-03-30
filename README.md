# BLDC Simulator

Professional BLDC and PMSM motor-control simulator with a desktop GUI, accessibility-first workflows, and repeatable calibration pipelines.

Current Version: 0.8.0

## Why This Project

This repository provides a practical environment to design, validate, and compare motor-control strategies before hardware deployment. It combines realistic electrical and mechanical modeling with tooling for analysis, regression checks, and parameter tuning.

## Core Capabilities

- BLDC/PMSM simulation with configurable motor, load, and supply models
- V/f and FOC control paths with PI-based loop tuning
- Accessibility-oriented PyQt6 interface (keyboard-first and screen-reader friendly)
- Real-time monitoring, data logging, FFT analysis, and plotting
- Motor profile import/export and repeatable calibration workflows
- Regression-oriented test suite and baseline validation

## Quick Start

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Launch the GUI

```bash
python main.py
```

### 3. Run tests (recommended before release)

```bash
pytest -q
```

## Project Layout

```text
.
├── main.py
├── src/
│   ├── control/
│   ├── core/
│   ├── hardware/
│   ├── ui/
│   ├── utils/
│   └── visualization/
├── examples/
├── tests/
├── docs/
├── data/
└── references/
```

## Documentation (Simplified)

To reduce navigation overhead, project guidance is centered on a small set of entry points:

- `README.md`: high-level overview, setup, release workflow
- `QUICKSTART.md`: practical usage flow for first simulations
- `CONTRIBUTING.md`: contribution and quality gates
- `docs/index.rst`: Sphinx documentation index
- `Roadmap/ROADMAP.md`: planned and completed milestones

## Versioning Workflow

Automatic version incrementing is configured with `bump2version`.

This repository also includes a pre-commit hook at `.githooks/pre-commit` to
auto-increment the patch version before each commit when hooks are enabled.

```bash
# Patch release (x.y.Z)
bump2version patch

# Minor release (x.Y.0)
bump2version minor

# Major release (X.0.0)
bump2version major
```

Tracked version locations include:

- `main.py`
- `src/__init__.py`
- `src/ui/main_window.py`
- `docs/conf.py`
- `README.md`

## Release Checklist

Before merging or publishing a release:

1. Run regression and baseline tests.
2. Verify generated artifacts and plots when control behavior changes.
3. Bump the version with `bump2version`.
4. Update release notes using `RELEASE_NOTES_TEMPLATE.md`.
5. Push only after local checks are green.

## References

- Control-theory and formula references are under `references/`.
- Sphinx API and user documentation live under `docs/`.

## License and Safety

- License: MIT (see `LICENSE`)
- This simulator is provided as-is for research, calibration, and educational use.
- Users are responsible for validation before applying settings to real hardware.

## Author

Amine Khettat
