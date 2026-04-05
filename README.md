# SPINOTOR

Advanced BLDC and PMSM motor-control simulator with a desktop GUI, accessibility-first workflows, and repeatable calibration pipelines.

Current Version: 0.9.5

## Why This Project

This repository provides a practical environment to design, validate, and compare motor-control strategies before hardware deployment. It combines realistic electrical and mechanical modeling with tooling for analysis, regression checks, and parameter tuning.

## Core Capabilities

The list below highlights major capabilities. For the complete implemented feature inventory,
see `docs/features.rst`.

- BLDC/PMSM simulation with configurable motor, load, and supply models
- FOC and V/f control, including sensored and sensorless FOC operation
- Auto-calibration workflows grounded in analytic and physics-based methods
- Field weakening for operation beyond base speed
- Space Vector Modulation (SVM) with optional PWM-modulation effects
- Inverter and power-stage realism with loss, ripple, thermal, and asymmetry modeling
- Phase-current sensing with 1-, 2-, or 3-shunt reconstruction and injectable amplifier gain/offset errors
- Star (wye) and delta motor topologies
- Deterministic constant-step RK4 integration
- NUMBA-accelerated compute paths with CPU/GPU backend selection and safe fallback behavior
- Power-factor correction (PFC) compensation plus power-factor and efficiency telemetry
- Standard startup sequencing from alignment to closed-loop handoff
- Real-time monitoring, data logging, FFT analysis, and plotting
- Control timing telemetry and MCU-load estimation helpers
- Hardware backend abstraction with mock DAQ support for dry-run integration
- Modular, scalable architecture across control, core, hardware, UI, and visualization layers
- Motor profile import/export, regression baselines, and repeatable validation workflows
- Accessibility-oriented PyQt6 interface (keyboard-first and screen-reader friendly)
- Optional audio assistance for key status and workflow events

## Quick Start

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Launch the GUI

```bash
python main.py
```

In the Calibration tab, use the single Auto-Calibrate action to run the full
analytic and physics-based calibration workflow from the local UI.

### 3. Run tests (recommended before release)

```bash
pytest -q
```

### 4. Run mandatory pre-commit gates

```bash
pip install -r requirements-dev.txt
python -m pre_commit run --all-files
```

Before each commit, the repository hook at `.githooks/pre-commit` now enforces:

- Documentation policy check (code/config changes must include relevant doc updates)
- Sphinx HTML regeneration (`python -m sphinx -b html docs docs/_build/html`)
- Quality gates (`ruff`, `mypy`, `bandit`)
- Full test run with coverage gate (`pytest --cov=src --cov-fail-under=100`)
- Dependency vulnerability audit (`pip-audit -r requirements.txt`)

Emergency bypass variables exist but are not intended for normal development:

- `SKIP_MANDATORY_GATES=1`
- `SKIP_DOC_POLICY_CHECK=1`

CI also enforces:

- Ruff lint checks
- Mypy type checks on `src/`
- Bandit security scan on `src/`
- Sphinx HTML documentation build
- Full tests with 100% `src/` coverage threshold
- `pip-audit` vulnerability scan for runtime dependencies

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
Set `SKIP_AUTO_BUMP=1` in the shell only when you intentionally want to bypass one commit.

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
4. Follow `.github/RELEASE_CHECKLIST.md`.
5. Update release notes using `RELEASE_NOTES_TEMPLATE.md`.
6. Push only after local checks are green.

## References

- Control-theory and formula references are under `references/`.
- Sphinx API and user documentation live under `docs/`.
- Security disclosure and handling policy is in `SECURITY.md`.

## License and Safety

- License: Custom restricted license (see `LICENSE`)
- This simulator is provided as-is for research, calibration, and educational use.
- Users are responsible for validation before applying settings to real hardware.

## Author

Amine Khettat
