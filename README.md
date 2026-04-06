# SPINOTOR

Advanced BLDC and PMSM motor-control simulator with a desktop GUI, accessibility-first workflows, and repeatable calibration pipelines.

Current Version: 0.10.0

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
- Accessibility-oriented PySide6 interface (keyboard-first and screen-reader friendly)
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

## Sensorless Angle Observers

SPINOTOR implements five observer modes selectable from the **Observer & Startup** tab.
All sensorless modes reconstruct the electrical angle from terminal voltages and
currents without requiring a rotor position sensor.

### Measured (reference mode)

Uses the true rotor angle directly from the simulation model. No estimation is
performed. This is the ideal reference mode for controller tuning and debugging
before enabling sensorless operation.

### PLL — Phase-Locked Loop

Reconstructs back-EMF in the stationary (α–β) frame from the stator voltage
equation and applies a phase-locked loop to extract electrical angle and speed.
Good first sensorless mode; simple gain structure (Kp, Ki).

Suggested gains: Kp 50–150, Ki 1000–5000.

### SMO — Sliding-Mode Observer

Sliding-mode-inspired observer with a sign-action error correction and a
low-pass filter (LPF) on the estimated EMF. More robust than PLL against supply
disturbances and load steps.

Suggested gains: Kslide 300–1000, LPF Alpha 0.05–0.2, Boundary 0.03–0.12 rad.

### STSMO — Super-Twisting Sliding-Mode Observer

Second-order Super-Twisting algorithm (Levant 1993) with a fully implicit
backward-Euler integrator for unconditional numerical stability at any time-step.
Key features:

- Gain k1 sets convergence speed (analytical calibration via `calibrate_stsmo_gains_analytical()`).
- Gain k2 adapts with electrical speed: `k2_eff = max(k2_min, k2_factor · ke · ωm · ωe)`,
  satisfying the Levant rotating-EMF tracking condition at all speeds.
- A Second-Order Generalized Integrator (SOGI) post-filter at the electrical
  frequency suppresses chattering before angle extraction; bypassed below 5 rad/s.
- z1 warm-start seeding from EMF at speed prevents startup transients.
- MRAS-based stator resistance drift correction is available as an optional add-on.

Typical calibration entry point:

```python
ctrl.calibrate_stsmo_gains_analytical(rated_rpm=3500.0)
```

Manual override parameters: k1 (default 18), k2_min (default 500 V/s),
k2_factor (default 1.0×), rated_rpm.

### ActiveFlux — Active Flux Observer

Based on the Active Flux concept (Boldea 2009). Defines the active flux vector
`ψa = ψs − Ld·is`, which is always aligned with the rotor d-axis regardless of
field-weakening id injection. Angle is extracted as `θe = arctan2(ψa_β, ψa_α)`.
Stator flux is integrated with a leaky (drift-correcting) pole at `dc_cutoff_hz`
(default 0.5 Hz) to suppress integrator offset. Particularly well-suited for
interior PM motors with significant saliency.

Manual override parameter: dc_cutoff_hz (default 0.5 Hz).

### Choosing an observer

For a clean bring-up path: start with **Measured**, tune d/q loops and speed
loop, then switch to **PLL**, then **SMO** if disturbance rejection is needed,
then **STSMO** for best chattering suppression and quantitative gain calibration.
Use **ActiveFlux** for IPM (interior PM) motors where d/q saliency is significant.

Full observer tuning guidance is in `docs/advanced.rst` and `docs/sensorless_observers.rst`.

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
