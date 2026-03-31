Quick Start Guide
=================

Installation
------------

1. Clone the repository::

    git clone https://github.com/aminekhettat/BLDC_motor_control.git
    cd BLDC_motor_control

2. Create a virtual environment::

    python -m venv .venv
    .venv\Scripts\activate  # On Windows
    source .venv/bin/activate  # On macOS/Linux

3. Install dependencies::

    pip install -r requirements.txt

4. Install development quality tools::

    pip install -r requirements-dev.txt

5. Run mandatory local commit gates::

    python -m pre_commit run --all-files

   Note: this repository uses ``.githooks/pre-commit`` via ``core.hooksPath``,
   so ``pre-commit install`` is not required.

    The hook enforces documentation updates, Sphinx HTML build, quality checks,
    full tests, and 100% coverage on ``src/`` before commit.

Running the GUI
---------------

Start the main application::

    python main.py

This will launch the BLDC Motor Control GUI with:
- Real-time motor simulation
- Interactive control modes (FOC/V/F)
- Parameter monitoring and adjustment
- Data logging and export

Current Measurement and FFT Workflow
-----------------------------------

The GUI now includes a measurement-oriented workflow for validating inverter current sensing and harmonic content:

1. Select the inverter current-sense topology (triple, double, or single shunt) in the GUI.
2. Choose whether FOC should use true phase currents or reconstructed measured currents as its feedback source.
3. Open the current spectrum window to inspect stacked FFT magnitude and phase plots.
4. Toggle amplitude units (linear or dB), phase units (degrees or radians), and per-axis linear/log scaling.
5. Export either FFT data as CSV or the current spectrum figure as an image.

The live bridge drawing reflects the active shunt topology. In single-shunt mode it uses the physically correct shared low-side return path.

Running Examples
----------------

Field-Oriented Control example::

    python examples/example_foc_control.py

Voltage-Frequency Control example::

    python examples/example_vf_control.py

Loaded no-field-weakening calibration example::

    python examples/calibrate_no_fw_loaded_point.py

This example writes a detailed report to ``data/logs/calibration_me1718_no_fw_loaded_point.json``.

Running Tests
-------------

Execute the test suite::

    pytest tests/

Or run accessibility tests::

    python tests/test_feature_accessibility_integration.py

Quality Gates
-------------

The CI pipeline enforces the following quality checks on pull requests:

- ``ruff check .``
- ``mypy src``
- ``bandit -c bandit.yaml -r src``
- ``pip-audit -r requirements.txt``

Core Concepts
-------------

**Motor Parameters**: The motor is characterized by:
- Stator resistance (Rs)
- Direct-axis inductance (Ld)
- Quadrature-axis inductance (Lq)
- Magnetic flux linkage (lambda_pm)
- Number of pole pairs (p)

**Control Modes**:
- **FOC (Field-Oriented Control)**: Decoupled control of torque and flux for high performance
- **V/F (Voltage-Frequency Control)**: Simplified scalar control for basic operation

**Space Vector Modulation**: PWM generation for 3-phase inverter control

**Loaded Calibration**: The loaded no-FW workflow first proves speed feasibility under load, then evaluates orthogonality and conditioned efficiency at the final operating point.

See the full documentation for detailed API reference and advanced topics.
