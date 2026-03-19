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

Running the GUI
---------------

Start the main application::

    python main.py

This will launch the BLDC Motor Control GUI with:
- Real-time motor simulation
- Interactive control modes (FOC/V/F)
- Parameter monitoring and adjustment
- Data logging and export

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

    python test_accessibility_fixes.py

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
