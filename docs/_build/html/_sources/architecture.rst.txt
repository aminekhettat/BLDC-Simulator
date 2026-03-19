Architecture
============

Project Structure
-----------------

::

    BLDC_motor_control/
    ├── src/
    │   ├── core/              # Core motor simulation
    │   │   ├── motor_model.py
    │   │   ├── power_model.py
    │   │   ├── load_model.py
    │   │   └── simulation_engine.py
    │   ├── control/           # Control algorithms
    │   │   ├── base_controller.py
    │   │   ├── foc_controller.py
    │   │   ├── vf_controller.py
    │   │   ├── svm_generator.py
    │   │   └── transforms.py
    │   ├── ui/                # User interface
    │   │   ├── main_window.py
    │   │   └── widgets/
    │   ├── visualization/     # Plotting & display
    │   │   └── visualization.py
    │   └── utils/             # Utilities
    │       ├── config.py
    │       ├── data_logger.py
    │       └── speech.py
    ├── examples/              # Example scripts
    ├── tests/                 # Test suite
    ├── docs/                  # Documentation (Sphinx)
    └── data/                  # Logs and plots

Design Patterns
---------------

**MVC-Style Architecture**
- Model: Motor simulation (core module)
- View: PyQt6 GUI (ui module)
- Controller: Control algorithms (control module)

**Strategy Pattern**
- Base controller class with FOC and V/F implementations
- Pluggable load models

**Observer Pattern**
- GUI updates on simulation events via signals/slots

**Factory Pattern**
- Motor configuration loading and initialization

Core Simulation Flow
--------------------

1. **Initialization**
   - Load motor parameters
   - Initialize control system
   - Setup GUI and plots

2. **Simulation Loop**
   - Update reference signals
   - Execute controller (FOC or V/F)
   - Generate PWM via SVM
   - Update motor dynamics
   - Calculate losses and power

3. **Display Update**
   - Refresh plots
   - Update status bar
   - Log data to CSV

Key Components
--------------

**Motor Model** (`src.core.motor_model`)
- Grid-based motor parameter storage
- Phase current and voltage tracking
- EMF calculation with profile-selected sinusoidal or trapezoidal waveform support
- Torque computation from currents

**Control Modules** (`src.control.*`)
- Base controller framework
- FOC with PI current loops
- V/F scalar control
- SVM PWM generation
- Clark/Park transformations

**Simulation Engine** (`src.core.simulation_engine`)
- Time-stepping with configurable dt
- Event dispatch for UI updates
- Data collection and logging

**UI Layer** (`src.ui.*`)
- PyQt6 main window
- Accessible widgets with ARIA labels
- Real-time plot management
- Parameter input/output controls

**Visualization** (`src.visualization.visualization`)
- Matplotlib integration
- Configurable grid and plot styles
- Export functionality

Loaded Calibration Workflow
---------------------------

The repository includes a dedicated loaded no-field-weakening calibration example:

- Script: `examples/calibrate_no_fw_loaded_point.py`
- Seed source: prior converged unloaded/no-FW tuning session
- Output: JSON report in `data/logs/`

The workflow is intentionally staged:

1. Stabilize at the practical no-field-weakening speed cap.
2. Increase load torque with a smooth ramp and retain the highest point that still passes speed tracking.
3. Retune the final operating point with orthogonality and conditioned-efficiency gates.

This keeps torque-feasibility discovery separate from the stricter final loaded-point quality checks.

Data Flow
---------

::

    User Input
        ↓
    [Controller (FOC/V/F)]
        ↓
    [SVM Generator]
        ↓
    [Motor Model]
        ↓
    [Visualization]
        ↓
    GUI Display & CSV Log

Threading Model
---------------

- **Main Thread**: PyQt6 GUI and event loop
- **QThread**: Simulation engine runs in background thread
- **Thread-Safe Communication**: Qt signals/slots for thread safety
- **Data Logging**: Asynchronous CSV writing without blocking UI

Extensibility Points
--------------------

1. **New Control Strategy**
   - Inherit from `BaseController`
   - Implement `update()` and `get_control_law()` methods
   - Register in main_window

2. **New Load Model**
   - Extend load parameter definitions
   - Override load torque calculation
   - Update configuration loading

3. **New Visualization**
   - Add plots to visualization module
   - Register with matplotlib axes
   - Connect to simulation signals

Performance Considerations
--------------------------

- **Numba JIT**: Optional compilation for inner loops
- **Numpy Vectorization**: Array operations instead of loops
- **Event-Driven Updates**: Only redraw plots when data changes
- **Configurable Simulation Rate**: Adjust dt and update frequency
- **Efficient Data Structures**: Numpy arrays and dataclasses
