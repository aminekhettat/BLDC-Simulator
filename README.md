# BLDC Motor Control Simulator - README

## Overview

Comprehensive Python GUI application for BLDC (Brushless DC) motor simulation with V/f voltage-to-frequency control. Designed with **full screen reader accessibility** and scalable architecture for future Field-Oriented Control (FOC) implementation.

## Key Features

### Motor Modeling

- **Complete BLDC motor model** with 3-phase electrical dynamics
- Trapezoidal back-EMF generation
- Mechanical dynamics (inertia, friction)
- Real-time numerical integration (RK4 method)
- Optimized calculations using NumPy

### Control System

- **V/f (Voltage-to-Frequency) controller** for open-loop speed control
- **Field-Oriented Control (FOC)** with selectable Clarke or Concordia transforms,
  auto-tunable PI current regulators, and Cartesian/polar output modes
- Space Vector Modulation (SVM) for 3-phase PWM generation
- Frequency slew-rate limiting (soft-start capability)
- Flexible control architecture for easy switching between algorithms
- Advanced inverter realism with individually switchable blocks for:
  device drop, current-dependent conduction loss, switching loss,
  direction-aware dead-time distortion, freewheel diode loss,
  minimum pulse suppression, DC-link ripple, thermal coupling, and phase mismatch

### Load Profiles

- Constant load
- Ramp-up load (time-dependent)
- Variable/cyclic load patterns
- Custom load functions

### Power Supply Profiles

- Configurable DC bus voltage model
- Constant or ramping voltage
- Logged alongside motor data for analysis

### Accessibility & GUI

- **Built with PyQt6** (no tkinter - full accessibility support)
- **Screen reader compatible** with NVDA, JAWS, etc.
- Tab key navigation throughout application
- Clear keyboard shortcuts (F5=Start, F6=Stop, F7=Reset, Ctrl+S=Export)
- Descriptive labels and help text for all controls

### Data & Visualization

- Real-time parameter monitoring
- CSV export with metadata
- Comprehensive plotting capabilities:
  - 3-phase currents, voltages, back-EMF
  - Speed and torque profiles
  - Phase portraits
  - Dedicated inverter analysis: DC-link voltage/ripple, loss breakdown,
    junction temperature, and common-mode voltage
- Custom multi-axis plotting for arbitrarily selected variables
- Vocal assistance announcements for plot generation and simulation events

### Architecture

- **Modular design** - Easy to extend with FOC, current control, etc.
- **Well-documented code** - Full Sphinx-style docstrings
- **Efficient computations** - Optimized for real-time performance
- **Type hints** - Clear interfaces for extension

## Project Structure

```
BLDC_motor_control/
├── main.py                      # Application entry point
├── requirements.txt             # Python dependencies
├── README.md
├── src/
│   ├── core/                    # Motor models and simulation
│   │   ├── motor_model.py       # BLDC motor physics
│   │   ├── load_model.py        # Load profiles
│   │   └── simulation_engine.py # Main simulation loop
│   ├── control/                 # Control algorithms
│   │   ├── base_controller.py   # Abstract controller interface
│   │   ├── svm_generator.py     # SVM modulation
│   │   └── vf_controller.py     # V/f speed controller
│   ├── ui/                      # GUI application
│   │   ├── main_window.py       # Main window
│   │   └── widgets/
│   │       └── accessible_widgets.py  # Accessible PyQt6 widgets
│   ├── utils/                   # Utilities
│   │   ├── config.py            # Configuration parameters
│   │   └── data_logger.py       # Data logging and export
│   └── visualization/           # Plotting and visualization
│       └── visualization.py     # matplotlib plots
├── data/
│   ├── logs/                    # CSV simulation data
│   └── plots/                   # Generated plots
└── tests/                       # Unit tests (future)
```

## Installation

### Requirements

- Python 3.9+
- Windows/Linux/macOS

### Setup

1. **Install dependencies**:

```bash
pip install -r requirements.txt
```

2. **Run application**:

```bash
python main.py
```

## Usage Guide

### Starting a Simulation

1. **Configure Motor Parameters** (Tab 1)
   - Enter motor nominal voltage, resistance, inductance
   - Set back-EMF constant, torque constant
   - Configure rotor inertia and friction

2. **Set Load Profile** (Tab 2)
   - Choose load type (Constant, Ramp, Variable)
   - Configure load parameters (torque, duration, etc.)

3. **Configure V/f Controller** (Tab 3)
   - Set nominal voltage and frequency
   - Adjust startup/ramp parameters
   - Set speed reference

4. **Start Simulation** (F5)
   - Monitor real-time parameters (Tab 4)
   - Controller and motor run simultaneously

5. **Analyze Results**
   - Generate plots (Tab 5)
   - Export data to CSV (Ctrl+S)

### Keyboard Shortcuts

- **F5**: Start simulation
- **F6**: Stop simulation
- **F7**: Reset to initial state
- **Ctrl+S**: Export data
- **Ctrl+Q**: Quit application
- **Tab**: Navigate between fields
- **Arrow keys**: Navigate lists when focused

## Screen Reader Usage

The application is designed for NVDA, JAWS, and other screen readers:

1. **Field Labels**: All input fields have descriptive labels
2. **Help Text**: Each parameter has accessibility descriptions
3. **Tab Navigation**: Fluent tab-key navigation between sections
4. **Status Updates**: Real-time values announced for monitoring
5. **Buttons**: Clear action descriptions

### Tips for Screen Reader Users

- Use Tab to move between tabs, then arrow keys to navigate within
- Listen for parameter descriptions before entering values
- Status values in monitoring tab update automatically during simulation
- Use Alt+F4 to close application, or Ctrl+Q

## Mathematical Background

### BLDC Motor Model

**Electrical Equation (Phase):**

```
v_phase = R*i_phase + L*di_phase/dt + e_back_emf
```

**Back-EMF (Trapezoidal):**

```
e = K_emf * ω * trapezoidal(θ)
```

**Mechanical Equation:**

```
dω/dt = (τ_electromagnetic - τ_load - f*ω) / J
```

### V/f Control

**Linear V/f Characteristic:**

```
V(f) = V_startup + K_vf * f
```

Where `K_vf = (V_nominal - V_startup) / f_nominal`

### SVM Modulation

Generates 3-phase PWM using sector-based space vector synthesis:

1. Identify voltage sector (1-6)
2. Calculate dwell times for adjacent vectors
3. Generate time-averaged phase voltages

## Configuration

Key parameters in `src/utils/config.py`:

- **Motor Defaults**: Nominal voltage, resistance, inductance, etc.
- **Simulation**: Time step (dt), maximum history size
- **V/f Controller**: Nominal V/f parameters, slew rates
- **GUI**: Update interval, plot buffer size

Modify these for different motor types or simulation speeds.

## Extension Points (for FOC Implementation)

The architecture is designed for FOC extension:

1. **BaseController Interface** (`src/control/base_controller.py`)
   - Inherit for new control algorithms
   - Implement `update()`, `reset()`, `get_state()`

2. **Coordinate Transformations** (Ready to add)
   - Clarke transform (3-phase → α-β)
   - Park transform (α-β → d-q)

3. **Current Control Loop** (Ready to add)
   - Inherit BaseController
   - Implement PI regulators for d and q axes
   - Generate voltage references for SVM

4. **Rotor Position Estimation** (Future)
   - Sensorless estimation from back-EMF
   - PLL for grid synchronization

## Performance Optimization

The simulator is optimized for real-time efficiency:

1. **NumPy Vectorization**: Batch calculations on arrays
2. **RK4 Integration**: Accurate state updates at O(dt^5)
3. **Threading**: Background simulation thread (non-blocking GUI)
4. **Selective Logging**: Optional data logging to reduce overhead
5. **Buffered History**: Circular buffer for limited memory usage

## Future Enhancements

- [ ] Field-Oriented Control (FOC) implementation
- [ ] Sensorless rotor position estimation
- [ ] Current control loops (PI regulators)
- [ ] Power factor correction
- [ ] Harmonic analysis
- [ ] 3D motor visualization
- [ ] Hardware interface (real-time DAQ)
- [ ] Multi-motor simulation

## License & Attribution

Educational use simulator. Based on standard BLDC motor control theory.

## Support & Documentation

- **Full Sphinx docstrings** in all modules
- **Type hints** for clarity
- **Examples** in docstrings

## Release & Governance Checklist

Before merge or release:

1. Regression gate is green in CI: `Regression Gates / regression`
2. Baseline integrity and regression tests pass locally:

```bash
pytest tests/test_baseline_integrity.py tests/test_regression_baseline.py tests/test_regression_baseline_foc.py tests/test_regression_reporting.py -v
```

3. If baseline JSON files changed, include in PR:
   - `Baseline rationale:`
   - `Drift evidence:`
4. Use templates:
   - PR template: `.github/PULL_REQUEST_TEMPLATE.md`
   - Release notes template: `RELEASE_NOTES_TEMPLATE.md`

Policy references:

- `CONTRIBUTING.md`
- `docs/branch_protection.rst`

Generate documentation:

```bash
sphinx-build -b html docs/ build/html
```

## References

- BLDC motor control fundamentals
- SVM modulation techniques
- V/f control characteristics
- PyQt6 accessibility documentation

---

**Author**: BLDC Control Team  
**Version**: 1.0.0  
**Date**: March 2026
