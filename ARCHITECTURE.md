# BLDC Motor Control - Development Notes

## Architecture Overview

This document provides insights into the project architecture and design decisions.

## Core Components Interaction

```
┌─────────────────────────────────────────────────────────┐
│               PyQt6 GUI Application                      │
│  ┌────────────────────────────────────────────────────┐ │
│  │  Main Window (BLDCMotorControlGUI)                │ │
│  │  ├─ Parameters Tab                                │ │
│  │  ├─ Load Profile Tab                             │ │
│  │  ├─ V/f Control Tab                              │ │
│  │  ├─ Monitoring Tab (Real-time)                   │ │
│  │  └─ Plotting Tab                                 │ │
│  └────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
        ▼                  ▼                  ▼
  ┌──────────────┐  ┌─────────────┐  ┌─────────────────┐
  │ Simulation   │  │ SVM         │  │ V/f Controller  │
  │ Engine       │  │ Generator   │  │                 │
  └──────────────┘  └─────────────┘  └─────────────────┘
        │
        ▼
  ┌──────────────────────────────────┐
  │  BLDC Motor Model                │
  │  ├─ Electrical Dynamics (3-phase)│
  │  ├─ Back-EMF Generation          │
  │  ├─ Mechanical Dynamics          │
  │  └─ RK4 Integration              │
  └──────────────────────────────────┘
        │
        ▼
  ┌──────────────────────────────────┐
  │  Load Profile                    │
  │  ├─ Constant Load                │
  │  ├─ Ramp Load                    │
  │  ├─ Variable Load                │
  │  └─ Cyclic Load                  │
  └──────────────────────────────────┘
```

## Motor Model Physics

### Electrical Equations

For each phase (a, b, c):

```
v_phase = R * i_phase + L * di_phase/dt + e_back_emf
```

Rearranged for numerical integration:

```
di_phase/dt = (1/L) * (v_phase - R*i_phase - e_back_emf)
```

### Back-EMF Generation

Trapezoidal model (realistic for BLDC):

```
e = K_emf * ω * f_trap(θ)
```

Where:

- K_emf: Back-EMF constant [V·s/rad]
- ω: Angular velocity [rad/s]
- f_trap(θ): Trapezoidal function based on rotor position
- θ: Rotor position [rad] (electrical angle = θ \* poles_pairs)

### Mechanical Equations

Newton's second law:

```
dω/dt = (1/J) * (τ_em - τ_load - f*ω)
dθ/dt = ω
```

Where:

- J: Rotor moment of inertia
- τ_em: Electromagnetic torque
- τ_load: Load torque
- f: Friction coefficient
- ω: Angular velocity

### Electromagnetic Torque

Proportional to phase current:

```
τ_em = K_t * i_active
```

For trapezoidal BLDC, active current is max of conducting phases.

## Numerical Integration

Uses 4th-order Runge-Kutta (RK4) for accuracy:

```
State: [i_a, i_b, i_c, ω, θ]

k1 = f(t_n, y_n)
k2 = f(t_n + h/2, y_n + h/2*k1)
k3 = f(t_n + h/2, y_n + h/2*k2)
k4 = f(t_n + h, y_n + h*k3)

y_{n+1} = y_n + (h/6)(k1 + 2k2 + 2k3 + k4)
```

Advantages:

- O(h^5) local truncation error
- Good stability for motor systems
- Accurate back-EMF calculation

## SVM Modulation

Space Vector Modulation synthesizes reference voltage using 6 active vectors:

```
Voltage Hexagon:
     V2 (120°)
      /\
    /    \
  V3       V1
  |        |
  |        |
  |        | Sector 1
  |        |
  V4       V6
    \    /
      \/
    V5 (240°)
```

Algorithm:

1. Identify sector from voltage angle
2. Calculate dwell times for adjacent vectors
3. Generate time-averaged PWM duty cycles

Result: Efficient voltage utilization (~66% of Vdc vs 50% for simple)

## V/f Control Characteristic

Linear V/f relationship:

```
V(f) = V_startup + K_vf * f

K_vf = (V_nominal - V_startup) / f_nominal
```

Benefits:

- Open-loop (no speed feedback needed)
- Soft-start (startup boost)
- Simple to tune
- Effective for constant-torque loads

Limitations:

- Torque varies with load
- No closed-loop speed regulation
- Limited dynamic performance

## Extensibility for FOC

The architecture is ready for Field-Oriented Control (FOC):

### Step 1: Coordinate Transforms (Ready to add)

- Clarke transform: 3-phase → α-β
- Park transform: α-β → d-q

### Step 2: Current Control (Ready to add)

- Inherit BaseController
- Implement PI regulators for Id, Iq
- Generate Vd, Vq references

### Step 3: Rotor Estimation (Ready to add)

- Phase lock loop (PLL) on back-EMF
- Sensorless position estimation
- Observer-based speed estimation

### Step 4: Field Control (Ready to add)

- Flux linkage regulation
- Torque vs speed optimization
- Efficiency improvement

## Accessibility Implementation

### Screen Reader Support

- All UI elements have AccessibleName
- Descriptions provided via AccessibleDescription
- Labels grouped with controls

### Keyboard Navigation

- Tab key cycles through controls
- Arrow keys navigate lists
- F-keys for common operations
- Enter for activation

### GUI Design

- Logical tab order
- Clear status announcements
- Descriptive button labels
- No visual-only information

## Performance Optimization

### Computation Efficiency

- NumPy vectorization for batch operations
- RK4 local calculations (no extra loops)
- Selective data logging (skip when not needed)

### Memory Management

- Circular history buffer (max_history)
- Efficient numpy array usage
- Automatic old data cleanup

### GUI Responsiveness

- Background threading for simulation
- Non-blocking updates
- Configurable update intervals
- Real-time monitoring

## Testing Strategy

### Unit Tests (tests/test_motor.py)

- Motor model accuracy
- Controller functionality
- Load profile behavior
- Integration between components

### Example Script (examples/example_vf_control.py)

- Demonstrates typical workflow
- Shows programmatic usage
- Serves as regression test

### Manual Testing

- Visual inspection of plots
- Parameter sweep verification
- Extreme condition testing

## File Organization Rationale

```
src/core/       - Physics (independent of UI/control)
src/control/    - Algorithm implementations
src/ui/         - User interface (PyQt6)
src/utils/      - Shared utilities
visualization/  - Plotting and data export
tests/          - Unit tests and verification
examples/       - Usage demonstrations
data/           - Output storage
```

Benefits:

- Core physics testable without GUI
- Control blocks reusable
- UI replaceable
- Clear separation of concerns

## Future Enhancement Roadmap

### Phase 1: Current State (Complete)

- ✅ V/f control
- ✅ SVM modulation
- ✅ GUI with accessibility
- ✅ Data logging/visualization

### Phase 2: FOC Implementation

- [ ] Coordinate transforms
- [ ] Current regulators (PI)
- [ ] d-q decoupling
- [ ] Rotor angle estimation (PLL)

### Phase 3: Advanced Features

- [ ] Sensorless control
- [ ] Power factor correction
- [ ] Efficiency optimization
- [ ] Multi-axis control

### Phase 4: Hardware Integration

- [ ] Real-time target support
- [ ] DAQ device drivers
- [ ] Real hardware motor interface
- [ ] FPGA implementation

## Configuration Parameters

Key tuning parameters in `src/utils/config.py`:

- **dt** (0.0001s): Simulation step - reduce for higher accuracy, increase for speed
- **max_history** (100000): Data buffer size - increase for longer simulation recording
- **update_interval** (100ms): GUI refresh rate - tweak for responsiveness

## Development Tips

### Adding a New Controller

1. Inherit from `BaseController`
2. Implement `update(dt)`, `reset()`, `get_state()`
3. Add to UI in main_window.py
4. Test with motor model

### Modifying Motor Parameters

1. Edit `src/core/motor_model.py` MotorParameters class
2. Update config defaults in `src/utils/config.py`
3. Add GUI fields in `src/ui/main_window.py`

### Adding Plots

1. Create function in `src/visualization/visualization.py`
2. Add button in plotting tab
3. Call from `BLDCMotorControlGUI._plot_*()` method

### Improving Accessibility

1. Add descriptive labels to all widgets
2. Use AccessibleName and AccessibleDescription
3. Test with screen reader (NVDA free on Windows)
4. Ensure Tab key navigation flow

---

For questions about implementation, review the comprehensive docstrings in the source code.
