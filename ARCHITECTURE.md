# BLDC Motor Control - Development Notes

## Architecture Overview

This document provides insights into the project architecture and design decisions.

## 2026 Runtime Architecture Extension

The simulator architecture now includes four additional runtime layers:

## Motor Parameter Import/Save Architecture

The simulator now supports parameter profile persistence for repeatable motor setup.

1. UI Interaction Layer

- File menu exposes:
  - Import Motor Parameters (JSON)
  - Save Motor Parameters (JSON)
  - Load Built-in Motor Profile (from `data/motor_profiles`)
- Imported values are written directly into motor parameter widgets and applied to
  simulation state.

2. Profile Persistence Layer

- Module: `src/utils/motor_profiles.py`
- Responsibilities:
  - Discover built-in profiles
  - Validate profile schema and motor parameter completeness
  - Normalize profile fields (pole-pairs derivation, Ld/Lq defaults)
  - Read/write JSON profiles

3. Storage Layer

- Directory: `data/motor_profiles`
- Schema id: `bldc.motor_profile.v1`
- Profile structure:
  - `motor_params`: simulation-ready motor model parameters
  - `rated_info`: vendor rated operating point
  - `source`: traceability (vendor URL / notes)

4. Startup Defaults and Built-ins

- Built-in menu is populated from all JSON profiles in `data/motor_profiles`.
- User-saved profiles are immediately discoverable by the built-in loader.

1. Startup Sequencing Layer

- FOC and V/f controllers each expose a phase-machine startup flow.
- FOC sequence supports: align -> open-loop -> closed-loop handoff.
- V/f sequence supports: align -> open-loop ramp -> run.

2. Inverter Realism Layer

- SVM remains the modulation core and is now wrapped by optional realism blocks.
- Each block is independently switchable and stateful where needed.
- Runtime telemetry includes effective bus voltage, loss breakdown,
  junction temperature, common-mode voltage, and minimum pulse events.

3. Communication Backend Layer

- SimulationEngine can route voltage commands through a hardware interface.
- A mock DAQ backend is provided for dry-run and integration testing.
- Fail-safe fallback keeps simulation running even if backend I/O fails.

4. Compute Backend Layer

- Compute backend policy is selectable (`auto`, `cpu`, `gpu`).
- GPU probing and fallback metadata are tracked in simulation state.
- Power metric evaluation can execute via CuPy when GPU backend is active.

## Auto-Tuning Convergence System (March 2026)

A production-grade auto-tuning framework has been integrated to optimize FOC PI controller parameters:

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Auto-Tune Until Convergence Script                     │
│  (examples/auto_tune_until_convergence.py)              │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌─ Stage 1: Current PI Optimization ─────────────┐    │
│  │  • Initial candidate generation                │    │
│  │  • Trial evaluation on search_time window      │    │
│  │  • Orthogonality gate (load-aware d/q decoup) │    │
│  │  • Best current gains selected                 │    │
│  └──────────────────────────────────────────────┘    │
│            │                                           │
│            ▼                                           │
│  ┌─ Stage 2: Speed PI Optimization ──────────────┐    │
│  │  • Constrain current PI (from Stage 1)        │    │
│  │  • Vary speed PI gains                        │    │
│  │  • Validation: settling time, steady-state   │    │
│  │  • Best speed gains selected                  │    │
│  └──────────────────────────────────────────────┘    │
│            │                                           │
│            ▼                                           │
│  ┌─ Stage 3: Unbounded Expansion (NEW!) ─────────┐    │
│  │  if --max-trials ≤ 0 and not converged:      │    │
│  │    • Expand parameter space (1.15× per round)│    │
│  │    • Build new neighbors from scaled seed    │    │
│  │    • Continue until convergence achieved     │    │
│  │    • Adaptive scaling factor up to 1.5×      │    │
│  └──────────────────────────────────────────────┘    │
│            │                                           │
│            ▼                                           │
│  ┌─ Verification: 20s Extended Run ──────────────┐    │
│  │  • Apply best candidate to motor             │    │
│  │  • Run verify_time_s window (default 8-20s) │    │
│  │  • Confirm full_converged=true               │    │
│  │  • Measure final speed ratio                 │    │
│  └──────────────────────────────────────────────┘    │
│            │                                           │
│            ▼                                           │
│  ┌─ Session Storage ─────────────────────────────┐    │
│  │  • JSON session file with all metrics       │    │
│  │  • Trial debug log (first N trials)         │    │
│  │  • Acceptance/convergence evidence          │    │
│  │  • Best candidate dictionary (PI gains)     │    │
│  └──────────────────────────────────────────────┘    │
│                                                       │
└─────────────────────────────────────────────────────────┘
```

### Key Components

**Trial Structure:**

- Each trial evaluates one (current_kp, current_ki, speed_kp, speed_ki, iq_limit_a) candidate
- Search phase: 0.6s default motor simulation to measure orthogonality & speed tracking
- Verification phase: 8-20s extended run to confirm stability and convergence

**Convergence Criteria:**

1. Speed error within ±2% tolerance band (configurable)
2. Tail mean error ≤ band upper (smooth tracking)
3. Tail max error ≤ 2× band upper (no large spikes)
4. Settling time achieved (stabilization detected)
5. Orthogonality gate pass (d/q decoupling quality)
6. Full verification window converged (20s extended run)

**Trial Limits:**

- Bounded mode: `--max-trials N` (N > 0) → search for N trials total
- Unbounded mode: `--max-trials 0` (or ≤ 0) → expand parameter space iteratively until convergence
- Overcurrent control:
  - `--overcurrent-limit-a A` (A > 0) → abort trial if phase current exceeds A amperes
  - `--overcurrent-limit-a 0` (or ≤ 0) → disable overcurrent abort (full search)

### Session Metadata Schema

Tuning sessions are saved as JSON with schema version v2:

```json
{
  "session_version": "v2",
  "scenario": "auto_tune_until_convergence",
  "trial_limit_mode": "unbounded|bounded",
  "trial_limit": -1 | 0 | N,
  "overcurrent_limit_a": null | 0.0 | A,
  "profiles_tuned": ["Motor Profile Name"],
  "results": {
    "tested_trials": <count>,
    "best_search_trial": { ...trial_dict... },
    "best_candidate": {
      "speed_pi": { "kp": X, "ki": Y },
      "current_pi": { "d_kp": X, "d_ki": Y },
      "iq_limit_a": <float>
    },
    "verification": {
      "converged": true|false,
      "full_converged": true|false,
      "final_speed_rpm": <float>,
      "final_speed_ratio": <float>,
      "tail_abs_mean_error_rpm": <float>,
      "tail_abs_max_error_rpm": <float>
    }
  },
  "acceptance": {
    "accepted": true|false,
    "reason": "convergence_ok" | "overcurrent_abort" | "search_exhausted"
  }
}
```

### Real-World Outcomes (March 2026)

All three motor profiles achieved successful convergence with unbounded mode:

| Motor Profile         | Target RPM | Trials | Final Speed | Ratio  | Status            |
| --------------------- | ---------- | ------ | ----------- | ------ | ----------------- |
| Innotec 255-EZS48-160 | 1500       | 71     | 1498.79     | 0.9992 | ✅ Full Converged |
| Motenergy ME1718 48V  | 1500       | 71     | 1506.52     | 1.0044 | ✅ Full Converged |
| Motenergy ME1719 48V  | 1500       | 71     | 1506.52     | 1.0044 | ✅ Full Converged |

All speed ratios within ±2% tolerance. Unbounded mode enabled convergence where bounded high-budget searches (5000 trials) failed due to finite candidate pool architecture.

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
