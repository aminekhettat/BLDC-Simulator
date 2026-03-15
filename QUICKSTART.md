"""
BLDC Motor Control - Quick Start Guide
=======================================

This guide will help you get started with the BLDC motor control simulator.
"""

# QUICK START GUIDE - BLDC Motor Control Simulator

## Installation & First Run (2 minutes)

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

(Optional) verify installation by running the unit tests:

```bash
python -m pytest -q
```

### 2. Launch Application

```bash
python main.py
```

You should see the main application window with tabs for motor configuration.

### 3. Optional: Verify GPU Backend Availability

GPU support is optional and safely falls back to CPU.

- Backend policy is configured in `src/utils/config.py` via
  `SIMULATION_PARAMS["compute_backend"]`.
- Valid values: `auto`, `cpu`, `gpu`.
- In `auto` mode, the simulator selects GPU only if CuPy and a CUDA device are
  available; otherwise CPU is used automatically.

---

## Your First Simulation (5 minutes)

### Step 1: Motor Parameters (Tab 1)

Accept all defaults or modify:

- **Nominal Voltage**: 48V
- **Phase Resistance**: 2.5Ω
- **Back-EMF Constant**: 0.1 V·s/rad
- **Torque Constant**: 0.1 N·m/A

These are typical values for a small hobby BLDC motor.

### Step 2: Load Profile (Tab 2)

- **Load Type**: Select "Constant"
- **Constant Load Torque**: 0.5 N·m

On this tab you can also configure a **Supply Profile** (constant or ramp) which
models the DC bus voltage feeding the inverter. This value is logged with the
motor data.

This simulates a constant mechanical load on the motor shaft.

### Step 3: Controller (Tab 3)

- Select **Control Mode**: choose "V/f" for open-loop speed control or "FOC" for
  field‑oriented control.

For V/f mode:

- **Nominal Voltage**: 48V
- **Nominal Frequency**: 100 Hz
- **Startup Voltage**: 1V
- **Speed Reference**: 50 Hz (half speed)

For FOC mode you can set d/q current references, choose output mode (polar vs
Cartesian) and auto‑tune the PI controllers with the provided buttons.

You can also enable startup sequencing for both modes:

- V/f: alignment + open-loop ramp + run
- FOC: alignment + forced open-loop ramp + observer handoff

These startup options are useful for robust low-speed behavior and reproducible
sensorless bring-up studies.

### Step 3b: Inverter and Timing (Control Tab)

Configure inverter realism and control-loop timing from the same tab:

- Enable/disable each inverter realism block independently.
- Set switching frequency (also defines control update period).
- Use MCU budget estimator fields to map host compute time to target MCU load.

These set up the voltage-to-frequency characteristic.

### Step 4: Start Simulation

- Press **F5** or click "Start Simulation"
- Watch the Monitoring tab (Tab 4) for real-time values

### Step 5: Export Results

- Press **Ctrl+S** or click "Export Data"
- Data is saved as CSV in `data/logs/`

### Step 6: View Plots

- Go to Plotting tab (Tab 5)
- Click "Plot 3-Phase Overview" or "Plot Currents"
- Plots open in new windows

---

## Understanding the Output

### Monitoring Tab Shows

- **Rotor Speed**: in RPM (acceleration ramps up)
- **Phase Currents**: A, B, C phase currents
- **Back-EMF**: Back-electromotive force per phase
- **Torque**: Electromagnetic torque vs load

### Plots Show

1. **3-Phase Currents**: Should see smooth 3-phase current waveforms
2. **Voltages**: Applied 3-phase voltage from SVM
3. **Back-EMF**: Induced voltage proportional to speed
4. **Speed Plot**: Should accelerate smoothly with V/f ramp
5. **Torque**: Electromagnetic torque vs load torque

### CSV Data Contains

- Time points
- Phase currents (A, B, C)
- Motor speed (RPM, rad/s)
- Position
- Back-EMF values
- Applied voltages
- Generated torques

---

## Keyboard Navigation (Screen Reader Users)

| Key             | Action                             |
| --------------- | ---------------------------------- |
| **F5**          | Start Simulation                   |
| **F6**          | Stop Simulation                    |
| **F7**          | Reset                              |
| **Ctrl+S**      | Export Data                        |
| **Ctrl+Q**      | Quit                               |
| **Tab**         | Navigate between tabs and fields   |
| **Shift+Tab**   | Navigate backwards                 |
| **Arrow Keys**  | Navigate within lists when focused |
| **Enter/Space** | Activate buttons                   |

All input fields have descriptive labels and help text for screen readers.

---

## Common Experiments

### Experiment 1: Speed Response

1. Set speed reference to 100 Hz
2. Set frequency slew rate to 100 Hz/s
3. Watch speed ramp up smoothly
4. Observe current and torque response

### Experiment 2: Load Effect

1. Set constant load to 3.0 N·m
2. Run simulation
3. Compare speed with 0 N·m load
4. Higher load = lower steady-state speed

### Experiment 3: Soft Start (Startup Boost)

1. Set startup voltage to 2V
2. Run simulation from 0 speed
3. Observe faster initial acceleration
4. Smooth transition to V/f characteristic

### Experiment 4: Ramping Load

1. Set load type to "Ramp"
2. Initial torque: 0, Final: 2 N·m, Duration: 1s
3. Watch how motor responds to increasing load
4. Current increases as load ramps up

---

## Interpreting Monitor Values

| Value             | Meaning                | Typical Range         |
| ----------------- | ---------------------- | --------------------- |
| **Speed (RPM)**   | Rotor rotation speed   | 0-10000               |
| **Phase Current** | Current in each phase  | -10...+10 A           |
| **Back-EMF**      | Induced voltage        | Proportional to speed |
| **Torque**        | Electromagnetic torque | 0-5 N·m               |
| **Omega (rad/s)** | Angular velocity       | Speed×2π/60           |

---

## Troubleshooting

### Simulation runs but motor doesn't accelerate

- **Check**: Speed reference > 0 (Tab 3)
- **Check**: DC voltage is sufficient (Tab 3)
- **Check**: Load torque isn't too high (Tab 2)

### Values all show zero

- **Check**: Has simulation started? (Press F5)
- **Check**: Look in Monitoring tab (Tab 4)

### Can't export data

- **Check**: Simulation must have run (at least one step)
- **Check**: Directory `data/logs/` must exist (created automatically)

### Screen reader not announcing values

- **Check**: Values update only during simulation
- **Setup**: Use Tab to navigate to Monitoring tab for focus
- **Listen**: Values announced automatically during updates

---

## Advanced: Programmatic Usage

For headless simulations, use the example script:

```bash
python examples/example_vf_control.py
```

This demonstrates:

- Creating motor model
- Setting up V/f controller
- Running simulation loop
- Exporting results
- Generating plots

Edit the example to modify parameters or simulate different scenarios.

---

## Project Structure

```
├── main.py              ← Run this to start GUI
├── examples/
│   └── example_vf_control.py    ← Headless example
├── src/
│   ├── core/            ← Motor model, simulation
│   ├── control/         ← Controllers (V/f, SVM)
│   ├── ui/              ← GUI and widgets
│   ├── utils/           ← Config and logging
│   └── visualization/   ← Plotting
├── data/
│   ├── logs/            ← Saved CSV data
│   └── plots/           ← Saved plot images
└── tests/
    └── test_motor.py    ← Unit tests (pytest)
```

---

## Configuration

Edit `src/utils/config.py` to change:

- Default motor parameters
- Simulation time step (dt)
- GUI update rate
- Plot styling

---

## Next Steps

1. **Run the GUI**: `python main.py`
2. **Try experiments** above
3. **Export data** and analyze in Excel/Python
4. **Generate plots** to visualize behavior
5. **Modify parameters** and see effects
6. **Read code comments** - fully documented with Sphinx docstrings

---

## For Screen Reader Users

The entire application is designed for accessibility:

- **Named fields**: Every input has a descriptive label
- **Help text**: Hover over fields or press F1 for help
- **Tab navigation**: Fluent tab-key flow through all controls
- **Status updates**: Real-time values announced during simulation
- **Keyboard shortcuts**: F5-F7 for common operations, Ctrl+S for export

**Recommended Screen Readers**:

- NVDA (Windows) - Free
- JAWS (Windows)
- VoiceOver (Mac)
- Orca (Linux)

---

## More Help

- **Full documentation**: See README.md
- **API documentation**: Read docstrings in source code
- **Examples**: See `examples/` folder
- **Tests**: See `tests/` folder for usage examples

---

**Happy simulating! 🎉**

For questions, check the README.md or review the comprehensive docstrings in the source code.
