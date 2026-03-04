Advanced Topics
================

Field-Oriented Control (FOC) Implementation
--------------------------------------------

**Overview**
FOC decouples torque and flux control by transforming stator currents into a rotating reference frame aligned with the rotor flux.

**Mathematical Foundation**

1. **Clark Transformation** (3-phase to 2-phase)
   ::
   
      iα = ia
      iβ = (ia + 2*ib) / √3

2. **Park Transformation** (stationary to rotating frame)
   ::
   
      id = iα*cos(θ) + iβ*sin(θ)
      iq = -iα*sin(θ) + iβ*cos(θ)

3. **Inverse Park Transformation**
   ::
   
      Uα = Ud*cos(θ) - Uq*sin(θ)
      Uβ = Ud*sin(θ) + Uq*cos(θ)

**Control Strategy**
- Direct-axis (d): Controls flux magnitude
- Quadrature-axis (q): Controls torque
- Independent PI controllers for each axis
- Decoupling terms for cross-coupling compensation

**Performance Metrics**
- Steady-state error: < 2% under load steps
- Response time: < 50 ms to step change
- Efficiency: 94-96% at rated operation

Space Vector Modulation (SVM)
------------------------------

**Principle**
SVM generates PWM signals that approximate a desired voltage vector using the eight voltage vectors of a 3-phase inverter.

**SVM Duty Cycles**
The three phases are controlled by calculating:
- Neutral point displacement
- Sector identification
- Time calculations for active vectors

**Advantages**
- Near-sinusoidal current reproduction
- 15% higher DC link utilization vs. sinusoidal PWM
- Better harmonic performance
- Lower acoustic noise

**Implementation**
See `src.control.svm_generator.SVMGenerator.generate_pwm()` for details.

Monte Carlo Simulation
----------------------

**Purpose**
Analyze motor performance under parameter uncertainties and variations.

**Methodology**
1. Define parameter ranges (±10% typical)
2. Generate random samples (e.g., 1000 simulations)
3. Run simulation for each sample
4. Collect statistics (mean, std dev, min, max)

**Typical Analysis**
- Impact of manufacturing tolerances
- Parameter sensitivity analysis
- Performance distribution at different operating points

**Usage Example**
::

    from src.core.motor_model import MotorParameters
    from src.core.simulation_engine import SimulationEngine
    import numpy as np
    
    # Define parameter ranges
    n_simulations = 1000
    variations = {
        'Rs': np.random.normal(2.0, 0.2, n_simulations),
        'Ld': np.random.normal(0.005, 0.0005, n_simulations),
        'Lq': np.random.normal(0.008, 0.0008, n_simulations),
    }
    
    results = []
    for i in range(n_simulations):
        params = MotorParameters(...)
        # Update with variations
        engine = SimulationEngine(params, control_mode='FOC')
        result = engine.run()
        results.append(result)

Inductance Management
---------------------

**Phase Inductance vs. d-q Inductances**

The motor model uses phase inductance (L_phase) for the base calculation but also supports direct d-q axis inductance specification (Ld, Lq):

- **Ld**: Direct-axis inductance (flux-weakening effect)
- **Lq**: Quadrature-axis inductance (load-bearing effect)
- **L_phase**: Single inductance value used in base equations

**Selection**
- Use L_phase (Ld/Lq = L_phase) for isotropic motors
- Specify Ld < Lq for interior PM (IPM) motors with saliency
- Ld ≈ Lq for surface PM (SPM) motors

See `INDUCTANCE_MANAGEMENT.md` for detailed explanation.

Power Loss Analysis
-------------------

**Loss Components**

1. **Copper Losses** (I²R)
   ::
   
      P_copper = Rs * (ia² + ib² + ic²)

2. **Iron Losses** (hysteresis + eddy current)
   ::
   
      P_iron ≈ k_h * f * B_peak² + k_e * f² * B_peak²

3. **Mechanical Losses** (friction + windage)
   ::
   
      P_mech ≈ f_loss * ω

**Efficiency Calculation**
::

    η = P_out / (P_out + P_total_loss) * 100%

**Loss Optimization**
- Copper: Minimize stator current magnitude
- Iron: Reduce operating flux density
- Mechanical: Optimize rotor design

GUI Accessibility Features
---------------------------

**Screen Reader Support**
- All widgets have proper accessible names and descriptions
- Labels and help text automatically read
- Numeric values prefixed with descriptive text (e.g., "Current value: 1.5 A")

**Keyboard Navigation**
- Tab order properly configured
- Shortcuts for common operations
- No mouse-only controls

**Visual Accessibility**
- High contrast plots available
- Adjustable font sizes
- Color scheme supporting colorblind users

**Implementation Details**
See `src.ui.widgets.accessible_widgets` for widget implementations.

Data Logger Features
--------------------

**CSV Export Format**
- Header row with column names
- Metadata saved in separate JSON file
- Custom export paths supported
- Timestamped default locations

**Recorded Data**
- Time, speed, currents (3-phase)
- Voltages (3-phase), EMF (3-phase)
- Torque, load torque
- Omega, theta (rotor angle)

**Usage Example**
::

    logger = DataLogger()
    result = logger.save_simulation_data(
        history_dict,
        metadata={'test': 'FOC performance'},
        filename='custom_path.csv',
        use_custom_path=True
    )

Performance Tuning
------------------

**Simulation Parameters**
- `dt`: Time step (default 0.0001 s) - smaller = more accuracy but slower
- `update_freq`: GUI update frequency (default 100 Hz)
- `max_iterations`: Stop after N steps

**Control Tuning**
- **Kp (Proportional gain)**: Affects response speed and overshoot
- **Ki (Integral gain)**: Removes steady-state error
- **Bandwidth**: Typically 100-500 Hz for current loop

**Optimization Tips**
1. Use Numba JIT for 5-10x speedup
2. Reduce plot update frequency for faster execution
3. Implement early stopping conditions
4. Use event-driven updates instead of polling

Contributing Control Algorithms
--------------------------------

To add a new control algorithm:

1. Create new class in `src/control/` inheriting from `BaseController`
2. Implement `__init__`, `update()`, and `get_control_law()` methods
3. Register in `main_window.py` UI dropdown
4. Add tests in `tests/` directory
5. Document in this guide

**Template**
::

    from src.control.base_controller import BaseController
    
    class MyController(BaseController):
        def __init__(self, params):
            super().__init__(params)
            # Your initialization
        
        def update(self, state):
            # Update internal state
            pass
        
        def get_control_law(self):
            # Return control voltages
            return Ua, Ub, Uc
