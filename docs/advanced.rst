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

Loaded No-Field-Weakening Calibration
-------------------------------------

The repository includes a staged loaded-point calibration workflow in `examples/calibrate_no_fw_loaded_point.py`.

**Why staged acceptance is used**

- A strict all-criteria gate can reject candidates too early when the real question is whether the motor can hold speed under load.
- The workflow therefore checks speed feasibility first, then orthogonality, then efficiency only when the mechanical power/load is large enough for efficiency to be meaningful.

**Acceptance stages**

1. ``speed_tracking_passed``
2. ``orthogonality_stage_passed``
3. ``efficiency_conditioned_passed``

**Current measured outcome**

- Practical no-FW target speed: about ``1617.44 rpm``
- Selected speed-feasible torque: about ``9.99 Nm``
- Final high-fidelity verification still fails orthogonality and conditioned efficiency, so the report remains informative rather than fully accepted.

Shunt Current Reconstruction Fidelity
-------------------------------------

This section summarizes measured-vs-true current fidelity for topology-aware
shunt reconstruction and documents the anti-saturation gain adaptation used
before benchmarking.

Validation scope
^^^^^^^^^^^^^^^^

- Unit and integration checks: ``tests/test_current_sense_verification.py``
- Real motor profiles:
    - ``data/motor_profiles/innotec_255_ezs48_160.json``
    - ``data/motor_profiles/motenergy_me1718_48v.json``
    - ``data/motor_profiles/motenergy_me1719_48v.json``

Automated verification status
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- ``46 passed`` on ``tests/test_current_sense_verification.py``
- Coverage includes:
    - triple-shunt direct measurement accuracy
    - double-shunt Kirchhoff reconstruction consistency
    - single-shunt sector-aware reconstruction across sectors 1..6
    - engine-history consistency (``currents_*`` vs ``currents_*_true``)

Adaptive amplifier-gain method (anti-saturation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before running fidelity metrics on each motor profile, the current-sense
amplifier gain is adapted from a peak-current probe.

1. Run a short probe with low gain (``probe_gain = 0.2``) to estimate
     ``I_peak`` from true physics currents.
2. Compute a safe gain with headroom:

     ``gain = min(v_offset, vcc - v_offset) * headroom / (r_shunt * I_peak)``

3. Clamp gain to implementation bounds.
4. Re-run fidelity metrics and verify zero ADC saturation.

Benchmark conditions
^^^^^^^^^^^^^^^^^^^^

- ``dt = 1e-4 s``
- ``n_steps = 800``
- load torque ``2.0 Nm``
- three-phase sinusoidal excitation, magnitude ``24.0 V``, frequency ``120 Hz``
- ``r_shunt = 1e-3 ohm``, ``vcc = 3.3 V``, ``offset = 1.65 V``
- gain headroom factor: ``0.80``

Adaptive gain and saturation summary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Per-profile calibrated gain and ADC saturation
     :header-rows: 1

     * - Motor profile
         - Estimated peak current (A)
         - Calibrated gain
         - Triple sat ratio
         - Double sat ratio
         - Single sat ratio
     * - Innotec 255-EZS48-160
         - 1685.820
         - 0.783002
         - 0.000
         - 0.000
         - 0.000
     * - Motenergy ME1718 48V
         - 1554.738
         - 0.849018
         - 0.000
         - 0.000
         - 0.000
     * - Motenergy ME1719 48V
         - 1554.738
         - 0.849018
         - 0.000
         - 0.000
         - 0.000

Representative measured-vs-true errors (ME1718, adaptive gain)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Error metrics by topology (A)
     :header-rows: 1

     * - Topology
         - Phase
         - Max abs error
         - RMS error
         - Median abs error
         - P95 abs error
     * - Triple
         - a
         - 3.185e-06
         - 1.337e-07
         - 6.392e-08
         - 1.219e-07
     * - Triple
         - b
         - 3.047e-06
         - 1.289e-07
         - 6.113e-08
         - 1.256e-07
     * - Triple
         - c
         - 3.047e-06
         - 1.295e-07
         - 6.389e-08
         - 1.250e-07
     * - Double
         - c (reconstructed)
         - 6.232e-06
         - 2.318e-07
         - 6.254e-08
         - 1.277e-07
     * - Single
         - a
         - 3.185e-06
         - 9.743e-07
         - 6.809e-07
         - 2.119e-06

Interpretation
^^^^^^^^^^^^^^

- With adaptive gain enabled, all evaluated real-motor profiles run with
    zero ADC clipping in triple, double, and single-shunt modes.
- Triple-shunt remains the closest to true currents (lowest RMS error).
- Double-shunt keeps low error on measured phases and controlled error on the
    reconstructed phase.
- Single-shunt remains physically consistent and benefits directly from proper
    gain tuning to preserve full observable current amplitude.

Raw benchmark reports are available in:

- ``data/logs/current_sense_fidelity_innotec_255_ezs48_160_adaptive_gain.json``
- ``data/logs/current_sense_fidelity_me1718_adaptive_gain.json``
- ``data/logs/current_sense_fidelity_motenergy_me1719_48v_adaptive_gain.json``

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

Research-Grade Fidelity Roadmap (Phase 1)
------------------------------------------

Phase 1 establishes deterministic regression checks before deeper physics and control upgrades.

**Objective**

- Freeze a reproducible KPI baseline over fixed scenarios
- Detect unintended behavior drift when changing motor/control models
- Keep changes measurable release-to-release

**Implemented Baseline Framework**

- `src.utils.regression_baseline`:
    - Defines fixed reference scenarios
    - Runs deterministic V/f simulations
    - Runs deterministic FOC simulations
    - Computes KPI set per scenario
    - Saves/loads baseline JSON
    - Compares current KPIs against frozen baseline tolerances
- `tests.test_regression_baseline`:
    - Executes the reference suite and checks drift against baseline
- `tests.test_regression_baseline_foc`:
    - Executes the FOC reference suite and checks drift against baseline
- `examples.generate_regression_baseline`:
    - Utility script to freeze or refresh baseline data
- `examples.generate_foc_regression_baseline`:
    - Utility script to freeze or refresh FOC baseline data
- `examples.report_regression_drift`:
    - Utility script to print KPI drift tables (full and failed rows)

**Reference Scenarios**

1. no_load_spinup
2. constant_load
3. ramp_load
4. supply_sag

**FOC Reference Scenarios**

1. foc_constant_load_clarke
2. foc_ramp_load_clarke
3. foc_supply_sag_concordia

**Current KPI Set**

- final_speed_rpm
- peak_speed_rpm
- peak_phase_current_a
- steady_state_speed_std_rpm
- torque_ripple_std_nm
- mean_supply_voltage_v
- mean_load_torque_nm

**Drift Diagnostics**

Baseline comparisons now include row-wise drift reporting with:

- absolute delta
- percentage delta
- tolerance percentage
- pass/fail/missing status

When regression tests fail, the failure output includes a compact drift table
to show exactly which KPI exceeded tolerance and by how much.

**CI Regression Gate**

GitHub Actions workflow:

- `.github/workflows/regression-gates.yml`

Behavior:

- Runs V/f and FOC baseline regression tests on pushes and pull requests
- Fails the CI job when KPI drift exceeds configured tolerance
- Always uploads an artifact bundle with:
    - `regression_drift_report.txt`
    - current frozen baseline JSON files

Branch protection setup and required status-check mapping are documented in:

- `docs/branch_protection.rst`

PR merge checklist template:

- `.github/PULL_REQUEST_TEMPLATE.md`

Contributor policy for baseline regeneration and required evidence:

- `CONTRIBUTING.md`

Release note template with explicit baseline-change section:

- `RELEASE_NOTES_TEMPLATE.md`

**How to Freeze Baseline**

::

        python examples/generate_regression_baseline.py

This writes baseline data to:

- `tests/baselines/reference_baseline.json`

To freeze FOC baseline:

::

    python examples/generate_foc_regression_baseline.py

This writes baseline data to:

- `tests/baselines/foc_reference_baseline.json`

**How to Run Regression Check**

::

        pytest tests/test_regression_baseline.py -v

Run both V/f and FOC regression checks:

::

    pytest tests/test_regression_baseline.py tests/test_regression_baseline_foc.py -v

Generate a human-readable drift report:

::

    python examples/report_regression_drift.py

**Important Notes**

- Keep `SIMULATION_PARAMS['dt']` fixed when comparing against the same baseline.
- Regenerate baseline only after intentional model/control updates.
- If baseline is regenerated, record the reason in commit history/release notes.

FOC Observer and Inverter Tuning Guide
--------------------------------------

Recent control updates add selectable angle observers and non-ideal inverter terms.
Use the guidance below as practical starting points before fine tuning.

Advanced inverter realism blocks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The inverter model now supports individually switchable realism blocks so the
simulation can be tuned from ideal-average behavior to a higher-fidelity
reduced-order bridge model.

Available feature toggles:

- Device drop
- Dead-time distortion
- Conduction loss
- Switching loss
- Freewheel diode path loss
- Minimum pulse suppression
- DC-link ripple
- Thermal coupling
- Phase asymmetry

Observer mode selection
^^^^^^^^^^^^^^^^^^^^^^^

- `Measured`: Uses model rotor angle directly. Best for reference simulations and debugging.
- `PLL`: Uses back-EMF angle tracking. Good first sensorless-like observer mode.
- `SMO`: Sliding-mode-inspired variant with robust error sign action and filtered speed estimate.

Suggested initial observer gains
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- PLL Kp: `50` to `150`
- PLL Ki: `1000` to `5000`
- SMO Kslide: `300` to `1000`
- SMO LPF Alpha: `0.05` to `0.2`
- SMO Boundary: `0.03` to `0.12` rad

Suggested inverter non-ideality starting points
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Device Drop: `0.2` to `1.0` V
- Dead-Time Loss: `0.005` to `0.03` pu
- Conduction Resistance: `0.005` to `0.05` ohm
- Switching Frequency: `8000` to `20000` Hz
- Switching Loss Coeff: `0.002` to `0.02` V/A/kHz
- Diode Drop: `0.2` to `1.0` V
- Minimum Pulse Fraction: `0.002` to `0.03` pu
- DC-Link Capacitance: `500e-6` to `5e-3` F
- DC-Link Source Resistance: `0.01` to `0.2` ohm
- DC-Link ESR: `0.001` to `0.05` ohm
- Thermal Resistance: `0.5` to `3.0` K/W
- Thermal Capacitance: `10` to `200` J/K
- Phase Mismatch Scales: `0.98` to `1.02`

Tuning workflow (recommended)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Start with `Measured` observer and all inverter non-idealities at zero.
- Tune d/q current loops and cascaded speed loop first.
- Switch to `PLL`, tune Kp then Ki to remove steady-state phase error.
- Switch to `SMO` if robust disturbance rejection is needed, then tune `Kslide` and `LPF Alpha`.
- Introduce inverter non-idealities one at a time: device drop, dead-time, then conduction resistance.
- Add switching and diode losses after the base current-loop tuning is stable.
- Enable DC-link ripple only after voltage-loop behavior is understood, because it couples supply and modulation limits.
- Enable thermal coupling for long-duration sweeps, not for first-pass controller tuning.
- Use phase asymmetry only when studying mismatch sensitivity or fault-like drift.
- Re-freeze baseline only after expected drift is verified and documented.

Telemetry exposed by the inverter model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The simulation history and inverter state now include:

- Effective DC-link voltage
- DC-link ripple magnitude
- DC-link bus current
- Device, conduction, switching, dead-time, and diode loss components
- Total inverter loss power
- Junction temperature
- Common-mode voltage
- Minimum pulse event count

Sensorless low-speed maturity enhancement
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The FOC observer path now uses confidence-and-speed weighted angle blending in
sensorless modes (`PLL`/`SMO`) to improve low-speed robustness:

- At low speed or low confidence, electrical angle remains closer to measured angle.
- As speed and confidence increase, control transitions smoothly toward full observer angle.
- This reduces abrupt observer phase behavior during weak back-EMF conditions.

Configuration entry point:

- `FOCController.set_sensorless_blend(enabled, min_speed_rpm, min_confidence)`

Runtime diagnostics exposed in controller state:

- `sensorless_blend_weight`
- `theta_sensorless_raw`

**Phase-2 Next Steps**

- Replace simplified torque paths with energy-consistent formulations
- Add adaptive handoff fallback tuning guidelines for low-speed recovery scenarios
- Add startup transition scenario diversity (load/supply/noise sweeps) for stress coverage
