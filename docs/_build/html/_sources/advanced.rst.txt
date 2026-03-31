Advanced Topics
================

.. contents:: Table of Contents
   :local:
   :depth: 2

Multi-Rate Control Loop Execution
----------------------------------

Real-world single-shunt FOC implementations on microcontrollers (e.g. STM32,
TMS320, dsPIC) use two distinct interrupt rates:

.. list-table:: Real MCU execution rates (Fpwm = 20 kHz)
   :header-rows: 1
   :widths: 40 20 20 20

   * - Task
     - Trigger
     - Period
     - Frequency
   * - Current reconstruction (single-shunt)
     - PWM interrupt
     - 50 µs
     - 20 kHz
   * - Clarke / Park transform
     - PWM interrupt
     - 50 µs
     - 20 kHz
   * - Rotor angle observer (SMO / PLL)
     - PWM interrupt
     - 50 µs
     - 20 kHz
   * - d/q current PI regulators
     - PWM interrupt
     - 50 µs
     - 20 kHz
   * - Speed PI regulator
     - Slow-loop flag (every N periods)
     - ~10 ms
     - ~100 Hz

Running the speed PI at the same rate as the current PI would make its
integrator gains unrealistically large and its transient response unrealistically
fast.  The simulator replicates this two-rate structure with the
``speed_loop_divider`` parameter on ``FOCController``.

**Configuration**::

    ctrl = FOCController(motor, enable_speed_loop=True)
    # Fpwm = 20 kHz  →  dt = 50 µs  →  divider = round(10 ms / 50 µs) = 200
    ctrl.set_speed_loop_divider(200)

The ``FOCController`` then:

* Runs Clarke/Park + observer + d/q current PI **every** ``update()`` call.
* Increments an internal counter and runs the speed PI only every
  ``speed_loop_divider`` calls, passing an effective ``dt_speed = N × dt``
  to the integrator.
* Holds the last Iq reference as a zero-order hold (ZOH) between speed-PI
  firings, matching the real interrupt-driven behaviour.

**Automatic setting in FW calibrator**

``FieldWeakeningCalibrator`` sets the divider automatically::

    _speed_divider = max(1, min(500, round(10e-3 / dt)))
    ctrl.set_speed_loop_divider(_speed_divider)

This keeps the speed-PI effective period at ≈ 10 ms regardless of the
simulation time-step chosen by the user.

**Backward compatibility**

``speed_loop_divider = 1`` (the default) restores the original behaviour where
both loops run at the same rate.  This is useful for unit-test comparisons
against pre-existing baselines.

Field-Weakening Root Cause Analysis and Fixes
----------------------------------------------

Prior to the March 2026 patch, all motor profiles were stuck at approximately
2720 RPM even though their rated speeds are 4000–5000 RPM.  Four root-cause
bugs were identified and resolved.

Bug 1 — FW headroom computed from unsaturated PI voltage (Critical)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Location:** ``FOCController.update()`` and
``_update_field_weakening_from_voltage()``

**Root cause:** The FW headroom ``h = V_lim − |v_unsat|`` was computed
*before* the d_priority saturation block clipped the PI outputs.  During
acceleration, the current PI output can reach 50–200 V; the formula then
gives ``h = 27.71 − 200 = −172 V``, a permanently negative headroom that
drives the FW integrator to its maximum at every step.

**Fix:** Saturation is applied *first*, then the saturated voltages
``(v_d_sat, v_q_sat)`` are passed to ``_update_field_weakening_from_voltage()``.
By construction ``|v_sat| ≤ V_lim``, so ``h ≥ 0`` at steady state.

*Reference: Kim & Sul, IEEE Trans. Ind. Appl. 33(2), 1997.*

Bug 2 — Decoupling feedforward using measured currents (High)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Location:** ``FOCController.update()`` — feedforward block

**Root cause:** The cross-coupling terms were ``v_d_ff = −ωe·Lq·Iq_measured``.
During braking transients, measured Iq can spike to −400 A, pushing
``v_d_ff`` to +28 V, which exceeds ``V_lim = 27.71 V``.  With d_priority
saturation, ``v_d = +27.71 V`` and ``v_q_sat = 0 V`` (zero torque), causing
the motor to decelerate freely through 0 RPM and reverse.

**Fix:** Feedforward uses **reference** currents ``id_ref_command`` /
``iq_ref_command``, which are always bounded by ``iq_limit_a`` and
``fw_id_max``.

*Reference: Holtz, IEEE Proc. 90(8), 2002; Briz et al., IEEE TIE 47(4), 2000.*

Bug 3 — Speed loop anti-windup gain kaw = 0.05 (Medium)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Location:** ``FieldWeakeningCalibrator._run_fw_simulation()``

**Root cause:** With ``kaw = 0.05`` the back-calculation drain rate was
≈ 86 A/s.  An integral accumulated to 2000 A during acceleration took
≈ 23 s to drain — longer than the 10 s simulation window.

**Fix:** ``kaw = 1.0`` → drain rate ≈ 2000 A/s → clears in ≈ 1 s.

*Reference: Åström & Hägglund, PID Controllers, ISA Press, 1995.*

Bug 4 — Step speed reference causing severe overshoot (Medium)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Location:** ``FieldWeakeningCalibrator._run_fw_simulation()``

**Root cause:** Speed reference was set to the full target RPM at t = 0.
The motor overshot rated speed by ≥ 30 %, triggering the reversal mechanism
of Bug 2.

**Fix:** Linear ramp from 0 to ``target_rpm`` over
``t_ramp = clip(0.4 × T_sim, 2 s, 6 s)``, keeping the speed error small
throughout acceleration.

*Reference: Morimoto et al., IEEE Trans. Ind. Appl. 30(4), 1994.*

**Post-fix results**

.. list-table:: Calibration results after all four fixes
   :header-rows: 1
   :widths: 30 20 20 30

   * - Motor
     - Rated RPM
     - Achieved RPM
     - Status
   * - MotEnergy ME1718 48 V
     - 4000
     - 4034
     - 3 / 4 PASS ✓
   * - MotEnergy ME1719 48 V
     - 4000
     - 3998
     - 3 / 4 PASS ✓
   * - Innotec 255-EZS48-160
     - 3000
     - 3000
     - 3 / 4 PASS ✓

The 60 % rated-load failure at rated FW speed is physically expected:
above base speed the motor operates in constant-power mode and maximum
available torque is limited by ``fw_id_max`` and the current circle.

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

FOC feedback path integration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- The FOC loop can now consume either the true motor phase currents or the reconstructed shunt currents.
- This makes it possible to compare ideal control performance against controller behavior that is limited by measurement topology.
- The simulation engine preserves both histories so plots and regression tests can compare controller-facing currents against the underlying physics state.

Spectrum analysis workflow
^^^^^^^^^^^^^^^^^^^^^^^^^^

- The current spectrum window renders stacked magnitude and phase plots for the selected phase current.
- Magnitude can be displayed in linear units or dB; phase can be shown in degrees or radians.
- Each FFT subplot supports independent linear/log axis scaling and grid toggles.
- Export actions preserve the active units in both CSV headers and saved figures.

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

**RK4 Stability Advisory (dt/PWM only)**

The simulator now computes and displays an RK4 stability advisory in the quick-info
panel and status bar. This advisory is based on the linear stability region of
explicit RK4 and recommends only simulation-setting actions:

- Decrease ``dt``
- Increase PWM/switching frequency (``f_pwm = 1/dt``)

Motor physical parameters are treated as fixed real-world constants for this guidance.

**Criteria used**

- RK4 linear stability bound: ``|lambda * dt| <= 2.785``
- Electrical mode: ``lambda_e = R/L``  ->  ``dt_max_e = 2.785 * (L/R)``
- Mechanical mode: ``lambda_m = b/J`` ->  ``dt_max_m = 2.785 * (J/b)``
- Effective limit: ``dt_max = min(dt_max_e, dt_max_m)``
- Recommended conservative target: ``dt_recommended = dt_max / 5``
- Recommended minimum PWM: ``f_pwm_recommended >= 1 / dt_recommended``

**Severity levels**

- ``unstable``: ``dt > dt_max``
- ``marginal``: ``0.5 * dt_max < dt <= dt_max``
- ``stable``: ``dt <= 0.5 * dt_max``
- ``unknown``: missing positive ``R/L`` or ``J/b`` pair

The GUI color coding is:

- ``stable`` -> green
- ``marginal`` -> orange
- ``unstable`` -> red
- ``unknown`` -> neutral gray

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
