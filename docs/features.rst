Features
========

Core Simulation Features
------------------------

- **3-Phase Motor Model**: Complete mathematical model of BLDC motor dynamics
- **Profile-Driven EMF Shapes**: Supports sinusoidal dq/PMSM-style profiles and trapezoidal BLDC waveforms
- **Accurate EMF Calculation**: Based on space vector PWM and motor parameters
- **Torque Computation**: Accounting for load dynamics and motor characteristics

Control Algorithms
------------------

1. **Field-Oriented Control (FOC)**
   - Decoupled torque and flux control
   - Park and Clarke transformations
   - PI compensators for current control
   - Sensored and sensorless operation with five selectable angle observers:
     Measured (reference), PLL, SMO, STSMO (Super-Twisting), and ActiveFlux
   - Superior performance and efficiency

2. **Voltage-Frequency Control (V/F)**
   - Scalar control strategy
   - Simple and robust operation
   - Suitable for basic applications

3. **Space Vector Modulation (SVM)**
   - PWM generation for 3-phase inverter
   - Near-sinusoidal current output
   - Optimized voltage utilization

Sensorless Angle Observers
--------------------------

FOC sensorless operation is supported through five selectable observer modes.

- **Measured** — True simulation angle; reference mode for controller tuning.
- **PLL** — Phase-Locked Loop on the reconstructed back-EMF vector; simple Kp/Ki
  gain structure; suited to surface-PM motors at medium and high speed.
- **SMO** — First-order Sliding-Mode Observer with boundary-layer softening and
  LPF smoothing; better disturbance rejection than PLL.
- **STSMO** — Super-Twisting SMO (second order):

  - Backward-Euler implicit integration — unconditionally stable at any time-step
    or gain value.
  - Speed-adaptive k2 satisfying the Levant rotating-EMF tracking condition
    ``k2 ≥ ke·ωm·ωe`` at all operating points.
  - SOGI post-filter at the electrical frequency to suppress chattering.
  - Analytical gain calibration from rated speed and back-EMF constant.
  - Optional MRAS resistance drift correction for thermal robustness.

- **ActiveFlux** — Tracks the active flux vector ``ψa = ψs − Ld·is`` (Boldea 2009);
  angle extraction is saliency-independent, making it naturally suited to IPM
  motors and field-weakening operation.

All sensorless modes participate in a confidence-weighted blend with the open-loop
angle for smooth low-speed handoff.  Detailed documentation: ``docs/sensorless_observers.rst``.

Advanced Features
-----------------

- **Monte Carlo Simulations**: Statistical analysis of motor performance across parameter variations
- **Real-time Monitoring**: Live display of motor parameters, currents, voltages, and power
- **Current Measurement Realism**: Triple-, double-, and single-shunt inverter sensing with topology-aware reconstruction and side-by-side true versus measured current history
- **Controller Feedback Selection**: FOC can run from true motor currents or reconstructed shunt currents to mirror real embedded feedback paths
- **Current Spectrum Analyzer**: Stacked FFT magnitude/phase views with per-axis linear/log scaling, optional dB amplitude, selectable phase units, grid toggles, and CSV/image export
- **Data Logging**: CSV export with metadata for offline analysis
- **Power Analysis**: Loss calculations including copper, iron, and mechanical losses
- **Power Factor Analysis**: Waveform-based active/reactive/apparent power and PF metrics with reactive compensation sizing helpers
- **PFC Telemetry Hook**: Optional closed-loop PF target tracking in simulation metadata/history (`pfc_command_var`, PF trend)
- **Efficiency Telemetry**: Input power, mechanical output power, estimated total loss, and system efficiency tracked in simulation metadata/history
- **Efficiency Tuning Suggestions**: Heuristic recommendations for switching-frequency and inverter-loss tradeoff tuning from measured efficiency/PF
- **Loaded No-FW Calibration**: Staged loaded-point search that separates speed feasibility, orthogonality, and conditioned efficiency acceptance
- **Standard Startup Sequencing**: Configurable alignment and open-loop startup flows for both FOC and V/f controllers
- **Inverter Realism Blocks**: Independently switchable drop/loss/ripple/thermal/asymmetry effects with runtime telemetry
- **Inverter Analysis Plot**: Dedicated multi-panel plotting for bus ripple, loss breakdown, junction temperature, and common-mode voltage
- **Hardware Backend Abstraction**: Communication backend interface with mock DAQ support for dry-run hardware integration
- **Control Timing Telemetry**: Control-cycle calculation duration and CPU-load metrics, including MCU load estimation helpers
- **Compute Backend Policy**: Runtime backend selection (`auto`, `cpu`, `gpu`) with safe fallback and CuPy acceleration for power metrics when available

User Interface
--------------

- **Accessible PySide6 GUI** (LGPL, enhanced accessibility event propagation)
  - Screen reader support (NVDA, JAWS, Orca compatible)
  - Keyboard-only navigation
  - High contrast display support
  - Spoken status updates and narrated plot/export workflow hooks
  - All interactive widgets carry accessible names and descriptions

- **Interactive Controls**
  - Real-time parameter adjustment
  - Simulation speed control
  - Grid display with customizable spacing
  - Plot export functionality
  - Current-sense topology selection and live inverter bridge visualization
  - FFT plot controls for amplitude/phase units and independent axis scaling

- **Monitoring Dashboard**
  - Status bar with simulation parameters
  - RK4 dt/PWM numerical-stability advisory with severity color coding
  - Real-time multi-channel oscilloscope (``OscilloscopeWidget``) with up to
    4 independently selectable signal strips; zoom/pan via pyqtgraph or
    matplotlib NavigationToolbar fallback; scrolling time-window selector
    (1 s / 5 s / 10 s / 30 s / All); Pause/Resume; Reset-Axes; ghost-run
    comparison overlay (previous run shown as translucent dashed trace)
  - Phase current and voltage visualization
  - Torque and power monitoring
  - True-versus-measured current comparison for current-sense validation

- **Post-Simulation Plot Customizer** (``PlotCustomizerDialog``)
  - Opens from "Customize …" buttons next to every post-sim plot button
  - Six tabs: Figure (title, size, DPI), Typography (font sizes), Traces
    (per-trace color-picker, line width, line style), Grid, Legend, Export
  - Publication-quality export presets: Screen, IEEE 2-col, IEEE 1-col,
    A4 Report, Presentation 16:9, High-DPI Print (600 dpi)
  - Live preview canvas inside the dialog; "Apply to original" propagates
    changes back to the figure shown in the separate matplotlib window
  - PNG / PDF / SVG / EPS save with configurable DPI via file dialog
  - ``PlotStyle`` dataclass is fully serializable (``to_dict()``) and
    injectable into any ``SimulationPlotter.create_*()`` call

Data & Export
-------------

- **CSV Export**: Save simulation results with metadata
- **Custom Export Paths**: Choose custom directories for data storage
- **JSON Metadata**: Simulation parameters saved with results
- **Full Simulation History**: All time-series data available for analysis
- **FFT CSV Export**: Save frequency, magnitude, and phase data using the active display units
- **FFT Image Export**: Capture the stacked magnitude/phase view as a shareable figure

Optimization
------------

- **Numba JIT Compilation**: Optional performance boost for computationally intensive calculations
- **Fallback Fallback Decorators**: Graceful degradation if Numba unavailable
- **Efficient Algorithms**: Optimized matrix operations and transformations

Extensibility
-------------

- **Modular Architecture**: Easy to add new control strategies
- **Plugin-Style Load Models**: Support for different load characteristics
- **Custom Parameter Sets**: Pre-defined motor configurations
- **Event-Driven Simulation**: Clear separation between control and visualization
