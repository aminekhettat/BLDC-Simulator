Features
========

Core Simulation Features
------------------------

- **3-Phase Motor Model**: Complete mathematical model of BLDC motor dynamics
- **Accurate EMF Calculation**: Based on space vector PWM and motor parameters
- **Torque Computation**: Accounting for load dynamics and motor characteristics

Control Algorithms
------------------

1. **Field-Oriented Control (FOC)**
   - Decoupled torque and flux control
   - Park and Clarke transformations
   - PI compensators for current control
   - Superior performance and efficiency

2. **Voltage-Frequency Control (V/F)**
   - Scalar control strategy
   - Simple and robust operation
   - Suitable for basic applications

3. **Space Vector Modulation (SVM)**
   - PWM generation for 3-phase inverter
   - Near-sinusoidal current output
   - Optimized voltage utilization

Advanced Features
-----------------

- **Monte Carlo Simulations**: Statistical analysis of motor performance across parameter variations
- **Real-time Monitoring**: Live display of motor parameters, currents, voltages, and power
- **Data Logging**: CSV export with metadata for offline analysis
- **Power Analysis**: Loss calculations including copper, iron, and mechanical losses

User Interface
--------------

- **Accessible PyQt6 GUI**
  - Screen reader support (NVDA, JAWS compatible)
  - Keyboard-only navigation
  - High contrast display support

- **Interactive Controls**
  - Real-time parameter adjustment
  - Simulation speed control
  - Grid display with customizable spacing
  - Plot export functionality

- **Monitoring Dashboard**
  - Status bar with simulation parameters
  - Real-time waveform plots
  - Phase current and voltage visualization
  - Torque and power monitoring

Data & Export
-------------

- **CSV Export**: Save simulation results with metadata
- **Custom Export Paths**: Choose custom directories for data storage
- **JSON Metadata**: Simulation parameters saved with results
- **Full Simulation History**: All time-series data available for analysis

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
