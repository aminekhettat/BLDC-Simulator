===========================
BLDC Motor Control
===========================

Welcome to the BLDC Motor Control documentation!

Version ``0.10.0`` adds a complete sensorless observer suite (PLL, SMO, Super-Twisting SMO with
backward-Euler integration and SOGI post-filter, and Active Flux), a restructured 5-tab GUI with
context-sensitive observer parameter visibility, and a full migration from PyQt6 to PySide6 for
improved accessibility event propagation.  Earlier releases consolidated topology-aware shunt
current reconstruction, controller-selectable measured-vs-true FOC current feedback, an expanded
FFT analyzer, and hardened CI quality gates (Ruff, Mypy, Bandit, pip-audit).

This project provides a comprehensive simulation and control framework for three-phase Brushless DC (BLDC) motors with advanced features including:

- **Field-Oriented Control (FOC)** and **Voltage-Frequency (V/F)** control modes
- **Five sensorless angle observers**: Measured, PLL, SMO, STSMO (Super-Twisting), ActiveFlux
- **Space Vector Modulation (SVM)** for PWM generation
- **Monte Carlo simulations** for performance analysis
- **Accessible PySide6 GUI** with screen reader support (NVDA, JAWS, Orca)
- **Real-time parameter monitoring** and data logging
- **Topology-aware current measurement realism** for triple-, double-, and single-shunt inverter sensing
- **Current spectrum analysis** with FFT magnitude/phase plots and CSV/image export

Table of Contents
=================

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   quickstart
   features

.. toctree::
   :maxdepth: 2
   :caption: Core Modules

   api/motor_model
   api/control
   api/visualization

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/core
   api/ui
   api/utils

.. toctree::
   :maxdepth: 2
   :caption: Advanced Topics

   architecture
   advanced
   sensorless_observers
   visualization
   branch_protection

Indices and Tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
