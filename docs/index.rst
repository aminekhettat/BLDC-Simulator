===========================
BLDC Motor Control
===========================

Welcome to the BLDC Motor Control documentation!

Version ``0.8.1`` consolidates the March 2026 control and measurement work: topology-aware shunt current reconstruction,
controller-selectable measured-vs-true FOC current feedback, and an expanded FFT analyzer with stacked magnitude/phase
plots, configurable units, axis scaling, and export tools. It also adds hardened code-quality gates in local hooks and CI
(Ruff, Mypy, Bandit, and dependency vulnerability auditing).

This project provides a comprehensive simulation and control framework for three-phase Brushless DC (BLDC) motors with advanced features including:

- **Field-Oriented Control (FOC)** and **Voltage-Frequency (V/F)** control modes
- **Space Vector Modulation (SVM)** for PWM generation
- **Monte Carlo simulations** for performance analysis
- **Accessible PyQt6 GUI** with screen reader support
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
   branch_protection

Indices and Tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
