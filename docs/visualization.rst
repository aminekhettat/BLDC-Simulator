Visualization System
====================

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

SPINOTOR provides two complementary visualization systems:

1. **Real-time oscilloscope** (``OscilloscopeWidget``) — live monitoring during
   simulation; powered by **pyqtgraph** when available and falls back to a
   matplotlib toolbar canvas otherwise.

2. **Post-simulation plot customizer** (``PlotCustomizerDialog`` /
   ``PlotStyleApplicator``) — style editor for every matplotlib figure produced
   by ``SimulationPlotter``; supports publication-quality export presets.


Real-Time Oscilloscope — OscilloscopeWidget
-------------------------------------------

Located in the **Analysis → Monitoring** tab, the oscilloscope replaces the
fixed speed-curve panel with a dynamic multi-channel display.

Features
^^^^^^^^

- **Up to 4 configurable channel strips**, each driven by an independent
  ``QComboBox`` channel selector.
- **20+ selectable signals** (all entries in ``CHANNEL_REGISTRY``): rotor
  speed, phase currents, phase voltages, back-EMFs, torque, d/q currents,
  observer confidence, DC-link voltage/ripple, efficiency, and more.
- **Scrolling time-window selector**: 1 s / 5 s / 10 s / 30 s / All data.
- **Pause / Resume** without stopping the simulation.
- **Reset-Axes** button: auto-scale Y to the visible data range.
- **Grid toggle** (single checkbox synchronised across all strips).
- **Vertical crosshair** spanning all strips; a readout label shows the
  instantaneous values of all active channels at the hovered time.
- **Ghost-run overlay**: when a new simulation starts, the previous run's
  traces are preserved as translucent dashed lines for before/after comparison.
- **Graceful backend selection**: pyqtgraph (preferred — native Qt, hardware
  anti-aliased, fast) → matplotlib NavigationToolbar2QT (fallback when
  ``pyqtgraph`` is not installed).

Usage
^^^^^

The oscilloscope is wired automatically.  No manual action is needed:

- Every simulation sample is pushed via ``OscilloscopeWidget.push_sample()``.
- Calling ``start_new_run()`` at simulation start ghosts the previous traces
  and clears the live buffers.
- ``set_paused(True/False)`` freezes the live update loop without discarding
  buffered data.

Headless / programmatic access
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The pure-Python ring buffer and channel registry do **not** require a running
``QApplication``:

.. code-block:: python

   from src.ui.widgets.oscilloscope_widget import _ChannelBuffer, CHANNEL_REGISTRY

   buf = _ChannelBuffer(maxlen=50_000)
   buf.push(0.001, 1000.0)          # (time_s, value)
   ts, vs = buf.get_window(5.0)     # last 5 s
   buf.snapshot_as_ghost()          # freeze for overlay
   buf.clear()                      # wipe live data

Install optional pyqtgraph backend
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   pip install pyqtgraph>=0.13.0

The application starts normally without it; a log message indicates the
matplotlib fallback is active.


Post-Simulation Plot Customizer
--------------------------------

After running a simulation, each plot button in the **Analysis → Plotting**
tab has a paired **Customize …** button.  Clicking it opens
``PlotCustomizerDialog`` with the last generated figure pre-loaded.

Dialog tabs
^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Tab
     - Controls
   * - **Figure**
     - Overall title (``suptitle``), figure width × height (inches), DPI
   * - **Typography**
     - Font sizes for subplot titles, axis labels, tick labels, and legends
   * - **Traces**
     - Per-trace color picker, global line width, per-trace line style
   * - **Grid**
     - Major / minor grid toggle, grid alpha slider
   * - **Legend**
     - Show / hide, position (``best``, ``upper right``, etc.), font size
   * - **Export**
     - One-click export preset selector; explicit Save-to-file button

Export presets
^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 20 10 45

   * - Preset name
     - Figure size
     - DPI
     - Intended use
   * - Screen (default)
     - 10 × 6 in
     - 96
     - On-screen display
   * - IEEE 2-column
     - 3.5 × 2.6 in
     - 300
     - Single column of an IEEE double-column paper
   * - IEEE 1-column (full width)
     - 7.16 × 4 in
     - 300
     - Full-width IEEE double-column figure
   * - A4 Report
     - 6.3 × 4 in
     - 150
     - Embedded in an A4 Word / PDF report
   * - Presentation 16:9
     - 10 × 5.6 in
     - 120
     - Slide deck (large text, thick lines)
   * - High-DPI Print (600 dpi)
     - 6.3 × 4 in
     - 600
     - High-resolution print-ready export

PlotStyle dataclass — programmatic API
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``PlotStyle`` is a plain dataclass (no Qt dependency) that can be constructed
and passed directly to any ``SimulationPlotter.create_*()`` method:

.. code-block:: python

   from src.ui.widgets.plot_customizer_dialog import PlotStyle, EXPORT_PRESETS
   from src.visualization.visualization import SimulationPlotter

   # From a preset
   ieee_style = PlotStyle.from_preset(
       next(p for p in EXPORT_PRESETS if "IEEE 2" in p.name)
   )

   # Custom style
   custom = PlotStyle(
       figsize=(7.0, 4.5),
       dpi=300,
       title_fontsize=10,
       axis_fontsize=9,
       tick_fontsize=8,
       linewidth=1.2,
       trace_colors={"Phase A": "#1565C0", "Phase B": "#2E7D32"},
       grid_major=True,
       grid_minor=False,
   )

   history = engine.get_history()
   fig = SimulationPlotter.create_current_plot(history, style=custom)
   SimulationPlotter.save_plot(fig, Path("output/currents.pdf"), dpi=300)

PlotStyleApplicator — standalone application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``PlotStyleApplicator`` is a stateless, Qt-free class that can be applied to
any existing matplotlib Figure after it has been created:

.. code-block:: python

   from src.ui.widgets.plot_customizer_dialog import PlotStyle, PlotStyleApplicator

   # Apply to a figure created any way you like
   PlotStyleApplicator.apply(fig, PlotStyle(linewidth=2.0, title_fontsize=14))

   # Save with a specific DPI (overrides style.dpi if provided)
   PlotStyleApplicator.save(fig, Path("figure.pdf"), dpi=300)


SimulationPlotter — available plot methods
-------------------------------------------

All methods accept an optional ``style: PlotStyle | None = None`` keyword argument.

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Method
     - Description
   * - ``create_3phase_plot(history)``
     - 6-panel: phase currents, voltages, back-EMF, speed, torque, current portrait
   * - ``create_current_plot(history)``
     - Single-panel 3-phase current comparison
   * - ``create_pfc_analysis_plot(history)``
     - 4-panel: PF trend, active / reactive power, PFC compensation command
   * - ``create_efficiency_analysis_plot(history)``
     - 4-panel: system efficiency, input / output power, estimated total loss
   * - ``create_inverter_analysis_plot(history)``
     - 5-panel: DC-link voltage & ripple, bus current, loss breakdown, junction temp, CMV
   * - ``create_multi_axis_plot(history, variables)``
     - N-panel with independent Y-axes for any list of history keys
   * - ``create_measured_vs_true_current_plot(history)``
     - Phase-wise overlay with rolling RMS error panel (requires ``*_true`` keys)
