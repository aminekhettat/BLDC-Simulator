"""
Regression tests for SimulationPlotter with PlotStyle integration.

Atomic features tested
----------------------
- All seven SimulationPlotter.create_* methods still return a Figure
- Backward-compatibility: calling without *style* parameter still works
- Passing a PlotStyle applies it and does not corrupt the figure
- save_plot writes a file
- _apply_style is a no-op when style=None
"""

from __future__ import annotations

import importlib.util
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest

from src.visualization.visualization import SimulationPlotter, _apply_style

# Load plot_customizer_dialog directly to avoid PySide6 dependency from
# src/ui/widgets/__init__.py in headless environments.
_DIALOG_PATH = (
    Path(__file__).parent.parent
    / "src" / "ui" / "widgets" / "plot_customizer_dialog.py"
)
_spec = importlib.util.spec_from_file_location("plot_customizer_dialog", _DIALOG_PATH)
_mod = importlib.util.module_from_spec(_spec)
sys.modules.setdefault("plot_customizer_dialog", _mod)
_spec.loader.exec_module(_mod)

PlotStyle = _mod.PlotStyle
EXPORT_PRESETS = _mod.EXPORT_PRESETS


# ── Shared fixture ─────────────────────────────────────────────────────────────

@pytest.fixture
def history():
    """Full-featured history dict that satisfies all SimulationPlotter methods."""
    t = np.linspace(0, 1, 200)
    zeros = np.zeros_like(t)
    sine = np.sin(2 * np.pi * t)
    return {
        "time": t,
        "currents_a": sine,
        "currents_b": np.sin(2 * np.pi * t - 2 * np.pi / 3),
        "currents_c": np.sin(2 * np.pi * t - 4 * np.pi / 3),
        "currents_a_true": sine * 1.01,
        "currents_b_true": np.sin(2 * np.pi * t - 2 * np.pi / 3) * 1.01,
        "currents_c_true": np.sin(2 * np.pi * t - 4 * np.pi / 3) * 1.01,
        "voltages_a": np.cos(2 * np.pi * t) * 24,
        "voltages_b": np.cos(2 * np.pi * t - 2 * np.pi / 3) * 24,
        "voltages_c": np.cos(2 * np.pi * t - 4 * np.pi / 3) * 24,
        "emf_a": sine * 12,
        "emf_b": np.sin(2 * np.pi * t - 2 * np.pi / 3) * 12,
        "emf_c": np.sin(2 * np.pi * t - 4 * np.pi / 3) * 12,
        "speed": t * 3500,
        "torque": 1.0 + 0.2 * sine,
        "load_torque": 0.8 + 0.1 * sine,
        # PFC
        "power_factor": np.clip(0.8 + 0.1 * sine, -1.0, 1.0),
        "input_power": 400 + 40 * sine,
        "pfc_command_var": 100 + 20 * np.cos(2 * np.pi * t),
        # Efficiency
        "mechanical_output_power": 300 + 25 * sine,
        "total_loss_power": 100 + 10 * sine,
        "efficiency": np.clip(0.75 + 0.05 * sine, 0.0, 1.0),
        # Inverter
        "effective_dc_voltage": 47 + 0.5 * sine,
        "dc_link_ripple_v": 0.5 + 0.1 * sine,
        "dc_link_bus_current_a": 10 + 1.5 * sine,
        "device_loss_power": 5 + 0.5 * sine,
        "conduction_loss_power": 8 + sine,
        "switching_loss_power": 4 + 0.5 * sine,
        "dead_time_loss_power": 2 + 0.2 * sine,
        "diode_loss_power": 1 + 0.1 * sine,
        "inverter_total_loss_power": 20 + 2 * sine,
        "inverter_junction_temp_c": 40 + 5 * sine,
        "common_mode_voltage": 0.3 * sine,
    }


@pytest.fixture
def default_style():
    return PlotStyle()


@pytest.fixture
def ieee_style():
    ieee = next((p for p in EXPORT_PRESETS if "IEEE" in p.name), EXPORT_PRESETS[0])
    return PlotStyle.from_preset(ieee)


# ── Backward compatibility (no style parameter) ───────────────────────────────

class TestBackwardCompatibility:
    def test_3phase_no_style(self, history):
        fig = SimulationPlotter.create_3phase_plot(history)
        assert fig.get_axes()
        plt.close(fig)

    def test_currents_no_style(self, history):
        fig = SimulationPlotter.create_current_plot(history)
        assert fig.get_axes()
        plt.close(fig)

    def test_pfc_no_style(self, history):
        fig = SimulationPlotter.create_pfc_analysis_plot(history)
        assert len(fig.get_axes()) == 4
        plt.close(fig)

    def test_efficiency_no_style(self, history):
        fig = SimulationPlotter.create_efficiency_analysis_plot(history)
        assert len(fig.get_axes()) == 4
        plt.close(fig)

    def test_inverter_no_style(self, history):
        fig = SimulationPlotter.create_inverter_analysis_plot(history)
        assert len(fig.get_axes()) == 5
        plt.close(fig)

    def test_multi_axis_no_style(self, history):
        fig = SimulationPlotter.create_multi_axis_plot(history, ["currents_a", "speed"])
        assert fig.get_axes()
        plt.close(fig)

    def test_measured_vs_true_no_style(self, history):
        fig = SimulationPlotter.create_measured_vs_true_current_plot(history)
        assert fig.get_axes()
        plt.close(fig)


# ── With PlotStyle (default) ──────────────────────────────────────────────────

class TestWithDefaultStyle:
    def test_3phase_with_style(self, history, default_style):
        fig = SimulationPlotter.create_3phase_plot(history, style=default_style)
        assert fig.get_axes()
        plt.close(fig)

    def test_currents_with_style(self, history, default_style):
        fig = SimulationPlotter.create_current_plot(history, style=default_style)
        assert fig.get_axes()
        plt.close(fig)

    def test_pfc_with_style(self, history, default_style):
        fig = SimulationPlotter.create_pfc_analysis_plot(history, style=default_style)
        assert len(fig.get_axes()) == 4
        plt.close(fig)

    def test_efficiency_with_style(self, history, default_style):
        fig = SimulationPlotter.create_efficiency_analysis_plot(history, style=default_style)
        assert len(fig.get_axes()) == 4
        plt.close(fig)

    def test_inverter_with_style(self, history, default_style):
        fig = SimulationPlotter.create_inverter_analysis_plot(history, style=default_style)
        assert len(fig.get_axes()) == 5
        plt.close(fig)

    def test_multi_axis_with_style(self, history, default_style):
        fig = SimulationPlotter.create_multi_axis_plot(
            history, ["currents_a", "speed", "torque"], style=default_style
        )
        assert fig.get_axes()
        plt.close(fig)

    def test_measured_vs_true_with_style(self, history, default_style):
        fig = SimulationPlotter.create_measured_vs_true_current_plot(
            history, style=default_style
        )
        assert fig.get_axes()
        plt.close(fig)


# ── With IEEE PlotStyle ────────────────────────────────────────────────────────

class TestWithIEEEStyle:
    def test_currents_ieee_style_applies_linewidth(self, history, ieee_style):
        fig = SimulationPlotter.create_current_plot(history, style=ieee_style)
        ax = fig.get_axes()[0]
        for line in ax.get_lines():
            assert line.get_linewidth() == pytest.approx(ieee_style.linewidth)
        plt.close(fig)

    def test_efficiency_ieee_style_title_fontsize(self, history, ieee_style):
        fig = SimulationPlotter.create_efficiency_analysis_plot(history, style=ieee_style)
        ax = fig.get_axes()[0]
        assert ax.title.get_fontsize() == pytest.approx(ieee_style.title_fontsize)
        plt.close(fig)


# ── save_plot ─────────────────────────────────────────────────────────────────

class TestSavePlot:
    def test_save_creates_png(self, history):
        fig = SimulationPlotter.create_current_plot(history)
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir) / "currents.png"
            SimulationPlotter.save_plot(fig, out, dpi=96)
            assert out.exists()
            assert out.stat().st_size > 0
        plt.close(fig)

    def test_save_creates_nested_directory(self, history):
        fig = SimulationPlotter.create_current_plot(history)
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir) / "a" / "b" / "speed.png"
            SimulationPlotter.save_plot(fig, out, dpi=72)
            assert out.exists()
        plt.close(fig)


# ── _apply_style edge cases ───────────────────────────────────────────────────

class TestApplyStyleEdgeCases:
    def test_noop_when_none(self):
        fig, ax = plt.subplots()
        ax.plot([0, 1], [0, 1])
        _apply_style(fig, None)     # must not raise
        plt.close(fig)

    def test_figure_with_no_axes(self):
        """Applying style to an empty figure must not raise."""
        fig = plt.figure()
        style = PlotStyle()
        _apply_style(fig, style)
        plt.close(fig)

    def test_figure_with_multiple_subplots(self, history):
        fig = SimulationPlotter.create_3phase_plot(history)
        style = PlotStyle(linewidth=2.5, tick_fontsize=8)
        _apply_style(fig, style)
        for ax in fig.get_axes():
            for line in ax.get_lines():
                assert line.get_linewidth() == pytest.approx(2.5)
        plt.close(fig)

    def test_suptitle_added_only_when_nonempty(self):
        fig, ax = plt.subplots()
        ax.plot([0], [0])
        _apply_style(fig, PlotStyle(suptitle=""))   # empty → no suptitle change
        _apply_style(fig, PlotStyle(suptitle="Hello"))
        assert fig._suptitle is not None
        plt.close(fig)

    def test_missing_history_key_raises_key_error(self):
        with pytest.raises(KeyError):
            SimulationPlotter.create_pfc_analysis_plot({"bad_key": np.zeros(5)})
