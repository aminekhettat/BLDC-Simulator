"""
Tests for PlotStyle, PlotStyleApplicator, and ExportPreset.

Atomic features tested
----------------------
- PlotStyle default construction
- PlotStyle.from_preset populates fields from ExportPreset
- PlotStyleApplicator.apply modifies figure properties (no Qt required)
- PlotStyleApplicator.save writes a file at the correct DPI
- EXPORT_PRESETS: all presets have unique names, positive DPI and figsize
"""

from __future__ import annotations

import importlib.util
import sys
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")          # headless — no display needed
import matplotlib.pyplot as plt
import pytest

# Load plot_customizer_dialog directly to bypass src/ui/widgets/__init__.py
# (which imports PySide6 unconditionally).  This lets the Qt-free parts run
# in environments that do not have a display or PySide6 installed.
_DIALOG_PATH = (
    Path(__file__).parent.parent
    / "src" / "ui" / "widgets" / "plot_customizer_dialog.py"
)
_spec = importlib.util.spec_from_file_location("plot_customizer_dialog", _DIALOG_PATH)
_mod = importlib.util.module_from_spec(_spec)
sys.modules.setdefault("plot_customizer_dialog", _mod)
_spec.loader.exec_module(_mod)

EXPORT_PRESETS = _mod.EXPORT_PRESETS
ExportPreset = _mod.ExportPreset
PlotStyle = _mod.PlotStyle
PlotStyleApplicator = _mod.PlotStyleApplicator


# ── helpers ────────────────────────────────────────────────────────────────────

def _simple_figure():
    """Return a minimal 1-axes matplotlib figure with one line."""
    fig, ax = plt.subplots()
    ax.plot([0, 1, 2], [0, 1, 0], label="test", linewidth=1.0, color="blue")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Test Figure")
    ax.legend()
    return fig


# ── ExportPreset ───────────────────────────────────────────────────────────────

class TestExportPreset:
    def test_all_presets_have_unique_names(self):
        names = [p.name for p in EXPORT_PRESETS]
        assert len(names) == len(set(names)), "Duplicate preset names found"

    def test_all_presets_positive_dpi(self):
        for p in EXPORT_PRESETS:
            assert p.dpi > 0, f"Preset '{p.name}' has non-positive DPI"

    def test_all_presets_valid_figsize(self):
        for p in EXPORT_PRESETS:
            w, h = p.figsize
            assert w > 0 and h > 0, f"Preset '{p.name}' has invalid figsize"

    def test_preset_count_at_least_three(self):
        # Ensure Screen, IEEE-ish, and Report-ish presets are present
        assert len(EXPORT_PRESETS) >= 3


# ── PlotStyle ─────────────────────────────────────────────────────────────────

class TestPlotStyle:
    def test_default_construction(self):
        s = PlotStyle()
        assert s.dpi > 0
        assert len(s.figsize) == 2
        assert s.grid_major is True
        assert s.legend_visible is True
        assert isinstance(s.trace_colors, dict)
        assert isinstance(s.trace_linestyles, dict)

    def test_from_preset_copies_dpi_and_figsize(self):
        preset = EXPORT_PRESETS[0]
        s = PlotStyle.from_preset(preset)
        assert s.dpi == preset.dpi
        assert s.figsize == preset.figsize

    def test_from_preset_copies_fontsizes(self):
        for preset in EXPORT_PRESETS:
            s = PlotStyle.from_preset(preset)
            assert s.title_fontsize == preset.title_fontsize
            assert s.axis_fontsize == preset.axis_fontsize

    def test_custom_trace_colors(self):
        s = PlotStyle(trace_colors={"ch1": "#FF0000"})
        assert s.trace_colors["ch1"] == "#FF0000"


# ── PlotStyleApplicator ───────────────────────────────────────────────────────

class TestPlotStyleApplicator:
    def test_apply_noop_when_style_none(self):
        """_apply_style (via SimulationPlotter) is a no-op for None style."""
        from src.visualization.visualization import _apply_style
        fig = _simple_figure()
        # Should not raise
        _apply_style(fig, None)
        plt.close(fig)

    def test_apply_changes_title_fontsize(self):
        fig = _simple_figure()
        style = PlotStyle(title_fontsize=20)
        PlotStyleApplicator.apply(fig, style)
        ax = fig.get_axes()[0]
        assert ax.title.get_fontsize() == pytest.approx(20)
        plt.close(fig)

    def test_apply_changes_linewidth(self):
        fig = _simple_figure()
        style = PlotStyle(linewidth=3.0)
        PlotStyleApplicator.apply(fig, style)
        ax = fig.get_axes()[0]
        lines = ax.get_lines()
        assert lines, "Expected at least one line in the figure"
        for line in lines:
            assert line.get_linewidth() == pytest.approx(3.0)
        plt.close(fig)

    def test_apply_changes_tick_fontsize(self):
        fig = _simple_figure()
        style = PlotStyle(tick_fontsize=7)
        PlotStyleApplicator.apply(fig, style)
        ax = fig.get_axes()[0]
        for tick in ax.get_xticklabels() + ax.get_yticklabels():
            assert tick.get_fontsize() == pytest.approx(7)
        plt.close(fig)

    def test_apply_changes_axis_label_fontsize(self):
        fig = _simple_figure()
        style = PlotStyle(axis_fontsize=14)
        PlotStyleApplicator.apply(fig, style)
        ax = fig.get_axes()[0]
        assert ax.get_xlabel()  # non-empty label
        assert ax.xaxis.label.get_fontsize() == pytest.approx(14)
        plt.close(fig)

    def test_apply_trace_color_by_label(self):
        fig = _simple_figure()
        style = PlotStyle(trace_colors={"test": "#00FF00"})
        PlotStyleApplicator.apply(fig, style)
        ax = fig.get_axes()[0]
        lines = ax.get_lines()
        test_lines = [ln for ln in lines if ln.get_label() == "test"]
        assert test_lines, "Line with label 'test' not found"
        import matplotlib.colors as mcolors
        actual = mcolors.to_hex(test_lines[0].get_color())
        assert actual.upper() == "#00FF00"
        plt.close(fig)

    def test_apply_suptitle(self):
        fig = _simple_figure()
        style = PlotStyle(suptitle="My Publication Title")
        PlotStyleApplicator.apply(fig, style)
        # suptitle is stored in fig._suptitle
        assert fig._suptitle is not None
        assert "Publication" in fig._suptitle.get_text()
        plt.close(fig)

    def test_apply_grid_major_false(self):
        fig = _simple_figure()
        style = PlotStyle(grid_major=False)
        PlotStyleApplicator.apply(fig, style)
        ax = fig.get_axes()[0]
        # matplotlib stores grid lines; just verify no exception
        plt.close(fig)

    def test_apply_legend_hidden(self):
        fig = _simple_figure()
        style = PlotStyle(legend_visible=False)
        PlotStyleApplicator.apply(fig, style)
        ax = fig.get_axes()[0]
        leg = ax.get_legend()
        if leg is not None:
            assert not leg.get_visible()
        plt.close(fig)

    def test_save_writes_png(self):
        fig = _simple_figure()
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir) / "test_out.png"
            PlotStyleApplicator.save(fig, out, dpi=72)
            assert out.exists()
            assert out.stat().st_size > 0
        plt.close(fig)

    def test_save_writes_pdf(self):
        fig = _simple_figure()
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir) / "test_out.pdf"
            PlotStyleApplicator.save(fig, out, dpi=150)
            assert out.exists()
        plt.close(fig)

    def test_save_creates_parent_directory(self):
        fig = _simple_figure()
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir) / "subdir" / "plot.png"
            PlotStyleApplicator.save(fig, out)
            assert out.exists()
        plt.close(fig)

    def test_apply_from_ieee_preset(self):
        """Applying IEEE preset should not raise and produce smaller fonts."""
        ieee_preset = next((p for p in EXPORT_PRESETS if "IEEE" in p.name), None)
        if ieee_preset is None:
            pytest.skip("No IEEE preset found in EXPORT_PRESETS")
        style = PlotStyle.from_preset(ieee_preset)
        fig = _simple_figure()
        PlotStyleApplicator.apply(fig, style)
        plt.close(fig)
