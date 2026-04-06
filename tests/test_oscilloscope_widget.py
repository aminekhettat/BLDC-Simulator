"""
Tests for OscilloscopeWidget and its channel buffer logic.

Atomic features tested
----------------------
- _ChannelBuffer: push, get_window, get_ghost, snapshot_as_ghost, clear, last_time/value
- OscilloscopeWidget: push_sample, set_paused, start_new_run, clear_data, set_available_keys
  (Qt-dependent tests require PySide6 and a QApplication; skipped when absent)
"""

from __future__ import annotations

import importlib.util
import math
import sys

import pytest

# ── Load oscilloscope_widget directly (bypass PySide6 __init__.py) ────────────
# The widget uses PySide6 but we want to test the pure-Python buffer
# logic even in headless environments.  PySide6 is only needed for the
# UI class, which is guarded by ``pytest.importorskip`` in the Qt section.

_WIDGET_PATH = (
    __import__("pathlib").Path(__file__).parent.parent
    / "src" / "ui" / "widgets" / "oscilloscope_widget.py"
)

_osc_mod_key = "_spinotor_oscilloscope_widget"
if _osc_mod_key not in sys.modules:
    _spec = importlib.util.spec_from_file_location(_osc_mod_key, _WIDGET_PATH)
    _omod = importlib.util.module_from_spec(_spec)
    sys.modules[_osc_mod_key] = _omod
    try:
        _spec.loader.exec_module(_omod)
    except Exception as _e:
        # If PySide6 is unavailable at module level, the pure-Python classes
        # are still accessible up to the point of the Qt import.  Any errors
        # will surface naturally in the Qt tests (where importorskip guards them).
        pass

_ChannelBuffer = sys.modules[_osc_mod_key]._ChannelBuffer
CHANNEL_REGISTRY = sys.modules[_osc_mod_key].CHANNEL_REGISTRY
_TIME_WINDOWS = sys.modules[_osc_mod_key]._TIME_WINDOWS


class TestChannelBuffer:
    def test_push_and_get_all(self):
        buf = _ChannelBuffer(maxlen=1000)
        for i in range(10):
            buf.push(float(i) * 0.1, float(i) * 2.0)
        ts, vs = buf.get_window(0)  # 0 = all data
        assert len(ts) == 10
        assert len(vs) == 10
        assert ts[0] == pytest.approx(0.0)
        assert vs[-1] == pytest.approx(18.0)

    def test_get_window_trims_old_data(self):
        buf = _ChannelBuffer(maxlen=1000)
        # push 0..19 seconds, 1 sample/s
        for i in range(20):
            buf.push(float(i), float(i))
        ts, vs = buf.get_window(5.0)   # last 5 s → samples 15..19
        assert len(ts) == 5
        assert ts[0] == pytest.approx(15.0)
        assert ts[-1] == pytest.approx(19.0)

    def test_get_window_empty_buffer(self):
        buf = _ChannelBuffer()
        ts, vs = buf.get_window(5.0)
        assert ts == []
        assert vs == []

    def test_snapshot_as_ghost_and_retrieve(self):
        buf = _ChannelBuffer()
        buf.push(0.0, 1.0)
        buf.push(0.1, 2.0)
        buf.snapshot_as_ghost()
        buf.push(0.2, 3.0)           # push after ghost — ghost should NOT change
        ghost_t, ghost_v = buf.get_ghost()
        assert len(ghost_t) == 2
        assert ghost_v[1] == pytest.approx(2.0)

    def test_clear_empties_live_data_not_ghost(self):
        buf = _ChannelBuffer()
        buf.push(0.0, 5.0)
        buf.snapshot_as_ghost()
        buf.push(0.1, 6.0)
        buf.clear()
        ts, vs = buf.get_window(0)
        assert ts == []
        ghost_t, ghost_v = buf.get_ghost()
        assert len(ghost_t) == 1       # ghost preserved

    def test_last_time_and_last_value(self):
        buf = _ChannelBuffer()
        assert buf.last_time() == 0.0
        assert math.isnan(buf.last_value())
        buf.push(1.5, 42.0)
        assert buf.last_time() == pytest.approx(1.5)
        assert buf.last_value() == pytest.approx(42.0)

    def test_maxlen_ring_eviction(self):
        buf = _ChannelBuffer(maxlen=5)
        for i in range(10):
            buf.push(float(i), float(i))
        ts, vs = buf.get_window(0)
        assert len(ts) == 5
        assert ts[0] == pytest.approx(5.0)   # oldest 5 evicted

    def test_snapshot_ghost_initially_empty(self):
        buf = _ChannelBuffer()
        gt, gv = buf.get_ghost()
        assert gt == []
        assert gv == []


# ── CHANNEL_REGISTRY sanity ────────────────────────────────────────────────────

class TestChannelRegistry:
    def test_speed_rpm_present(self):
        assert "speed_rpm" in CHANNEL_REGISTRY

    def test_all_entries_have_label_and_unit(self):
        for key, (label, unit) in CHANNEL_REGISTRY.items():
            assert isinstance(key, str) and key
            assert isinstance(label, str) and label
            assert isinstance(unit, str)           # unit may be empty string

    def test_time_windows_sorted_ascending(self):
        seconds = [s for _, s in _TIME_WINDOWS if s > 0]
        assert seconds == sorted(seconds)


# ── Qt-dependent tests ────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def qapp():
    """Return (or create) a QApplication for the module."""
    PySide6 = pytest.importorskip("PySide6.QtWidgets")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    return app


class TestOscilloscopeWidget:
    def test_instantiation(self, qapp):
        from src.ui.widgets.oscilloscope_widget import OscilloscopeWidget
        w = OscilloscopeWidget()
        assert w is not None

    def test_push_sample_does_not_raise(self, qapp):
        from src.ui.widgets.oscilloscope_widget import OscilloscopeWidget
        w = OscilloscopeWidget()
        w.push_sample(0.0, {"speed_rpm": 1000.0, "torque": 1.5})
        w.push_sample(0.001, {"speed_rpm": 1010.0, "torque": 1.6})

    def test_set_paused_toggles(self, qapp):
        from src.ui.widgets.oscilloscope_widget import OscilloscopeWidget
        w = OscilloscopeWidget()
        w.set_paused(True)
        assert w._paused is True
        w.set_paused(False)
        assert w._paused is False

    def test_start_new_run_clears_buffers(self, qapp):
        from src.ui.widgets.oscilloscope_widget import OscilloscopeWidget
        w = OscilloscopeWidget()
        # Push some data
        for i in range(5):
            w.push_sample(float(i) * 0.1, {"speed_rpm": float(i * 100)})
        # start_new_run ghosts and clears
        w.start_new_run()
        # All live buffers should now be empty
        for buf in w._buffers.values():
            ts, vs = buf.get_window(0)
            assert ts == []

    def test_clear_data_removes_ghost_and_live(self, qapp):
        from src.ui.widgets.oscilloscope_widget import OscilloscopeWidget
        w = OscilloscopeWidget()
        for i in range(3):
            w.push_sample(float(i) * 0.1, {"speed_rpm": float(i * 100)})
        w.start_new_run()   # snapshot to ghost
        w.clear_data()
        for buf in w._buffers.values():
            gt, gv = buf.get_ghost()
            assert gt == []
            ts, vs = buf.get_window(0)
            assert ts == []

    def test_set_available_keys_updates_combo(self, qapp):
        from src.ui.widgets.oscilloscope_widget import OscilloscopeWidget
        w = OscilloscopeWidget()
        keys = ["speed_rpm", "torque", "currents_a"]
        w.set_available_keys(keys)
        # No exception should be raised; just verify the call completes
