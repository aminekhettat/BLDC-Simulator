"""
Atomic features tested in this module:
- ld lq controls and simulation params info are available
"""

import sys

from PyQt6.QtWidgets import QApplication

from src.ui.main_window import BLDCMotorControlGUI

app = QApplication.instance() or QApplication(sys.argv)


def test_ld_lq_controls_and_simulation_params_info_are_available():
    win = BLDCMotorControlGUI()

    assert hasattr(win, "param_ld")
    assert hasattr(win, "param_lq")

    info = win.get_simulation_params_info()
    assert "dt" in info or "Simulation time step" in info
    assert "Recommended action: adjust dt or PWM frequency" in info


def test_rk4_advisory_accessibility_text_is_exposed(monkeypatch):
    win = BLDCMotorControlGUI()

    # Build one monitoring update and verify advisory text is surfaced for AT.
    win._update_monitoring({"speed_rpm": 0.0, "time": 0.0})

    assert "RK4 stability advisory" in win.lbl_stability.text()
    assert win.lbl_stability.toolTip() == win.lbl_stability.text()
    assert win.status_bar_stability.toolTip() == win.status_bar_stability.text()
    assert "RK4 stability status" in win.status_bar_stability.accessibleName()


def test_rk4_advisory_voice_announces_on_severity_change(monkeypatch):
    win = BLDCMotorControlGUI()
    spoken: list[str] = []

    monkeypatch.setattr("src.ui.main_window.is_audio_assistance_enabled", lambda: True)
    monkeypatch.setattr("src.ui.main_window.speak", lambda msg, **_kwargs: spoken.append(msg))

    win._last_stability_severity_announced = None
    win._announce_stability_advisory_if_needed(
        severity="unstable",
        dt_recommended_s=5.0e-5,
        pwm_recommended_min_hz=20000.0,
        margin=0.8,
    )
    assert any("unstable" in s.lower() for s in spoken)

    # Same severity should not re-announce.
    count_after_first = len(spoken)
    win._announce_stability_advisory_if_needed(
        severity="unstable",
        dt_recommended_s=5.0e-5,
        pwm_recommended_min_hz=20000.0,
        margin=0.8,
    )
    assert len(spoken) == count_after_first

    # Transition to stable should announce once.
    win._announce_stability_advisory_if_needed(
        severity="stable",
        dt_recommended_s=5.0e-5,
        pwm_recommended_min_hz=20000.0,
        margin=3.2,
    )
    assert any("healthy" in s.lower() for s in spoken)
