"""Tests for visualization utilities."""

import numpy as np

from src.visualization.visualization import SimulationPlotter


def make_dummy_history():
    t = np.linspace(0, 1, 100)
    return {
        "time": t,
        "currents_a": np.sin(2 * np.pi * t),
        "voltages_a": np.cos(2 * np.pi * t),
        "speed": t * 100,
        "power_factor": np.clip(0.75 + 0.2 * np.sin(2 * np.pi * t), -1.0, 1.0),
        "input_power": 400.0 + 40.0 * np.sin(2 * np.pi * t),
        "pfc_command_var": 100.0 + 20.0 * np.cos(2 * np.pi * t),
    }


def test_create_multi_axis_plot():
    history = make_dummy_history()
    fig = SimulationPlotter.create_multi_axis_plot(
        history, ["currents_a", "voltages_a", "speed"]
    )
    # simple sanity: figure should have axes
    assert fig.get_axes()


def test_create_pfc_analysis_plot():
    history = make_dummy_history()
    fig = SimulationPlotter.create_pfc_analysis_plot(history)
    axes = fig.get_axes()

    assert len(axes) == 4
    assert "Power Factor" in axes[0].get_title()
    assert "Reactive" in axes[2].get_title()
