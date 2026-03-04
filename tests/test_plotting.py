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
    }


def test_create_multi_axis_plot():
    history = make_dummy_history()
    fig = SimulationPlotter.create_multi_axis_plot(
        history, ["currents_a", "voltages_a", "speed"]
    )
    # simple sanity: figure should have axes
    assert fig.get_axes()
