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
        "mechanical_output_power": 300.0 + 25.0 * np.sin(2 * np.pi * t),
        "total_loss_power": 100.0 + 15.0 * np.cos(2 * np.pi * t),
        "efficiency": np.clip(0.75 + 0.05 * np.sin(2 * np.pi * t), 0.0, 1.0),
        "effective_dc_voltage": 47.0 + 0.6 * np.sin(2 * np.pi * t),
        "dc_link_ripple_v": 1.0 + 0.2 * np.cos(2 * np.pi * t),
        "dc_link_bus_current_a": 12.0 + 1.5 * np.sin(2 * np.pi * t),
        "device_loss_power": 8.0 + 1.0 * np.sin(2 * np.pi * t),
        "conduction_loss_power": 12.0 + 2.0 * np.cos(2 * np.pi * t),
        "switching_loss_power": 6.0 + 0.8 * np.sin(2 * np.pi * t),
        "dead_time_loss_power": 3.0 + 0.4 * np.cos(2 * np.pi * t),
        "diode_loss_power": 2.0 + 0.3 * np.sin(2 * np.pi * t),
        "inverter_total_loss_power": 31.0 + 2.5 * np.sin(2 * np.pi * t),
        "inverter_junction_temp_c": 35.0 + 4.0 * np.sin(2 * np.pi * t),
        "common_mode_voltage": 0.5 * np.sin(2 * np.pi * t),
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


def test_create_efficiency_analysis_plot():
    history = make_dummy_history()
    fig = SimulationPlotter.create_efficiency_analysis_plot(history)
    axes = fig.get_axes()

    assert len(axes) == 4
    assert "Efficiency" in axes[0].get_title()
    assert "Loss" in axes[3].get_title()


def test_create_inverter_analysis_plot():
    history = make_dummy_history()
    fig = SimulationPlotter.create_inverter_analysis_plot(history)
    axes = fig.get_axes()

    assert len(axes) == 5
    assert "DC-Link" in axes[0].get_title()
    assert "Loss Breakdown" in axes[2].get_title()
