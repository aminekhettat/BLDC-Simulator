"""
Atomic features tested in this module:
- create 3phase plot and current plot variants
- multi axis plot skips missing variable with warning
- pfc efficiency inverter raise on missing or empty time
- save plot writes file
- base controller abstract method bodies
- ui conf setup adds css file
"""

import logging
from pathlib import Path
from typing import cast

import matplotlib.pyplot as plt
import numpy as np
import pytest

from src.control.base_controller import BaseController
from src.ui import conf as ui_conf
from src.visualization.visualization import SimulationPlotter


def _full_history(n: int = 50) -> dict[str, np.ndarray]:
    t = np.linspace(0.0, 1.0, n)
    return {
        "time": t,
        "currents_a": np.sin(2 * np.pi * t),
        "currents_b": np.sin(2 * np.pi * t - 2.0 * np.pi / 3.0),
        "currents_c": np.sin(2 * np.pi * t + 2.0 * np.pi / 3.0),
        "voltages_a": 24.0 * np.cos(2 * np.pi * t),
        "voltages_b": 24.0 * np.cos(2 * np.pi * t - 2.0 * np.pi / 3.0),
        "voltages_c": 24.0 * np.cos(2 * np.pi * t + 2.0 * np.pi / 3.0),
        "emf_a": 4.0 * np.sin(2 * np.pi * t),
        "emf_b": 4.0 * np.sin(2 * np.pi * t - 2.0 * np.pi / 3.0),
        "emf_c": 4.0 * np.sin(2 * np.pi * t + 2.0 * np.pi / 3.0),
        "speed": 1000.0 * t,
        "torque": 2.0 + 0.2 * np.sin(2 * np.pi * t),
        "load_torque": 1.5 + 0.1 * np.cos(2 * np.pi * t),
        "power_factor": np.clip(0.85 + 0.05 * np.sin(2 * np.pi * t), -1.0, 1.0),
        "input_power": 500.0 + 20.0 * np.sin(2 * np.pi * t),
        "pfc_command_var": 80.0 + 5.0 * np.cos(2 * np.pi * t),
        "mechanical_output_power": 400.0 + 20.0 * np.sin(2 * np.pi * t),
        "total_loss_power": 100.0 + 5.0 * np.cos(2 * np.pi * t),
        "efficiency": np.clip(0.8 + 0.03 * np.sin(2 * np.pi * t), 0.0, 1.0),
        "effective_dc_voltage": 47.0 + 0.5 * np.sin(2 * np.pi * t),
        "dc_link_ripple_v": 0.8 + 0.1 * np.cos(2 * np.pi * t),
        "dc_link_bus_current_a": 10.0 + 1.0 * np.sin(2 * np.pi * t),
        "inverter_total_loss_power": 30.0 + 3.0 * np.sin(2 * np.pi * t),
        "device_loss_power": 8.0 + 0.5 * np.sin(2 * np.pi * t),
        "conduction_loss_power": 10.0 + 1.0 * np.cos(2 * np.pi * t),
        "switching_loss_power": 6.0 + 0.4 * np.sin(2 * np.pi * t),
        "dead_time_loss_power": 3.0 + 0.3 * np.cos(2 * np.pi * t),
        "diode_loss_power": 2.0 + 0.2 * np.sin(2 * np.pi * t),
        "inverter_junction_temp_c": 40.0 + 2.0 * np.sin(2 * np.pi * t),
        "common_mode_voltage": 0.5 * np.sin(2 * np.pi * t),
    }


def test_create_3phase_plot_and_current_plot_variants():
    h = _full_history()
    fig1 = SimulationPlotter.create_3phase_plot(
        h,
        grid_on=True,
        grid_spacing=0.1,
        minor_grid=True,
        grid_spacing_y=0.5,
    )
    fig2 = SimulationPlotter.create_current_plot(
        h,
        grid_on=True,
        grid_spacing=0.2,
        minor_grid=True,
        grid_spacing_y=0.2,
    )
    assert len(fig1.get_axes()) == 6
    assert len(fig2.get_axes()) == 1
    plt.close(fig1)
    plt.close(fig2)


def test_multi_axis_plot_skips_missing_variable_with_warning(caplog):
    h = _full_history()
    with caplog.at_level(logging.WARNING):
        fig = SimulationPlotter.create_multi_axis_plot(
            h,
            ["currents_a", "not_in_history", "speed"],
            grid_on=True,
            grid_spacing=0.1,
            minor_grid=True,
            grid_spacing_y=1.0,
        )
    assert "not in history, skipping" in caplog.text
    assert fig.get_axes()
    plt.close(fig)


def test_pfc_efficiency_inverter_raise_on_missing_or_empty_time():
    with pytest.raises(KeyError):
        SimulationPlotter.create_pfc_analysis_plot({})
    with pytest.raises(KeyError):
        SimulationPlotter.create_efficiency_analysis_plot({})
    with pytest.raises(KeyError):
        SimulationPlotter.create_inverter_analysis_plot({})

    empty = {"time": np.array([], dtype=np.float64)}
    with pytest.raises(ValueError):
        SimulationPlotter.create_pfc_analysis_plot(empty)
    with pytest.raises(ValueError):
        SimulationPlotter.create_efficiency_analysis_plot(empty)
    with pytest.raises(ValueError):
        SimulationPlotter.create_inverter_analysis_plot(empty)


def test_save_plot_writes_file(tmp_path: Path):
    h = _full_history()
    fig = SimulationPlotter.create_current_plot(h)
    out = tmp_path / "plots" / "currents.png"
    SimulationPlotter.save_plot(fig, out, dpi=120)
    assert out.exists()
    assert out.stat().st_size > 0
    plt.close(fig)


def test_base_controller_abstract_method_bodies():
    dummy = cast(BaseController, object())
    assert BaseController.update(dummy, 0.01) is None
    assert BaseController.reset(dummy) is None
    assert BaseController.get_state(dummy) is None


def test_ui_conf_setup_adds_css_file():
    called: list[str] = []

    class DummyApp:
        def add_css_file(self, name: str) -> None:
            called.append(name)

    ui_conf.setup(DummyApp())
    assert called == ["custom.css"]
    assert ui_conf.project == "SPINOTOR"
