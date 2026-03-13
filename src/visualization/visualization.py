"""
Visualization Module
====================

Handles plotting and visualization of simulation results.

Uses matplotlib for real-time and saved plots.

:author: BLDC Control Team
:version: 1.0.0
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from pathlib import Path
from typing import Dict, Optional
import logging

logger = logging.getLogger(__name__)


class SimulationPlotter:
    """
    Real-time and post-simulation plotting.

    Generates standard plots for:
    - Currents vs time
    - Voltages vs time
    - Speed vs time
    - Torque vs time
    - 3-phase relationships
    """

    @staticmethod
    def create_3phase_plot(
        history: Dict[str, np.ndarray],
        figsize: tuple = (12, 8),
        grid_on: bool = True,
        grid_spacing: Optional[float] = None,
        minor_grid: bool = False,
        grid_spacing_y: Optional[float] = None,
    ) -> Figure:
        """
        Create figure with 3-phase motor variables.

        :param history: History dictionary from SimulationEngine
        :type history: dict
        :param figsize: Figure size
        :type figsize: tuple
        :return: Matplotlib figure
        :rtype: Figure
        """
        from src.utils.config import PLOT_STYLE

        fig = plt.figure(figsize=figsize)
        time = history["time"]

        # Currents
        ax1 = fig.add_subplot(3, 2, 1)
        ax1.plot(
            time,
            history["currents_a"],
            label="Phase A",
            color=PLOT_STYLE["current_color"],
            linewidth=1.5,
        )
        ax1.plot(
            time, history["currents_b"], label="Phase B", linestyle="--", linewidth=1.5
        )
        ax1.plot(
            time, history["currents_c"], label="Phase C", linestyle=":", linewidth=1.5
        )
        ax1.set_xlabel("Time (s)", fontsize=10)
        ax1.set_ylabel("Current (A)", fontsize=10)
        ax1.set_title("3-Phase Currents", fontsize=11, fontweight="bold")
        ax1.grid(grid_on, alpha=0.3)
        if minor_grid:
            ax1.minorticks_on()
            ax1.grid(which="minor", alpha=0.1, linestyle=":")
        if grid_spacing and grid_spacing > 0:
            from matplotlib.ticker import MultipleLocator

            ax1.xaxis.set_major_locator(MultipleLocator(grid_spacing))
        if grid_spacing_y and grid_spacing_y > 0:
            from matplotlib.ticker import MultipleLocator

            ax1.yaxis.set_major_locator(MultipleLocator(grid_spacing_y))
        ax1.legend(loc="best")

        # Voltages
        ax2 = fig.add_subplot(3, 2, 2)
        ax2.plot(
            time,
            history["voltages_a"],
            label="Phase A",
            color=PLOT_STYLE["voltage_color"],
            linewidth=1.5,
        )
        ax2.plot(
            time, history["voltages_b"], label="Phase B", linestyle="--", linewidth=1.5
        )
        ax2.plot(
            time, history["voltages_c"], label="Phase C", linestyle=":", linewidth=1.5
        )
        ax2.set_xlabel("Time (s)", fontsize=10)
        ax2.set_ylabel("Voltage (V)", fontsize=10)
        ax2.set_title("3-Phase Voltages", fontsize=11, fontweight="bold")
        ax2.grid(grid_on, alpha=0.3)
        if minor_grid:
            ax2.minorticks_on()
            ax2.grid(which="minor", alpha=0.1, linestyle=":")
        if grid_spacing and grid_spacing > 0:
            from matplotlib.ticker import MultipleLocator

            ax2.xaxis.set_major_locator(MultipleLocator(grid_spacing))
        if grid_spacing_y and grid_spacing_y > 0:
            from matplotlib.ticker import MultipleLocator

            ax2.yaxis.set_major_locator(MultipleLocator(grid_spacing_y))
        ax2.legend(loc="best")

        # Back-EMF
        ax3 = fig.add_subplot(3, 2, 3)
        ax3.plot(time, history["emf_a"], label="Phase A", color="purple", linewidth=1.5)
        ax3.plot(time, history["emf_b"], label="Phase B", linestyle="--", linewidth=1.5)
        ax3.plot(time, history["emf_c"], label="Phase C", linestyle=":", linewidth=1.5)
        ax3.set_xlabel("Time (s)", fontsize=10)
        ax3.set_ylabel("Back-EMF (V)", fontsize=10)
        ax3.set_title("Back-EMF", fontsize=11, fontweight="bold")
        ax3.grid(grid_on, alpha=0.3)
        if minor_grid:
            ax3.minorticks_on()
            ax3.grid(which="minor", alpha=0.1, linestyle=":")
        if grid_spacing and grid_spacing > 0:
            from matplotlib.ticker import MultipleLocator

            ax3.xaxis.set_major_locator(MultipleLocator(grid_spacing))
        if grid_spacing_y and grid_spacing_y > 0:
            from matplotlib.ticker import MultipleLocator

            ax3.yaxis.set_major_locator(MultipleLocator(grid_spacing_y))
        ax3.legend(loc="best")

        # Speed
        ax4 = fig.add_subplot(3, 2, 4)
        ax4.plot(time, history["speed"], color=PLOT_STYLE["speed_color"], linewidth=2)
        ax4.set_xlabel("Time (s)", fontsize=10)
        ax4.set_ylabel("Speed (RPM)", fontsize=10)
        ax4.set_title("Rotor Speed", fontsize=11, fontweight="bold")
        ax4.grid(grid_on, alpha=0.3)
        if minor_grid:
            ax4.minorticks_on()
            ax4.grid(which="minor", alpha=0.1, linestyle=":")
        if grid_spacing and grid_spacing > 0:
            from matplotlib.ticker import MultipleLocator

            ax4.xaxis.set_major_locator(MultipleLocator(grid_spacing))
        if grid_spacing_y and grid_spacing_y > 0:
            from matplotlib.ticker import MultipleLocator

            ax4.yaxis.set_major_locator(MultipleLocator(grid_spacing_y))

        # Torque
        ax5 = fig.add_subplot(3, 2, 5)
        ax5.plot(
            time,
            history["torque"],
            label="Electromagnetic",
            color=PLOT_STYLE["torque_color"],
            linewidth=1.5,
        )
        ax5.plot(
            time, history["load_torque"], label="Load", linestyle="--", linewidth=1.5
        )
        ax5.set_xlabel("Time (s)", fontsize=10)
        ax5.set_ylabel("Torque (N*m)", fontsize=10)
        ax5.set_title("Torque Comparison", fontsize=11, fontweight="bold")
        ax5.grid(grid_on, alpha=0.3)
        if minor_grid:
            ax5.minorticks_on()
            ax5.grid(which="minor", alpha=0.1, linestyle=":")
        if grid_spacing and grid_spacing > 0:
            from matplotlib.ticker import MultipleLocator

            ax5.xaxis.set_major_locator(MultipleLocator(grid_spacing))
        if grid_spacing_y and grid_spacing_y > 0:
            from matplotlib.ticker import MultipleLocator

            ax5.yaxis.set_major_locator(MultipleLocator(grid_spacing_y))
        ax5.legend(loc="best")

        # Phase portrait (Phase A vs B currents)
        ax6 = fig.add_subplot(3, 2, 6)
        ax6.plot(
            history["currents_a"],
            history["currents_b"],
            color="green",
            linewidth=1,
            alpha=0.7,
        )
        ax6.set_xlabel("Current A (A)", fontsize=10)
        ax6.set_ylabel("Current B (A)", fontsize=10)
        ax6.set_title("Current Phase Portrait (A vs B)", fontsize=11, fontweight="bold")
        ax6.grid(grid_on, alpha=0.3)
        if minor_grid:
            ax6.minorticks_on()
            ax6.grid(which="minor", alpha=0.1, linestyle=":")
        if grid_spacing and grid_spacing > 0:
            from matplotlib.ticker import MultipleLocator

            ax6.xaxis.set_major_locator(MultipleLocator(grid_spacing))
        if grid_spacing_y and grid_spacing_y > 0:
            from matplotlib.ticker import MultipleLocator

            ax6.yaxis.set_major_locator(MultipleLocator(grid_spacing_y))

        plt.tight_layout()
        return fig

    @staticmethod
    def create_current_plot(
        history: Dict[str, np.ndarray],
        grid_on: bool = True,
        grid_spacing: Optional[float] = None,
        minor_grid: bool = False,
        grid_spacing_y: Optional[float] = None,
    ) -> Figure:
        """
        Create dedicated current analysis plot.

        :param history: History dictionary
        :type history: dict
        :param grid_on: Show grid
        :type grid_on: bool
        :param grid_spacing: X-axis spacing (s)
        :type grid_spacing: Optional[float]
        :param minor_grid: Show minor grid lines
        :type minor_grid: bool
        :param grid_spacing_y: Y-axis spacing (A), 0 or None for auto
        :type grid_spacing_y: Optional[float]
        :return: Matplotlib figure
        :rtype: Figure
        """
        from matplotlib.ticker import MultipleLocator

        fig = plt.figure(figsize=(10, 6))
        time = history["time"]

        ax = fig.add_subplot(1, 1, 1)
        ax.plot(time, history["currents_a"], label="Phase A", linewidth=2)
        ax.plot(
            time, history["currents_b"], label="Phase B", linestyle="--", linewidth=2
        )
        ax.plot(
            time, history["currents_c"], label="Phase C", linestyle=":", linewidth=2
        )

        ax.set_xlabel("Time (s)", fontsize=11, fontweight="bold")
        ax.set_ylabel("Current (A)", fontsize=11, fontweight="bold")
        ax.set_title("3-Phase Motor Currents", fontsize=12, fontweight="bold")
        ax.grid(grid_on, alpha=0.3)

        # Apply minor grid if requested
        if minor_grid:
            ax.minorticks_on()
            ax.grid(which="minor", alpha=0.1, linestyle=":")

        # Apply X-axis spacing
        if grid_spacing and grid_spacing > 0:
            ax.xaxis.set_major_locator(MultipleLocator(grid_spacing))

        # Apply Y-axis spacing
        if grid_spacing_y and grid_spacing_y > 0:
            ax.yaxis.set_major_locator(MultipleLocator(grid_spacing_y))

        ax.legend(loc="best", fontsize=10)

        plt.tight_layout()
        return fig

    @staticmethod
    def create_pfc_analysis_plot(
        history: Dict[str, np.ndarray],
        figsize: tuple = (11, 9),
        grid_on: bool = True,
        grid_spacing: Optional[float] = None,
        minor_grid: bool = False,
        grid_spacing_y: Optional[float] = None,
    ) -> Figure:
        """Create a dedicated PFC telemetry plot with PF, power, and command trends."""
        from matplotlib.ticker import MultipleLocator

        if "time" not in history:
            raise KeyError("history must include 'time'")

        time = np.asarray(history["time"], dtype=np.float64)
        if time.size == 0:
            raise ValueError("history contains no samples")

        pf = np.asarray(history.get("power_factor", np.zeros_like(time)), dtype=np.float64)
        active_power = np.asarray(
            history.get("input_power", np.zeros_like(time)), dtype=np.float64
        )
        pfc_command = np.asarray(
            history.get("pfc_command_var", np.zeros_like(time)), dtype=np.float64
        )

        pf_abs = np.clip(np.abs(pf), 1e-6, None)
        apparent_power = np.abs(active_power) / pf_abs
        reactive_power = np.sqrt(np.maximum(apparent_power**2 - active_power**2, 0.0))

        fig, axes = plt.subplots(4, 1, figsize=figsize, sharex=True)

        def _style_axis(ax):
            ax.grid(grid_on, alpha=0.3)
            if minor_grid:
                ax.minorticks_on()
                ax.grid(which="minor", alpha=0.1, linestyle=":")
            if grid_spacing and grid_spacing > 0:
                ax.xaxis.set_major_locator(MultipleLocator(grid_spacing))
            if grid_spacing_y and grid_spacing_y > 0:
                ax.yaxis.set_major_locator(MultipleLocator(grid_spacing_y))

        axes[0].plot(time, pf, color="#2E7D32", linewidth=1.8)
        axes[0].set_ylabel("PF")
        axes[0].set_title("Power Factor Trend", fontsize=11, fontweight="bold")
        axes[0].set_ylim(-1.05, 1.05)
        _style_axis(axes[0])

        axes[1].plot(time, active_power, color="#1565C0", linewidth=1.6)
        axes[1].set_ylabel("P (W)")
        axes[1].set_title("Input Active Power", fontsize=11, fontweight="bold")
        _style_axis(axes[1])

        axes[2].plot(time, reactive_power, color="#EF6C00", linewidth=1.6)
        axes[2].set_ylabel("Q (VAR)")
        axes[2].set_title("Estimated Reactive Power", fontsize=11, fontweight="bold")
        _style_axis(axes[2])

        axes[3].plot(time, pfc_command, color="#6A1B9A", linewidth=1.6)
        axes[3].set_ylabel("Cmd (VAR)")
        axes[3].set_xlabel("Time (s)")
        axes[3].set_title("PFC Compensation Command", fontsize=11, fontweight="bold")
        _style_axis(axes[3])

        plt.tight_layout()
        return fig

    @staticmethod
    def create_efficiency_analysis_plot(
        history: Dict[str, np.ndarray],
        figsize: tuple = (11, 9),
        grid_on: bool = True,
        grid_spacing: Optional[float] = None,
        minor_grid: bool = False,
        grid_spacing_y: Optional[float] = None,
    ) -> Figure:
        """Create a dedicated efficiency plot with input/output/loss power trends."""
        from matplotlib.ticker import MultipleLocator

        if "time" not in history:
            raise KeyError("history must include 'time'")

        time = np.asarray(history["time"], dtype=np.float64)
        if time.size == 0:
            raise ValueError("history contains no samples")

        input_power = np.asarray(
            history.get("input_power", np.zeros_like(time)), dtype=np.float64
        )
        mech_power = np.asarray(
            history.get("mechanical_output_power", np.zeros_like(time)),
            dtype=np.float64,
        )
        loss_power = np.asarray(
            history.get("total_loss_power", np.zeros_like(time)), dtype=np.float64
        )
        efficiency = np.asarray(
            history.get("efficiency", np.zeros_like(time)), dtype=np.float64
        )

        fig, axes = plt.subplots(4, 1, figsize=figsize, sharex=True)

        def _style_axis(ax):
            ax.grid(grid_on, alpha=0.3)
            if minor_grid:
                ax.minorticks_on()
                ax.grid(which="minor", alpha=0.1, linestyle=":")
            if grid_spacing and grid_spacing > 0:
                ax.xaxis.set_major_locator(MultipleLocator(grid_spacing))
            if grid_spacing_y and grid_spacing_y > 0:
                ax.yaxis.set_major_locator(MultipleLocator(grid_spacing_y))

        axes[0].plot(time, efficiency, color="#00695C", linewidth=1.8)
        axes[0].set_ylabel("eta")
        axes[0].set_title("System Efficiency", fontsize=11, fontweight="bold")
        axes[0].set_ylim(-0.05, 1.05)
        _style_axis(axes[0])

        axes[1].plot(time, input_power, color="#1565C0", linewidth=1.6)
        axes[1].set_ylabel("Pin (W)")
        axes[1].set_title("Electrical Input Power", fontsize=11, fontweight="bold")
        _style_axis(axes[1])

        axes[2].plot(time, mech_power, color="#2E7D32", linewidth=1.6)
        axes[2].set_ylabel("Pout (W)")
        axes[2].set_title("Mechanical Output Power", fontsize=11, fontweight="bold")
        _style_axis(axes[2])

        axes[3].plot(time, loss_power, color="#C62828", linewidth=1.6)
        axes[3].set_ylabel("Loss (W)")
        axes[3].set_xlabel("Time (s)")
        axes[3].set_title("Estimated Total Loss", fontsize=11, fontweight="bold")
        _style_axis(axes[3])

        plt.tight_layout()
        return fig

    @staticmethod
    def create_multi_axis_plot(
        history: Dict[str, np.ndarray],
        variables: list,
        figsize: tuple = (10, 6),
        grid_on: bool = True,
        grid_spacing: Optional[float] = None,
        minor_grid: bool = False,
        grid_spacing_y: Optional[float] = None,
    ) -> Figure:
        """
        Create a multi-axis plot for an arbitrary list of variables.

        Each variable will be plotted on its own y-axis to allow very different
        magnitudes to coexist (multi-scaling). Colors and legend are handled
        automatically.

        :param history: History dictionary from SimulationEngine
        :type history: dict
        :param variables: List of history keys to plot (must be in history)
        :type variables: list
        :param figsize: Figure size
        :type figsize: tuple
        :param grid_on: Show grid
        :type grid_on: bool
        :param grid_spacing: X-axis spacing (s)
        :type grid_spacing: Optional[float]
        :param minor_grid: Show minor grid lines
        :type minor_grid: bool
        :param grid_spacing_y: Y-axis spacing, 0 or None for auto
        :type grid_spacing_y: Optional[float]
        :return: Matplotlib figure
        :rtype: Figure
        """
        from matplotlib.ticker import MultipleLocator

        fig, ax_primary = plt.subplots(figsize=figsize)
        time = history["time"]

        axes = [ax_primary]
        colors = plt.cm.tab10.colors

        for idx, var in enumerate(variables):
            if var not in history:
                logger.warning(f"Variable '{var}' not in history, skipping.")
                continue
            if idx == 0:
                ax = ax_primary
            else:
                # create additional y-axis on the right
                ax = ax_primary.twinx()
                # offset spine to avoid overlap
                ax.spines["right"].set_position(("axes", 1 + 0.1 * (idx - 1)))
                axes.append(ax)

            ax.plot(
                time,
                history[var],
                label=var,
                color=colors[idx % len(colors)],
                linewidth=1.5,
            )
            ax.set_ylabel(var)
            ax.grid(grid_on, alpha=0.2)

            # Apply minor grid if requested
            if minor_grid:
                ax.minorticks_on()
                ax.grid(which="minor", alpha=0.1, linestyle=":")

            # Apply X-axis spacing
            if grid_spacing and grid_spacing > 0:
                ax.xaxis.set_major_locator(MultipleLocator(grid_spacing))

            # Apply Y-axis spacing
            if grid_spacing_y and grid_spacing_y > 0:
                ax.yaxis.set_major_locator(MultipleLocator(grid_spacing_y))

        axes[0].set_xlabel("Time (s)")
        # combine legends
        handles = []
        labels = []
        for ax in axes:
            h, lab = ax.get_legend_handles_labels()
            handles.extend(h)
            labels.extend(lab)
        if handles:
            axes[0].legend(handles, labels, loc="best")
        plt.tight_layout()
        return fig

    @staticmethod
    def save_plot(fig: Figure, filepath: Path, dpi: int = 100) -> None:
        """
        Save figure to file.

        :param fig: Matplotlib figure
        :type fig: Figure
        :param filepath: Output path
        :type filepath: Path
        :param dpi: Resolution DPI
        :type dpi: int
        """
        filepath.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(filepath, dpi=dpi, bbox_inches="tight")
        logger.info(f"Saved plot to {filepath}")
