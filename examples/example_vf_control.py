"""
Example Script - BLDC Motor Standalone Simulation
==================================================

This script demonstrates how to use the BLDC motor simulation engine
independently of the GUI for programmatic control and data analysis.

Usage:
    python examples/example_vf_control.py

This example shows:
1. Motor parameter configuration
2. Load profile setup
3. V/f controller operation
4. SVM modulation
5. Simulation execution
6. Data export and plotting
"""

from pathlib import Path

import numpy as np

from src.control import SVMGenerator, VFController
from src.core import BLDCMotor, MotorParameters, RampLoad, SimulationEngine
from src.utils.config import (
    DEFAULT_MOTOR_PARAMS,
    SIMULATION_PARAMS,
    VF_CONTROLLER_PARAMS,
)
from src.utils.data_logger import DataLogger
from src.visualization.visualization import SimulationPlotter


def main():
    """Run example BLDC motor simulation with V/f control."""

    print("=" * 60)
    print("BLDC Motor Simulation - V/f Control Example")
    print("=" * 60)

    # ===== STEP 1: Configure Motor =====
    print("\n[1] Configuring Motor...")
    motor_params = MotorParameters(
        nominal_voltage=DEFAULT_MOTOR_PARAMS["nominal_voltage"],
        phase_resistance=DEFAULT_MOTOR_PARAMS["phase_resistance"],
        phase_inductance=DEFAULT_MOTOR_PARAMS["phase_inductance"],
        back_emf_constant=DEFAULT_MOTOR_PARAMS["back_emf_constant"],
        torque_constant=DEFAULT_MOTOR_PARAMS["torque_constant"],
        rotor_inertia=DEFAULT_MOTOR_PARAMS["rotor_inertia"],
        friction_coefficient=DEFAULT_MOTOR_PARAMS["friction_coefficient"],
        num_poles=DEFAULT_MOTOR_PARAMS["num_poles"],
        poles_pairs=DEFAULT_MOTOR_PARAMS["poles_pairs"],
    )

    motor = BLDCMotor(motor_params, dt=SIMULATION_PARAMS["dt"])
    print(f"  Motor: {motor_params.num_poles} poles, {motor_params.nominal_voltage}V nominal")

    # ===== STEP 2: Configure Load =====
    print("\n[2] Configuring Load Profile...")
    # Ramping load: 0 N·m at start, 2 N·m at t=2s, stays at 2 N·m
    load = RampLoad(initial=0.0, final=2.0, duration=2.0)
    print("  Load: Ramp from 0 to 2 N·m over 2 seconds")

    # ===== STEP 3: Create Simulation Engine =====
    print("\n[3] Creating Simulation Engine...")
    engine = SimulationEngine(motor, load, dt=SIMULATION_PARAMS["dt"], max_history=50000)
    print(f"  Engine: dt={SIMULATION_PARAMS['dt']}s, max history {engine.max_history} samples")

    # ===== STEP 4: Create Control Blocks =====
    print("\n[4] Creating Control Blocks...")

    # SVM Generator
    svm = SVMGenerator(dc_voltage=SIMULATION_PARAMS["dc_voltage"])
    print(f"  SVM: DC voltage {SIMULATION_PARAMS['dc_voltage']}V")
    print(f"       Max linear voltage: {svm.get_maximum_voltage():.2f}V")

    # V/f Controller
    controller = VFController(
        v_nominal=VF_CONTROLLER_PARAMS["v_nominal"],
        f_nominal=VF_CONTROLLER_PARAMS["f_nominal"],
        dc_voltage=SIMULATION_PARAMS["dc_voltage"],
        v_startup=VF_CONTROLLER_PARAMS["v_startup"],
    )
    controller.set_frequency_slew_rate(50.0)  # Hz/s
    controller.set_speed_reference(100.0)  # 100 Hz speed command
    print(
        f"  V/f Controller: {VF_CONTROLLER_PARAMS['v_nominal']}V at {VF_CONTROLLER_PARAMS['f_nominal']}Hz"
    )
    print("                  Speed reference: 100 Hz")

    # ===== STEP 5: Run Simulation =====
    print("\n[5] Running Simulation...")
    simulation_time = 5.0  # 5 seconds
    num_steps = int(simulation_time / engine.dt)

    for step in range(num_steps):
        # Get V/f controller output
        voltage_magnitude, angle = controller.update(engine.dt)

        # Generate SVM voltages
        voltages = svm.modulate(voltage_magnitude, angle)

        # Execute motor step
        engine.step(voltages, log_data=True)

        # Progress indicator
        if (step + 1) % 10000 == 0:
            percentage = ((step + 1) / num_steps) * 100
            print(f"  Progress: {percentage:.1f}% ({step + 1}/{num_steps} steps)")

    print(f"  Simulation complete! Simulated {simulation_time}s in {num_steps} steps")

    # ===== STEP 6: Analyze Results =====
    print("\n[6] Analyzing Results...")
    history = engine.get_history()

    # Find steady-state speed (last 500 samples)
    final_speed = np.mean(history["speed"][-500:])
    max_speed = np.max(history["speed"])
    max_current = np.max(np.abs(history["currents_a"]))
    max_torque = np.max(history["torque"])

    print(f"  Final Speed: {final_speed:.2f} RPM")
    print(f"  Max Speed: {max_speed:.2f} RPM")
    print(f"  Max Phase Current: {max_current:.3f} A")
    print(f"  Max Electromagnetic Torque: {max_torque:.4f} N·m")

    # ===== STEP 7: Export Data =====
    print("\n[7] Exporting Data...")
    logger = DataLogger()

    metadata = {
        "example": "V/f Control with Ramping Load",
        "simulation_time": simulation_time,
        "motor": "Default BLDC Motor",
        "control": "V/f with 100 Hz reference",
        "load": "Ramp 0-2 N·m over 2s",
    }

    csv_file = logger.save_simulation_data(history, metadata, "example_run")
    print(f"  Data saved to: {csv_file}")

    # ===== STEP 8: Generate Plots =====
    print("\n[8] Generating Plots...")

    # 3-phase overview plot
    fig_3phase = SimulationPlotter.create_3phase_plot(history)
    plot_dir = Path(__file__).parent.parent / "data" / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    plot_file_3phase = plot_dir / "example_3phase_overview.png"
    SimulationPlotter.save_plot(fig_3phase, plot_file_3phase)
    print(f"  3-phase plot saved to: {plot_file_3phase}")

    # Current detail plot
    fig_current = SimulationPlotter.create_current_plot(history)
    plot_file_current = plot_dir / "example_currents.png"
    SimulationPlotter.save_plot(fig_current, plot_file_current)
    print(f"  Current plot saved to: {plot_file_current}")

    print("\n" + "=" * 60)
    print("Example Complete!")
    print("Check data/logs/ for CSV files and data/plots/ for plots")
    print("=" * 60)


if __name__ == "__main__":
    main()
