"""Example script demonstrating FOC control simulation.

Runs a simple simulation using FOCController with constant current
references and a variable supply profile to illustrate dynamic behavior.
"""

import numpy as np
from src.core import (
    BLDCMotor,
    MotorParameters,
    SimulationEngine,
    RampLoad,
    RampSupply,
)
from src.control import SVMGenerator, FOCController

# create motor and parameters
params = MotorParameters(
    nominal_voltage=48.0,
    phase_resistance=0.5,
    phase_inductance=0.001,
    back_emf_constant=0.1,
    torque_constant=0.1,
    rotor_inertia=0.001,
    friction_coefficient=0.0001,
    num_poles=8,
    poles_pairs=4,
)
motor = BLDCMotor(params, dt=0.0001)

# simple ramp load
load = RampLoad(initial=0.0, final=1.0, duration=5.0)

# supply sag from 48V down to 36V over 2s
supply = RampSupply(initial=48.0, final=36.0, duration=2.0)

engine = SimulationEngine(motor, load, dt=0.0001, supply_profile=supply)

# SVM generator
svm = SVMGenerator(dc_voltage=48.0)

# FOC controller
ctrl = FOCController(motor=motor)
ctrl.set_current_references(id_ref=0.0, iq_ref=2.0)
ctrl.set_speed_reference(1000.0)  # rpm

print("Starting FOC simulation...")
for i in range(20000):
    dt = engine.dt
    mag, ang = ctrl.update(dt)
    # convert polar to three-phase for motor
    voltages = svm.modulate(mag, ang)
    engine.step(voltages)

history = engine.get_history()
print("Simulation complete. Final speed (RPM):", history["speed"][-1])
