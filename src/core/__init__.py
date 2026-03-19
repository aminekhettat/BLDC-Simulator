"""
BLDC Motor Control - Core Module
=================================

This package contains the core motor models and simulation engine for BLDC motor control.

:author: BLDC Control Team
:version: 0.8.0
:date: 2026-03-03
"""

from .motor_model import BLDCMotor, MotorParameters
from .load_model import LoadProfile, ConstantLoad, RampLoad, VariableLoad
from .simulation_engine import SimulationEngine
from .power_model import (
    SupplyProfile,
    ConstantSupply,
    RampSupply,
    VariableSupply,
    PowerFactorController,
    compute_efficiency_metrics,
    compute_power_metrics,
    recommend_efficiency_adjustments,
    required_reactive_compensation,
)

__all__ = [
    "BLDCMotor",
    "MotorParameters",
    "LoadProfile",
    "ConstantLoad",
    "RampLoad",
    "VariableLoad",
    "SimulationEngine",
    "SupplyProfile",
    "ConstantSupply",
    "RampSupply",
    "VariableSupply",
    "PowerFactorController",
    "compute_efficiency_metrics",
    "compute_power_metrics",
    "recommend_efficiency_adjustments",
    "required_reactive_compensation",
]
