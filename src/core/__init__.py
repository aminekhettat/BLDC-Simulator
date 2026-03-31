"""
BLDC Motor Control - Core Module
=================================

This package contains the core motor models and simulation engine for BLDC motor control.

:author: BLDC Control Team
:version: 0.8.0
:date: 2026-03-03
"""

from .load_model import ConstantLoad, LoadProfile, RampLoad, VariableLoad
from .motor_model import BLDCMotor, MotorParameters
from .power_model import (
    ConstantSupply,
    PowerFactorController,
    RampSupply,
    SupplyProfile,
    VariableSupply,
    compute_efficiency_metrics,
    compute_power_metrics,
    recommend_efficiency_adjustments,
    required_reactive_compensation,
)
from .simulation_engine import SimulationEngine

__all__ = [
    "BLDCMotor",
    "ConstantLoad",
    "ConstantSupply",
    "LoadProfile",
    "MotorParameters",
    "PowerFactorController",
    "RampLoad",
    "RampSupply",
    "SimulationEngine",
    "SupplyProfile",
    "VariableLoad",
    "VariableSupply",
    "compute_efficiency_metrics",
    "compute_power_metrics",
    "recommend_efficiency_adjustments",
    "required_reactive_compensation",
]
