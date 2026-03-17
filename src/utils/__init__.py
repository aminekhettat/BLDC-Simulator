"""
Utilities Package Module

This package contains utility modules for configuration and data logging.

:author: BLDC Control Team
:version: 1.0.0
"""

from .config import (
    DEFAULT_MOTOR_PARAMS,
    SIMULATION_PARAMS,
    VF_CONTROLLER_PARAMS,
    LOGS_DIR,
    PLOTS_DIR,
    MOTOR_PROFILES_DIR,
)
from .data_logger import DataLogger
from .motor_profiles import (
    PROFILE_SCHEMA,
    list_motor_profiles,
    load_motor_profile,
    save_motor_profile,
)

__all__ = [
    "DEFAULT_MOTOR_PARAMS",
    "SIMULATION_PARAMS",
    "VF_CONTROLLER_PARAMS",
    "LOGS_DIR",
    "PLOTS_DIR",
    "MOTOR_PROFILES_DIR",
    "DataLogger",
    "PROFILE_SCHEMA",
    "list_motor_profiles",
    "load_motor_profile",
    "save_motor_profile",
]
