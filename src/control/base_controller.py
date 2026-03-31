"""
Base Controller Module
======================

This module provides the abstract base class for all control blocks.

Defines the interface that all controllers must implement for consistency
and future device integration (e.g., FOC, sensorless control).

:author: BLDC Control Team
:version: 0.8.0

.. versionadded:: 0.8.0
    Initial base controller implementation
"""

from abc import ABC, abstractmethod


class BaseController(ABC):
    """
    Abstract base class for motor controllers.

    All control algorithms (V/f, FOC, etc.) inherit from this class.

    **Minimum interface:**
    - update(dt): Calculate control output
    - reset(): Reset to initial state
    - get_state(): Return current state
    """

    @abstractmethod
    def update(self, dt: float):
        """
        Update controller and calculate control output.

        :param dt: Time step [seconds]
        :type dt: float
        :return: Control output (controller-specific)
        """

    @abstractmethod
    def reset(self) -> None:
        """
        Reset controller to initial state.
        """

    @abstractmethod
    def get_state(self) -> dict:
        """
        Get controller state variables.

        :return: Dictionary with controller state
        :rtype: dict
        """
