"""
Field-Oriented Control (FOC) Module
===================================

Implements basic FOC algorithm with PI current regulators and optional
auto-tuning functionality. Supports choosing Clarke or Concordia
transforms and running in either polar or Cartesian output modes.

:author: BLDC Control Team
:version: 1.0.0

.. versionadded:: 1.0.0
    Initial FOC implementation with PI regulators and auto-tune stub
"""

from typing import Tuple, Optional
import numpy as np
from .base_controller import BaseController
from .transforms import (
    clarke_transform,
    inverse_clarke,
    park_transform,
    inverse_park,
    concordia_transform,
    inverse_concordia,
)
from src.core.motor_model import BLDCMotor
import logging

logger = logging.getLogger(__name__)


def _pi_update(state: dict, error: float, dt: float) -> float:
    """Update simple PI controller with state dict containing kp, ki, integral."""
    kp = state["kp"]
    ki = state["ki"]
    state["integral"] += error * dt
    return kp * error + ki * state["integral"]


class FOCController(BaseController):
    """
    Basic Field-Oriented Controller supporting current loops and speed
    reference.  Designed for integration with SVM generator, returning either
    (magnitude, angle) or Cartesian (v_alpha, v_beta) depending on mode.

    **Features:**
      - d/q current PI regulators
      - speed reference (via iq setpoint)
      - optional auto-tuning stub
      - transform choice (clarke or concordia)
      - output in polar or Cartesian
    """

    def __init__(
        self,
        motor: BLDCMotor,
        use_concordia: bool = False,
        output_cartesian: bool = False,
    ):
        """Initialize FOC controller.

        :param motor: BLDCMotor instance for state feedback
        :param use_concordia: Choose concordia transform instead of Clarke
        :param output_cartesian: If True, return (v_alpha, v_beta) instead of
                                 magnitude/angle
        """
        super().__init__()
        self.motor = motor
        self.use_concordia = use_concordia
        self.output_cartesian = output_cartesian

        # PI states for d and q axes
        self.pi_d = {"kp": 1.0, "ki": 0.1, "integral": 0.0}
        self.pi_q = {"kp": 1.0, "ki": 0.1, "integral": 0.0}

        # References
        self.id_ref = 0.0
        self.iq_ref = 0.0
        self.speed_ref = 0.0

        # Internal angle state
        self.theta = 0.0

    def set_current_references(self, id_ref: float, iq_ref: float) -> None:
        """Set d/q current references."""
        self.id_ref = id_ref
        self.iq_ref = iq_ref

    def set_speed_reference(self, speed_rpm: float) -> None:
        """Set speed reference (rpm). qi reference will be derived from it."""
        self.speed_ref = speed_rpm

    def auto_tune_pi(self, axis: str = "q", bandwidth: float = 50.0) -> None:
        """Auto-tune PI parameters for given axis ('d' or 'q').

        This is a placeholder using simple heuristics.  A real implementation
        would perform step responses to estimate Ku and Tu and apply
        Ziegler–Nichols or IMC rules.
        """
        state = self.pi_d if axis == "d" else self.pi_q
        # heuristic: make kp proportional to bandwidth and rotor inertia
        J = self.motor.params.rotor_inertia
        state["kp"] = bandwidth * J * 10.0
        state["ki"] = state["kp"] * 10.0  # arbitrary
        logger.info(
            f"Auto-tuned {axis}-axis PI: kp={state['kp']:.3f}, ki={state['ki']:.3f}"
        )

    def update(self, dt: float) -> Tuple[float, float]:
        """Update controller and return voltage command.

        :param dt: Time step [s]
        :return: Either (magnitude, angle) or (v_alpha, v_beta) depending on
                 `output_cartesian` setting.
        """
        # compute electrical angle from motor position
        theta_e = self.motor.theta * self.motor.params.poles_pairs
        self.theta = theta_e % (2 * np.pi)

        # read phase currents and transform
        ia, ib, ic = self.motor.currents
        if self.use_concordia:
            v_alpha, v_beta = concordia_transform(ia, ib, ic)
        else:
            v_alpha, v_beta = clarke_transform(ia, ib, ic)

        # park transform to get d/q currents
        id_val, iq_val = park_transform(v_alpha, v_beta, self.theta)

        # update speed reference → iq_ref mapping simple proportional
        # assume linear relation (for demonstration)
        self.iq_ref = (self.speed_ref / 60.0) * (2 * np.pi) * self.pi_q["kp"]

        # compute errors
        error_d = self.id_ref - id_val
        error_q = self.iq_ref - iq_val

        # PI controller outputs Vd and Vq
        v_d = _pi_update(self.pi_d, error_d, dt)
        v_q = _pi_update(self.pi_q, error_q, dt)

        # inverse park to alpha-beta voltages
        v_alpha_cmd, v_beta_cmd = inverse_park(v_d, v_q, self.theta)

        if self.output_cartesian:
            return v_alpha_cmd, v_beta_cmd
        else:
            mag = np.hypot(v_alpha_cmd, v_beta_cmd)
            ang = np.arctan2(v_beta_cmd, v_alpha_cmd)
            return mag, ang

    def reset(self) -> None:
        """Reset controller state."""
        self.pi_d["integral"] = 0.0
        self.pi_q["integral"] = 0.0
        self.id_ref = 0.0
        self.iq_ref = 0.0
        self.speed_ref = 0.0
        self.theta = 0.0

    def get_state(self) -> dict:
        """Return controller state variables."""
        return {
            "id_ref": self.id_ref,
            "iq_ref": self.iq_ref,
            "speed_ref": self.speed_ref,
            "v_d": self.pi_d,
            "v_q": self.pi_q,
        }
