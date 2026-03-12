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

from typing import Dict, Optional, Tuple
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


def _wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _pi_update(state: dict, error: float, dt: float) -> float:
    """Update simple PI controller with state dict containing kp, ki, integral."""
    kp = state["kp"]
    ki = state["ki"]
    state["integral"] += error * dt
    return kp * error + ki * state["integral"]


def _pi_update_anti_windup(
    state: Dict[str, float],
    error: float,
    dt: float,
    limit: Optional[float] = None,
) -> float:
    """PI update with optional back-calculation anti-windup and output limiting."""
    kp = state["kp"]
    ki = state["ki"]
    kaw = state.get("kaw", 0.0)

    integral = state["integral"]
    u_unsat = kp * error + ki * integral

    if limit is not None:
        u_sat = float(np.clip(u_unsat, -limit, limit))
    else:
        u_sat = u_unsat

    # Back-calculation anti-windup term uses (u_sat - u_unsat).
    state["integral"] += (error + kaw * (u_sat - u_unsat)) * dt
    return u_sat


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
        enable_speed_loop: bool = False,
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
        self.enable_speed_loop = enable_speed_loop

        # PI states for d and q axes
        self.pi_d = {"kp": 1.0, "ki": 0.1, "kaw": 0.5, "integral": 0.0}
        self.pi_q = {"kp": 1.0, "ki": 0.1, "kaw": 0.5, "integral": 0.0}
        self.pi_speed = {"kp": 0.02, "ki": 1.0, "kaw": 0.3, "integral": 0.0}

        # References
        self.id_ref = 0.0
        self.iq_ref = 0.0
        self.speed_ref = 0.0
        self.speed_error = 0.0

        # Cascaded-loop limits.
        self.iq_limit_a = 30.0
        self.vdq_limit = self.motor.params.nominal_voltage / np.sqrt(3.0)

        # Optional decoupling feed-forward terms for current loops.
        self.enable_decouple_d = False
        self.enable_decouple_q = False
        self.last_v_d_ff = 0.0
        self.last_v_q_ff = 0.0

        # Internal angle state
        self.theta = 0.0
        self.last_v_d = 0.0
        self.last_v_q = 0.0

        # Angle observer configuration and states.
        self.observer_target_mode = "Measured"
        self.angle_observer_mode = "Measured"  # Measured | PLL | SMO
        self.pll = {"kp": 80.0, "ki": 2000.0, "integral": 0.0}
        self.smo = {
            "k_slide": 600.0,
            "lpf_alpha": 0.08,
            "boundary": 0.06,
            "omega_est": 0.0,
        }
        self.theta_est_pll = 0.0
        self.theta_est_smo = 0.0
        self.theta_meas_emf = 0.0
        self.theta_error_pll = 0.0
        self.theta_error_smo = 0.0
        self.emf_observer_mag = 0.0

        # Sensorless startup transition (initial observer -> target observer).
        self.startup_transition_enabled = False
        self.startup_initial_mode = "Measured"
        self.startup_min_speed_rpm = 300.0
        self.startup_min_elapsed_s = 0.05
        self.startup_min_emf_v = 0.5
        self.startup_elapsed_s = 0.0
        self.startup_transition_done = True
        self.startup_ready_to_switch = False

    @staticmethod
    def _normalize_observer_mode(mode: str) -> str:
        normalized = str(mode).strip().upper()
        mapping = {
            "MEASURED": "Measured",
            "PLL": "PLL",
            "SMO": "SMO",
            "SLIDING": "SMO",
            "SLIDING MODE": "SMO",
        }
        if normalized not in mapping:
            raise ValueError("Unsupported angle observer mode")
        return mapping[normalized]

    def set_angle_observer(self, mode: str = "Measured") -> None:
        """Set electrical angle observer mode: Measured, PLL or SMO."""
        self.observer_target_mode = self._normalize_observer_mode(mode)
        if not self.startup_transition_enabled or self.startup_transition_done:
            self.angle_observer_mode = self.observer_target_mode

    def set_startup_transition(
        self,
        enabled: bool,
        initial_mode: str = "Measured",
        min_speed_rpm: float = 300.0,
        min_elapsed_s: float = 0.05,
        min_emf_v: float = 0.5,
    ) -> None:
        """Configure observer startup handoff from an initial to target mode."""
        if min_speed_rpm < 0.0:
            raise ValueError("min_speed_rpm must be non-negative")
        if min_elapsed_s < 0.0:
            raise ValueError("min_elapsed_s must be non-negative")
        if min_emf_v < 0.0:
            raise ValueError("min_emf_v must be non-negative")

        self.startup_transition_enabled = bool(enabled)
        self.startup_initial_mode = self._normalize_observer_mode(initial_mode)
        self.startup_min_speed_rpm = float(min_speed_rpm)
        self.startup_min_elapsed_s = float(min_elapsed_s)
        self.startup_min_emf_v = float(min_emf_v)

        self.startup_elapsed_s = 0.0
        self.startup_ready_to_switch = False
        self.startup_transition_done = not self.startup_transition_enabled
        self.angle_observer_mode = (
            self.startup_initial_mode
            if self.startup_transition_enabled
            else self.observer_target_mode
        )

    def _resolve_observer_mode(self, dt: float, emf_mag: float) -> str:
        """Resolve active observer mode considering startup transition logic."""
        if not self.startup_transition_enabled:
            self.startup_transition_done = True
            self.startup_ready_to_switch = False
            return self.observer_target_mode

        self.startup_elapsed_s += dt
        speed_ready = abs(self.motor.speed_rpm) >= self.startup_min_speed_rpm
        time_ready = self.startup_elapsed_s >= self.startup_min_elapsed_s
        emf_ready = emf_mag >= self.startup_min_emf_v
        self.startup_ready_to_switch = bool(speed_ready and time_ready and emf_ready)

        if not self.startup_transition_done and self.startup_ready_to_switch:
            self.startup_transition_done = True

        if self.startup_transition_done:
            return self.observer_target_mode
        return self.startup_initial_mode

    def set_pll_gains(self, kp: float, ki: float) -> None:
        """Set PLL observer gains."""
        self.pll["kp"] = float(kp)
        self.pll["ki"] = float(ki)

    def set_smo_gains(
        self,
        k_slide: float,
        lpf_alpha: float,
        boundary: float = 0.06,
    ) -> None:
        """Set SMO-like observer gains and smoothing constants."""
        if lpf_alpha <= 0.0 or lpf_alpha > 1.0:
            raise ValueError("lpf_alpha must be in (0, 1]")
        if boundary <= 0.0:
            raise ValueError("boundary must be positive")
        self.smo["k_slide"] = float(k_slide)
        self.smo["lpf_alpha"] = float(lpf_alpha)
        self.smo["boundary"] = float(boundary)

    def _estimate_theta_electrical(self, dt: float) -> float:
        """Estimate electrical angle from selected observer mode."""
        theta_measured = (self.motor.theta * self.motor.params.poles_pairs) % (
            2 * np.pi
        )

        emf_a, emf_b, emf_c = self.motor.back_emf
        if self.use_concordia:
            emf_alpha, emf_beta = concordia_transform(emf_a, emf_b, emf_c)
        else:
            emf_alpha, emf_beta = clarke_transform(emf_a, emf_b, emf_c)

        emf_mag = float(np.hypot(emf_alpha, emf_beta))
        self.emf_observer_mag = emf_mag
        self.angle_observer_mode = self._resolve_observer_mode(dt, emf_mag)

        if abs(emf_alpha) + abs(emf_beta) > 1e-12:
            self.theta_meas_emf = float(np.arctan2(emf_beta, emf_alpha)) % (2 * np.pi)

        mode = self.angle_observer_mode
        if mode == "Measured":
            self.theta_error_pll = 0.0
            self.theta_error_smo = 0.0
            return theta_measured

        if mode == "PLL":
            err = _wrap_angle(self.theta_meas_emf - self.theta_est_pll)
            self.theta_error_pll = err
            self.pll["integral"] += err * dt
            omega_correction = (
                self.pll["kp"] * err + self.pll["ki"] * self.pll["integral"]
            )
            omega_base = self.motor.omega * self.motor.params.poles_pairs
            self.theta_est_pll = (
                self.theta_est_pll + (omega_base + omega_correction) * dt
            ) % (2 * np.pi)
            return self.theta_est_pll

        # Sliding-mode-inspired observer variant.
        err = _wrap_angle(self.theta_meas_emf - self.theta_est_smo)
        self.theta_error_smo = err
        boundary = self.smo["boundary"]
        slide = float(np.tanh(err / boundary))
        omega_target = (
            self.motor.omega * self.motor.params.poles_pairs
            + self.smo["k_slide"] * slide
        )
        alpha = self.smo["lpf_alpha"]
        self.smo["omega_est"] = (1.0 - alpha) * self.smo[
            "omega_est"
        ] + alpha * omega_target
        self.theta_est_smo = (self.theta_est_smo + self.smo["omega_est"] * dt) % (
            2 * np.pi
        )
        return self.theta_est_smo

    def set_cascaded_speed_loop(
        self, enabled: bool, iq_limit_a: Optional[float] = None
    ) -> None:
        """Enable/disable cascaded speed->iq loop and optionally set iq clamp."""
        self.enable_speed_loop = enabled
        if iq_limit_a is not None:
            self.iq_limit_a = float(abs(iq_limit_a))

    def set_speed_pi_gains(self, kp: float, ki: float, kaw: float = 0.3) -> None:
        """Set speed loop PI gains for cascaded control mode."""
        self.pi_speed["kp"] = float(kp)
        self.pi_speed["ki"] = float(ki)
        self.pi_speed["kaw"] = float(kaw)

    def set_decoupling(self, enable_d: bool = False, enable_q: bool = False) -> None:
        """Enable/disable optional d/q decoupling feed-forward compensation."""
        self.enable_decouple_d = bool(enable_d)
        self.enable_decouple_q = bool(enable_q)

    def set_current_pi_gains(
        self,
        d_kp: float,
        d_ki: float,
        q_kp: float,
        q_ki: float,
        kaw: float = 0.5,
    ) -> None:
        """Set d/q current PI gains and anti-windup gain."""
        self.pi_d.update({"kp": float(d_kp), "ki": float(d_ki), "kaw": float(kaw)})
        self.pi_q.update({"kp": float(q_kp), "ki": float(q_ki), "kaw": float(kaw)})

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
        # compute electrical angle from selected observer
        self.theta = self._estimate_theta_electrical(dt)

        # read phase currents and transform
        ia, ib, ic = self.motor.currents
        if self.use_concordia:
            v_alpha, v_beta = concordia_transform(ia, ib, ic)
        else:
            v_alpha, v_beta = clarke_transform(ia, ib, ic)

        # park transform to get d/q currents
        id_val, iq_val = park_transform(v_alpha, v_beta, self.theta)

        if self.enable_speed_loop:
            speed_ref_rad_s = (self.speed_ref / 60.0) * (2 * np.pi)
            self.speed_error = speed_ref_rad_s - self.motor.omega
            self.iq_ref = _pi_update_anti_windup(
                self.pi_speed, self.speed_error, dt, limit=self.iq_limit_a
            )
        else:
            # Backward-compatible legacy mapping for existing scenarios.
            self.iq_ref = (self.speed_ref / 60.0) * (2 * np.pi) * self.pi_q["kp"]
            self.speed_error = 0.0

        # compute errors
        error_d = self.id_ref - id_val
        error_q = self.iq_ref - iq_val

        # PI controller outputs Vd and Vq with anti-windup clamps.
        v_d = _pi_update_anti_windup(self.pi_d, error_d, dt, limit=self.vdq_limit)
        v_q = _pi_update_anti_windup(self.pi_q, error_q, dt, limit=self.vdq_limit)

        omega_elec = self.motor.omega * self.motor.params.poles_pairs
        v_d_ff = (
            -omega_elec * self.motor.params.lq * iq_val
            if self.enable_decouple_d
            else 0.0
        )
        v_q_ff = (
            omega_elec * self.motor.params.ld * id_val
            if self.enable_decouple_q
            else 0.0
        )
        v_d += v_d_ff
        v_q += v_q_ff

        # Enforce vector saturation to available voltage circle.
        vdq_mag = np.hypot(v_d, v_q)
        if vdq_mag > self.vdq_limit and vdq_mag > 1e-12:
            scale = self.vdq_limit / vdq_mag
            v_d *= scale
            v_q *= scale

        self.last_v_d = v_d
        self.last_v_q = v_q
        self.last_v_d_ff = v_d_ff
        self.last_v_q_ff = v_q_ff

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
        self.pi_speed["integral"] = 0.0
        self.id_ref = 0.0
        self.iq_ref = 0.0
        self.speed_ref = 0.0
        self.speed_error = 0.0
        self.theta = 0.0
        self.last_v_d = 0.0
        self.last_v_q = 0.0
        self.last_v_d_ff = 0.0
        self.last_v_q_ff = 0.0
        self.pll["integral"] = 0.0
        self.smo["omega_est"] = 0.0
        self.theta_est_pll = 0.0
        self.theta_est_smo = 0.0
        self.theta_meas_emf = 0.0
        self.theta_error_pll = 0.0
        self.theta_error_smo = 0.0
        self.emf_observer_mag = 0.0
        self.startup_elapsed_s = 0.0
        self.startup_ready_to_switch = False
        self.startup_transition_done = not self.startup_transition_enabled
        self.angle_observer_mode = (
            self.startup_initial_mode
            if self.startup_transition_enabled
            else self.observer_target_mode
        )

    def get_state(self) -> dict:
        """Return controller state variables."""
        return {
            "id_ref": self.id_ref,
            "iq_ref": self.iq_ref,
            "speed_ref": self.speed_ref,
            "speed_error": self.speed_error,
            "speed_loop_enabled": self.enable_speed_loop,
            "decouple_d_enabled": self.enable_decouple_d,
            "decouple_q_enabled": self.enable_decouple_q,
            "angle_observer_mode": self.angle_observer_mode,
            "theta_electrical": self.theta,
            "theta_meas_emf": self.theta_meas_emf,
            "theta_error_pll": self.theta_error_pll,
            "theta_error_smo": self.theta_error_smo,
            "emf_observer_mag": self.emf_observer_mag,
            "observer_target_mode": self.observer_target_mode,
            "startup_transition_enabled": self.startup_transition_enabled,
            "startup_transition_done": self.startup_transition_done,
            "startup_ready_to_switch": self.startup_ready_to_switch,
            "startup_elapsed_s": self.startup_elapsed_s,
            "startup_initial_mode": self.startup_initial_mode,
            "startup_min_speed_rpm": self.startup_min_speed_rpm,
            "startup_min_elapsed_s": self.startup_min_elapsed_s,
            "startup_min_emf_v": self.startup_min_emf_v,
            "v_d": self.pi_d,
            "v_q": self.pi_q,
            "v_d_cmd": self.last_v_d,
            "v_q_cmd": self.last_v_q,
            "v_d_ff": self.last_v_d_ff,
            "v_q_ff": self.last_v_q_ff,
            "speed_pi": self.pi_speed,
            "pll": self.pll,
            "smo": self.smo,
        }
