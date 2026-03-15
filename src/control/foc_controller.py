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
    park_transform,
    inverse_park,
    concordia_transform,
)
from src.core.motor_model import BLDCMotor
import logging

logger = logging.getLogger(__name__)


def _wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _blend_angles(theta_a: float, theta_b: float, weight_b: float) -> float:
    """Blend two wrapped angles using vector interpolation."""
    w = float(np.clip(weight_b, 0.0, 1.0))
    s = (1.0 - w) * np.sin(theta_a) + w * np.sin(theta_b)
    c = (1.0 - w) * np.cos(theta_a) + w * np.cos(theta_b)
    return float(np.arctan2(s, c) % (2 * np.pi))


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
        self.startup_min_confidence = 0.6
        self.startup_confidence_hold_s = 0.02
        self.startup_confidence_hysteresis = 0.1
        self.startup_fallback_enabled = True
        self.startup_fallback_hold_s = 0.03
        self.startup_elapsed_s = 0.0
        self.startup_confidence_elapsed_s = 0.0
        self.startup_fallback_elapsed_s = 0.0
        self.startup_transition_done = True
        self.startup_ready_to_switch = False
        self.startup_fallback_event_count = 0
        self.observer_confidence = 0.0
        self.observer_confidence_emf = 0.0
        self.observer_confidence_speed = 0.0
        self.observer_confidence_coherence = 0.0
        self.observer_confidence_ema = 0.0
        self.observer_confidence_trend = 0.0
        self.observer_confidence_prev_above = False
        self.observer_confidence_above_threshold_time_s = 0.0
        self.observer_confidence_below_threshold_time_s = 0.0
        self.observer_confidence_crossings_up = 0
        self.observer_confidence_crossings_down = 0
        self.startup_handoff_count = 0
        self.startup_last_handoff_time_s = 0.0
        self.startup_last_handoff_confidence = 0.0
        self.startup_handoff_confidence_peak = 0.0
        self.startup_handoff_quality = 0.0
        self.startup_handoff_stability_ratio = 1.0

        # Standard startup sequence: align -> open-loop ramp -> closed-loop.
        self.startup_sequence_enabled = False
        self.startup_phase = "closed_loop"
        self.startup_sequence_elapsed_s = 0.0
        self.startup_phase_elapsed_s = 0.0
        self.startup_align_duration_s = 0.05
        self.startup_align_current_a = 1.5
        self.startup_align_angle = 0.0
        self.startup_open_loop_initial_speed_rpm = 30.0
        self.startup_open_loop_target_speed_rpm = 300.0
        self.startup_open_loop_ramp_time_s = 0.2
        self.startup_open_loop_id_ref_a = 0.0
        self.startup_open_loop_iq_ref_a = 2.0
        self.startup_open_loop_speed_rpm = 0.0
        self.startup_open_loop_angle = 0.0
        self.id_ref_command = 0.0
        self.iq_ref_command = 0.0

        # Sensorless maturity: blend measured and observer angle at low confidence
        # to improve low-speed robustness and reduce abrupt phase behavior.
        self.sensorless_blend_enabled = True
        self.sensorless_blend_min_speed_rpm = 250.0
        self.sensorless_blend_min_confidence = 0.65
        self.sensorless_blend_weight = 0.0
        self.theta_sensorless_raw = 0.0

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
        min_confidence: float = 0.6,
        confidence_hold_s: float = 0.02,
        confidence_hysteresis: float = 0.1,
        fallback_enabled: bool = True,
        fallback_hold_s: float = 0.03,
    ) -> None:
        """Configure observer startup handoff from an initial to target mode."""
        if min_speed_rpm < 0.0:
            raise ValueError("min_speed_rpm must be non-negative")
        if min_elapsed_s < 0.0:
            raise ValueError("min_elapsed_s must be non-negative")
        if min_emf_v < 0.0:
            raise ValueError("min_emf_v must be non-negative")
        if min_confidence < 0.0 or min_confidence > 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if confidence_hold_s < 0.0:
            raise ValueError("confidence_hold_s must be non-negative")
        if confidence_hysteresis < 0.0 or confidence_hysteresis > 1.0:
            raise ValueError("confidence_hysteresis must be in [0, 1]")
        if fallback_hold_s < 0.0:
            raise ValueError("fallback_hold_s must be non-negative")

        self.startup_transition_enabled = bool(enabled)
        self.startup_initial_mode = self._normalize_observer_mode(initial_mode)
        self.startup_min_speed_rpm = float(min_speed_rpm)
        self.startup_min_elapsed_s = float(min_elapsed_s)
        self.startup_min_emf_v = float(min_emf_v)
        self.startup_min_confidence = float(min_confidence)
        self.startup_confidence_hold_s = float(confidence_hold_s)
        self.startup_confidence_hysteresis = float(confidence_hysteresis)
        self.startup_fallback_enabled = bool(fallback_enabled)
        self.startup_fallback_hold_s = float(fallback_hold_s)

        self.startup_elapsed_s = 0.0
        self.startup_confidence_elapsed_s = 0.0
        self.startup_fallback_elapsed_s = 0.0
        self.startup_ready_to_switch = False
        self.startup_transition_done = not self.startup_transition_enabled
        self.startup_fallback_event_count = 0
        self.startup_handoff_count = 0
        self.startup_last_handoff_time_s = 0.0
        self.startup_last_handoff_confidence = 0.0
        self.startup_handoff_confidence_peak = 0.0
        self.startup_handoff_quality = 0.0
        self.startup_handoff_stability_ratio = 1.0
        self.observer_confidence_prev_above = False
        self.observer_confidence_above_threshold_time_s = 0.0
        self.observer_confidence_below_threshold_time_s = 0.0
        self.observer_confidence_crossings_up = 0
        self.observer_confidence_crossings_down = 0
        self.angle_observer_mode = (
            self.startup_initial_mode
            if self.startup_transition_enabled
            else self.observer_target_mode
        )

    def set_startup_sequence(
        self,
        enabled: bool,
        align_duration_s: float = 0.05,
        align_current_a: float = 1.5,
        align_angle_deg: float = 0.0,
        open_loop_initial_speed_rpm: float = 30.0,
        open_loop_target_speed_rpm: float = 300.0,
        open_loop_ramp_time_s: float = 0.2,
        open_loop_id_ref_a: float = 0.0,
        open_loop_iq_ref_a: float = 2.0,
    ) -> None:
        """Configure a standard sensored/sensorless startup sequence."""
        if align_duration_s < 0.0:
            raise ValueError("align_duration_s must be non-negative")
        if align_current_a < 0.0:
            raise ValueError("align_current_a must be non-negative")
        if open_loop_initial_speed_rpm < 0.0:
            raise ValueError("open_loop_initial_speed_rpm must be non-negative")
        if open_loop_target_speed_rpm < 0.0:
            raise ValueError("open_loop_target_speed_rpm must be non-negative")
        if open_loop_ramp_time_s < 0.0:
            raise ValueError("open_loop_ramp_time_s must be non-negative")

        self.startup_sequence_enabled = bool(enabled)
        self.startup_align_duration_s = float(align_duration_s)
        self.startup_align_current_a = float(align_current_a)
        self.startup_align_angle = float(np.deg2rad(align_angle_deg) % (2 * np.pi))
        self.startup_open_loop_initial_speed_rpm = float(open_loop_initial_speed_rpm)
        self.startup_open_loop_target_speed_rpm = float(open_loop_target_speed_rpm)
        self.startup_open_loop_ramp_time_s = float(open_loop_ramp_time_s)
        self.startup_open_loop_id_ref_a = float(open_loop_id_ref_a)
        self.startup_open_loop_iq_ref_a = float(open_loop_iq_ref_a)
        self._reset_startup_sequence_runtime()

    def _has_position_sensor(self) -> bool:
        """Treat direct measured angle mode as sensored operation."""
        return self.observer_target_mode == "Measured"

    def _enter_startup_phase(self, phase: str) -> None:
        """Switch startup phase and reset phase-local timers."""
        self.startup_phase = phase
        self.startup_phase_elapsed_s = 0.0
        if phase == "align":
            self.angle_observer_mode = "Alignment"
            self.startup_open_loop_angle = self.startup_align_angle
        elif phase == "open_loop":
            self.angle_observer_mode = "OpenLoop"
            measured_theta = (self.motor.theta * self.motor.params.poles_pairs) % (
                2 * np.pi
            )
            self.startup_open_loop_angle = measured_theta
            self.startup_open_loop_speed_rpm = self.startup_open_loop_initial_speed_rpm
            self.startup_elapsed_s = 0.0
            self.startup_confidence_elapsed_s = 0.0
            self.startup_fallback_elapsed_s = 0.0
            self.startup_ready_to_switch = False
            self.startup_transition_done = False
        else:
            self.angle_observer_mode = self.observer_target_mode

    def _reset_startup_sequence_runtime(self) -> None:
        """Reset phase runtime for the standard startup sequence."""
        self.startup_sequence_elapsed_s = 0.0
        self.startup_phase_elapsed_s = 0.0
        self.startup_open_loop_speed_rpm = self.startup_open_loop_initial_speed_rpm
        self.startup_open_loop_angle = self.startup_align_angle
        if not self.startup_sequence_enabled:
            self.startup_phase = "closed_loop"
            return
        if self.startup_align_duration_s > 0.0:
            self._enter_startup_phase("align")
        elif self._has_position_sensor():
            self._enter_startup_phase("closed_loop")
        else:
            self._enter_startup_phase("open_loop")

    def _update_target_observer_theta(self, dt: float) -> Tuple[float, float, float]:
        """Update target observer states and return measured/target angles."""
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

        if abs(emf_alpha) + abs(emf_beta) > 1e-12:
            self.theta_meas_emf = float(np.arctan2(emf_beta, emf_alpha)) % (2 * np.pi)

        if self.observer_target_mode == "PLL":
            err = _wrap_angle(self.theta_meas_emf - self.theta_est_pll)
            self.theta_error_pll = err
            self.theta_error_smo = 0.0
            self.pll["integral"] += err * dt
            omega_correction = (
                self.pll["kp"] * err + self.pll["ki"] * self.pll["integral"]
            )
            omega_base = self.motor.omega * self.motor.params.poles_pairs
            self.theta_est_pll = (
                self.theta_est_pll + (omega_base + omega_correction) * dt
            ) % (2 * np.pi)
            theta_target = self.theta_est_pll
        elif self.observer_target_mode == "SMO":
            err = _wrap_angle(self.theta_meas_emf - self.theta_est_smo)
            self.theta_error_smo = err
            self.theta_error_pll = 0.0
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
            theta_target = self.theta_est_smo
        else:
            self.theta_error_pll = 0.0
            self.theta_error_smo = 0.0
            theta_target = theta_measured

        speed_norm = min(
            abs(self.motor.speed_rpm) / max(self.startup_min_speed_rpm, 1e-9), 1.0
        )
        emf_norm = min(emf_mag / max(self.startup_min_emf_v, 1e-9), 1.0)
        phase_err = abs(_wrap_angle(self.theta_meas_emf - theta_target))
        coherence = 1.0 - min(phase_err / np.pi, 1.0)

        self.observer_confidence_emf = float(np.clip(emf_norm, 0.0, 1.0))
        self.observer_confidence_speed = float(np.clip(speed_norm, 0.0, 1.0))
        self.observer_confidence_coherence = float(np.clip(coherence, 0.0, 1.0))
        self.observer_confidence = float(
            np.clip(
                0.45 * self.observer_confidence_emf
                + 0.35 * self.observer_confidence_speed
                + 0.20 * self.observer_confidence_coherence,
                0.0,
                1.0,
            )
        )

        conf_prev = self.observer_confidence_ema
        conf_alpha = 0.15
        self.observer_confidence_ema = float(
            (1.0 - conf_alpha) * conf_prev + conf_alpha * self.observer_confidence
        )
        self.observer_confidence_trend = float(
            np.clip(self.observer_confidence_ema - conf_prev, -1.0, 1.0)
        )

        conf_above = self.observer_confidence >= self.startup_min_confidence
        if conf_above:
            self.observer_confidence_above_threshold_time_s += dt
        else:
            self.observer_confidence_below_threshold_time_s += dt
        if conf_above and not self.observer_confidence_prev_above:
            self.observer_confidence_crossings_up += 1
        elif (not conf_above) and self.observer_confidence_prev_above:
            self.observer_confidence_crossings_down += 1
        self.observer_confidence_prev_above = conf_above
        self.theta_sensorless_raw = theta_target
        return theta_measured, theta_target, emf_mag

    def _apply_sensorless_blend(
        self, theta_measured: float, theta_target: float
    ) -> float:
        """Blend measured and observer angles for low-speed robustness."""
        if self.observer_target_mode == "Measured":
            self.sensorless_blend_weight = 0.0
            return theta_measured
        if not self.sensorless_blend_enabled:
            self.sensorless_blend_weight = 1.0
            return theta_target

        w_speed = np.clip(
            abs(self.motor.speed_rpm) / max(self.sensorless_blend_min_speed_rpm, 1e-9),
            0.0,
            1.0,
        )
        w_conf = np.clip(
            self.observer_confidence / max(self.sensorless_blend_min_confidence, 1e-9),
            0.0,
            1.0,
        )
        self.sensorless_blend_weight = float(w_speed * w_conf)
        return _blend_angles(theta_measured, theta_target, self.sensorless_blend_weight)

    def _evaluate_sensorless_handoff(
        self, dt: float, emf_mag: float, confidence: float
    ) -> bool:
        """Evaluate when it is safe to leave forced open-loop startup."""
        self.startup_elapsed_s += dt

        conf_hi = self.startup_min_confidence
        speed_mag = abs(self.motor.speed_rpm)
        speed_ready = speed_mag >= self.startup_min_speed_rpm
        time_ready = self.startup_elapsed_s >= self.startup_min_elapsed_s
        emf_ready = emf_mag >= self.startup_min_emf_v
        static_ready = bool(speed_ready and time_ready and emf_ready)

        if confidence >= conf_hi:
            self.startup_confidence_elapsed_s += dt
        else:
            self.startup_confidence_elapsed_s = 0.0

        adaptive_ready = (
            self.startup_confidence_elapsed_s >= self.startup_confidence_hold_s
        )
        self.startup_ready_to_switch = bool(static_ready and adaptive_ready)
        self.startup_handoff_confidence_peak = max(
            self.startup_handoff_confidence_peak, confidence
        )
        return self.startup_transition_enabled and self.startup_ready_to_switch

    def _record_sensorless_handoff(self, confidence: float, emf_mag: float) -> None:
        """Store open-loop to closed-loop handoff diagnostics."""
        speed_mag = abs(self.motor.speed_rpm)
        speed_score = min(speed_mag / max(self.startup_min_speed_rpm, 1e-9), 1.0)
        emf_score = min(emf_mag / max(self.startup_min_emf_v, 1e-9), 1.0)
        time_score = min(
            self.startup_elapsed_s / max(self.startup_min_elapsed_s, 1e-9), 1.0
        )
        self.startup_handoff_quality = float(
            np.clip(
                0.5 * confidence
                + 0.2 * speed_score
                + 0.2 * emf_score
                + 0.1 * time_score,
                0.0,
                1.0,
            )
        )
        self.startup_handoff_count += 1
        self.startup_last_handoff_time_s = self.startup_sequence_elapsed_s
        self.startup_last_handoff_confidence = float(confidence)
        self.startup_transition_done = True
        self.startup_ready_to_switch = True
        self.startup_fallback_elapsed_s = 0.0

    def _maybe_apply_sensorless_fallback(
        self, dt: float, emf_mag: float, confidence: float
    ) -> bool:
        """Return True when closed-loop should fall back to forced open-loop."""
        if not self.startup_fallback_enabled:
            return False

        conf_lo = max(
            0.0, self.startup_min_confidence - self.startup_confidence_hysteresis
        )
        speed_release = self.startup_min_speed_rpm * 0.9
        emf_release = self.startup_min_emf_v * 0.9
        degraded = (
            confidence <= conf_lo
            or abs(self.motor.speed_rpm) < speed_release
            or emf_mag < emf_release
        )
        if degraded:
            self.startup_fallback_elapsed_s += dt
        else:
            self.startup_fallback_elapsed_s = 0.0

        if self.startup_fallback_elapsed_s < self.startup_fallback_hold_s:
            return False

        self.startup_fallback_elapsed_s = 0.0
        self.startup_transition_done = False
        self.startup_ready_to_switch = False
        self.startup_elapsed_s = 0.0
        self.startup_confidence_elapsed_s = 0.0
        self.startup_fallback_event_count += 1
        total_transitions = (
            self.startup_handoff_count + self.startup_fallback_event_count
        )
        self.startup_handoff_stability_ratio = (
            self.startup_handoff_count / total_transitions
            if total_transitions > 0
            else 1.0
        )
        return True

    def _estimate_theta_with_startup_sequence(self, dt: float) -> float:
        """Resolve control angle using the standard startup phase machine."""
        theta_measured, theta_target, emf_mag = self._update_target_observer_theta(dt)
        self.startup_sequence_elapsed_s += dt
        self.startup_phase_elapsed_s += dt

        if self.startup_phase == "align":
            self.angle_observer_mode = "Alignment"
            self.sensorless_blend_weight = 0.0
            if self.startup_phase_elapsed_s >= self.startup_align_duration_s:
                if self._has_position_sensor():
                    self.startup_transition_done = True
                    self.startup_ready_to_switch = True
                    self._enter_startup_phase("closed_loop")
                else:
                    self.startup_transition_done = False
                    self._enter_startup_phase("open_loop")
            return self.startup_align_angle

        if self.startup_phase == "open_loop":
            self.angle_observer_mode = "OpenLoop"
            self.sensorless_blend_weight = 0.0
            sign = -1.0 if self.speed_ref < 0.0 else 1.0
            if self.startup_open_loop_ramp_time_s <= 0.0:
                ramp_ratio = 1.0
            else:
                ramp_ratio = min(
                    self.startup_phase_elapsed_s / self.startup_open_loop_ramp_time_s,
                    1.0,
                )
            self.startup_open_loop_speed_rpm = (
                self.startup_open_loop_initial_speed_rpm
                + (
                    self.startup_open_loop_target_speed_rpm
                    - self.startup_open_loop_initial_speed_rpm
                )
                * ramp_ratio
            )
            omega_elec = (
                sign
                * (self.startup_open_loop_speed_rpm / 60.0)
                * (2 * np.pi)
                * self.motor.params.poles_pairs
            )
            self.startup_open_loop_angle = (
                self.startup_open_loop_angle + omega_elec * dt
            ) % (2 * np.pi)

            if self._evaluate_sensorless_handoff(dt, emf_mag, self.observer_confidence):
                self._record_sensorless_handoff(self.observer_confidence, emf_mag)
                self._enter_startup_phase("closed_loop")
                return self._apply_sensorless_blend(theta_measured, theta_target)
            return self.startup_open_loop_angle

        self.startup_transition_done = True
        self.startup_ready_to_switch = True
        self.angle_observer_mode = self.observer_target_mode
        if not self._has_position_sensor() and self._maybe_apply_sensorless_fallback(
            dt, emf_mag, self.observer_confidence
        ):
            self._enter_startup_phase("open_loop")
            return self.startup_open_loop_angle
        return self._apply_sensorless_blend(theta_measured, theta_target)

    def _get_active_references(self, dt: float) -> Tuple[float, float]:
        """Return d/q references after applying startup phase overrides."""
        id_ref = self.id_ref

        if self.enable_speed_loop:
            speed_ref_rad_s = (self.speed_ref / 60.0) * (2 * np.pi)
            self.speed_error = speed_ref_rad_s - self.motor.omega
            iq_ref = _pi_update_anti_windup(
                self.pi_speed, self.speed_error, dt, limit=self.iq_limit_a
            )
        else:
            iq_ref = (self.speed_ref / 60.0) * (2 * np.pi) * self.pi_q["kp"]
            self.speed_error = 0.0

        if not self.startup_sequence_enabled:
            return id_ref, iq_ref

        if self.startup_phase == "align":
            return self.startup_align_current_a, 0.0
        if self.startup_phase == "open_loop":
            return self.startup_open_loop_id_ref_a, self.startup_open_loop_iq_ref_a
        return id_ref, iq_ref

    def _resolve_observer_mode(
        self, dt: float, emf_mag: float, confidence: float
    ) -> str:
        """Resolve active observer mode considering startup transition logic."""
        if not self.startup_transition_enabled:
            self.startup_transition_done = True
            self.startup_ready_to_switch = False
            return self.observer_target_mode

        self.startup_elapsed_s += dt
        speed_mag = abs(self.motor.speed_rpm)
        conf_hi = self.startup_min_confidence
        conf_lo = max(0.0, conf_hi - self.startup_confidence_hysteresis)

        speed_ready = abs(self.motor.speed_rpm) >= self.startup_min_speed_rpm
        time_ready = self.startup_elapsed_s >= self.startup_min_elapsed_s
        emf_ready = emf_mag >= self.startup_min_emf_v
        static_ready = bool(speed_ready and time_ready and emf_ready)

        if confidence >= conf_hi:
            self.startup_confidence_elapsed_s += dt
        else:
            self.startup_confidence_elapsed_s = 0.0

        adaptive_ready = (
            self.startup_confidence_elapsed_s >= self.startup_confidence_hold_s
        )
        self.startup_ready_to_switch = bool(static_ready and adaptive_ready)

        if not self.startup_transition_done:
            self.startup_handoff_confidence_peak = max(
                self.startup_handoff_confidence_peak, confidence
            )

        if not self.startup_transition_done and self.startup_ready_to_switch:
            speed_score = min(speed_mag / max(self.startup_min_speed_rpm, 1e-9), 1.0)
            emf_score = min(emf_mag / max(self.startup_min_emf_v, 1e-9), 1.0)
            time_score = min(
                self.startup_elapsed_s / max(self.startup_min_elapsed_s, 1e-9), 1.0
            )
            self.startup_handoff_quality = float(
                np.clip(
                    0.5 * confidence
                    + 0.2 * speed_score
                    + 0.2 * emf_score
                    + 0.1 * time_score,
                    0.0,
                    1.0,
                )
            )
            self.startup_handoff_count += 1
            self.startup_last_handoff_time_s = self.startup_elapsed_s
            self.startup_last_handoff_confidence = float(confidence)
            self.startup_transition_done = True
            self.startup_fallback_elapsed_s = 0.0

        if self.startup_transition_done and self.startup_fallback_enabled:
            speed_release = self.startup_min_speed_rpm * 0.9
            emf_release = self.startup_min_emf_v * 0.9
            degraded = (
                confidence <= conf_lo
                or speed_mag < speed_release
                or emf_mag < emf_release
            )
            if degraded:
                self.startup_fallback_elapsed_s += dt
            else:
                self.startup_fallback_elapsed_s = 0.0

            if self.startup_fallback_elapsed_s >= self.startup_fallback_hold_s:
                self.startup_transition_done = False
                self.startup_ready_to_switch = False
                self.startup_elapsed_s = 0.0
                self.startup_confidence_elapsed_s = 0.0
                self.startup_fallback_elapsed_s = 0.0
                self.startup_fallback_event_count += 1
                total_transitions = (
                    self.startup_handoff_count + self.startup_fallback_event_count
                )
                self.startup_handoff_stability_ratio = (
                    self.startup_handoff_count / total_transitions
                    if total_transitions > 0
                    else 1.0
                )
                return self.startup_initial_mode

        total_transitions = (
            self.startup_handoff_count + self.startup_fallback_event_count
        )
        self.startup_handoff_stability_ratio = (
            self.startup_handoff_count / total_transitions
            if total_transitions > 0
            else 1.0
        )

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

    def set_sensorless_blend(
        self,
        enabled: bool = True,
        min_speed_rpm: float = 250.0,
        min_confidence: float = 0.65,
    ) -> None:
        """Configure low-speed sensorless angle blending behavior."""
        if min_speed_rpm < 0.0:
            raise ValueError("min_speed_rpm must be non-negative")
        if min_confidence < 0.0 or min_confidence > 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        self.sensorless_blend_enabled = bool(enabled)
        self.sensorless_blend_min_speed_rpm = float(min_speed_rpm)
        self.sensorless_blend_min_confidence = float(min_confidence)

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

        if abs(emf_alpha) + abs(emf_beta) > 1e-12:
            self.theta_meas_emf = float(np.arctan2(emf_beta, emf_alpha)) % (2 * np.pi)

        speed_norm = min(
            abs(self.motor.speed_rpm) / max(self.startup_min_speed_rpm, 1e-9), 1.0
        )
        emf_norm = min(emf_mag / max(self.startup_min_emf_v, 1e-9), 1.0)

        if self.observer_target_mode == "PLL":
            target_theta = self.theta_est_pll
        elif self.observer_target_mode == "SMO":
            target_theta = self.theta_est_smo
        else:
            target_theta = theta_measured

        phase_err = abs(_wrap_angle(self.theta_meas_emf - target_theta))
        coherence = 1.0 - min(phase_err / np.pi, 1.0)

        self.observer_confidence_emf = float(np.clip(emf_norm, 0.0, 1.0))
        self.observer_confidence_speed = float(np.clip(speed_norm, 0.0, 1.0))
        self.observer_confidence_coherence = float(np.clip(coherence, 0.0, 1.0))

        self.observer_confidence = float(
            np.clip(
                0.45 * self.observer_confidence_emf
                + 0.35 * self.observer_confidence_speed
                + 0.20 * self.observer_confidence_coherence,
                0.0,
                1.0,
            )
        )

        conf_prev = self.observer_confidence_ema
        conf_alpha = 0.15
        self.observer_confidence_ema = float(
            (1.0 - conf_alpha) * conf_prev + conf_alpha * self.observer_confidence
        )
        self.observer_confidence_trend = float(
            np.clip(self.observer_confidence_ema - conf_prev, -1.0, 1.0)
        )

        conf_above = self.observer_confidence >= self.startup_min_confidence
        if conf_above:
            self.observer_confidence_above_threshold_time_s += dt
        else:
            self.observer_confidence_below_threshold_time_s += dt
        if conf_above and not self.observer_confidence_prev_above:
            self.observer_confidence_crossings_up += 1
        elif (not conf_above) and self.observer_confidence_prev_above:
            self.observer_confidence_crossings_down += 1
        self.observer_confidence_prev_above = conf_above

        self.angle_observer_mode = self._resolve_observer_mode(
            dt, emf_mag, self.observer_confidence
        )

        mode = self.angle_observer_mode
        if mode == "Measured":
            self.theta_error_pll = 0.0
            self.theta_error_smo = 0.0
            self.sensorless_blend_weight = 0.0
            self.theta_sensorless_raw = theta_measured
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
            theta_obs = self.theta_est_pll
        else:
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
            theta_obs = self.theta_est_smo

        self.theta_sensorless_raw = theta_obs
        if not self.sensorless_blend_enabled:
            self.sensorless_blend_weight = 1.0
            return theta_obs

        w_speed = np.clip(
            abs(self.motor.speed_rpm) / max(self.sensorless_blend_min_speed_rpm, 1e-9),
            0.0,
            1.0,
        )
        w_conf = np.clip(
            self.observer_confidence / max(self.sensorless_blend_min_confidence, 1e-9),
            0.0,
            1.0,
        )
        self.sensorless_blend_weight = float(w_speed * w_conf)
        return _blend_angles(theta_measured, theta_obs, self.sensorless_blend_weight)

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
        # compute electrical angle from selected observer or startup sequencer
        if self.startup_sequence_enabled:
            self.theta = self._estimate_theta_with_startup_sequence(dt)
        else:
            self.theta = self._estimate_theta_electrical(dt)

        # read phase currents and transform
        ia, ib, ic = self.motor.currents
        if self.use_concordia:
            v_alpha, v_beta = concordia_transform(ia, ib, ic)
        else:
            v_alpha, v_beta = clarke_transform(ia, ib, ic)

        # park transform to get d/q currents
        id_val, iq_val = park_transform(v_alpha, v_beta, self.theta)

        self.id_ref_command, self.iq_ref_command = self._get_active_references(dt)

        # compute errors
        error_d = self.id_ref_command - id_val
        error_q = self.iq_ref_command - iq_val

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
        self.startup_confidence_elapsed_s = 0.0
        self.startup_fallback_elapsed_s = 0.0
        self.startup_ready_to_switch = False
        self.startup_transition_done = not self.startup_transition_enabled
        self.startup_fallback_event_count = 0
        self.observer_confidence = 0.0
        self.observer_confidence_emf = 0.0
        self.observer_confidence_speed = 0.0
        self.observer_confidence_coherence = 0.0
        self.observer_confidence_ema = 0.0
        self.observer_confidence_trend = 0.0
        self.observer_confidence_prev_above = False
        self.observer_confidence_above_threshold_time_s = 0.0
        self.observer_confidence_below_threshold_time_s = 0.0
        self.observer_confidence_crossings_up = 0
        self.observer_confidence_crossings_down = 0
        self.startup_handoff_count = 0
        self.startup_last_handoff_time_s = 0.0
        self.startup_last_handoff_confidence = 0.0
        self.startup_handoff_confidence_peak = 0.0
        self.startup_handoff_quality = 0.0
        self.startup_handoff_stability_ratio = 1.0
        self.startup_sequence_elapsed_s = 0.0
        self.startup_phase_elapsed_s = 0.0
        self.startup_open_loop_speed_rpm = self.startup_open_loop_initial_speed_rpm
        self.startup_open_loop_angle = self.startup_align_angle
        self.sensorless_blend_weight = 0.0
        self.theta_sensorless_raw = 0.0
        self.id_ref_command = 0.0
        self.iq_ref_command = 0.0
        self.angle_observer_mode = (
            self.startup_initial_mode
            if self.startup_transition_enabled
            else self.observer_target_mode
        )
        self._reset_startup_sequence_runtime()

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
            "startup_confidence_elapsed_s": self.startup_confidence_elapsed_s,
            "startup_fallback_elapsed_s": self.startup_fallback_elapsed_s,
            "startup_initial_mode": self.startup_initial_mode,
            "startup_min_speed_rpm": self.startup_min_speed_rpm,
            "startup_min_elapsed_s": self.startup_min_elapsed_s,
            "startup_min_emf_v": self.startup_min_emf_v,
            "startup_min_confidence": self.startup_min_confidence,
            "startup_confidence_hold_s": self.startup_confidence_hold_s,
            "startup_confidence_hysteresis": self.startup_confidence_hysteresis,
            "startup_fallback_enabled": self.startup_fallback_enabled,
            "startup_fallback_hold_s": self.startup_fallback_hold_s,
            "startup_fallback_event_count": self.startup_fallback_event_count,
            "observer_confidence": self.observer_confidence,
            "observer_confidence_emf": self.observer_confidence_emf,
            "observer_confidence_speed": self.observer_confidence_speed,
            "observer_confidence_coherence": self.observer_confidence_coherence,
            "observer_confidence_ema": self.observer_confidence_ema,
            "observer_confidence_trend": self.observer_confidence_trend,
            "observer_confidence_above_threshold_time_s": self.observer_confidence_above_threshold_time_s,
            "observer_confidence_below_threshold_time_s": self.observer_confidence_below_threshold_time_s,
            "observer_confidence_crossings_up": self.observer_confidence_crossings_up,
            "observer_confidence_crossings_down": self.observer_confidence_crossings_down,
            "startup_handoff_count": self.startup_handoff_count,
            "startup_last_handoff_time_s": self.startup_last_handoff_time_s,
            "startup_last_handoff_confidence": self.startup_last_handoff_confidence,
            "startup_handoff_confidence_peak": self.startup_handoff_confidence_peak,
            "startup_handoff_quality": self.startup_handoff_quality,
            "startup_handoff_stability_ratio": self.startup_handoff_stability_ratio,
            "startup_sequence_enabled": self.startup_sequence_enabled,
            "startup_phase": self.startup_phase,
            "startup_phase_code": {
                "align": 1.0,
                "open_loop": 2.0,
                "closed_loop": 3.0,
            }.get(self.startup_phase, 0.0),
            "startup_sequence_elapsed_s": self.startup_sequence_elapsed_s,
            "startup_phase_elapsed_s": self.startup_phase_elapsed_s,
            "startup_has_position_sensor": self._has_position_sensor(),
            "startup_align_duration_s": self.startup_align_duration_s,
            "startup_align_current_a": self.startup_align_current_a,
            "startup_align_angle_rad": self.startup_align_angle,
            "startup_open_loop_initial_speed_rpm": self.startup_open_loop_initial_speed_rpm,
            "startup_open_loop_target_speed_rpm": self.startup_open_loop_target_speed_rpm,
            "startup_open_loop_ramp_time_s": self.startup_open_loop_ramp_time_s,
            "startup_open_loop_id_ref_a": self.startup_open_loop_id_ref_a,
            "startup_open_loop_iq_ref_a": self.startup_open_loop_iq_ref_a,
            "startup_open_loop_speed_rpm": self.startup_open_loop_speed_rpm,
            "sensorless_blend_enabled": self.sensorless_blend_enabled,
            "sensorless_blend_min_speed_rpm": self.sensorless_blend_min_speed_rpm,
            "sensorless_blend_min_confidence": self.sensorless_blend_min_confidence,
            "sensorless_blend_weight": self.sensorless_blend_weight,
            "theta_sensorless_raw": self.theta_sensorless_raw,
            "id_ref_command": self.id_ref_command,
            "iq_ref_command": self.iq_ref_command,
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
