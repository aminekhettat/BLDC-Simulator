"""Adaptive control-loop tuning using stability margins and state-space checks.

This module provides a lightweight, dependency-free alternative to classic
control-toolbox workflows for the BLDC/PMSM simulator. It linearizes the current
and speed loops, evaluates gain/phase margins numerically, checks
controllability/observability of closed-loop state-space models, and searches
for PI gains that satisfy requested robustness targets.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple

import numpy as np

from src.core.motor_model import MotorParameters
from .foc_controller import FOCController


@dataclass(frozen=True)
class MarginResult:
    """Frequency-domain robustness margins for one SISO loop."""

    gain_margin_db: float
    phase_margin_deg: float
    gain_crossover_hz: Optional[float]
    phase_crossover_hz: Optional[float]


@dataclass(frozen=True)
class LoopDesignTargets:
    """Target robustness values used by the adaptive tuner."""

    min_gain_margin_db: float = 6.0
    min_phase_margin_deg: float = 45.0


@dataclass(frozen=True)
class PIGainCandidate:
    """One PI candidate and its loop analysis summary."""

    kp: float
    ki: float
    margin: MarginResult
    controllable: bool
    observable: bool
    score: float


@dataclass(frozen=True)
class AdaptiveTuningResult:
    """Selected gains and diagnostics for speed/current loops."""

    current_kp: float
    current_ki: float
    speed_kp: float
    speed_ki: float
    current_margin: MarginResult
    speed_margin: MarginResult
    current_controllable: bool
    current_observable: bool
    speed_controllable: bool
    speed_observable: bool


class AdaptiveFOCTuner:
    """Adaptive PI tuning for FOC loops using margin-driven optimization."""

    def __init__(
        self,
        params: MotorParameters,
        current_targets: Optional[LoopDesignTargets] = None,
        speed_targets: Optional[LoopDesignTargets] = None,
    ) -> None:
        self.params = params
        self.current_targets = current_targets or LoopDesignTargets()
        self.speed_targets = speed_targets or LoopDesignTargets()

    @staticmethod
    def _pi_frequency_response(kp: float, ki: float, omega: np.ndarray) -> np.ndarray:
        s = 1j * omega
        return kp + (ki / s)

    @staticmethod
    def _first_order_frequency_response(
        num: float,
        den_a: float,
        den_b: float,
        omega: np.ndarray,
    ) -> np.ndarray:
        s = 1j * omega
        return num / (den_a * s + den_b)

    @staticmethod
    def _interpolate_crossing(x1: float, y1: float, x2: float, y2: float) -> float:
        if abs(y2 - y1) < 1e-15:
            return x2
        t = -y1 / (y2 - y1)
        return x1 + t * (x2 - x1)

    def _estimate_margins(
        self, open_loop: np.ndarray, omega: np.ndarray
    ) -> MarginResult:
        mag = np.abs(open_loop)
        phase_deg = np.unwrap(np.angle(open_loop)) * 180.0 / np.pi

        gain_cross = None
        for k in range(len(mag) - 1):
            y1 = mag[k] - 1.0
            y2 = mag[k + 1] - 1.0
            if y1 == 0.0:
                gain_cross = omega[k]
                break
            if y1 * y2 < 0.0:
                gain_cross = self._interpolate_crossing(omega[k], y1, omega[k + 1], y2)
                break

        phase_margin = -180.0
        if gain_cross is not None:
            gc_phase = np.interp(gain_cross, omega, phase_deg)
            phase_margin = 180.0 + gc_phase

        phase_cross = None
        phase_with_offset = phase_deg + 180.0
        for k in range(len(phase_with_offset) - 1):
            y1 = phase_with_offset[k]
            y2 = phase_with_offset[k + 1]
            if y1 == 0.0:
                phase_cross = omega[k]
                break
            if y1 * y2 < 0.0:
                phase_cross = self._interpolate_crossing(omega[k], y1, omega[k + 1], y2)
                break

        gain_margin_db = np.inf
        if phase_cross is not None:
            mag_at_pc = np.interp(phase_cross, omega, mag)
            if mag_at_pc > 1e-15:
                gain_margin_db = 20.0 * np.log10(1.0 / mag_at_pc)

        return MarginResult(
            gain_margin_db=float(gain_margin_db),
            phase_margin_deg=float(phase_margin),
            gain_crossover_hz=(
                None if gain_cross is None else float(gain_cross / (2.0 * np.pi))
            ),
            phase_crossover_hz=(
                None if phase_cross is None else float(phase_cross / (2.0 * np.pi))
            ),
        )

    @staticmethod
    def _controllability_rank(a: np.ndarray, b: np.ndarray) -> int:
        n = a.shape[0]
        cols = [b]
        for _ in range(1, n):
            cols.append(a @ cols[-1])
        cm = np.hstack(cols)
        return int(np.linalg.matrix_rank(cm))

    @staticmethod
    def _observability_rank(a: np.ndarray, c: np.ndarray) -> int:
        n = a.shape[0]
        rows = [c]
        for _ in range(1, n):
            rows.append(rows[-1] @ a)
        om = np.vstack(rows)
        return int(np.linalg.matrix_rank(om))

    def _current_loop_state_space(
        self, kp: float, ki: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        r = max(self.params.phase_resistance, 1e-12)
        phase_l = max(self.params.phase_inductance, 1e-12)

        a = np.array(
            [
                [-(r + kp) / phase_l, ki / phase_l],
                [-1.0, 0.0],
            ],
            dtype=np.float64,
        )
        b = np.array([[kp / phase_l], [1.0]], dtype=np.float64)
        c = np.array([[1.0, 0.0]], dtype=np.float64)
        return a, b, c

    def _speed_loop_state_space(
        self, kp: float, ki: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        j = max(self.params.rotor_inertia, 1e-12)
        b_fric = max(self.params.friction_coefficient, 1e-12)
        kt = max(self.params.torque_constant, 1e-12)

        a = np.array(
            [
                [-(b_fric + kt * kp) / j, (kt * ki) / j],
                [-1.0, 0.0],
            ],
            dtype=np.float64,
        )
        b = np.array([[(kt * kp) / j], [1.0]], dtype=np.float64)
        c = np.array([[1.0, 0.0]], dtype=np.float64)
        return a, b, c

    def analyze_current_loop(self, kp: float, ki: float) -> Dict[str, object]:
        omega = np.logspace(0, 6, 2000)
        plant = self._first_order_frequency_response(
            num=1.0,
            den_a=max(self.params.phase_inductance, 1e-12),
            den_b=max(self.params.phase_resistance, 1e-12),
            omega=omega,
        )
        controller = self._pi_frequency_response(kp, ki, omega)
        loop = controller * plant
        margin = self._estimate_margins(loop, omega)

        a, b, c = self._current_loop_state_space(kp, ki)
        controllable = self._controllability_rank(a, b) == a.shape[0]
        observable = self._observability_rank(a, c) == a.shape[0]

        return {
            "margin": margin,
            "controllable": controllable,
            "observable": observable,
        }

    def analyze_speed_loop(self, kp: float, ki: float) -> Dict[str, object]:
        omega = np.logspace(-1, 4, 2000)
        plant = self._first_order_frequency_response(
            num=max(self.params.torque_constant, 1e-12),
            den_a=max(self.params.rotor_inertia, 1e-12),
            den_b=max(self.params.friction_coefficient, 1e-12),
            omega=omega,
        )
        controller = self._pi_frequency_response(kp, ki, omega)
        loop = controller * plant
        margin = self._estimate_margins(loop, omega)

        a, b, c = self._speed_loop_state_space(kp, ki)
        controllable = self._controllability_rank(a, b) == a.shape[0]
        observable = self._observability_rank(a, c) == a.shape[0]

        return {
            "margin": margin,
            "controllable": controllable,
            "observable": observable,
        }

    @staticmethod
    def _search_space(lo: float, hi: float, samples: int) -> Iterable[float]:
        return np.logspace(np.log10(lo), np.log10(hi), samples)

    @staticmethod
    def _candidate_score(
        margin: MarginResult,
        targets: LoopDesignTargets,
        controllable: bool,
        observable: bool,
    ) -> float:
        if not controllable or not observable:
            return -1e12

        gm_term = min(margin.gain_margin_db - targets.min_gain_margin_db, 30.0)
        pm_term = min(margin.phase_margin_deg - targets.min_phase_margin_deg, 90.0)
        penalty = 0.0
        if gm_term < 0.0:
            penalty += 200.0 * gm_term
        if pm_term < 0.0:
            penalty += 200.0 * pm_term

        return gm_term + pm_term + penalty

    def _optimize_loop(
        self,
        analyzer,
        targets: LoopDesignTargets,
        kp_range: Tuple[float, float],
        ki_range: Tuple[float, float],
        grid_size: int,
    ) -> PIGainCandidate:
        best: Optional[PIGainCandidate] = None

        for kp in self._search_space(kp_range[0], kp_range[1], grid_size):
            for ki in self._search_space(ki_range[0], ki_range[1], grid_size):
                result = analyzer(float(kp), float(ki))
                margin: MarginResult = result["margin"]  # type: ignore[assignment]
                controllable = bool(result["controllable"])
                observable = bool(result["observable"])
                score = self._candidate_score(
                    margin=margin,
                    targets=targets,
                    controllable=controllable,
                    observable=observable,
                )
                candidate = PIGainCandidate(
                    kp=float(kp),
                    ki=float(ki),
                    margin=margin,
                    controllable=controllable,
                    observable=observable,
                    score=float(score),
                )
                if best is None or candidate.score > best.score:
                    best = candidate

        if best is None:
            raise RuntimeError("Unable to generate PI candidates")
        return best

    def tune(
        self,
        current_kp_range: Tuple[float, float] = (1e-3, 5.0),
        current_ki_range: Tuple[float, float] = (1e-1, 5e3),
        speed_kp_range: Tuple[float, float] = (1e-6, 2.0),
        speed_ki_range: Tuple[float, float] = (1e-4, 5e2),
        grid_size: int = 12,
    ) -> AdaptiveTuningResult:
        current_best = self._optimize_loop(
            analyzer=self.analyze_current_loop,
            targets=self.current_targets,
            kp_range=current_kp_range,
            ki_range=current_ki_range,
            grid_size=grid_size,
        )
        speed_best = self._optimize_loop(
            analyzer=self.analyze_speed_loop,
            targets=self.speed_targets,
            kp_range=speed_kp_range,
            ki_range=speed_ki_range,
            grid_size=grid_size,
        )

        return AdaptiveTuningResult(
            current_kp=current_best.kp,
            current_ki=current_best.ki,
            speed_kp=speed_best.kp,
            speed_ki=speed_best.ki,
            current_margin=current_best.margin,
            speed_margin=speed_best.margin,
            current_controllable=current_best.controllable,
            current_observable=current_best.observable,
            speed_controllable=speed_best.controllable,
            speed_observable=speed_best.observable,
        )

    @staticmethod
    def apply_to_foc(controller: FOCController, tuning: AdaptiveTuningResult) -> None:
        """Apply adaptive-tuning PI gains to a FOC controller instance."""
        controller.set_current_pi_gains(
            d_kp=tuning.current_kp,
            d_ki=tuning.current_ki,
            q_kp=tuning.current_kp,
            q_ki=tuning.current_ki,
            kaw=0.2,
        )
        controller.set_speed_pi_gains(
            kp=tuning.speed_kp,
            ki=tuning.speed_ki,
            kaw=0.05,
        )
