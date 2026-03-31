"""
Field-Weakening Auto-Calibrator
================================

Automatic calibration of field-weakening (FW) parameters for BLDC/PMSM motors
under FOC.  The calibrator performs two complementary steps:

1. **Analytical design** – derives physically grounded FW parameters from motor
   nameplate data (Ke, L, R, rated current, rated speed, supply voltage).

2. **Simulation validation** – sweeps a small grid of controller gains and
   verifies closed-loop stability and speed-tracking in a full-order RK4 motor
   simulation, picking the best validated set.

Motor model physics recap
--------------------------
In the d-q frame the q-axis voltage equation is (steady-state, R neglected)::

    Vq ≈ ωe × λpm_eff

where  ``λpm_eff = λpm × flux_ratio``,  ``λpm = Ke / pp``.

The ``flux_ratio`` is reduced by negative Id injection according to the
motor-model parameter ``flux_weakening_id_coefficient`` (k_fw)::

    flux_ratio = clip(1 + k_fw × min(Id, 0),  min_ratio, 1)

To reach rated speed ``N_rated`` the voltage demand must stay within the
phase-voltage limit ``Vphase = Vnom / √3``::

    Vphase ≥ ωe_rated × λpm × flux_ratio
    → flux_ratio ≤ Vphase / (ωe_rated × λpm)
                 = Vphase / (Ke × ωmech_rated)       [since ωe×λpm = Ke×ωmech]

This gives ``ratio_required``.  The calibrator solves for the k_fw that
achieves this ratio when Id = -fw_id_max_a::

    k_fw = (1 – ratio_required) × safety / fw_id_max_a

The FOC controller injects negative Id via a proportional-integral headroom
controller::

    d(Id_inj)/dt = gain × (headroom_target – (Vlim – |Vdq_unsat|))

Since ``|Vdq_unsat|`` can far exceed ``Vlim`` when the motor is deep in the
field-weakening region, the effective error is large and FW engages quickly
even with modest gain values.

Usage
------
::

    from src.control.field_weakening_calibrator import (
        FieldWeakeningCalibrator, FWCalibrationResult
    )

    result = FieldWeakeningCalibrator.from_profile_file(
        "data/motor_profiles/motenergy_me1718_48v.json"
    ).calibrate()
    print(result)
"""

from __future__ import annotations

import json
import math

# ---------------------------------------------------------------------------
# Project imports (resolved at call-time to avoid circular imports)
# ---------------------------------------------------------------------------
import sys as _sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from pathlib import Path as _Path
from typing import Any

import numpy as np

from src.core.load_model import LoadProfile

_PROJECT_ROOT = _Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in _sys.path:
    _sys.path.insert(0, str(_PROJECT_ROOT))


# ---------------------------------------------------------------------------
# Public data-classes
# ---------------------------------------------------------------------------


@dataclass
class FWPhysicsParams:
    """Physics-based FW parameters derived from motor nameplate data."""

    # Motor-model parameter controlling PM flux reduction
    flux_weakening_id_coefficient: float  # k_fw  [1/A]
    flux_weakening_min_ratio: float  # λpm_eff / λpm floor

    # FOC controller FW block parameters
    fw_start_rpm: float  # speed at which FW injection begins [RPM]
    fw_gain: float  # integrator gain  [A / (V·s)]
    fw_id_max_a: float  # max |Id| injection  [A]
    fw_headroom_target_v: float  # voltage headroom target  [V]

    # Diagnostics
    no_fw_max_rpm: float  # max speed without FW  [RPM]
    rated_speed_rpm: float  # rated speed from nameplate  [RPM]
    ratio_required: float  # min flux ratio to reach rated speed
    fw_needed: bool  # True when rated speed > no-FW speed


@dataclass
class FWValidationMetrics:
    """Per-operating-point simulation metrics."""

    target_rpm: float
    load_torque_nm: float
    achieved_rpm: float
    speed_error_rpm: float
    speed_error_pct: float
    fw_injection_a: float  # mean steady-state FW Id injection
    id_a: float  # mean steady-state Id
    iq_a: float  # mean steady-state Iq
    efficiency_pct: float
    stable: bool
    status: str  # "PASS" | "FAIL" | "UNSTABLE"


@dataclass
class FWCalibrationResult:
    """Complete field-weakening calibration result for one motor."""

    motor_profile_name: str
    motor_params_summary: dict[str, Any]

    # Calibrated parameters
    physics_params: FWPhysicsParams
    selected_gain: float
    selected_fw_id_max_a: float

    # Validation results
    operating_points: list[FWValidationMetrics]

    # Summary
    all_passed: bool
    best_achieved_rpm: float
    calibration_time_s: float
    notes: str = ""

    def __repr__(self) -> str:  # pragma: no cover
        pts = len(self.operating_points)
        passed = sum(1 for p in self.operating_points if p.status == "PASS")
        return (
            f"FWCalibrationResult({self.motor_profile_name!r}, "
            f"rated={self.physics_params.rated_speed_rpm:.0f} RPM, "
            f"best={self.best_achieved_rpm:.0f} RPM, "
            f"{passed}/{pts} pts PASS)"
        )

    def to_dict(self) -> dict[str, Any]:
        """Serialise to a JSON-compatible dictionary."""
        d = {
            "schema": "bldc.fw_calibration.v1",
            "motor_profile_name": self.motor_profile_name,
            "motor_params_summary": self.motor_params_summary,
            "physics_params": asdict(self.physics_params),
            "selected_gain": self.selected_gain,
            "selected_fw_id_max_a": self.selected_fw_id_max_a,
            "operating_points": [asdict(p) for p in self.operating_points],
            "all_passed": self.all_passed,
            "best_achieved_rpm": self.best_achieved_rpm,
            "calibration_time_s": self.calibration_time_s,
            "notes": self.notes,
        }
        return d


# ---------------------------------------------------------------------------
# Internal simulation helper
# ---------------------------------------------------------------------------


class _RampLoad(LoadProfile):
    """Smooth cubic-ease-in-out load ramp from 0 to *torque* over [t0, t1]."""

    def __init__(self, t0: float, t1: float, torque: float) -> None:
        self.t0 = float(t0)
        self.t1 = float(t1)
        self.torque = float(torque)

    def get_torque(self, t: float) -> float:
        if t <= self.t0:
            return 0.0
        if t >= self.t1:
            return self.torque
        x = (t - self.t0) / max(self.t1 - self.t0, 1e-9)
        s = x * x * (3.0 - 2.0 * x)
        return self.torque * s


def _run_fw_simulation(
    params,  # MotorParameters (already has k_fw set)
    speed_kp: float,
    speed_ki: float,
    cur_kp: float,
    cur_ki: float,
    iq_limit_a: float,
    target_rpm: float,
    load_torque_nm: float,
    fw_start_rpm: float,
    fw_gain: float,
    fw_id_max_a: float,
    fw_headroom_v: float,
    dt: float,
    sim_duration: float,
    flux_ratio_at_target: float = 1.0,
) -> dict[str, Any] | None:
    """
    Run a single closed-loop FOC+FW simulation and return steady-state metrics.

    Returns None if the simulation diverges.

    The ``flux_ratio_at_target`` argument is used to scale the speed-loop
    gains so that the speed PI remains well-tuned even when the effective
    motor torque constant Kt is reduced by field-weakening (Kt_eff = Kt ×
    flux_ratio).  Without this scaling the PI is too aggressive in FW mode,
    causing overshoot and instability above the no-FW speed limit.
    """
    from src.control.foc_controller import FOCController
    from src.control.svm_generator import SVMGenerator
    from src.control.transforms import clarke_transform, park_transform
    from src.core.motor_model import BLDCMotor
    from src.core.simulation_engine import SimulationEngine

    # ── Speed PI scaling for FW region ───────────────────────────────────
    # With the back-EMF estimation approach for FW headroom (Kim & Sul 1997),
    # the FW integrator no longer over-injects Id during startup transients.
    # The speed loop gain does NOT need to be artificially reduced:
    #
    #   • In the sub-FW region: full gains give crisp acceleration response.
    #   • In the FW region: Kt_eff = Kt × flux_ratio is reduced, which
    #     naturally de-rates the plant gain.  A proportionally faster speed
    #     PI (gain = 1.0 × session_gains) therefore produces roughly the same
    #     closed-loop bandwidth as the original session tuning — no detuning
    #     needed.
    #
    # A fixed fw_scale = 1.0 avoids artificially slow startup behaviour that
    # occurred when flux_ratio_at_target (the deep-FW equilibrium value,
    # e.g. 0.383 for ME1718 at 4000 RPM) was applied from t = 0.
    fw_scale = 1.0
    speed_kp_fw = speed_kp * fw_scale
    speed_ki_fw = speed_ki * fw_scale

    motor = BLDCMotor(params, dt=dt)
    load = _RampLoad(0.4, 1.2, load_torque_nm)
    engine = SimulationEngine(motor, load, dt=dt, compute_backend="auto")

    ctrl = FOCController(motor=motor, enable_speed_loop=True)
    ctrl.set_cascaded_speed_loop(True, iq_limit_a=iq_limit_a)
    # kaw=1.0: aggressive back-calculation anti-windup on the speed loop.
    # With kaw=0.05 the integral takes ~22 s to drain when Iq saturates
    # (common during the long acceleration to FW speeds).  kaw=1.0 drains
    # it in ~1 s, eliminating the sustained overshoot/oscillation that
    # prevented the motor from settling at rated RPM.
    ctrl.set_speed_pi_gains(kp=speed_kp_fw, ki=speed_ki_fw, kaw=1.0)
    # Multi-rate: configure ALL loop rates to match a real MCU timing model.
    #
    #   Current loop (Clarke/Park + d/q PI + observer) : every dt      = T_pwm
    #   Speed PI + FW integrator (slow ISR)            : every N×dt   ≈ 10 ms
    #
    # configure_mcu_timing() sets both speed_loop_divider and fw_loop_divider
    # to N = round(10 ms / dt), clamped so the slow-ISR period stays ≈ 10 ms
    # regardless of the simulation timestep chosen by _safe_dt().
    ctrl.configure_mcu_timing(pwm_freq_hz=1.0 / dt, speed_loop_hz=100.0)
    ctrl.set_current_pi_gains(
        d_kp=cur_kp,
        d_ki=cur_ki,
        q_kp=cur_kp,
        q_ki=cur_ki,
        kaw=0.2,
    )
    ctrl.set_voltage_saturation(
        mode="d_priority",
        coupled_antiwindup_enabled=True,
        coupled_antiwindup_gain=0.3,
    )
    ctrl.set_current_references(id_ref=0.0, iq_ref=0.0)
    ctrl.set_angle_observer("Measured")
    ctrl.set_startup_transition(enabled=False, initial_mode="Measured")
    ctrl.set_startup_sequence(enabled=False)
    ctrl._enter_startup_phase("closed_loop")
    ctrl.startup_transition_done = True
    ctrl.startup_ready_to_switch = True

    ctrl.set_field_weakening(
        enabled=True,
        start_speed_rpm=fw_start_rpm,
        gain=fw_gain,
        max_negative_id_a=fw_id_max_a,
        headroom_target_v=fw_headroom_v,
    )
    ctrl.set_decoupling(enable_d=True, enable_q=True)
    # Do NOT set the speed reference yet; we ramp it below.

    svm = SVMGenerator(dc_voltage=params.nominal_voltage)
    svm.set_sample_time(dt)

    n_steps = int(sim_duration / dt)
    speeds = np.zeros(n_steps)
    ids = np.zeros(n_steps)
    iqs = np.zeros(n_steps)
    fw_inj = np.zeros(n_steps)
    p_in = np.zeros(n_steps)
    p_out = np.zeros(n_steps)

    # Speed ramp: ramp from 0 → target_rpm over the first 40 % of the
    # simulation (≥ 2 s, ≤ 6 s).  This keeps the speed-loop error small
    # and prevents Iq from saturating during acceleration, which caused
    # the speed PI integral to wind up and the motor to oscillate badly
    # even when kaw was large.  The motor naturally enters FW as it passes
    # the no-FW speed limit during the ramp, giving a clean steady-state
    # for the tail evaluation window.
    t_ramp = float(np.clip(0.4 * sim_duration, 2.0, 6.0))

    for k in range(n_steps):
        t = (k + 1) * dt

        # Update speed reference along the ramp
        ramp_frac = min(t / t_ramp, 1.0)
        ctrl.set_speed_reference(target_rpm * ramp_frac)

        svm.set_phase_currents(motor.currents)
        mag, ang = ctrl.update(dt)

        if not (np.isfinite(mag) and np.isfinite(ang)):
            return None

        vabc = np.array(svm.modulate(mag, ang), dtype=np.float64)
        ia, ib, ic = float(motor.currents[0]), float(motor.currents[1]), float(motor.currents[2])
        engine.step(vabc, log_data=False)

        if abs(motor.speed_rpm) > 1e5 or not np.isfinite(motor.omega):
            return None

        theta_e = float((motor.theta * params.poles_pairs) % (2.0 * math.pi))
        i_al, i_be = clarke_transform(ia, ib, ic)
        i_d, i_q = park_transform(i_al, i_be, theta_e)

        speeds[k] = float(motor.speed_rpm)
        ids[k] = float(i_d)
        iqs[k] = float(i_q)
        fw_inj[k] = float(ctrl.field_weakening_id_injection_a)
        p_in[k] = float(abs(np.dot(vabc, np.array([ia, ib, ic]))))
        p_out[k] = float(abs(load.get_torque(t) * motor.omega))

    # Steady-state window: last 25 % of simulation (≥ 1.5 s worth)
    # Using the trailing quarter ensures we see true steady-state after
    # any transient from the load ramp and FW engagement.
    tail_samples = max(int(0.25 * n_steps), int(1.5 / dt))
    tail = slice(-tail_samples, None)

    speed_mean = float(np.mean(speeds[tail]))
    speed_err = float(speed_mean - target_rpm)
    speed_err_p = 100.0 * speed_err / max(abs(target_rpm), 1.0)
    id_mean = float(np.mean(ids[tail]))
    iq_mean = float(np.mean(iqs[tail]))
    fw_mean = float(np.mean(fw_inj[tail]))
    p_in_mean = float(np.mean(p_in[tail]))
    p_out_mean = float(np.mean(p_out[tail]))
    eff = 100.0 * p_out_mean / max(p_in_mean, 0.1)

    return {
        "speed_mean": speed_mean,
        "speed_err": speed_err,
        "speed_err_pct": speed_err_p,
        "id_mean": id_mean,
        "iq_mean": iq_mean,
        "fw_injection": fw_mean,
        "p_in": p_in_mean,
        "p_out": p_out_mean,
        "efficiency": eff,
    }


# ---------------------------------------------------------------------------
# Main calibrator class
# ---------------------------------------------------------------------------


class FieldWeakeningCalibrator:
    """
    Physics-based field-weakening auto-calibrator for BLDC/PMSM motors.

    Parameters
    ----------
    motor_params_dict : dict
        Raw ``motor_params`` section from a motor profile JSON.
    rated_info : dict
        Raw ``rated_info`` section from a motor profile JSON.
    profile_name : str
        Human-readable motor name (for reports).
    fw_current_fraction : float
        Fraction of rated current to use as max FW Id injection (default 0.25).
    safety_factor : float
        Extra flux-reduction margin on top of the minimum required (default 1.15).
    sim_duration : float
        Closed-loop simulation duration in seconds (default 6.0).
    verbose : bool
        Print progress messages (default True).
    """

    def __init__(
        self,
        motor_params_dict: dict[str, Any],
        rated_info: dict[str, Any],
        profile_name: str = "Unknown",
        fw_current_fraction: float = 0.25,
        safety_factor: float = 1.15,
        sim_duration: float = 10.0,
        verbose: bool = True,
    ) -> None:
        self.mp_dict = motor_params_dict
        self.rated_info = rated_info
        self.profile_name = profile_name
        self.fw_frac = fw_current_fraction
        self.safety = safety_factor
        self.sim_duration = sim_duration
        self.verbose = verbose

        # Parse motor parameters
        self._R = float(motor_params_dict["phase_resistance"])
        self._L = float(motor_params_dict["phase_inductance"])
        self._Ke = float(motor_params_dict["back_emf_constant"])
        self._Kt = float(motor_params_dict.get("torque_constant", self._Ke))
        self._J = float(motor_params_dict.get("rotor_inertia", 0.001))
        self._B = float(motor_params_dict.get("friction_coefficient", 0.001))
        self._pp = int(
            motor_params_dict.get("poles_pairs", int(motor_params_dict.get("num_poles", 10)) // 2)
        )
        self._Vnom = float(motor_params_dict["nominal_voltage"])
        self._Ld = float(motor_params_dict.get("ld", self._L))
        self._Lq = float(motor_params_dict.get("lq", self._L))

        # Rated info
        self._N_rated = float(rated_info.get("rated_speed_rpm", 4000.0))
        # Accept either rated_current_a or rated_current_a_rms
        self._I_rated = float(
            rated_info.get("rated_current_a", rated_info.get("rated_current_a_rms", 100.0))
        )
        self._T_rated = float(rated_info.get("rated_torque_nm", self._Kt * self._I_rated))

    # ------------------------------------------------------------------
    # Class-method constructors
    # ------------------------------------------------------------------

    @classmethod
    def from_profile_file(
        cls,
        profile_path: str | Path,
        **kwargs: Any,
    ) -> FieldWeakeningCalibrator:
        """Create a calibrator from a motor profile JSON file."""
        data = json.loads(Path(profile_path).read_text())
        return cls(
            motor_params_dict=data["motor_params"],
            rated_info=data.get("rated_info", {}),
            profile_name=data.get("profile_name", Path(profile_path).stem),
            **kwargs,
        )

    @classmethod
    def from_session_and_profile(
        cls,
        profile_path: str | Path,
        session_path: str | Path,
        **kwargs: Any,
    ) -> tuple[FieldWeakeningCalibrator, dict[str, Any]]:
        """
        Create calibrator + extract best PI gains from a tuning-session JSON.

        Returns
        -------
        (calibrator, best_gains_dict)
            ``best_gains_dict`` contains keys:
            ``speed_kp, speed_ki, current_kp, current_ki, iq_limit_a``.
        """
        cal = cls.from_profile_file(profile_path, **kwargs)
        session = json.loads(Path(session_path).read_text())
        bc = session.get("best_candidate", {})
        gains = {
            "speed_kp": float(bc.get("speed_pi", {}).get("kp", 0.5)),
            "speed_ki": float(bc.get("speed_pi", {}).get("ki", 2.0)),
            "current_kp": float(
                bc.get("current_pi", {}).get("d_kp") or bc.get("current_pi", {}).get("kp", 0.15)
            ),
            "current_ki": float(
                bc.get("current_pi", {}).get("d_ki") or bc.get("current_pi", {}).get("ki", 20.0)
            ),
            "iq_limit_a": float(bc.get("iq_limit_a", 100.0)),
        }
        return cal, gains

    # ------------------------------------------------------------------
    # Physics computation
    # ------------------------------------------------------------------

    def compute_physics_params(self) -> FWPhysicsParams:
        """
        Derive field-weakening parameters analytically from motor data.

        Physics basis (Kim & Sul 1997, Bolognani 1999)
        ------------------------------------------------
        The FOCController uses ``vdq_limit = Vnom/√3`` (full inverter linear
        range, no 0.95 derating).  All analytical FW parameters are therefore
        derived against that same limit so the headroom equilibrium matches
        what the controller actually enforces.

        Feedforward Id formula (SPMSM, Ld = Lq = L)
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        At steady-state speed ωmech, with demagnetising current Id (< 0),
        the q-axis voltage is::

            Vq = R·Iq + ωe·L·Id + ωe·λpm·flux_ratio

        where ``flux_ratio = 1 + k_fw·Id`` (motor model convention).

        The effective total inductance for Id in the voltage equation is::

            L_eff = L + λpm·k_fw   [H]

        Setting |Vdq|² = Vdq_limit² and solving for Id gives the analytical
        feedforward::

            Id_ff(ω, Iq) = (√(Vlim² − (ωe·L·Iq)²) − ωe·λpm) / (ωe·L_eff)

        This expression is negative when ωe·λpm > Vlim (FW region) and
        converges smoothly to zero as speed drops to the no-FW limit.

        Headroom target
        ~~~~~~~~~~~~~~~
        A 5 % headroom (0.05 × Vdq_limit) gives the integrator room to react
        to transients while staying close to maximum voltage utilisation.  The
        FW gain is designed so the integrator reaches the equilibrium Id in
        ≈ 2 s starting from zero — slow enough to avoid interactions with the
        speed loop (bandwidth ≈ 20–50 rad/s) yet fast enough to track the
        speed ramp.

        Returns
        -------
        FWPhysicsParams
        """
        # ── Voltage limits ─────────────────────────────────────────────────
        # The FOCController sets vdq_limit = Vnom/√3 (no derating factor).
        # Use the same convention so analytical equilibria match the controller.
        Vdq_limit = self._Vnom / math.sqrt(3.0)  # e.g. 27.71 V for 48 V bus

        # Conservative physics bound (used only for ratio_required reporting)
        V_phase = Vdq_limit * 0.95

        # Flux linkage per electrical radian
        lam_pm = self._Ke / self._pp  # [Wb]

        # ── No-FW max speed ────────────────────────────────────────────────
        # At no-load (Iq = 0), the no-FW max speed is where Vq_back = Vdq_limit:
        #   Vdq_limit = ωe·λpm = pp·ωmech·λpm = Ke·ωmech
        omega_noFW = Vdq_limit / self._Ke  # mech rad/s (no-load, no FW)
        rpm_noFW = omega_noFW * 60.0 / (2.0 * math.pi)

        # ── Rated-speed voltage demand ─────────────────────────────────────
        omega_rated = self._N_rated * 2.0 * math.pi / 60.0
        Vback_rated = self._Ke * omega_rated  # back-EMF without FW

        fw_needed = Vback_rated > Vdq_limit

        if not fw_needed:
            return FWPhysicsParams(
                flux_weakening_id_coefficient=0.0,
                flux_weakening_min_ratio=0.95,
                fw_start_rpm=rpm_noFW * 0.95,
                fw_gain=1.0,
                fw_id_max_a=self._I_rated * self.fw_frac,
                fw_headroom_target_v=Vdq_limit * 0.05,
                no_fw_max_rpm=rpm_noFW,
                rated_speed_rpm=self._N_rated,
                ratio_required=1.0,
                fw_needed=False,
            )

        # ── Minimum flux ratio at rated speed ─────────────────────────────
        ratio_req = V_phase / Vback_rated  # < 1 → FW needed

        # ── Max FW Id (default 25 % of rated, increased if ratio needs more) ─
        fw_id_max = self.fw_frac * self._I_rated  # e.g. 25 A for ME1718

        # ── Motor model k_fw coefficient ──────────────────────────────────
        # flux_ratio at Id = -fw_id_max must reach ratio_req/safety:
        #   flux_ratio = 1 + k_fw·(-fw_id_max) = ratio_req / safety
        #   k_fw = (1 - ratio_req/safety) / fw_id_max
        k_fw = (1.0 - ratio_req / self.safety) / fw_id_max

        # Effective inductance for Id in the voltage equation (motor model):
        #   L_eff = L + λpm·k_fw
        L_eff = self._Ld + lam_pm * k_fw  # [H]

        # ── Feedforward Id at rated speed (no load, Iq = 0) ──────────────
        # This is the equilibrium Id the FW integrator should converge to at
        # rated speed under no load.  With Iq = 0:
        #   Id_ff = (Vdq_limit - ωe·λpm) / (ωe·L_eff)
        omega_e_rated = omega_rated * self._pp
        id_ff_rated_noload = (Vdq_limit - omega_e_rated * lam_pm) / (
            omega_e_rated * L_eff
        )  # negative value expected
        # Clip to within the rated FW budget (should be within fw_id_max)
        id_ff_rated_noload = float(np.clip(id_ff_rated_noload, -fw_id_max, 0.0))

        # Minimum flux ratio (allow 15 % extra below ratio_req for transients)
        min_ratio = max(ratio_req / self.safety * 0.85, 0.04)

        # ── FW start speed ────────────────────────────────────────────────
        fw_start = 0.90 * rpm_noFW

        # ── Headroom target ───────────────────────────────────────────────
        # 5 % of vdq_limit: leaves room for the integrator to respond to
        # dynamic changes without wasteful voltage under-utilisation.
        headroom_v = 0.05 * Vdq_limit

        # ── Integrator gain design ─────────────────────────────────────────
        # Target: reach equilibrium Id_ff within T_fw ≈ 2 s after FW onset.
        # When voltage saturates (headroom ≈ 0), headroom_error ≈ headroom_v.
        # Integrator: dId/dt = gain × headroom_error
        # → gain = |Id_ff_rated| / (headroom_v × T_fw)
        T_fw = 2.0  # target settling time [s]
        id_magnitude = max(abs(id_ff_rated_noload), 1.0)
        fw_gain = float(np.clip(id_magnitude / (headroom_v * T_fw), 0.5, 30.0))

        return FWPhysicsParams(
            flux_weakening_id_coefficient=float(k_fw),
            flux_weakening_min_ratio=float(min_ratio),
            fw_start_rpm=float(fw_start),
            fw_gain=float(fw_gain),
            fw_id_max_a=float(fw_id_max),
            fw_headroom_target_v=float(headroom_v),
            no_fw_max_rpm=float(rpm_noFW),
            rated_speed_rpm=float(self._N_rated),
            ratio_required=float(ratio_req),
            fw_needed=True,
        )

    # ------------------------------------------------------------------
    # Build MotorParameters with FW model coefficients
    # ------------------------------------------------------------------

    def _build_motor_params(self, physics: FWPhysicsParams):
        """Construct a MotorParameters object with k_fw applied."""
        from src.core.motor_model import MotorParameters

        return MotorParameters(
            model_type="dq",
            emf_shape="sinusoidal",
            nominal_voltage=self._Vnom,
            phase_resistance=self._R,
            phase_inductance=self._L,
            back_emf_constant=self._Ke,
            torque_constant=self._Kt,
            rotor_inertia=self._J,
            friction_coefficient=self._B,
            num_poles=self._pp * 2,
            poles_pairs=self._pp,
            ld=self._Ld,
            lq=self._Lq,
            flux_weakening_id_coefficient=physics.flux_weakening_id_coefficient,
            flux_weakening_min_ratio=physics.flux_weakening_min_ratio,
        )

    # ------------------------------------------------------------------
    # Safe simulation time-step (from current-loop bandwidth)
    # ------------------------------------------------------------------

    def _safe_dt(self) -> float:
        """
        Compute a numerically stable time-step from the current-loop bandwidth.

        Rule: dt ≤ π / (5 × ωc)  where  ωc = 30 × R/L.
        Clamped to [50 µs, 400 µs].
        """
        omega_c = 30.0 * self._R / self._L
        dt_ideal = math.pi / (5.0 * omega_c)
        return float(np.clip(dt_ideal, 5e-5, 4e-4))

    # ------------------------------------------------------------------
    # Main calibration entry-point
    # ------------------------------------------------------------------

    def calibrate(  # noqa: C901  # TODO: split into phase sub-methods to reduce complexity (27)
        self,
        speed_kp: float | None = None,
        speed_ki: float | None = None,
        current_kp: float | None = None,
        current_ki: float | None = None,
        iq_limit_a: float | None = None,
        gain_candidates: list[float] | None = None,
        fw_id_frac_candidates: list[float] | None = None,
    ) -> FWCalibrationResult:
        """
        Run the full field-weakening calibration.

        PI gains can be supplied explicitly (from a prior tuning session) or
        will be estimated analytically from motor parameters.

        Parameters
        ----------
        speed_kp, speed_ki : float, optional
            Speed PI gains.  Auto-estimated if not provided.
        current_kp, current_ki : float, optional
            Current PI gains.  Auto-estimated if not provided.
        iq_limit_a : float, optional
            Iq saturation limit.  Defaults to 2 × rated current.
        gain_candidates : list of float, optional
            FW controller gain values to sweep.  Default: [1.0, 2.0, 5.0, 10.0].
        fw_id_frac_candidates : list of float, optional
            Fractions of rated current to try as fw_id_max.
            Default: [0.20, 0.25, 0.30].

        Returns
        -------
        FWCalibrationResult
        """
        t_start = time.monotonic()

        # ── 1. Analytical FW parameters ──────────────────────────────
        physics = self.compute_physics_params()
        if self.verbose:
            print(f"\n{'─' * 64}")
            print(f"  Motor : {self.profile_name}")
            print(
                f"  Rated : {physics.rated_speed_rpm:.0f} RPM, "
                f"{self._T_rated:.1f} Nm, {self._I_rated:.0f} A"
            )
            print(f"  No-FW max speed : {physics.no_fw_max_rpm:.0f} RPM")
            print(f"  FW needed       : {physics.fw_needed}")
            if physics.fw_needed:
                print(f"  ratio_required  : {physics.ratio_required:.3f}")
                print(f"  k_fw (initial)  : {physics.flux_weakening_id_coefficient:.5f}")
                print(f"  fw_id_max       : {physics.fw_id_max_a:.1f} A")
                print(f"  fw_start        : {physics.fw_start_rpm:.0f} RPM")
                print(f"  fw_gain         : {physics.fw_gain:.2f}")
                print(f"  headroom_target : {physics.fw_headroom_target_v:.3f} V")
            print(f"{'─' * 64}")

        # ── 2. Fallback PI gains ──────────────────────────────────────
        omega_c = 30.0 * self._R / self._L
        speed_kp = speed_kp if speed_kp is not None else omega_c / 100.0 * self._J / self._Kt
        speed_ki = speed_ki if speed_ki is not None else omega_c / 100.0 * self._B / self._Kt * 10.0
        current_kp = current_kp if current_kp is not None else omega_c * self._L  # = 30·R
        current_ki = current_ki if current_ki is not None else omega_c * self._R  # = 30·R²/L
        iq_limit_a = iq_limit_a if iq_limit_a is not None else 2.0 * self._I_rated

        if self.verbose:
            print(f"  PI gains  speed: kp={speed_kp:.4f}, ki={speed_ki:.4f}")
            print(f"            curr : kp={current_kp:.4f}, ki={current_ki:.4f}")
            print(f"  Iq limit  : {iq_limit_a:.1f} A")

        # ── 3. Stable dt ─────────────────────────────────────────────
        dt = self._safe_dt()
        if self.verbose:
            print(f"  Safe dt   : {dt * 1e6:.0f} µs")

        # ── 4. If FW not needed → quick result ────────────────────────
        if not physics.fw_needed:
            pts = [
                FWValidationMetrics(
                    target_rpm=physics.rated_speed_rpm,
                    load_torque_nm=0.0,
                    achieved_rpm=physics.rated_speed_rpm,
                    speed_error_rpm=0.0,
                    speed_error_pct=0.0,
                    fw_injection_a=0.0,
                    id_a=0.0,
                    iq_a=self._T_rated / self._Kt,
                    efficiency_pct=95.0,
                    stable=True,
                    status="PASS",
                )
            ]
            return FWCalibrationResult(
                motor_profile_name=self.profile_name,
                motor_params_summary=self._params_summary(),
                physics_params=physics,
                selected_gain=physics.fw_gain,
                selected_fw_id_max_a=physics.fw_id_max_a,
                operating_points=pts,
                all_passed=True,
                best_achieved_rpm=physics.rated_speed_rpm,
                calibration_time_s=time.monotonic() - t_start,
                notes="Field weakening not required for this motor at rated voltage.",
            )

        # ── 5. Gain / fw_id_max sweep ─────────────────────────────────
        # Default gain sweep: covers fast (good tracking) to slow (no overshoot).
        # Anchor around the analytically designed gain from compute_physics_params.
        if gain_candidates is None:
            g0 = physics.fw_gain  # analytically designed gain
            gain_candidates = sorted(
                {
                    round(g0 * 0.25, 2),
                    round(g0 * 0.5, 2),
                    round(g0, 2),
                    round(g0 * 2.0, 2),
                    round(g0 * 4.0, 2),
                }
            )
        if fw_id_frac_candidates is None:
            fw_id_frac_candidates = [0.20, 0.25, 0.30]

        # In FW mode the d-axis and q-axis currents share the rated-current
        # budget: |Id|² + |Iq|² ≤ I_rated².  The session iq_limit (set for
        # transient no-FW operation, often 2-3× I_rated) is far too large in
        # FW mode: it lets the speed-PI command Iq values whose cross-coupling
        # term ωe·L·Iq dominates the d-axis voltage, consuming the entire
        # voltage vector and starving the q-axis of the voltage needed to
        # overcome back-EMF.
        # → Constrain Iq to the current-circle intersection with the largest
        #   fw_id candidate: Iq_max = √(I_rated² − fw_id_max²).
        fw_id_max_max = max(fw_id_frac_candidates) * self._I_rated
        iq_limit_fw = float(np.sqrt(max(self._I_rated**2 - fw_id_max_max**2, 1.0)))
        if self.verbose:
            print(
                f"  Iq limit (FW current-circle): {iq_limit_fw:.1f} A "
                f"(I_rated={self._I_rated:.0f} A, fw_id_max≤{fw_id_max_max:.0f} A)"
            )

        best_params: tuple[float, float, float] | None = None  # (gain, fw_id_max, fr)
        best_score = float("inf")
        best_metrics: dict[str, Any] | None = None

        # We sweep at the rated speed with light load (30% rated torque)
        # to find parameters that reliably reach rated speed
        test_speed = physics.rated_speed_rpm
        test_torque = self._T_rated * 0.30

        motor_params = self._build_motor_params(physics)  # starts with analytical k_fw

        if self.verbose:
            total = len(gain_candidates) * len(fw_id_frac_candidates)
            print(
                f"\n  Sweeping {total} FW gain × id_max combinations "
                f"at {test_speed:.0f} RPM, {test_torque:.1f} Nm load …"
            )

        # flux_ratio at rated speed with max FW — needed to scale speed PI gains
        ratio_req = physics.ratio_required  # minimum flux ratio required

        for fw_id_frac in fw_id_frac_candidates:
            fw_id_max = fw_id_frac * self._I_rated
            # Re-compute k_fw for this fw_id_max so the motor model scales correctly
            k_fw_adj = (1.0 - ratio_req / self.safety) / fw_id_max
            motor_params.flux_weakening_id_coefficient = float(k_fw_adj)
            # Effective flux ratio at rated speed with this Id_max
            fr_at_rated = float(
                np.clip(1.0 + k_fw_adj * (-fw_id_max), physics.flux_weakening_min_ratio, 1.0)
            )

            for gain in gain_candidates:
                # Iq limit for this specific fw_id_max (current circle)
                iq_lim_this = float(np.sqrt(max(self._I_rated**2 - fw_id_max**2, 1.0)))
                metrics = _run_fw_simulation(
                    params=motor_params,
                    speed_kp=speed_kp,
                    speed_ki=speed_ki,
                    cur_kp=current_kp,
                    cur_ki=current_ki,
                    iq_limit_a=iq_lim_this,
                    target_rpm=test_speed,
                    load_torque_nm=test_torque,
                    fw_start_rpm=physics.fw_start_rpm,
                    fw_gain=gain,
                    fw_id_max_a=fw_id_max,
                    fw_headroom_v=physics.fw_headroom_target_v,
                    dt=dt,
                    sim_duration=self.sim_duration,
                    flux_ratio_at_target=fr_at_rated,
                )

                if metrics is None:
                    if self.verbose:
                        print(f"    gain={gain:5.1f}  fw_id={fw_id_max:5.1f}A  → UNSTABLE")
                    continue

                speed_err_pct = abs(metrics["speed_err_pct"])
                stable_ok = metrics["speed_mean"] > 0.0
                # Score: penalise speed error heavily; reward efficiency
                score_val = speed_err_pct * 10.0 + max(0.0, 50.0 - metrics["efficiency"])

                if self.verbose:
                    print(
                        f"    gain={gain:5.1f}  fw_id={fw_id_max:5.1f}A  "
                        f"→ {metrics['speed_mean']:7.1f} RPM  "
                        f"err={metrics['speed_err_pct']:+6.2f}%  "
                        f"eff={metrics['efficiency']:5.1f}%  "
                        f"Id={metrics['id_mean']:+6.2f}A  "
                        f"FW={metrics['fw_injection']:+6.2f}A  "
                        f"score={score_val:.2f}"
                    )

                if stable_ok and score_val < best_score:
                    best_score = score_val
                    best_params = (gain, fw_id_max, fr_at_rated)
                    best_metrics = metrics
                    # Update the k_fw in physics for reporting
                    physics = FWPhysicsParams(
                        flux_weakening_id_coefficient=float(k_fw_adj),
                        flux_weakening_min_ratio=float(max(ratio_req / self.safety * 0.85, 0.04)),
                        fw_start_rpm=physics.fw_start_rpm,
                        fw_gain=gain,
                        fw_id_max_a=fw_id_max,
                        fw_headroom_target_v=physics.fw_headroom_target_v,
                        no_fw_max_rpm=physics.no_fw_max_rpm,
                        rated_speed_rpm=physics.rated_speed_rpm,
                        ratio_required=ratio_req,
                        fw_needed=True,
                    )

        # ── 6. If no candidate passed → report failure ─────────────────
        if best_params is None or best_metrics is None:
            elapsed = time.monotonic() - t_start
            note = (
                "No FW parameter combination produced stable operation at rated speed. "
                "Consider checking motor parameters or increasing sim_duration."
            )
            if self.verbose:
                print(f"\n  ⚠  {note}")
            return FWCalibrationResult(
                motor_profile_name=self.profile_name,
                motor_params_summary=self._params_summary(),
                physics_params=physics,
                selected_gain=gain_candidates[0],
                selected_fw_id_max_a=fw_id_frac_candidates[0] * self._I_rated,
                operating_points=[],
                all_passed=False,
                best_achieved_rpm=0.0,
                calibration_time_s=elapsed,
                notes=note,
            )

        sel_gain, sel_id_max, sel_flux_ratio = best_params

        # ── 7. Multi-point validation with best parameters ────────────
        motor_params.flux_weakening_id_coefficient = physics.flux_weakening_id_coefficient
        motor_params.flux_weakening_min_ratio = physics.flux_weakening_min_ratio

        if self.verbose:
            print(
                f"\n  Best  gain={sel_gain}, fw_id_max={sel_id_max:.1f} A — "
                f"validating at multiple operating points …"
            )

        # Validation operating points:
        #   • 80 % rated speed, 50 % rated torque  (below FW, sanity check)
        #   • 100 % rated speed, 30 % rated torque  (FW region, light load)
        #   • 100 % rated speed, 60 % rated torque  (FW region, moderate load)
        #   • 100 % rated speed, 0 % load           (FW region, free run)
        val_points = [
            (physics.no_fw_max_rpm * 0.95, self._T_rated * 0.50, "pre-FW 95% no-FW speed"),
            (physics.rated_speed_rpm, 0.0, "rated speed, no load"),
            (physics.rated_speed_rpm, self._T_rated * 0.30, "rated speed, 30% load"),
            (physics.rated_speed_rpm, self._T_rated * 0.60, "rated speed, 60% load"),
        ]

        op_results: list[FWValidationMetrics] = []
        all_ok = True

        for tgt_rpm, tgt_torq, label in val_points:
            if self.verbose:
                print(
                    f"    [{label}]  target={tgt_rpm:.0f} RPM, load={tgt_torq:.1f} Nm …",
                    end=" ",
                    flush=True,
                )

            # For the pre-FW point use nominal flux ratio and full Iq limit
            in_fw = tgt_rpm > physics.no_fw_max_rpm * 1.05
            pt_flux_ratio = sel_flux_ratio if in_fw else 1.0
            pt_iq_limit = iq_limit_fw if in_fw else iq_limit_a

            m = _run_fw_simulation(
                params=motor_params,
                speed_kp=speed_kp,
                speed_ki=speed_ki,
                cur_kp=current_kp,
                cur_ki=current_ki,
                iq_limit_a=pt_iq_limit,
                target_rpm=tgt_rpm,
                load_torque_nm=tgt_torq,
                fw_start_rpm=physics.fw_start_rpm,
                fw_gain=sel_gain,
                fw_id_max_a=sel_id_max,
                fw_headroom_v=physics.fw_headroom_target_v,
                dt=dt,
                sim_duration=self.sim_duration,
                flux_ratio_at_target=pt_flux_ratio,
            )

            if m is None:
                status = "UNSTABLE"
                all_ok = False
                vm = FWValidationMetrics(
                    target_rpm=tgt_rpm,
                    load_torque_nm=tgt_torq,
                    achieved_rpm=0.0,
                    speed_error_rpm=tgt_rpm,
                    speed_error_pct=100.0,
                    fw_injection_a=0.0,
                    id_a=0.0,
                    iq_a=0.0,
                    efficiency_pct=0.0,
                    stable=False,
                    status="UNSTABLE",
                )
            else:
                pass_speed = abs(m["speed_err_pct"]) < 5.0
                # In FW region efficiency is inherently lower due to reactive
                # demagnetising current → only check efficiency for no-load points
                # above the no-FW speed where mechanical power is meaningful.
                in_fw_region = tgt_rpm > physics.no_fw_max_rpm * 1.05
                if tgt_torq < 0.01 or in_fw_region:
                    pass_eff = True  # don't penalise FW operating points
                else:
                    pass_eff = m["efficiency"] > 60.0
                status = "PASS" if (pass_speed and pass_eff) else "FAIL"
                if status != "PASS":
                    all_ok = False
                vm = FWValidationMetrics(
                    target_rpm=tgt_rpm,
                    load_torque_nm=tgt_torq,
                    achieved_rpm=m["speed_mean"],
                    speed_error_rpm=m["speed_err"],
                    speed_error_pct=m["speed_err_pct"],
                    fw_injection_a=m["fw_injection"],
                    id_a=m["id_mean"],
                    iq_a=m["iq_mean"],
                    efficiency_pct=m["efficiency"],
                    stable=True,
                    status=status,
                )

            op_results.append(vm)
            if self.verbose:
                if status == "PASS":
                    print(f"✓ {vm.achieved_rpm:.0f} RPM  err={vm.speed_error_pct:+.2f}%")
                else:
                    print(
                        f"✗ {vm.achieved_rpm:.0f} RPM  err={vm.speed_error_pct:+.2f}%  [{status}]"
                    )

        best_rpm = max((v.achieved_rpm for v in op_results), default=0.0)

        elapsed = time.monotonic() - t_start
        notes = (
            f"Sweep over {len(gain_candidates) * len(fw_id_frac_candidates)} combinations; "
            f"best gain={sel_gain}, fw_id_max={sel_id_max:.1f} A, "
            f"k_fw={physics.flux_weakening_id_coefficient:.5f}."
        )

        if self.verbose:
            status_str = "✅ ALL PASS" if all_ok else "⚠  PARTIAL"
            print(f"\n  {status_str}  — best achieved: {best_rpm:.0f} RPM ({elapsed:.1f} s)")

        return FWCalibrationResult(
            motor_profile_name=self.profile_name,
            motor_params_summary=self._params_summary(),
            physics_params=physics,
            selected_gain=sel_gain,
            selected_fw_id_max_a=sel_id_max,
            operating_points=op_results,
            all_passed=all_ok,
            best_achieved_rpm=best_rpm,
            calibration_time_s=elapsed,
            notes=notes,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _params_summary(self) -> dict[str, Any]:
        return {
            "R": self._R,
            "L": self._L,
            "Ke": self._Ke,
            "Kt": self._Kt,
            "J": self._J,
            "pp": self._pp,
            "Vnom": self._Vnom,
            "N_rated": self._N_rated,
            "I_rated": self._I_rated,
            "T_rated": self._T_rated,
        }

    def print_physics_summary(self) -> None:  # pragma: no cover
        """Pretty-print the physics-based FW parameter analysis."""
        p = self.compute_physics_params()
        V_ph = self._Vnom / math.sqrt(3.0)
        print(f"\n{'=' * 64}")
        print(f"  Field-Weakening Physics Analysis: {self.profile_name}")
        print(f"{'=' * 64}")
        print(f"  Supply: {self._Vnom:.0f} V DC  →  Vphase = {V_ph:.2f} V")
        print(f"  Ke = {self._Ke:.4f} V·s/rad(mech),  pp = {self._pp}")
        print(f"  Rated speed: {self._N_rated:.0f} RPM")
        print(f"  Back-EMF at rated: {self._Ke * self._N_rated * 2 * math.pi / 60:.2f} V")
        print(f"  No-FW max speed  : {p.no_fw_max_rpm:.0f} RPM")
        print(f"  FW needed        : {p.fw_needed}")
        if p.fw_needed:
            print(
                f"  Flux ratio needed: {p.ratio_required:.3f}  "
                f"(factor {1 / p.ratio_required:.2f}× reduction)"
            )
            print(f"  k_fw             : {p.flux_weakening_id_coefficient:.5f} A⁻¹")
            print(
                f"  fw_id_max        : {p.fw_id_max_a:.1f} A  "
                f"({p.fw_id_max_a / self._I_rated * 100:.0f}% of I_rated)"
            )
            print(f"  fw_start_rpm     : {p.fw_start_rpm:.0f} RPM")
            print(f"  fw_gain          : {p.fw_gain:.2f} A/(V·s)")
            print(f"  headroom_target  : {p.fw_headroom_target_v:.3f} V")
        print(f"{'=' * 64}\n")
