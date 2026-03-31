"""Regression baseline utilities for deterministic BLDC scenario checks.

Phase-1 research-grade workflow:
- run a fixed set of reference scenarios
- compute KPIs consistently
- compare against a frozen baseline JSON
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TypedDict, cast

import numpy as np

from src.control import FOCController, SVMGenerator, VFController
from src.core import (
    BLDCMotor,
    ConstantLoad,
    ConstantSupply,
    MotorParameters,
    RampLoad,
    RampSupply,
    SimulationEngine,
)
from src.utils.config import DEFAULT_MOTOR_PARAMS, SIMULATION_PARAMS

BASELINE_SCHEMA_VERSION = 1


class DriftRow(TypedDict):
    """One KPI comparison row used in regression drift reporting."""

    scenario: str
    kpi: str
    expected: float
    actual: float
    delta_abs: float
    delta_pct: float
    allowed_abs: float
    tolerance_pct: float
    status: str


class ThresholdRow(TypedDict):
    """One threshold evaluation row for startup-transition diagnostics."""

    kpi: str
    actual: float
    warn_min: float | None
    warn_max: float | None
    fail_min: float | None
    fail_max: float | None
    status: str


# Conservative defaults intended to prevent severe regressions while avoiding
# fragile failures from small numeric drift across environments.
DEFAULT_STARTUP_TRANSITION_THRESHOLDS: dict[str, dict[str, float]] = {
    "startup_handoff_count": {"fail_min": 1.0, "warn_min": 1.0},
    "startup_fallback_event_count": {"fail_min": 1.0, "warn_min": 1.0},
    "startup_last_handoff_confidence": {"fail_min": 0.10, "warn_min": 0.20},
    "startup_handoff_confidence_peak": {"fail_min": 0.10, "warn_min": 0.20},
    "startup_handoff_quality": {"fail_min": 0.05, "warn_min": 0.20},
    "startup_handoff_stability_ratio": {"fail_min": 0.05, "warn_min": 0.20},
    "observer_confidence_above_threshold_time_s": {
        "fail_min": 0.001,
        "warn_min": 0.003,
    },
    "observer_confidence_below_threshold_time_s": {
        "fail_min": 0.001,
        "warn_min": 0.003,
    },
    "observer_confidence_crossings_up": {"fail_min": 1.0, "warn_min": 1.0},
    "observer_confidence_crossings_down": {"fail_min": 1.0, "warn_min": 1.0},
}


@dataclass(frozen=True)
class Scenario:
    """Reference simulation scenario used for KPI baselines."""

    name: str
    duration_s: float
    speed_ref_hz: float
    load_kind: str
    load_params: dict[str, float]
    supply_kind: str
    supply_params: dict[str, float]


@dataclass(frozen=True)
class FOCScenario:
    """Reference simulation scenario used for FOC KPI baselines."""

    name: str
    duration_s: float
    speed_ref_rpm: float
    id_ref_a: float
    iq_ref_a: float
    use_concordia: bool
    load_kind: str
    load_params: dict[str, float]
    supply_kind: str
    supply_params: dict[str, float]


def _default_motor() -> BLDCMotor:
    dt = cast(float, SIMULATION_PARAMS["dt"])
    params = MotorParameters(
        nominal_voltage=DEFAULT_MOTOR_PARAMS["nominal_voltage"],
        phase_resistance=DEFAULT_MOTOR_PARAMS["phase_resistance"],
        phase_inductance=DEFAULT_MOTOR_PARAMS["phase_inductance"],
        back_emf_constant=DEFAULT_MOTOR_PARAMS["back_emf_constant"],
        torque_constant=DEFAULT_MOTOR_PARAMS["torque_constant"],
        rotor_inertia=DEFAULT_MOTOR_PARAMS["rotor_inertia"],
        friction_coefficient=DEFAULT_MOTOR_PARAMS["friction_coefficient"],
        num_poles=int(DEFAULT_MOTOR_PARAMS["num_poles"]),
        poles_pairs=int(DEFAULT_MOTOR_PARAMS["poles_pairs"]),
    )
    return BLDCMotor(parameters=params, dt=dt)


def _make_load(kind: str, params: dict[str, float]):
    if kind == "constant":
        return ConstantLoad(torque=float(params["torque"]))
    if kind == "ramp":
        return RampLoad(
            initial=float(params["initial"]),
            final=float(params["final"]),
            duration=float(params["duration"]),
        )
    raise ValueError(f"Unsupported load kind: {kind}")


def _make_supply(kind: str, params: dict[str, float]):
    if kind == "constant":
        return ConstantSupply(voltage=float(params["voltage"]))
    if kind == "ramp":
        return RampSupply(
            initial=float(params["initial"]),
            final=float(params["final"]),
            duration=float(params["duration"]),
        )
    raise ValueError(f"Unsupported supply kind: {kind}")


def _compute_kpis(history: dict[str, np.ndarray]) -> dict[str, float]:
    speed = history["speed"]
    currents = np.vstack(
        [
            np.abs(history["currents_a"]),
            np.abs(history["currents_b"]),
            np.abs(history["currents_c"]),
        ]
    )
    torque = history["torque"]
    load_torque = history["load_torque"]
    supply = history.get("supply_voltage", np.zeros_like(speed))

    if speed.size == 0:
        return {
            "final_speed_rpm": 0.0,
            "peak_speed_rpm": 0.0,
            "peak_phase_current_a": 0.0,
            "steady_state_speed_std_rpm": 0.0,
            "torque_ripple_std_nm": 0.0,
            "mean_supply_voltage_v": 0.0,
            "mean_load_torque_nm": 0.0,
        }

    tail = max(10, int(0.1 * speed.size))
    speed_tail = speed[-tail:]
    torque_tail = torque[-tail:]

    return {
        "final_speed_rpm": float(np.mean(speed_tail)),
        "peak_speed_rpm": float(np.max(speed)),
        "peak_phase_current_a": float(np.max(currents)),
        "steady_state_speed_std_rpm": float(np.std(speed_tail)),
        "torque_ripple_std_nm": float(np.std(torque_tail)),
        "mean_supply_voltage_v": float(np.mean(supply)),
        "mean_load_torque_nm": float(np.mean(load_torque)),
    }


def run_scenario(s: Scenario) -> dict[str, float]:
    """Run a deterministic V/f scenario and return computed KPIs."""
    motor = _default_motor()
    load = _make_load(s.load_kind, s.load_params)
    supply = _make_supply(s.supply_kind, s.supply_params)

    dt = cast(float, SIMULATION_PARAMS["dt"])
    max_history = cast(int, SIMULATION_PARAMS["max_history"])
    dc_voltage = cast(float, SIMULATION_PARAMS["dc_voltage"])
    engine = SimulationEngine(
        motor=motor,
        load_profile=load,
        dt=dt,
        max_history=max_history,
        supply_profile=supply,
    )

    controller = VFController(
        v_nominal=DEFAULT_MOTOR_PARAMS["nominal_voltage"],
        f_nominal=100.0,
        dc_voltage=float(s.supply_params.get("voltage", dc_voltage)),
        v_startup=1.0,
    )
    controller.set_frequency_slew_rate(50.0)
    controller.set_speed_reference(s.speed_ref_hz)

    svm = SVMGenerator(dc_voltage=dc_voltage)

    num_steps = int(s.duration_s / engine.dt)
    for _ in range(num_steps):
        mag, angle = controller.update(engine.dt)
        svm.set_dc_voltage(engine.supply_profile.get_voltage(engine.time))
        v_abc = svm.modulate(mag, angle)
        engine.step(v_abc, log_data=True)

    return _compute_kpis(engine.get_history())


def reference_scenarios() -> list[Scenario]:
    """Return the fixed scenario set used for baseline regression checks."""
    return [
        Scenario(
            name="no_load_spinup",
            duration_s=2.0,
            speed_ref_hz=80.0,
            load_kind="constant",
            load_params={"torque": 0.0},
            supply_kind="constant",
            supply_params={"voltage": 48.0},
        ),
        Scenario(
            name="constant_load",
            duration_s=2.5,
            speed_ref_hz=80.0,
            load_kind="constant",
            load_params={"torque": 1.0},
            supply_kind="constant",
            supply_params={"voltage": 48.0},
        ),
        Scenario(
            name="ramp_load",
            duration_s=3.0,
            speed_ref_hz=90.0,
            load_kind="ramp",
            load_params={"initial": 0.0, "final": 2.0, "duration": 1.5},
            supply_kind="constant",
            supply_params={"voltage": 48.0},
        ),
        Scenario(
            name="supply_sag",
            duration_s=3.0,
            speed_ref_hz=90.0,
            load_kind="constant",
            load_params={"torque": 0.8},
            supply_kind="ramp",
            supply_params={"initial": 48.0, "final": 36.0, "duration": 2.0},
        ),
    ]


def run_reference_suite() -> dict[str, dict[str, float]]:
    """Run all reference scenarios and return KPI dictionary keyed by scenario name."""
    return {scenario.name: run_scenario(scenario) for scenario in reference_scenarios()}


def run_foc_scenario(s: FOCScenario) -> dict[str, float]:
    """Run a deterministic FOC scenario and return computed KPIs."""
    motor = _default_motor()
    load = _make_load(s.load_kind, s.load_params)
    supply = _make_supply(s.supply_kind, s.supply_params)

    dt = cast(float, SIMULATION_PARAMS["dt"])
    max_history = cast(int, SIMULATION_PARAMS["max_history"])
    dc_voltage = cast(float, SIMULATION_PARAMS["dc_voltage"])
    engine = SimulationEngine(
        motor=motor,
        load_profile=load,
        dt=dt,
        max_history=max_history,
        supply_profile=supply,
    )

    controller = FOCController(
        motor=motor,
        use_concordia=s.use_concordia,
        output_cartesian=False,
    )
    controller.set_current_references(id_ref=s.id_ref_a, iq_ref=s.iq_ref_a)
    controller.set_speed_reference(s.speed_ref_rpm)

    svm = SVMGenerator(dc_voltage=dc_voltage)

    num_steps = int(s.duration_s / engine.dt)
    for _ in range(num_steps):
        mag, angle = controller.update(engine.dt)
        svm.set_dc_voltage(engine.supply_profile.get_voltage(engine.time))
        v_abc = svm.modulate(mag, angle)
        engine.step(v_abc, log_data=True)

    return _compute_kpis(engine.get_history())


def reference_foc_scenarios() -> list[FOCScenario]:
    """Return the fixed FOC scenario set used for baseline regression checks."""
    return [
        FOCScenario(
            name="foc_constant_load_clarke",
            duration_s=2.0,
            speed_ref_rpm=1200.0,
            id_ref_a=0.0,
            iq_ref_a=0.5,
            use_concordia=False,
            load_kind="constant",
            load_params={"torque": 0.4},
            supply_kind="constant",
            supply_params={"voltage": 48.0},
        ),
        FOCScenario(
            name="foc_ramp_load_clarke",
            duration_s=2.5,
            speed_ref_rpm=1500.0,
            id_ref_a=0.0,
            iq_ref_a=0.8,
            use_concordia=False,
            load_kind="ramp",
            load_params={"initial": 0.2, "final": 1.2, "duration": 1.0},
            supply_kind="constant",
            supply_params={"voltage": 48.0},
        ),
        FOCScenario(
            name="foc_supply_sag_concordia",
            duration_s=2.5,
            speed_ref_rpm=1500.0,
            id_ref_a=0.0,
            iq_ref_a=0.7,
            use_concordia=True,
            load_kind="constant",
            load_params={"torque": 0.6},
            supply_kind="ramp",
            supply_params={"initial": 48.0, "final": 38.0, "duration": 1.5},
        ),
    ]


def run_foc_reference_suite() -> dict[str, dict[str, float]]:
    """Run all FOC reference scenarios and return KPI dictionary keyed by scenario name."""
    return {scenario.name: run_foc_scenario(scenario) for scenario in reference_foc_scenarios()}


def run_foc_startup_transition_diagnostics() -> dict[str, float]:
    """Run a deterministic startup-transition exercise and return reliability KPIs.

    This is used for CI/report artifacts so transition quality and stability
    trends can be reviewed release-to-release.
    """
    motor = _default_motor()
    ctrl = FOCController(motor=motor)
    ctrl.set_angle_observer("PLL")
    ctrl.set_startup_transition(
        enabled=True,
        initial_mode="Measured",
        min_speed_rpm=100.0,
        min_elapsed_s=0.005,
        min_emf_v=0.05,
        min_confidence=0.2,
        confidence_hold_s=0.002,
        confidence_hysteresis=0.1,
        fallback_enabled=True,
        fallback_hold_s=0.005,
    )

    # Phase A: satisfy handoff conditions.
    motor.state[3] = 95.0
    motor.state[4] = 0.55
    for _ in range(30):
        motor._last_emf = motor._calculate_back_emf(motor.theta)
        ctrl.update(0.001)

    # Phase B: degrade conditions and trigger fallback.
    motor.state[3] = 2.0
    motor.state[4] = 0.05
    for _ in range(20):
        motor._last_emf = motor._calculate_back_emf(motor.theta)
        ctrl.update(0.001)

    s = ctrl.get_state()
    keys = [
        "startup_handoff_count",
        "startup_fallback_event_count",
        "startup_last_handoff_time_s",
        "startup_last_handoff_confidence",
        "startup_handoff_confidence_peak",
        "startup_handoff_quality",
        "startup_handoff_stability_ratio",
        "observer_confidence_above_threshold_time_s",
        "observer_confidence_below_threshold_time_s",
        "observer_confidence_crossings_up",
        "observer_confidence_crossings_down",
    ]
    return {k: float(s[k]) for k in keys}


def save_baseline(output_path: Path, tolerances: dict[str, dict[str, float]] | None = None) -> Path:
    """Run suite and persist a baseline JSON file."""
    results = run_reference_suite()

    payload: dict[str, Any] = {
        "schema_version": BASELINE_SCHEMA_VERSION,
        "sim_dt": SIMULATION_PARAMS["dt"],
        "scenarios": results,
    }
    if tolerances:
        payload["tolerances"] = tolerances

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return output_path


def save_foc_baseline(
    output_path: Path, tolerances: dict[str, dict[str, float]] | None = None
) -> Path:
    """Run FOC suite and persist a baseline JSON file."""
    results = run_foc_reference_suite()

    payload: dict[str, Any] = {
        "schema_version": BASELINE_SCHEMA_VERSION,
        "sim_dt": SIMULATION_PARAMS["dt"],
        "scenarios": results,
    }
    if tolerances:
        payload["tolerances"] = tolerances

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return output_path


def load_baseline(path: Path) -> dict[str, Any]:
    """Load baseline JSON file."""
    return cast(dict[str, Any], json.loads(path.read_text(encoding="utf-8")))


def build_drift_report(
    current: dict[str, dict[str, float]], baseline_payload: dict[str, Any]
) -> list[DriftRow]:
    """Build row-wise KPI drift report against baseline.

    Status values:
    - pass: drift is within configured tolerance
    - fail: drift exceeds configured tolerance
    - missing: scenario or KPI not found in current run
    """
    rows: list[DriftRow] = []

    baseline = baseline_payload.get("scenarios", {})
    tolerances = baseline_payload.get("tolerances", {})
    min_abs_tolerance = float(baseline_payload.get("min_abs_tolerance", 1e-3))

    for scenario_name, baseline_kpis in baseline.items():
        if scenario_name not in current:
            rows.append(
                DriftRow(
                    scenario=scenario_name,
                    kpi="*scenario*",
                    expected=0.0,
                    actual=0.0,
                    delta_abs=0.0,
                    delta_pct=0.0,
                    allowed_abs=0.0,
                    tolerance_pct=0.0,
                    status="missing",
                )
            )
            continue

        current_kpis = current[scenario_name]
        scenario_tols = tolerances.get(scenario_name, {})

        for kpi_name, expected in baseline_kpis.items():
            if kpi_name not in current_kpis:
                rows.append(
                    DriftRow(
                        scenario=scenario_name,
                        kpi=kpi_name,
                        expected=float(expected),
                        actual=0.0,
                        delta_abs=0.0,
                        delta_pct=0.0,
                        allowed_abs=0.0,
                        tolerance_pct=float(scenario_tols.get(kpi_name, 0.10)),
                        status="missing",
                    )
                )
                continue

            actual = float(current_kpis[kpi_name])
            expected = float(expected)
            tol = float(scenario_tols.get(kpi_name, 0.10))
            delta = abs(actual - expected)
            allowed = max(min_abs_tolerance, abs(expected) * tol)

            if abs(expected) > 1e-12:
                delta_pct = (delta / abs(expected)) * 100.0
            else:
                delta_pct = 0.0 if delta <= 1e-12 else float("inf")

            rows.append(
                DriftRow(
                    scenario=scenario_name,
                    kpi=kpi_name,
                    expected=expected,
                    actual=actual,
                    delta_abs=delta,
                    delta_pct=delta_pct,
                    allowed_abs=allowed,
                    tolerance_pct=tol,
                    status="pass" if delta <= allowed else "fail",
                )
            )

    return rows


def format_drift_report(rows: list[DriftRow], failed_only: bool = False) -> str:
    """Format drift rows into a compact text table for logs and test errors."""
    if failed_only:
        rows = [r for r in rows if r["status"] in ("fail", "missing")]

    if not rows:
        return "No drift rows to report."

    lines = [
        "scenario | kpi | expected | actual | delta | delta% | tol% | status",
        "-" * 92,
    ]
    for r in rows:
        delta_pct_str = f"{r['delta_pct']:.2f}" if np.isfinite(r["delta_pct"]) else "inf"
        lines.append(
            f"{r['scenario']} | {r['kpi']} | {r['expected']:.6g} | {r['actual']:.6g} | "
            f"{r['delta_abs']:.6g} | {delta_pct_str} | {r['tolerance_pct'] * 100:.2f} | {r['status']}"  # noqa: E501
        )

    return "\n".join(lines)


def compare_to_baseline(
    current: dict[str, dict[str, float]],
    baseline_payload: dict[str, Any],
) -> list[str]:
    """Compare current KPI set to baseline payload and return violation messages."""
    failures: list[str] = []

    rows = build_drift_report(current=current, baseline_payload=baseline_payload)

    for row in rows:
        if row["status"] == "missing":
            if row["kpi"] == "*scenario*":
                failures.append(f"Missing scenario result: {row['scenario']}")
            else:
                failures.append(f"{row['scenario']}: missing KPI {row['kpi']}")
        elif row["status"] == "fail":
            failures.append(
                f"{row['scenario']}:{row['kpi']} baseline={row['expected']:.6g}, "
                f"actual={row['actual']:.6g}, delta={row['delta_abs']:.6g} "
                f"({row['delta_pct']:.2f}%) exceeds tol={row['tolerance_pct']:.2%}"
            )

    return failures


def _threshold_bounds(
    threshold_spec: dict[str, float] | None,
) -> tuple[float | None, float | None, float | None, float | None]:
    spec = threshold_spec or {}
    return (
        float(spec["warn_min"]) if "warn_min" in spec else None,
        float(spec["warn_max"]) if "warn_max" in spec else None,
        float(spec["fail_min"]) if "fail_min" in spec else None,
        float(spec["fail_max"]) if "fail_max" in spec else None,
    )


def evaluate_startup_transition_thresholds(
    diagnostics: dict[str, float],
    thresholds: dict[str, dict[str, float]] | None = None,
) -> list[ThresholdRow]:
    """Evaluate startup-transition diagnostics against warn/fail thresholds."""
    spec = thresholds or DEFAULT_STARTUP_TRANSITION_THRESHOLDS
    rows: list[ThresholdRow] = []

    for kpi in sorted(spec.keys()):
        actual = float(diagnostics.get(kpi, float("nan")))
        warn_min, warn_max, fail_min, fail_max = _threshold_bounds(spec.get(kpi))

        status = "pass"
        if (
            not np.isfinite(actual)
            or (fail_min is not None and actual < fail_min)
            or (fail_max is not None and actual > fail_max)
        ):
            status = "fail"
        elif (warn_min is not None and actual < warn_min) or (
            warn_max is not None and actual > warn_max
        ):
            status = "warn"

        rows.append(
            ThresholdRow(
                kpi=kpi,
                actual=actual,
                warn_min=warn_min,
                warn_max=warn_max,
                fail_min=fail_min,
                fail_max=fail_max,
                status=status,
            )
        )

    return rows


def format_startup_threshold_report(rows: list[ThresholdRow]) -> str:
    """Format startup threshold rows as a compact text table."""
    if not rows:
        return "No threshold rows to report."

    def _bound_str(v: float | None) -> str:
        return "-" if v is None else f"{v:.6g}"

    lines = [
        "kpi | actual | warn_min | warn_max | fail_min | fail_max | status",
        "-" * 88,
    ]
    for row in rows:
        lines.append(
            f"{row['kpi']} | {row['actual']:.6g} | "
            f"{_bound_str(row['warn_min'])} | {_bound_str(row['warn_max'])} | "
            f"{_bound_str(row['fail_min'])} | {_bound_str(row['fail_max'])} | {row['status']}"
        )

    return "\n".join(lines)
