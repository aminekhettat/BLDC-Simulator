"""
Atomic features tested in this module:
- baseline files exist and non empty
- baseline kpi shape and types
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, cast

BASELINES = [
    Path("tests/baselines/reference_baseline.json"),
    Path("tests/baselines/foc_reference_baseline.json"),
]

REQUIRED_KPIS = {
    "final_speed_rpm",
    "peak_speed_rpm",
    "peak_phase_current_a",
    "steady_state_speed_std_rpm",
    "torque_ripple_std_nm",
    "mean_supply_voltage_v",
    "mean_load_torque_nm",
}


def _load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(payload, dict)
    return cast(dict[str, Any], payload)


def test_baseline_files_exist_and_non_empty():
    for path in BASELINES:
        assert path.exists(), f"Missing baseline file: {path}"

        payload = _load_json(path)
        assert isinstance(payload, dict), f"Baseline is not a JSON object: {path}"
        assert "schema_version" in payload, f"Missing schema_version in {path}"
        assert "sim_dt" in payload, f"Missing sim_dt in {path}"
        assert "scenarios" in payload, f"Missing scenarios in {path}"

        scenarios = payload["scenarios"]
        assert isinstance(scenarios, dict), f"scenarios must be an object in {path}"
        assert scenarios, f"scenarios must not be empty in {path}"


def test_baseline_kpi_shape_and_types():
    for path in BASELINES:
        payload = _load_json(path)
        scenarios = payload["scenarios"]

        for scenario_name, kpis in scenarios.items():
            assert isinstance(kpis, dict), f"{path}:{scenario_name} KPI block must be object"

            missing = REQUIRED_KPIS - set(kpis.keys())
            assert not missing, f"{path}:{scenario_name} missing KPIs: {sorted(missing)}"

            for kpi_name in REQUIRED_KPIS:
                val = kpis[kpi_name]
                assert isinstance(val, (int, float)), (
                    f"{path}:{scenario_name}:{kpi_name} must be numeric"
                )
