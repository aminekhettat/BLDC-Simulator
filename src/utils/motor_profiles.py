"""Motor profile persistence helpers.

Provides import/export utilities for BLDC/PMSM motor parameter profiles.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from src.utils.config import DEFAULT_MOTOR_PARAMS, MOTOR_PROFILES_DIR

PROFILE_SCHEMA = "bldc.motor_profile.v1"

_REQUIRED_PARAM_KEYS = (
    "nominal_voltage",
    "phase_resistance",
    "phase_inductance",
    "back_emf_constant",
    "torque_constant",
    "rotor_inertia",
    "friction_coefficient",
    "num_poles",
)


def list_motor_profiles(directory: Path | None = None) -> list[Path]:
    """Return all JSON motor profile files in the configured directory."""
    profile_dir = directory or MOTOR_PROFILES_DIR
    profile_dir.mkdir(parents=True, exist_ok=True)
    return sorted(profile_dir.glob("*.json"))


def save_motor_profile(
    file_path: Path,
    motor_params: dict[str, Any],
    profile_name: str,
    rated_info: dict[str, Any] | None = None,
    source_info: dict[str, Any] | None = None,
) -> Path:
    """Save motor profile JSON to disk."""
    payload = {
        "schema": PROFILE_SCHEMA,
        "profile_name": profile_name,
        "motor_params": _normalize_motor_params(motor_params),
        "rated_info": rated_info or {},
        "source": source_info or {},
    }
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return file_path


def load_motor_profile(file_path: Path) -> dict[str, Any]:
    """Load and validate a motor profile JSON file."""
    raw = json.loads(file_path.read_text(encoding="utf-8"))
    schema = raw.get("schema")
    if schema != PROFILE_SCHEMA:
        raise ValueError(f"Unsupported motor profile schema: {schema}")

    normalized_params = _normalize_motor_params(raw.get("motor_params", {}))
    return {
        "schema": PROFILE_SCHEMA,
        "profile_name": raw.get("profile_name") or file_path.stem,
        "motor_params": normalized_params,
        "rated_info": raw.get("rated_info", {}),
        "source": raw.get("source", {}),
    }


def _normalize_motor_params(input_params: dict[str, Any]) -> dict[str, Any]:
    """Return validated motor parameters with safe defaults for missing keys."""
    params: dict[str, Any] = dict(DEFAULT_MOTOR_PARAMS)
    params.update(input_params or {})

    missing = [k for k in _REQUIRED_PARAM_KEYS if k not in params]
    if missing:
        raise ValueError(f"Missing motor parameters: {', '.join(missing)}")

    num_poles = int(params["num_poles"])
    if num_poles < 2:
        raise ValueError("num_poles must be >= 2")
    if num_poles % 2 != 0:
        raise ValueError("num_poles must be even")

    params["num_poles"] = num_poles
    params["poles_pairs"] = int(num_poles / 2)
    params.setdefault("ld", float(params["phase_inductance"]))
    params.setdefault("lq", float(params["phase_inductance"]))
    params.setdefault("model_type", "dq")
    params.setdefault("emf_shape", "sinusoidal")

    return params
