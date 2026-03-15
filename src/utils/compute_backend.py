"""Compute backend selection helpers.

This module selects a compute backend with safe fallback:
- "gpu": require GPU backend (CuPy) or raise
- "cpu": force NumPy CPU backend
- "auto": use GPU when available, otherwise CPU
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ComputeBackendState:
    """Resolved compute backend metadata."""

    requested: str
    selected: str
    gpu_available: bool
    reason: str


def _probe_cupy() -> tuple[bool, str]:
    """Check whether CuPy is importable and has at least one CUDA device."""
    try:
        import cupy as cp  # type: ignore

        count = int(cp.cuda.runtime.getDeviceCount())
        if count > 0:
            return True, f"cupy_cuda_devices={count}"
        return False, "cupy_found_but_no_cuda_device"
    except Exception as exc:  # pragma: no cover - environment-dependent
        return False, f"cupy_unavailable:{exc.__class__.__name__}"


def resolve_compute_backend(requested: str = "auto") -> ComputeBackendState:
    """Resolve requested backend to a concrete runtime backend."""
    req = str(requested).strip().lower()
    if req not in {"auto", "cpu", "gpu"}:
        raise ValueError("requested backend must be one of: auto, cpu, gpu")

    gpu_ok, reason = _probe_cupy()

    if req == "cpu":
        return ComputeBackendState(
            requested=req,
            selected="cpu",
            gpu_available=gpu_ok,
            reason="forced_cpu",
        )

    if req == "gpu":
        if not gpu_ok:
            raise RuntimeError(
                f"GPU backend requested but unavailable ({reason}). "
                "Install CuPy with CUDA support and ensure a CUDA-capable GPU is visible."
            )
        return ComputeBackendState(
            requested=req,
            selected="gpu",
            gpu_available=True,
            reason=reason,
        )

    # auto mode
    selected = "gpu" if gpu_ok else "cpu"
    return ComputeBackendState(
        requested=req,
        selected=selected,
        gpu_available=gpu_ok,
        reason=reason if gpu_ok else f"fallback_cpu:{reason}",
    )


def as_dict(state: ComputeBackendState) -> dict[str, Any]:
    """Convert backend state into a serializable dict."""
    return {
        "requested": state.requested,
        "selected": state.selected,
        "gpu_available": state.gpu_available,
        "reason": state.reason,
    }
