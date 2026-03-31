"""Compute backend selection helpers.

This module selects a compute backend with safe fallback:
- "gpu": require GPU backend (CuPy) or raise
- "cpu": force NumPy CPU backend
- "auto": use GPU when available, otherwise CPU
"""

from __future__ import annotations

import os
import platform
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class ComputeBackendState:
    """Resolved compute backend metadata."""

    requested: str
    selected: str
    gpu_available: bool
    reason: str


def _is_valid_cuda_root(path: Path) -> bool:
    """Return True when path looks like a usable CUDA toolkit root."""
    bin_dir = path / "bin"
    include_cuda_h = path / "include" / "cuda.h"
    has_runtime_dll = any(bin_dir.glob("cudart64_*.dll")) if bin_dir.exists() else False
    return bool(path.exists() and bin_dir.exists() and (has_runtime_dll or include_cuda_h.exists()))


def _ensure_cuda_path_windows() -> None:
    """Best-effort CUDA_PATH setup for Windows CuPy environments.

    CuPy can work without CUDA_PATH when wheels bundle required runtime pieces,
    but it emits a warning if CUDA_PATH is not set. This helper keeps behavior
    unchanged while reducing noisy warnings in logs.
    """
    if platform.system() != "Windows":
        return
    if os.environ.get("CUDA_PATH"):
        return

    candidates: list[Path] = []
    for env_name in ("CUDA_PATH", "CUDA_HOME"):
        env_value = os.environ.get(env_name)
        if env_value:
            candidates.append(Path(env_value))

    toolkit_root = Path("C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA")
    if toolkit_root.exists():
        version_dirs = sorted(
            [p for p in toolkit_root.iterdir() if p.is_dir()],
            key=lambda p: p.name,
            reverse=True,
        )
        candidates.extend(version_dirs)

    for candidate in candidates:
        if not _is_valid_cuda_root(candidate):
            continue
        os.environ["CUDA_PATH"] = str(candidate)
        bin_dir = str(candidate / "bin")
        current_path = os.environ.get("PATH", "")
        if bin_dir and bin_dir not in current_path.split(";"):
            os.environ["PATH"] = f"{bin_dir};{current_path}" if current_path else bin_dir
        return


def _probe_cupy() -> tuple[bool, str]:
    """Check whether CuPy is importable and has at least one CUDA device."""
    try:
        _ensure_cuda_path_windows()
        with warnings.catch_warnings():
            warnings.filterwarnings(
                "ignore",
                message="CUDA path could not be detected.*",
                category=UserWarning,
            )
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
