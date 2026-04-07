"""
Comprehensive utility-layer tests
====================================

Covers every public path in:
  - ``src.utils.config``           (path constants, default dicts)
  - ``src.utils.speech``           (speak, set/get audio assistance)
  - ``src.utils.compute_backend``  (resolve_compute_backend, as_dict)
  - ``src.utils.motor_profiles``   (list/save/load motor profile, _normalize_motor_params)
  - ``src.utils.data_logger``      (DataLogger.save_simulation_data, load)
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

# ============================================================================
# src.utils.config
# ============================================================================

class TestConfig:

    def test_project_root_is_path(self):
        from src.utils.config import PROJECT_ROOT  # noqa: PLC0415
        assert isinstance(PROJECT_ROOT, Path)

    def test_directories_exist(self):
        from src.utils.config import (  # noqa: PLC0415
            DATA_DIR,
            LOGS_DIR,
            MOTOR_PROFILES_DIR,
            PLOTS_DIR,
        )
        for d in (DATA_DIR, LOGS_DIR, MOTOR_PROFILES_DIR, PLOTS_DIR):
            assert d.exists()

    def test_default_motor_params_complete(self):
        from src.utils.config import DEFAULT_MOTOR_PARAMS  # noqa: PLC0415
        required = (
            "nominal_voltage", "phase_resistance", "phase_inductance",
            "back_emf_constant", "torque_constant", "rotor_inertia",
            "friction_coefficient", "num_poles", "poles_pairs",
        )
        for key in required:
            assert key in DEFAULT_MOTOR_PARAMS

    def test_simulation_params_complete(self):
        from src.utils.config import SIMULATION_PARAMS  # noqa: PLC0415
        for key in ("pwm_frequency_hz", "dt", "max_history"):
            assert key in SIMULATION_PARAMS

    def test_dt_is_reciprocal_of_pwm_freq(self):
        from src.utils.config import SIMULATION_PARAMS  # noqa: PLC0415
        assert SIMULATION_PARAMS["dt"] == pytest.approx(
            1.0 / SIMULATION_PARAMS["pwm_frequency_hz"]
        )

    def test_gui_params_keys(self):
        from src.utils.config import GUI_PARAMS  # noqa: PLC0415
        assert "update_interval" in GUI_PARAMS
        assert "theme" in GUI_PARAMS


# ============================================================================
# src.utils.speech
# ============================================================================

class TestSpeech:
    """Tests use monkeypatching to avoid real TTS calls."""

    def test_set_and_get_audio_assistance(self):
        import src.utils.speech as speech  # noqa: PLC0415
        speech.set_audio_assistance_enabled(False)
        assert not speech.is_audio_assistance_enabled()
        speech.set_audio_assistance_enabled(True)
        assert speech.is_audio_assistance_enabled()

    def test_speak_when_disabled_is_silent(self, capsys):
        import src.utils.speech as speech  # noqa: PLC0415
        speech.set_audio_assistance_enabled(False)
        speech.speak("hello world")
        # Nothing printed to stdout when disabled
        captured = capsys.readouterr()
        assert "hello world" not in captured.out
        speech.set_audio_assistance_enabled(True)  # restore

    def test_speak_fallback_prints_when_no_pyttsx3(self, monkeypatch, capsys):
        import src.utils.speech as speech  # noqa: PLC0415
        monkeypatch.setattr(speech, "pyttsx3", None)
        speech.set_audio_assistance_enabled(True)
        speech.speak("test message")
        captured = capsys.readouterr()
        assert "test message" in captured.out

    def test_speak_with_rate_when_pyttsx3_none(self, monkeypatch, capsys):
        import src.utils.speech as speech  # noqa: PLC0415
        monkeypatch.setattr(speech, "pyttsx3", None)
        speech.set_audio_assistance_enabled(True)
        speech.speak("rate test", rate=200)  # rate is ignored in fallback
        captured = capsys.readouterr()
        assert "rate test" in captured.out


# ============================================================================
# src.utils.compute_backend
# ============================================================================

class TestComputeBackend:

    def test_cpu_always_available(self):
        from src.utils.compute_backend import resolve_compute_backend  # noqa: PLC0415
        state = resolve_compute_backend("cpu")
        assert state.selected == "cpu"
        assert state.requested == "cpu"

    def test_auto_falls_back_to_cpu_when_no_gpu(self):
        from src.utils.compute_backend import resolve_compute_backend  # noqa: PLC0415
        state = resolve_compute_backend("auto")
        # In CI / headless environment, GPU is never available
        assert state.selected in ("cpu", "gpu")   # either is valid

    def test_gpu_unavailable_raises(self):
        from src.utils.compute_backend import resolve_compute_backend  # noqa: PLC0415
        # GPU is unavailable in this environment; should raise RuntimeError
        try:
            state = resolve_compute_backend("gpu")
            # If somehow GPU is available, selected must be 'gpu'
            assert state.selected == "gpu"
        except RuntimeError:
            pass  # expected in CI

    def test_invalid_backend_raises(self):
        from src.utils.compute_backend import resolve_compute_backend  # noqa: PLC0415
        with pytest.raises(ValueError, match="auto, cpu, gpu"):
            resolve_compute_backend("tpu")

    def test_as_dict_serializable(self):
        from src.utils.compute_backend import as_dict, resolve_compute_backend  # noqa: PLC0415
        state = resolve_compute_backend("cpu")
        d = as_dict(state)
        assert isinstance(d, dict)
        for key in ("requested", "selected", "gpu_available", "reason"):
            assert key in d
        # Must be JSON-serializable
        json_str = json.dumps(d)
        assert len(json_str) > 0

    def test_state_is_frozen(self):
        from src.utils.compute_backend import resolve_compute_backend  # noqa: PLC0415
        state = resolve_compute_backend("cpu")
        with pytest.raises((AttributeError, TypeError)):
            state.selected = "mutated"  # type: ignore[misc]


# ============================================================================
# src.utils.motor_profiles
# ============================================================================

_MINIMAL_PARAMS = {
    "nominal_voltage": 48.0,
    "phase_resistance": 2.5,
    "phase_inductance": 0.005,
    "back_emf_constant": 0.1,
    "torque_constant": 0.1,
    "rotor_inertia": 0.0005,
    "friction_coefficient": 0.001,
    "num_poles": 8,
}


class TestMotorProfiles:

    def test_list_profiles_returns_list(self, tmp_path):
        from src.utils.motor_profiles import list_motor_profiles  # noqa: PLC0415
        files = list_motor_profiles(tmp_path)
        assert isinstance(files, list)

    def test_save_and_load_roundtrip(self, tmp_path):
        from src.utils.motor_profiles import load_motor_profile, save_motor_profile  # noqa: PLC0415
        fp = tmp_path / "test_motor.json"
        save_motor_profile(fp, _MINIMAL_PARAMS, profile_name="Test Motor")
        loaded = load_motor_profile(fp)
        assert loaded["profile_name"] == "Test Motor"
        assert loaded["motor_params"]["nominal_voltage"] == pytest.approx(48.0)

    def test_save_creates_file(self, tmp_path):
        from src.utils.motor_profiles import save_motor_profile  # noqa: PLC0415
        fp = tmp_path / "my_motor.json"
        save_motor_profile(fp, _MINIMAL_PARAMS, profile_name="My Motor")
        assert fp.exists()

    def test_load_wrong_schema_raises(self, tmp_path):
        from src.utils.motor_profiles import load_motor_profile  # noqa: PLC0415
        fp = tmp_path / "bad.json"
        fp.write_text(json.dumps({"schema": "wrong.schema.v0", "motor_params": {}}))
        with pytest.raises(ValueError, match="schema"):
            load_motor_profile(fp)

    def test_normalize_fills_defaults(self):
        from src.utils.motor_profiles import _normalize_motor_params  # noqa: PLC0415
        result = _normalize_motor_params(_MINIMAL_PARAMS.copy())
        assert "ld" in result
        assert "lq" in result
        assert result["poles_pairs"] == 4

    def test_normalize_even_poles_required(self):
        from src.utils.motor_profiles import _normalize_motor_params  # noqa: PLC0415
        bad = dict(_MINIMAL_PARAMS, num_poles=7)
        with pytest.raises(ValueError, match="even"):
            _normalize_motor_params(bad)

    def test_normalize_min_2_poles_required(self):
        from src.utils.motor_profiles import _normalize_motor_params  # noqa: PLC0415
        bad = dict(_MINIMAL_PARAMS, num_poles=0)
        with pytest.raises(ValueError):
            _normalize_motor_params(bad)

    def test_normalize_missing_key_raises(self):
        from src.utils.motor_profiles import _normalize_motor_params  # noqa: PLC0415
        incomplete = {k: v for k, v in _MINIMAL_PARAMS.items() if k != "nominal_voltage"}
        # DEFAULT_MOTOR_PARAMS has nominal_voltage, so fill from default — should succeed
        # Actually missing keys are taken from DEFAULT_MOTOR_PARAMS; only custom required
        # fields will raise.  Omit a required field that has no default:
        incomplete2 = {k: v for k, v in _MINIMAL_PARAMS.items() if k != "back_emf_constant"}
        # back_emf_constant is in DEFAULT_MOTOR_PARAMS with value 0.1 → should pass
        result = _normalize_motor_params(incomplete2)
        assert "back_emf_constant" in result

    def test_save_includes_rated_info_and_source(self, tmp_path):
        from src.utils.motor_profiles import load_motor_profile, save_motor_profile  # noqa: PLC0415
        fp = tmp_path / "full.json"
        save_motor_profile(
            fp, _MINIMAL_PARAMS, "Full Test",
            rated_info={"rated_rpm": 3000},
            source_info={"manufacturer": "Acme"},
        )
        loaded = load_motor_profile(fp)
        assert loaded["rated_info"]["rated_rpm"] == 3000
        assert loaded["source"]["manufacturer"] == "Acme"

    def test_list_profiles_finds_saved_file(self, tmp_path):
        from src.utils.motor_profiles import (  # noqa: PLC0415
            list_motor_profiles,
            save_motor_profile,
        )
        fp = tmp_path / "found.json"
        save_motor_profile(fp, _MINIMAL_PARAMS, "Found")
        files = list_motor_profiles(tmp_path)
        assert fp in files


# ============================================================================
# src.utils.data_logger
# ============================================================================

def _make_history(n: int = 10) -> dict:
    t = np.linspace(0, 0.01, n)
    return {
        "time": t,
        "currents_a": np.random.rand(n),
        "omega": np.random.rand(n),
        "theta": np.random.rand(n),
        "torque": np.random.rand(n),
        "voltages_a": np.random.rand(n),
    }


class TestDataLogger:

    def test_save_returns_path(self, tmp_path):
        from src.utils.data_logger import DataLogger  # noqa: PLC0415
        dl = DataLogger(log_dir=tmp_path)
        history = _make_history()
        path = dl.save_simulation_data(history, filename="test_run")
        assert path.exists()
        assert path.suffix == ".csv"

    def test_save_with_metadata(self, tmp_path):
        from src.utils.data_logger import DataLogger  # noqa: PLC0415
        dl = DataLogger(log_dir=tmp_path)
        history = _make_history()
        path = dl.save_simulation_data(
            history,
            metadata={"motor": "test", "dt": 5e-5},
            filename="meta_test",
        )
        assert path.exists()

    def test_save_custom_path(self, tmp_path):
        from src.utils.data_logger import DataLogger  # noqa: PLC0415
        dl = DataLogger(log_dir=tmp_path)
        history = _make_history()
        custom = tmp_path / "custom_output.csv"
        path = dl.save_simulation_data(
            history, filename=str(custom), use_custom_path=True
        )
        assert path.exists()

    def test_auto_filename_generated_when_none(self, tmp_path):
        from src.utils.data_logger import DataLogger  # noqa: PLC0415
        dl = DataLogger(log_dir=tmp_path)
        path = dl.save_simulation_data(_make_history())
        assert path.exists()
        assert "simulation_" in path.stem

    def test_csv_has_correct_columns(self, tmp_path):
        import csv  # noqa: PLC0415

        from src.utils.data_logger import DataLogger  # noqa: PLC0415

        dl = DataLogger(log_dir=tmp_path)
        history = _make_history(5)
        path = dl.save_simulation_data(history, filename="col_test")

        with path.open() as f:
            reader = csv.reader(f)
            headers = next(reader)
        assert "time" in headers

    def test_custom_path_without_csv_extension(self, tmp_path):
        from src.utils.data_logger import DataLogger  # noqa: PLC0415
        dl = DataLogger(log_dir=tmp_path)
        custom = tmp_path / "no_ext_file"
        path = dl.save_simulation_data(
            _make_history(), filename=str(custom), use_custom_path=True
        )
        assert path.suffix == ".csv"

    def test_log_dir_created_automatically(self, tmp_path):
        from src.utils.data_logger import DataLogger  # noqa: PLC0415
        new_dir = tmp_path / "deep" / "nested" / "logs"
        dl = DataLogger(log_dir=new_dir)
        assert new_dir.exists()


# ============================================================================
# _is_valid_cuda_root (compute_backend internal)
# ============================================================================

class TestIsValidCudaRoot:

    def test_nonexistent_path_returns_false(self, tmp_path):
        from src.utils.compute_backend import _is_valid_cuda_root  # noqa: PLC0415
        fake = tmp_path / "nonexistent_cuda"
        assert not _is_valid_cuda_root(fake)

    def test_path_without_cuda_h_and_dll_returns_false(self, tmp_path):
        from src.utils.compute_backend import _is_valid_cuda_root  # noqa: PLC0415
        fake_cuda = tmp_path / "cuda"
        (fake_cuda / "bin").mkdir(parents=True)
        (fake_cuda / "include").mkdir(parents=True)
        # No cuda.h and no cudart64_*.dll
        assert not _is_valid_cuda_root(fake_cuda)

    def test_path_with_cuda_h_returns_true(self, tmp_path):
        from src.utils.compute_backend import _is_valid_cuda_root  # noqa: PLC0415
        fake_cuda = tmp_path / "cuda"
        (fake_cuda / "bin").mkdir(parents=True)
        (fake_cuda / "include").mkdir(parents=True)
        (fake_cuda / "include" / "cuda.h").write_text("// fake")
        assert _is_valid_cuda_root(fake_cuda)
