"""
Atomic features tested in this module:
- src lazy import and missing attribute
- variable load uses callback
- variable load interpolates and clamps
- variable load validates input
- cyclic load validation and output
- constant load reset is noop
- speech can be disabled
- speech falls back to console
- speech uses engine and rate
- compute backend cuda root validation
- compute backend sets cuda path on windows
- compute backend resolve modes and as dict
- data logger saves default name and loads
- data logger custom path and non numeric load
"""

import csv
from pathlib import Path

import numpy as np
import pytest

import src
from src.core.load_model import ConstantLoad, CyclicLoad, VariableLoad
from src.utils import compute_backend, speech
from src.utils.data_logger import DataLogger


def _sample_history() -> dict[str, np.ndarray]:
    return {
        "time": np.array([0.0, 0.1, 0.2], dtype=np.float64),
        "speed": np.array([0.0, 10.0, 20.0], dtype=np.float64),
        "currents_a": np.array([0.0, 1.0, 2.0], dtype=np.float64),
        "currents_b": np.array([0.0, 1.5, 2.5], dtype=np.float64),
        "currents_c": np.array([0.0, 2.0, 3.0], dtype=np.float64),
        "omega": np.array([0.0, 1.0, 2.0], dtype=np.float64),
        "theta": np.array([0.0, 0.1, 0.2], dtype=np.float64),
        "emf_a": np.array([0.0, 1.0, 2.0], dtype=np.float64),
        "emf_b": np.array([0.0, 1.0, 2.0], dtype=np.float64),
        "emf_c": np.array([0.0, 1.0, 2.0], dtype=np.float64),
        "torque": np.array([0.0, 0.5, 1.0], dtype=np.float64),
        "load_torque": np.array([0.0, 0.2, 0.4], dtype=np.float64),
        "voltages_a": np.array([0.0, 5.0, 10.0], dtype=np.float64),
        "voltages_b": np.array([0.0, 5.0, 10.0], dtype=np.float64),
        "voltages_c": np.array([0.0, 5.0, 10.0], dtype=np.float64),
    }


def test_src_lazy_import_and_missing_attribute():
    assert src.core.__name__ == "src.core"
    with pytest.raises(AttributeError):
        _ = src.not_a_real_subpackage


def test_variable_load_uses_callback():
    load = VariableLoad(torque_func=lambda t: t * 2.0)
    assert load.get_torque(1.25) == pytest.approx(2.5)


def test_variable_load_interpolates_and_clamps():
    load = VariableLoad(
        time_points=[0.0, 1.0, 2.0],
        torque_points=[0.0, 2.0, 4.0],
    )
    assert load.get_torque(-1.0) == pytest.approx(0.0)
    assert load.get_torque(0.5) == pytest.approx(1.0)
    assert load.get_torque(3.0) == pytest.approx(4.0)


def test_variable_load_validates_input():
    with pytest.raises(ValueError):
        VariableLoad()
    with pytest.raises(ValueError):
        VariableLoad(time_points=[0.0], torque_points=None)
    with pytest.raises(ValueError):
        VariableLoad(time_points=[0.0, 1.0], torque_points=[0.0])
    with pytest.raises(ValueError):
        VariableLoad(time_points=[1.0, 0.5], torque_points=[0.0, 1.0])


def test_cyclic_load_validation_and_output():
    with pytest.raises(ValueError):
        CyclicLoad(frequency=-1.0)

    load = CyclicLoad(offset=1.0, amplitude=2.0, frequency=1.0, phase=0.0)
    assert load.get_torque(0.0) == pytest.approx(1.0)
    assert load.get_torque(0.25) == pytest.approx(3.0)


def test_constant_load_reset_is_noop():
    load = ConstantLoad(torque=3.0)
    load.reset()
    assert load.get_torque(2.0) == pytest.approx(3.0)


def test_speech_can_be_disabled(monkeypatch):
    class FailingEngineModule:
        @staticmethod
        def init():
            raise AssertionError("speech engine should not be created when disabled")

    monkeypatch.setattr(speech, "pyttsx3", FailingEngineModule)
    speech.set_audio_assistance_enabled(False)
    try:
        speech.speak("quiet")
    finally:
        speech.set_audio_assistance_enabled(True)


def test_speech_falls_back_to_console(monkeypatch, capsys):
    monkeypatch.setattr(speech, "pyttsx3", None)
    speech.set_audio_assistance_enabled(True)
    speech.speak("hello world")
    captured = capsys.readouterr()
    assert "[Speech] hello world" in captured.out


def test_speech_uses_engine_and_rate(monkeypatch):
    calls: list[tuple[str, object]] = []

    class DummyEngine:
        def setProperty(self, name, value):
            calls.append((name, value))

        def say(self, message):
            calls.append(("say", message))

        def runAndWait(self):
            calls.append(("run", None))

    class DummyTts:
        @staticmethod
        def init():
            return DummyEngine()

    monkeypatch.setattr(speech, "pyttsx3", DummyTts)
    speech.set_audio_assistance_enabled(True)
    speech.speak("ready", rate=150)
    assert calls == [("rate", 150), ("say", "ready"), ("run", None)]


def test_compute_backend_cuda_root_validation(tmp_path: Path):
    valid_root = tmp_path / "cuda"
    (valid_root / "bin").mkdir(parents=True)
    (valid_root / "include").mkdir(parents=True)
    (valid_root / "include" / "cuda.h").write_text("// stub", encoding="utf-8")
    assert compute_backend._is_valid_cuda_root(valid_root) is True
    assert compute_backend._is_valid_cuda_root(tmp_path / "missing") is False


def test_compute_backend_sets_cuda_path_on_windows(monkeypatch, tmp_path: Path):
    root = tmp_path / "CUDA" / "v12.0"
    (root / "bin").mkdir(parents=True)
    (root / "include").mkdir(parents=True)
    (root / "include" / "cuda.h").write_text("// stub", encoding="utf-8")

    monkeypatch.setattr(compute_backend.platform, "system", lambda: "Windows")
    monkeypatch.delenv("CUDA_PATH", raising=False)
    monkeypatch.delenv("CUDA_HOME", raising=False)
    monkeypatch.setenv("PATH", "")
    monkeypatch.setattr(
        compute_backend,
        "Path",
        lambda p="": root.parent if str(p).startswith("C:/Program Files") else Path(p),
    )

    compute_backend._ensure_cuda_path_windows()

    assert Path(compute_backend.os.environ["CUDA_PATH"]) == root
    assert str(root / "bin") in compute_backend.os.environ["PATH"]


def test_compute_backend_resolve_modes_and_as_dict(monkeypatch):
    monkeypatch.setattr(compute_backend, "_probe_cupy", lambda: (False, "no_gpu"))
    cpu_state = compute_backend.resolve_compute_backend("cpu")
    assert cpu_state.selected == "cpu"

    auto_state = compute_backend.resolve_compute_backend("auto")
    assert auto_state.selected == "cpu"
    assert auto_state.reason == "fallback_cpu:no_gpu"

    with pytest.raises(RuntimeError):
        compute_backend.resolve_compute_backend("gpu")

    monkeypatch.setattr(compute_backend, "_probe_cupy", lambda: (True, "cupy_cuda_devices=1"))
    gpu_state = compute_backend.resolve_compute_backend("gpu")
    assert compute_backend.as_dict(gpu_state) == {
        "requested": "gpu",
        "selected": "gpu",
        "gpu_available": True,
        "reason": "cupy_cuda_devices=1",
    }

    with pytest.raises(ValueError):
        compute_backend.resolve_compute_backend("bad")


def test_data_logger_saves_default_name_and_loads(tmp_path: Path):
    logger = DataLogger(log_dir=tmp_path)
    csv_path = logger.save_simulation_data(_sample_history(), metadata={"scenario": "unit"})

    assert csv_path.exists()
    assert csv_path.suffix == ".csv"
    metadata_path = csv_path.with_name(f"{csv_path.stem}_metadata.json")
    assert metadata_path.exists()

    loaded = logger.load_simulation_data(csv_path)
    assert set(["time", "speed", "currents_a"]).issubset(loaded)
    assert loaded["speed"].dtype == np.float64
    assert loaded["speed"][2] == pytest.approx(20.0)


def test_data_logger_custom_path_and_non_numeric_load(tmp_path: Path):
    logger = DataLogger(log_dir=tmp_path)
    custom_path = tmp_path / "nested" / "export_name"
    csv_path = logger.save_simulation_data(
        _sample_history(),
        filename=str(custom_path),
        use_custom_path=True,
    )

    assert csv_path == custom_path.with_suffix(".csv")
    assert csv_path.exists()
    assert csv_path.with_name("export_name_metadata.json").exists()

    broken_csv = tmp_path / "broken.csv"
    with broken_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time", "speed"])
        writer.writerow(["0.0", "not-a-number"])

    loaded = logger.load_simulation_data(broken_csv)
    assert loaded["speed"][0] == pytest.approx(0.0)
