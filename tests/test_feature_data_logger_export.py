"""
Atomic features tested in this module:
- data logger custom path export creates csv and metadata
"""
from pathlib import Path

import numpy as np

from src.utils.data_logger import DataLogger


def test_data_logger_custom_path_export_creates_csv_and_metadata(tmp_path):
    logger = DataLogger()

    test_history = {
        "time": np.array([0.0, 0.1, 0.2]),
        "speed": np.array([0.0, 100.0, 200.0]),
        "currents_a": np.array([0.0, 1.0, 2.0]),
        "currents_b": np.array([0.0, 1.5, 2.5]),
        "currents_c": np.array([0.0, 2.0, 3.0]),
        "omega": np.array([0.0, 10.0, 20.0]),
        "theta": np.array([0.0, 0.1, 0.2]),
        "emf_a": np.array([0.0, 1.0, 2.0]),
        "emf_b": np.array([0.0, 1.0, 2.0]),
        "emf_c": np.array([0.0, 1.0, 2.0]),
        "torque": np.array([0.0, 1.0, 2.0]),
        "load_torque": np.array([0.0, 0.5, 1.0]),
        "voltages_a": np.array([0.0, 10.0, 20.0]),
        "voltages_b": np.array([0.0, 10.0, 20.0]),
        "voltages_c": np.array([0.0, 10.0, 20.0]),
    }

    custom_path = Path(tmp_path) / "my_export.csv"
    result = logger.save_simulation_data(
        test_history,
        metadata={"test": "data"},
        filename=str(custom_path),
        use_custom_path=True,
    )

    assert result.exists()
    assert result.name == "my_export.csv"

    metadata_path = custom_path.parent / "my_export_metadata.json"
    assert metadata_path.exists()

    with open(result, "r", encoding="utf-8") as f:
        lines = f.readlines()
    assert len(lines) > 1






