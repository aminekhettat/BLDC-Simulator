"""
Data Logger Module
==================

Handles data logging, CSV export, and history management.

:author: BLDC Control Team
:version: 0.8.0
"""

import csv
import json
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional
import numpy as np
import logging

logger = logging.getLogger(__name__)


class DataLogger:
    """
    Data logger for simulation results.

    Handles:
    - CSV export
    - JSON metadata
    - Data formatting
    - File management
    """

    def __init__(self, log_dir: Optional[Path] = None):
        """
        Initialize data logger.

        :param log_dir: Directory for log files
        :type log_dir: Path, optional
        """
        if log_dir is None:
            from .config import LOGS_DIR

            log_dir = LOGS_DIR

        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

    def save_simulation_data(
        self,
        history: Dict[str, np.ndarray],
        metadata: Optional[Dict[str, Any]] = None,
        filename: Optional[str] = None,
        use_custom_path: bool = False,
    ) -> Path:
        """
        Save simulation data to CSV and metadata to JSON.

        :param history: History dictionary from SimulationEngine.get_history()
        :type history: dict
        :param metadata: Optional metadata to save
        :type metadata: dict, optional
        :param filename: Optional filename (without extension) or full path if use_custom_path=True
        :type filename: str, optional
        :param use_custom_path: If True, filename is treated as full path
        :type use_custom_path: bool
        :return: Path to saved CSV file
        :rtype: Path
        """
        # Generate filename if not provided
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"simulation_{timestamp}"

        if use_custom_path:
            # Use full path provided by user
            csv_file = Path(filename)
            if csv_file.suffix != ".csv":
                csv_file = csv_file.with_suffix(".csv")
            # Create metadata file with same name but _metadata suffix before extension
            json_file = csv_file.parent / f"{csv_file.stem}_metadata.json"
            # Ensure directory exists
            csv_file.parent.mkdir(parents=True, exist_ok=True)
        else:
            # Use default logs directory
            csv_file = self.log_dir / f"{filename}.csv"
            json_file = self.log_dir / f"{filename}_metadata.json"

        # Save CSV
        self._save_csv(history, csv_file)
        logger.info(f"Saved simulation data to {csv_file}")

        # Save metadata
        if metadata is None:
            metadata = {}

        metadata_with_info = {
            "timestamp": datetime.now().isoformat(),
            "num_samples": len(history["time"]),
            "duration": float(history["time"][-1]) if len(history["time"]) > 0 else 0,
            **metadata,
        }

        self._save_json(metadata_with_info, json_file)

        return csv_file

    def _save_csv(self, history: Dict[str, np.ndarray], filepath: Path) -> None:
        """
        Save history to CSV file.

        :param history: History dictionary
        :type history: dict
        :param filepath: Output CSV file path
        :type filepath: Path
        """
        # Extract keys that should be in CSV (skip duplicate alpha/beta etc)
        csv_keys = [
            "time",
            "currents_a",
            "currents_b",
            "currents_c",
            "omega",
            "theta",
            "speed",
            "emf_a",
            "emf_b",
            "emf_c",
            "torque",
            "load_torque",
            "voltages_a",
            "voltages_b",
            "voltages_c",
            "supply_voltage",
            "effective_dc_voltage",
            "dc_link_ripple_v",
            "dc_link_bus_current_a",
            "device_loss_power",
            "conduction_loss_power",
            "switching_loss_power",
            "dead_time_loss_power",
            "diode_loss_power",
            "inverter_total_loss_power",
            "inverter_junction_temp_c",
            "common_mode_voltage",
            "min_pulse_event_count",
            "input_power",
            "mechanical_output_power",
            "total_loss_power",
            "efficiency",
        ]

        available_keys = [k for k in csv_keys if k in history]

        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)

            # Write header
            writer.writerow(available_keys)

            # Write data
            num_rows = len(history[available_keys[0]]) if available_keys else 0
            for i in range(num_rows):
                row = [history[key][i] for key in available_keys]
                writer.writerow(row)

    def _save_json(self, data: Dict[str, Any], filepath: Path) -> None:
        """
        Save metadata to JSON file.

        :param data: Data dictionary
        :type data: dict
        :param filepath: Output JSON file path
        :type filepath: Path
        """
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2, default=str)

    def load_simulation_data(self, filepath: Path) -> Dict[str, np.ndarray]:
        """
        Load simulation data from CSV.

        :param filepath: Path to CSV file
        :type filepath: Path
        :return: History dictionary with numpy arrays
        :rtype: dict
        """
        history = {}

        with open(filepath, "r") as f:
            reader = csv.reader(f)
            headers = next(reader)

            # Initialize lists
            for header in headers:
                history[header] = []

            # Read data
            for row in reader:
                for header, value in zip(headers, row):
                    try:
                        history[header].append(float(value))
                    except ValueError:
                        history[header].append(0.0)

        # Convert to numpy arrays
        for key in history:
            history[key] = np.array(history[key], dtype=np.float64)

        logger.info(f"Loaded simulation data from {filepath}")

        return history
