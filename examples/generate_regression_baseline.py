"""Generate and freeze KPI baseline for deterministic regression scenarios.

Run:
    python examples/generate_regression_baseline.py
"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.utils.regression_baseline import save_baseline

if __name__ == "__main__":
    baseline_path = Path("tests") / "baselines" / "reference_baseline.json"
    save_baseline(baseline_path)
    print(f"Baseline saved: {baseline_path}")
