"""Generate and freeze KPI baseline for deterministic FOC regression scenarios.

Run:
    python examples/generate_foc_regression_baseline.py
"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.utils.regression_baseline import save_foc_baseline


if __name__ == "__main__":
    baseline_path = Path("tests") / "baselines" / "foc_reference_baseline.json"
    save_foc_baseline(baseline_path)
    print(f"FOC baseline saved: {baseline_path}")
