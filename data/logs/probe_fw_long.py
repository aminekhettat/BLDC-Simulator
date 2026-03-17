import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from examples.calibrate_fw_loaded_point import (
    PROFILE_PATH,
    SESSION_PATH,
    Candidate,
    evaluate_case,
    to_motor_params,
)

profile = json.loads(Path(PROFILE_PATH).read_text(encoding="utf-8"))
params = to_motor_params(profile)
session = json.loads(Path(SESSION_PATH).read_text(encoding="utf-8"))
b = session["best_candidate"]

seed = Candidate(
    speed_kp=float(b["speed_pi"]["kp"]) * 0.11,
    speed_ki=float(b["speed_pi"]["ki"]) * 0.11,
    current_kp=float(b["current_pi"]["d_kp"]),
    current_ki=float(b["current_pi"]["d_ki"]),
    iq_limit_a=90.0,
    use_d_priority=True,
    coupled_aw_gain=0.25,
    fw_start_rpm=1100.0,
    fw_gain=1.1,
    fw_id_max_a=10.0,
    fw_headroom_target_v=1.0,
)

for coeff in [0.016, 0.02, 0.03, 0.04, 0.06]:
    params.flux_weakening_id_coefficient = coeff
    params.flux_weakening_min_ratio = 0.08
    r = evaluate_case(
        params,
        seed,
        target_speed_rpm=4000.0,
        final_load_nm=0.0,
        dt=1.0e-3,
        sim_end_s=2.8,
        compute_margins=False,
    )
    m = r.get("metrics", {})
    print(
        coeff,
        r.get("speed_tracking_passed"),
        m.get("mean_speed_rpm_last_1s"),
        m.get("mean_speed_error_rpm_last_1s"),
        m.get("fw_injection_dc_a_last_1s"),
    )
