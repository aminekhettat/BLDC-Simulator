import itertools
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

params.flux_weakening_id_coefficient = 0.016
params.flux_weakening_min_ratio = 0.08

best = None
for skf, iql, fwg, idmax, hr in itertools.product(
    [0.09, 0.10, 0.11],
    [90.0, 100.0, 110.0],
    [1.0, 1.1, 1.2],
    [10.0, 12.0, 14.0],
    [1.0, 1.2, 1.4],
):
    cand = Candidate(
        speed_kp=float(b["speed_pi"]["kp"]) * skf,
        speed_ki=float(b["speed_pi"]["ki"]) * skf,
        current_kp=float(b["current_pi"]["d_kp"]),
        current_ki=float(b["current_pi"]["d_ki"]),
        iq_limit_a=float(iql),
        use_d_priority=True,
        coupled_aw_gain=0.25,
        fw_start_rpm=1100.0,
        fw_gain=float(fwg),
        fw_id_max_a=float(idmax),
        fw_headroom_target_v=float(hr),
    )
    r = evaluate_case(
        params,
        cand,
        target_speed_rpm=4000.0,
        final_load_nm=0.0,
        dt=0.001,
        sim_end_s=0.9,
        compute_margins=False,
    )
    e = abs(r["metrics"]["mean_speed_error_rpm_last_1s"])
    if best is None or e < best[0]:
        best = (e, cand, r)
    if r["speed_tracking_passed"]:
        print("FOUND", cand)
        print("SPEED", r["metrics"]["mean_speed_rpm_last_1s"])
        print("FW", r["metrics"]["fw_injection_dc_a_last_1s"])
        raise SystemExit(0)

print("NO_FOUND")
print("BEST_ERR", best[0])
print("BEST_CAND", best[1])
print("BEST_SPEED", best[2]["metrics"]["mean_speed_rpm_last_1s"])
