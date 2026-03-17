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
for skf, iql, fwg, idmax in itertools.product(
    [0.08, 0.11, 0.2, 0.4, 0.8, 1.0, 1.4, 2.0],
    [90.0, 120.0, 180.0, 260.0],
    [0.8, 1.0, 1.2, 1.6],
    [10.0, 20.0, 30.0, 40.0],
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
        fw_headroom_target_v=1.0,
    )
    r = evaluate_case(
        params,
        cand,
        target_speed_rpm=4000.0,
        final_load_nm=0.0,
        dt=1.0e-3,
        sim_end_s=2.8,
        compute_margins=False,
    )
    m = r.get("metrics", {})
    speed = float(m.get("mean_speed_rpm_last_1s", 0.0))
    err = abs(speed - 4000.0)
    row = (
        err,
        speed,
        skf,
        iql,
        fwg,
        idmax,
        r.get("speed_tracking_passed"),
        m.get("fw_injection_dc_a_last_1s"),
    )
    if best is None or err < best[0]:
        best = row
    if r.get("speed_tracking_passed"):
        print("FOUND", row)
        raise SystemExit(0)

print("BEST", best)
