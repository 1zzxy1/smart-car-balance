"""Convert 19-column telemetry rows (commit 428848e / 2537b82) into the
31-column schema produced by commit 8e6b015, padding missing fields with NaN.

19-col layout (from code/app/hmi_app.c @ 428848e:267-289):
    0  tick                              -> 31col[0]
    1  scheduler_mission_state           -> 31col[1]
    2  motor_total_distance_m            -> 31col[3]
    3  motor_actual_speed_mps            -> 31col[4]
    4  yaw                               -> 31col[9]
    5  scheduler_turn_progress           -> 31col[16]
    6  scheduler_turn_remaining          -> 31col[17]
    7  expect_angle                      -> 31col[12]
    8  target_angle                      -> 31col[13]
    9  pitch                             -> 31col[19]
    10 balance_angle_feedback            -> 31col[22]
    11 target_gyro                       -> 31col[23]
    12 gyro_y_rate                       -> 31col[20]
    13 servo_output                      -> 31col[25]
    14 servo_last_duty (PWM)             -> 31col[26]
    15 angle_pid.kp                      -> 31col[27]
    16 angle_pid.kd                      -> 31col[28]
    17 scheduler_open_turn_angle         -> 31col[14] AND 31col[29]
    18 scheduler_run_speed_mps           -> 31col[30]

Missing in 19-col (filled with NaN): 31col[2,5,6,7,8,10,11,15,18,21,24]
    2  balance_heading_enabled
    5  scheduler_start_yaw
    6  scheduler_turn_target_yaw
    7  yaw_target
    8  balance_target_yaw_smooth
    10 yaw_error
    11 steering_pid.out
    15 scheduler_turn_delta
    18 roll
    21 gyro_z_rate
    24 balance_gyro_feedback
"""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence

NAN = float("nan")

# Source(19col) -> Target(31col). Multi-mapping handled separately.
INDEX_MAP = {
    0:  0,
    1:  1,
    2:  3,
    3:  4,
    4:  9,
    5:  16,
    6:  17,
    7:  12,
    8:  13,
    9:  19,
    10: 22,
    11: 23,
    12: 20,
    13: 25,
    14: 26,
    15: 27,
    16: 28,
    # 17 -> 14 and 29 (handled in code)
    18: 30,
}

# Names for the 31-col target schema (commit 8e6b015 hmi_send_telemetry order).
COL31_NAMES = [
    "tick",                       # 0
    "mission_state",              # 1
    "heading_enabled",            # 2  (NaN in 19col)
    "distance_m",                 # 3
    "speed_mps",                  # 4
    "start_yaw",                  # 5  (NaN)
    "turn_target_yaw",            # 6  (NaN)
    "yaw_target",                 # 7  (NaN)
    "yaw_smooth",                 # 8  (NaN)
    "yaw",                        # 9
    "yaw_error",                  # 10 (NaN)
    "steering_pid_out",           # 11 (NaN)
    "expect_angle",               # 12
    "target_angle",               # 13
    "open_turn_angle",            # 14
    "turn_delta",                 # 15 (NaN)
    "turn_progress",              # 16
    "turn_remaining",             # 17
    "roll",                       # 18 (NaN)
    "pitch",                      # 19
    "gyro_y_rate",                # 20
    "gyro_z_rate",                # 21 (NaN)
    "balance_angle_feedback",     # 22
    "target_gyro",                # 23
    "balance_gyro_feedback",      # 24 (NaN)
    "servo_output",               # 25
    "servo_pwm_duty",             # 26
    "angle_kp",                   # 27
    "angle_kd",                   # 28
    "open_turn_angle_dup",        # 29  (duplicate of 14 in source code)
    "run_speed_mps",              # 30
]


def _to_float(token) -> float:
    if isinstance(token, (int, float)):
        return float(token)
    s = str(token).strip()
    if not s:
        return NAN
    try:
        return float(s)
    except ValueError:
        return NAN


def from_19col(row: Sequence) -> List[float]:
    """Convert one 19-column row (list/tuple/CSV-split) to a 31-column row.

    Missing fields are filled with float('nan'). Returns a list of 31 floats.
    Accepts either numeric values or stringified tokens.
    """
    if len(row) != 19:
        raise ValueError(f"expected 19 columns, got {len(row)}")

    out = [NAN] * 31
    for src, dst in INDEX_MAP.items():
        out[dst] = _to_float(row[src])

    # source col 17 (open_turn_angle) is duplicated at 31col[14] and 31col[29]
    open_turn = _to_float(row[17])
    out[14] = open_turn
    out[29] = open_turn

    return out


def convert_csv(in_path: str, out_path: str, skip_blank: bool = True) -> int:
    """Convert a 19-column CSV file to a 31-column CSV file.

    Returns number of rows written. Lines that aren't 19-column numeric
    are skipped (e.g. the timestamp banner header line in 4.txt).
    """
    written = 0
    with open(in_path, "r", encoding="utf-8", errors="replace") as fin, \
         open(out_path, "w", encoding="utf-8", newline="") as fout:
        fout.write(",".join(COL31_NAMES) + "\n")
        for raw in fin:
            line = raw.strip()
            if skip_blank and not line:
                continue
            tokens = line.split(",")
            if len(tokens) != 19:
                # banner / corrupt line - skip silently
                continue
            try:
                row31 = from_19col(tokens)
            except ValueError:
                continue
            fout.write(",".join(
                "" if (isinstance(v, float) and math.isnan(v)) else f"{v:g}"
                for v in row31
            ) + "\n")
            written += 1
    return written


def _self_test() -> None:
    sample = "0,0,0.000,0.000,0.00,0.0,360.0,0.00,0.00,0.00,0.00,0.00,0.00,0,5940,22.00,-0.3000,6.00,1.00"
    row = sample.split(",")
    out = from_19col(row)
    assert len(out) == 31
    assert out[0] == 0.0          # tick
    assert out[1] == 0.0          # state
    assert math.isnan(out[2])     # heading_enabled missing
    assert out[14] == 6.0         # open_turn_angle
    assert out[26] == 5940.0      # pwm
    assert out[27] == 22.0        # angle_kp
    assert abs(out[28] + 0.3) < 1e-9  # angle_kd
    assert out[29] == 6.0         # duplicated open_turn_angle
    assert out[30] == 1.0         # run_speed
    print("self_test ok")


if __name__ == "__main__":
    import sys
    if len(sys.argv) == 1:
        _self_test()
    elif len(sys.argv) == 3:
        n = convert_csv(sys.argv[1], sys.argv[2])
        print(f"wrote {n} rows -> {sys.argv[2]}")
    else:
        print("usage: convert_19col_to_31col.py [<in> <out>]   (no args runs self-test)")
