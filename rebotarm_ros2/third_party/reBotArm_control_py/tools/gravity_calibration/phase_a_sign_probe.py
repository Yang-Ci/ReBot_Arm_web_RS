"""Phase A — sign calibration by hand (NO torque, motors stay disabled).

Streams mechPos (0x7019) of motors 1-6 at ~5 Hz and logs deltas vs start.
Johnny moves one joint at a time by hand; the delta sign vs the physical
direction fixes the motor->URDF sign convention for the RS build.

Run: .demo/bin/python phase_a_sign_probe.py [duration_s]
Output: prints a line whenever any joint moves >0.03 rad from its last
printed value; writes full trace to phase_a_trace.jsonl.
"""

import json
import sys
import time
from pathlib import Path

from motorbridge import Controller

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
MECH_POS = 0x7019
MODELS = ["rs-06", "rs-06", "rs-06", "rs-00", "rs-00", "rs-00"]

ctrl = Controller("can0")
motors = {}
for mid, model in zip(range(1, 7), MODELS):
    motors[mid] = ctrl.add_robstride_motor(mid, 0xFD, model)

def read_all():
    out = {}
    for mid, m in motors.items():
        try:
            out[mid] = m.robstride_get_param_f32(MECH_POS)
        except Exception:
            out[mid] = None
    return out

start = read_all()
print("START pose:", {k: round(v, 4) for k, v in start.items()}, flush=True)
last_printed = dict(start)

trace = open(Path(__file__).parent / "phase_a_trace.jsonl", "w")
t0 = time.time()
while time.time() - t0 < DUR:
    q = read_all()
    trace.write(json.dumps({"t": round(time.time() - t0, 3), "q": q}) + "\n")
    trace.flush()
    for mid, v in q.items():
        if v is None or last_printed[mid] is None:
            continue
        if abs(v - last_printed[mid]) > 0.03:
            print(
                f"MOVE motor{mid}: {v:+.4f} rad (delta vs start {v - start[mid]:+.4f})",
                flush=True,
            )
            last_printed[mid] = v
    time.sleep(0.2)

print("END pose:", {k: (round(v, 4) if v is not None else None) for k, v in read_all().items()}, flush=True)
