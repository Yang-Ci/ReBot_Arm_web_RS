"""测试我们自己的 IKSolver。"""

from __future__ import annotations

import numpy as np

from rebot_b601_rs_sim.control.ik import IKSolver


def test_ik_current_pose() -> None:
    ik = IKSolver()
    q6 = np.array([0.0, 0.5, 1.0, 0.0, 0.5, 0.0])
    pos, rot = ik.forward_kinematics(q6)
    print(f"FK pos: {pos}", flush=True)

    q_target, success = ik.solve(pos, target_rot=rot, q_init=q6)
    print(f"IK success: {success}, q: {q_target}", flush=True)
    assert success
