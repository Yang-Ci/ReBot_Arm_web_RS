"""测试 Pinocchio / SDK 的 IK。"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "reBotArm_control_py")
)

from reBotArm_control_py.kinematics import compute_fk, load_robot_model
from reBotArm_control_py.kinematics.inverse_kinematics import (
    IKParams,
    pos_rot_to_se3,
    solve_ik,
)
from rebot_b601_rs_sim.config import END_EFFECTOR_FRAME, NUM_JOINTS, RS_URDF_PATH


def test_pinocchio_fk() -> None:
    model = load_robot_model(str(RS_URDF_PATH))
    print(f"nq: {model.nq}, expected controlled joints: {NUM_JOINTS}", flush=True)
    q6 = np.array([0.0, 0.5, 1.0, 0.0, 0.5, 0.0])
    q_full = np.zeros(model.nq)
    q_full[:NUM_JOINTS] = q6
    pos, rot, _ = compute_fk(model, q_full, frame_name=END_EFFECTOR_FRAME)
    print(f"FK pos: {pos}", flush=True)
    assert model.nq == 8
    assert pos.shape == (3,)


def test_pinocchio_ik_current_pose() -> None:
    """以当前 FK 位姿为目标，IK 应该能收敛。"""
    model = load_robot_model(str(RS_URDF_PATH))
    data = model.createData()
    frame_id = model.getFrameId(END_EFFECTOR_FRAME)

    q6 = np.array([0.0, 0.5, 1.0, 0.0, 0.5, 0.0])
    q_full = np.zeros(model.nq)
    q_full[:NUM_JOINTS] = q6

    pos, rot, _ = compute_fk(model, q_full, frame_name=END_EFFECTOR_FRAME)
    target = pos_rot_to_se3(pos, rot)

    result = solve_ik(
        model, data, frame_id, target, q_full,
        params=IKParams(max_iter=1000, tolerance=1e-4),
        controlled_joints=NUM_JOINTS,
    )
    print(
        f"IK to current pose: success={result.success}, "
        f"error={result.error:.6e}, iters={result.iterations}",
        flush=True,
    )
    print(f"result q: {result.q}", flush=True)
    assert result.success
