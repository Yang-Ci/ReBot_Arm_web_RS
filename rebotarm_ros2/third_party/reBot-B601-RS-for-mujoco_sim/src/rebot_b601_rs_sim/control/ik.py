"""逆运动学接口：调用 SDK 求解，映射到 MuJoCo 关节空间。"""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from ..config import END_EFFECTOR_FRAME, NUM_JOINTS, RS_URDF_PATH


class IKSolver:
    """基于 reBotArm_control_py / Pinocchio 的 IK 求解器。"""

    def __init__(self, urdf_path: str | None = None, end_effector_frame: str | None = None) -> None:
        """
        Args:
            urdf_path: URDF 文件路径，默认使用 SDK 中的 B601-RS URDF。
            end_effector_frame: 末端执行器帧名称，默认 ``gripper_end``。
        """
        from reBotArm_control_py.kinematics import load_robot_model

        self.urdf_path = urdf_path or str(RS_URDF_PATH)
        self.end_effector_frame = end_effector_frame or END_EFFECTOR_FRAME

        self.model = load_robot_model(self.urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(self.end_effector_frame)
        if self.frame_id < 0:
            raise ValueError(f"End-effector frame '{self.end_effector_frame}' not found in URDF")

    def solve(
        self,
        target_pos: np.ndarray,
        target_rot: np.ndarray | None = None,
        q_init: np.ndarray | None = None,
        *,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        max_iter: int = 1000,
        tolerance: float = 1e-4,
        step_size: float = 0.5,
        damping: float = 1e-6,
    ) -> tuple[np.ndarray, bool]:
        """求解目标位姿对应的关节角。

        Args:
            target_pos: 目标位置 (3,)，单位：米。
            target_rot: 目标旋转矩阵 (3, 3)，可选。
            q_init: 初始关节角 (NUM_JOINTS,)。若为 None 使用零位。
            roll, pitch, yaw: ZYX 欧拉角（弧度），仅当 target_rot 为 None 时生效。
            max_iter, tolerance, step_size, damping: CLIK 求解器参数。

        Returns:
            (q_target, success)
        """
        from reBotArm_control_py.kinematics.inverse_kinematics import (
            IKParams,
            pos_rot_to_se3,
            solve_ik,
        )

        if q_init is None:
            q_init = pin.neutral(self.model)
        else:
            q_init = np.asarray(q_init, dtype=float)
            if q_init.shape[0] != NUM_JOINTS:
                # 尝试补零到模型维度
                padded = np.zeros(self.model.nq)
                padded[: min(q_init.shape[0], NUM_JOINTS)] = q_init[:NUM_JOINTS]
                q_init = padded

        target = pos_rot_to_se3(
            np.asarray(target_pos, dtype=float), target_rot, roll, pitch, yaw
        )
        params = IKParams(
            max_iter=max_iter,
            tolerance=tolerance,
            step_size=step_size,
            damping=damping,
        )
        result = solve_ik(
            self.model,
            self.data,
            self.frame_id,
            target,
            q_init,
            params,
            controlled_joints=NUM_JOINTS,
        )
        return result.q, result.success

    def forward_kinematics(
        self, q: np.ndarray, frame_name: str | None = None
    ) -> tuple[np.ndarray, np.ndarray]:
        """给定关节角，计算末端（或指定帧）位置与旋转矩阵。"""
        from reBotArm_control_py.kinematics import compute_fk

        q_full = np.zeros(self.model.nq)
        q_full[: min(q.shape[0], NUM_JOINTS)] = q[:NUM_JOINTS]
        frame = frame_name or self.end_effector_frame
        pos, rot, _ = compute_fk(self.model, q_full, frame_name=frame)
        return pos, rot
