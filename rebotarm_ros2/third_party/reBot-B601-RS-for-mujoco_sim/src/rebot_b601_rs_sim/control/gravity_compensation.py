"""重力补偿：调用 SDK 计算 g(q)，作为 MuJoCo 前馈力矩。"""

from __future__ import annotations

import numpy as np

from ..config import NUM_JOINTS, RS_URDF_PATH


class GravityCompensator:
    """基于 Pinocchio 的重力补偿力矩计算。"""

    def __init__(self, urdf_path: str | None = None) -> None:
        """
        Args:
            urdf_path: URDF 文件路径，默认使用 SDK 中的 B601-RS URDF。
        """
        from reBotArm_control_py.dynamics import compute_generalized_gravity
        from reBotArm_control_py.kinematics import load_robot_model

        self.urdf_path = urdf_path or str(RS_URDF_PATH)
        self.model = load_robot_model(self.urdf_path)
        self.data = self.model.createData()
        self._compute_fn = compute_generalized_gravity

    def compute(self, q: np.ndarray) -> np.ndarray:
        """计算当前关节构型下的重力补偿力矩。

        Args:
            q: 关节位置 (NUM_JOINTS,)。

        Returns:
            tau_g (NUM_JOINTS,)，单位：N·m。
        """
        q = np.asarray(q, dtype=float)
        q_full = np.zeros(self.model.nq)
        q_full[: min(q.shape[0], NUM_JOINTS)] = q[:NUM_JOINTS]

        tau_g_full = self._compute_fn(self.model, q_full, self.data)
        return tau_g_full[:NUM_JOINTS]
