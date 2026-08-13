"""底层关节控制器封装。"""

from __future__ import annotations

import numpy as np

from ..config import NUM_JOINTS


class JointPositionController:
    """简单的 PD 位置控制器，输出目标位置给 position actuator。"""

    def __init__(self, kp: float = 100.0, kd: float = 10.0) -> None:
        self.kp = kp
        self.kd = kd

    def compute(
        self,
        q_target: np.ndarray,
        q_current: np.ndarray,
        dq_current: np.ndarray | None = None,
    ) -> np.ndarray:
        """返回 position actuator 的目标位置。

        对于 MuJoCo 的 position actuator，直接返回目标位置即可；
        kp/kd 用于后续扩展为力矩模式或外部实现。
        """
        _ = q_current
        _ = dq_current
        return np.asarray(q_target, dtype=float)


class TorqueController:
    """力矩控制器：将期望力矩映射到 MuJoCo actuator。"""

    def __init__(self, num_joints: int = NUM_JOINTS) -> None:
        self.num_joints = num_joints

    def compute(
        self,
        tau_desired: np.ndarray,
        tau_gravity: np.ndarray | None = None,
    ) -> np.ndarray:
        """返回总力矩。"""
        tau = np.asarray(tau_desired, dtype=float)
        if tau_gravity is not None:
            tau = tau + np.asarray(tau_gravity, dtype=float)
        return tau
