"""机器人状态观测封装。"""

from __future__ import annotations

import numpy as np

from .model import RobotModel


class RobotState:
    """从 MuJoCo data 中读取机器人状态。"""

    def __init__(self, robot: RobotModel) -> None:
        self.robot = robot

    @property
    def q(self) -> np.ndarray:
        return self.robot.get_q()

    @property
    def dq(self) -> np.ndarray:
        return self.robot.get_dq()

    @property
    def tau(self) -> np.ndarray:
        """关节力矩（基于执行器 ctrl 或 sensor，当前返回 ctrl）。"""
        ctrl = np.zeros(self.robot.model.nu)
        ctrl[:] = self.robot.data.ctrl.copy()
        # 映射到关节顺序
        out = np.zeros(len(self.robot.actuator_ids))
        for i, aid in enumerate(self.robot.actuator_ids):
            if aid >= 0:
                out[i] = ctrl[aid]
        return out

    @property
    def ee_position(self) -> np.ndarray:
        pos, _ = self.robot.get_ee_pose()
        return pos

    @property
    def ee_rotation(self) -> np.ndarray:
        _, rot = self.robot.get_ee_pose()
        return rot
