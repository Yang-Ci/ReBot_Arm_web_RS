"""MuJoCo 仿真主循环。"""

from __future__ import annotations

from typing import Callable

import mujoco
import numpy as np

from ..config import NUM_JOINTS
from ..control.controller import JointPositionController, TorqueController
from ..control.gravity_compensation import GravityCompensator
from ..robot.model import RobotModel


class Simulator:
    """封装 MuJoCo 仿真循环，支持位置控制、力矩控制、重力补偿模式。"""

    def __init__(
        self,
        robot: RobotModel,
        *,
        gravity_compensator: GravityCompensator | None = None,
    ) -> None:
        self.robot = robot
        self.gravity_compensator = gravity_compensator
        self.pos_controller = JointPositionController()
        self.torque_controller = TorqueController()

    def reset(self, q0: np.ndarray | None = None) -> None:
        """重置仿真。"""
        self.robot.reset(q0)

    def step_position(self, q_target: np.ndarray) -> None:
        """位置控制模式：驱动关节到目标位置。"""
        q_current = self.robot.get_q()
        ctrl = self.pos_controller.compute(q_target, q_current)
        self.robot.set_ctrl(ctrl)
        self.robot.step()

    def step_torque(self, tau_desired: np.ndarray) -> None:
        """力矩控制模式：直接施加力矩（需要 actuator 为 motor 类型）。"""
        ctrl = self.torque_controller.compute(tau_desired)
        self.robot.set_ctrl(ctrl)
        self.robot.step()

    def step_gravity_compensation(self) -> None:
        """重力补偿模式：计算 g(q) 并通过 qfrc_applied 施加。

        同时将 position actuator 的目标设为当前位置，避免 actuator 把关节拉回零位。
        """
        if self.gravity_compensator is None:
            raise RuntimeError("GravityCompensator not provided")
        q = self.robot.get_q()
        tau_g = self.gravity_compensator.compute(q)
        # 让 position actuator 保持当前位置，不干扰重力补偿
        self.robot.set_ctrl(q)
        self.robot.set_qfrc_applied(tau_g)
        self.robot.step()

    def run(
        self,
        controller: Callable[[np.ndarray, np.ndarray], np.ndarray],
        duration: float,
        q0: np.ndarray | None = None,
    ) -> None:
        """运行一个开环/闭环仿真。

        Args:
            controller: 控制器函数 (q, dq) -> ctrl。
            duration: 仿真时长（秒）。
            q0: 初始关节角。
        """
        self.reset(q0)
        steps = int(duration / self.robot.model.opt.timestep)
        for _ in range(steps):
            q = self.robot.get_q()
            dq = self.robot.get_dq()
            ctrl = controller(q, dq)
            self.robot.set_ctrl(ctrl)
            self.robot.step()
