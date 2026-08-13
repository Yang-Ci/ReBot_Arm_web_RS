"""测试重力补偿力矩计算与施加（无 viewer）。"""

from __future__ import annotations

import numpy as np

import mujoco

from rebot_b601_rs_sim.control.gravity_compensation import GravityCompensator
from rebot_b601_rs_sim.robot.model import RobotModel


def test_gravity_comp_no_actuator() -> None:
    """验证无 actuator 时，重力补偿力矩能使关节运动（不被锁定在零位）。"""
    robot = RobotModel()
    gc = GravityCompensator()
    nq = robot.model.nq
    nv = robot.model.nv
    N_ARM_JOINTS = 6

    q0 = np.array([0.0, 0.8, 1.2, 0.0, 0.5, 0.0])
    robot.reset(q0)

    # 若模型中存在执行器，将其 ctrl 设为当前位置，避免对臂运动产生额外力矩
    if robot.model.nu > 0:
        for i in range(robot.model.nu):
            robot.data.ctrl[i] = robot.data.qpos[robot.model.actuator_trnid[i, 0]]

    # 测试目的是验证“无执行器锁定”时重力补偿能让关节运动；
    # 临时将 actuator gear 置零，避免新版 position servo 增益过高导致关节被锁住。
    original_gears = robot.model.actuator_gear[:, 0].copy()
    robot.model.actuator_gear[:, 0] = 0.0

    kp_hold = np.zeros(nv)
    kd_hold = np.zeros(nv)
    kp_hold[:N_ARM_JOINTS] = 8.0
    kd_hold[:N_ARM_JOINTS] = 2.4
    q_hold = np.zeros(nq)
    q_hold[:N_ARM_JOINTS] = q0

    try:
        for _ in range(200):
            q = robot.data.qpos[:nq].copy()
            qd = robot.data.qvel[:nv].copy()
            tau_g = gc.compute(q[:N_ARM_JOINTS])
            mujoco.mj_forward(robot.model, robot.data)
            tau_dyn = robot.data.qfrc_bias[:nv].copy()
            tau = tau_dyn + kp_hold * (q_hold[:nv] - q[:nv]) - kd_hold * qd
            robot.data.qfrc_applied[:N_ARM_JOINTS] = tau[:N_ARM_JOINTS]
            mujoco.mj_step(robot.model, robot.data)

        q_final = robot.get_q()
        print(f"q_final: {q_final}", flush=True)
        # 至少有一个关节离开了初始位置（否则说明被锁定）
        assert np.linalg.norm(q_final - q0) > 0.01, "Joints appear locked"
    finally:
        robot.model.actuator_gear[:, 0] = original_gears
