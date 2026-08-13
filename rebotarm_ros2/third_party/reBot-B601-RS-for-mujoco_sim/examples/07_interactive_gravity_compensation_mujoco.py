#!/usr/bin/env python3
"""MuJoCo 交互式重力补偿仿真。

用法:
    python examples/07_interactive_gravity_compensation_mujoco.py

交互:
    在 MuJoCo viewer 中拖动机器人关节后松手，关节会悬浮在当前位置。
    b / home / zero: 回归零点
    o / open: 张开夹爪
    c / close: 闭合夹爪
    q / quit / exit: 退出
"""

from __future__ import annotations

import queue
import signal
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rebot_b601_rs_sim.config import SCENE_PATH
from rebot_b601_rs_sim.control.gravity_compensation import GravityCompensator


def input_thread_fn(cmd_queue: queue.Queue, stop_event: threading.Event) -> None:
    """后台线程：读取终端输入并放入队列。"""
    while not stop_event.is_set():
        try:
            line = input("命令 > ")
        except EOFError:
            cmd_queue.put("quit")
            break
        if line.strip():
            cmd_queue.put(line.strip())


def main() -> None:
    # ── 加载模型 ──────────────────────────────────────────────────────────────
    mj_model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    mj_data = mujoco.MjData(mj_model)
    N_ARM_JOINTS = 6
    # XML 中夹爪包含驱动关节 joint7 + joint_left + joint_right，
    # qpos 顺序为 6 个臂关节后依次是 joint7/left/right。
    N_GRIPPER_JOINTS = 3
    N_ROBOT_JOINTS = N_ARM_JOINTS + N_GRIPPER_JOINTS  # 9

    gc = GravityCompensator()

    # 保持目标：静止时自动记录当前位置，松手后悬浮在这里
    q_hold = np.zeros(N_ROBOT_JOINTS)
    drag_threshold = 0.3  # rad/s，超过此值认为正在被拖动
    still_counter = 0

    # 夹爪开合目标 (joint7, joint_left, joint_right)
    # joint7 与左右指通过 equality 1:1 联动，因此三者目标相同。
    gripper_open = np.array([0.04, 0.04, 0.04])  # 略小于上限 0.05
    gripper_close = np.zeros(3)

    # 控制增益
    kp_hold = np.full(N_ROBOT_JOINTS, 8.0)
    kd_hold = np.full(N_ROBOT_JOINTS, 2.4)
    kp_drag = np.full(N_ROBOT_JOINTS, 0.5)
    kd_drag = np.full(N_ROBOT_JOINTS, 0.2)

    # ── 交互线程 ──────────────────────────────────────────────────────────────
    cmd_queue: queue.Queue[str] = queue.Queue()
    stop_event = threading.Event()

    def _signal_handler(sig, frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _signal_handler)

    threading.Thread(
        target=input_thread_fn,
        args=(cmd_queue, stop_event),
        daemon=True,
    ).start()

    print("=" * 60)
    print("MuJoCo 重力补偿仿真已启动")
    print("在 MuJoCo viewer 中拖动机器人关节后松手，关节会悬浮在当前位置")
    print("      b / home / zero: 回归零点")
    print("      o / open: 张开夹爪")
    print("      c / close: 闭合夹爪")
    print("      q / quit / exit: 退出")
    print("=" * 60)

    # ── 主仿真循环 ────────────────────────────────────────────────────────────
    dt = float(mj_model.opt.timestep)
    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        while viewer.is_running() and not stop_event.is_set():
            # 处理终端命令
            try:
                line = cmd_queue.get_nowait()
            except queue.Empty:
                line = None

            if line is not None:
                cmd = line.lower()
                if cmd in ("q", "quit", "exit"):
                    stop_event.set()
                    break
                if cmd in ("b", "home", "zero"):
                    print("  回归零点")
                    q_hold[:] = 0.0
                if cmd in ("o", "open"):
                    print("  张开夹爪")
                    q_hold[N_ARM_JOINTS : N_ARM_JOINTS + N_GRIPPER_JOINTS] = gripper_open
                if cmd in ("c", "close"):
                    print("  闭合夹爪")
                    q_hold[N_ARM_JOINTS : N_ARM_JOINTS + N_GRIPPER_JOINTS] = gripper_close

            q = mj_data.qpos[:N_ROBOT_JOINTS].copy()
            qd = mj_data.qvel[:N_ROBOT_JOINTS].copy()
            qd_norm = float(np.linalg.norm(qd[:N_ARM_JOINTS]))

            # 用 Pinocchio 计算重力力矩（显示用）
            tau_g = gc.compute(q[:N_ARM_JOINTS])

            # 用 MuJoCo 自身的偏置力做完全动态补偿，保证物理一致
            mujoco.mj_forward(mj_model, mj_data)
            tau_dyn = mj_data.qfrc_bias[:N_ROBOT_JOINTS].copy()

            # 控制策略：
            #   - 被拖动时（速度大）：保持目标跟随当前位置，弱 PD/阻尼
            #   - 静止时（速度小）：锁定保持位置，用 PD 悬浮
            if qd_norm < drag_threshold:
                still_counter += 1
                if still_counter > 5:
                    q_hold[:N_ARM_JOINTS] = q[:N_ARM_JOINTS]
                tau = tau_dyn + kp_hold * (q_hold - q) - kd_hold * qd
            else:
                still_counter = 0
                q_hold[:N_ARM_JOINTS] = q[:N_ARM_JOINTS]
                tau = tau_dyn + kp_drag * (q_hold - q) - kd_drag * qd

            # 对机械臂关节施加广义外力
            mj_data.qfrc_applied[:N_ARM_JOINTS] = tau[:N_ARM_JOINTS]
            # 夹爪直接锁定到目标位置 (joint7 + left + right)
            mj_data.qpos[N_ARM_JOINTS : N_ARM_JOINTS + N_GRIPPER_JOINTS] = q_hold[
                N_ARM_JOINTS : N_ARM_JOINTS + N_GRIPPER_JOINTS
            ]
            mj_data.qvel[N_ARM_JOINTS : N_ARM_JOINTS + N_GRIPPER_JOINTS] = 0.0

            mujoco.mj_step(mj_model, mj_data)
            viewer.sync()

            # 每 50 帧打印一次重力力矩
            if int(mj_data.time / dt) % 50 == 0:
                print(
                    f"  tau_g (N·m): [{' '.join(f'{x:6.3f}' for x in tau_g)}] "
                    f"qd_norm={qd_norm:.2f}"
                )

            time.sleep(max(0.0, dt - 0.001))

    print("\n退出仿真。")


if __name__ == "__main__":
    main()
