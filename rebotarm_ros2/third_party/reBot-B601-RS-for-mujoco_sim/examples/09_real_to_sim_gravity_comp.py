#!/usr/bin/env python3
"""示例 9：真实机械臂重力补偿 + MuJoCo 数字孪生（无抓取方块/夹爪力反馈）。

功能：
    1. 真实 B601-RS 机械臂进入 MIT 重力补偿模式，可自由拖动。
    2. 真实臂/夹爪状态实时同步到 MuJoCo 数字孪生。
    3. 夹爪只跟随真实电机位置，不提供接触力反馈，方便手动自由开合。

主要参数：
    --mock              使用模拟数据，不连接真实硬件。
    --headless          不打开 MuJoCo 可视化窗口。
    --can can0          CAN 接口名（默认 can0）。
    --hw-yaml FILE      SDK 硬件配置文件（默认 rebotarm_rs.yaml）。
    --rate HZ           控制/同步频率（默认 50）。
    --no-hold           使能后电机不保持位置，方便手动拖动。

臂重力补偿：
    --kp-arm 0.0        臂关节 MIT kp（默认 0，纯重力补偿可拖动）。
    --kd-arm 0.2        臂关节 MIT kd。
    --gravity-scale     重力补偿缩放。支持单个值或 6 个逗号分隔值，
                        例如 --gravity-scale 1.0,1.2,1.0,1.0,1.0,1.0。
                        也可以在文件顶部直接修改 GRAVITY_SCALE 数组。
    --tau-arm-limit 20.0  臂前馈力矩上限（N·m）。

夹爪同步：
    --kp-gripper 0.0    夹爪 MIT kp（默认 0，纯位置跟随，可自由拖动）。
    --kd-gripper 0.05   夹爪 MIT kd。
    --gripper-scale 0.00833  真实夹爪电机位置（rad）到 MuJoCo 直线位移（m）的缩放。

用法示例：
    # 连接真实机械臂（需先启动 CAN 接口）
    python examples/09_real_to_sim_gravity_comp.py --no-hold

    # 模拟模式（无硬件）
    python examples/09_real_to_sim_gravity_comp.py --mock --headless

真实机械臂连接前请确保：
    sudo ip link set can0 up type can bitrate 500000

交互提示：
    - 在真实模式下，用手拖动机械臂和夹爪，MuJoCo 数字孪生会实时跟随。
    - 夹爪没有力反馈，可自由开合，适合不需要抓取物体的场景。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rebot_b601_rs_sim.bridge.real_to_sim import (
    MockRealRobot,
    RealToSimBridge,
    RebotArmClient,
    create_real_arm,
)
from rebot_b601_rs_sim.control.gravity_compensation import GravityCompensator
from rebot_b601_rs_sim.robot.model import RobotModel
from rebot_b601_rs_sim.utils import limit_array, parse_gravity_scale, run_passive_loop

# 按关节 1~6 的重力补偿缩放系数。
GRAVITY_SCALE = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

# mock 模式下的初始关节角：臂 6-DOF + 夹爪 1-DOF
Q0_MOCK = np.array([0.0, 1.17, 0.51, 0.0, 0.0, 0.0, np.deg2rad(220.0)])


def _build_parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="真实机械臂重力补偿 + MuJoCo 数字孪生同步（无夹爪力反馈）。"
    )
    parser.add_argument("--mock", action="store_true", help="使用模拟数据，不连接真实硬件")
    parser.add_argument("--headless", action="store_true", help="不打开 MuJoCo 可视化窗口")
    parser.add_argument("--can", type=str, default="can0", help="CAN 接口名（默认 can0）")
    parser.add_argument(
        "--hw-yaml", type=str, default="rebotarm_rs.yaml",
        help="SDK 硬件配置文件路径（默认 rebotarm_rs.yaml）",
    )
    parser.add_argument("--rate", type=float, default=50.0, help="控制与同步频率（Hz，默认 50）")
    parser.add_argument(
        "--no-hold", action="store_true",
        help="使能后电机不保持位置，方便手动拖动",
    )
    parser.add_argument(
        "--gripper-scale", type=float, default=0.05 / 6.021,
        help="真实夹爪电机位置（rad）到 MuJoCo 直线位移（m）的缩放。"
             "默认：0.00830",
    )
    parser.add_argument(
        "--gripper-offset", type=float, default=0.0,
        help="真实夹爪电机在完全闭合时对应的角度（rad）。默认：0.0 rad",
    )
    parser.add_argument(
        "--kp-arm", type=float, default=0.0,
        help="臂关节 MIT kp（默认 0.0，设为 0 时可手动拖动）",
    )
    parser.add_argument(
        "--kd-arm", type=float, default=0.2,
        help="臂关节 MIT kd（默认 0.2，提供少量阻尼提高稳定性）",
    )
    parser.add_argument(
        "--gravity-scale", type=parse_gravity_scale, default=GRAVITY_SCALE.copy(),
        help="各关节重力补偿力矩缩放系数。示例：--gravity-scale 1.0,1.2,1.0,1.0,1.0,1.0",
    )
    parser.add_argument(
        "--kp-gripper", type=float, default=0.0,
        help="夹爪 MIT kp（默认 0.0，设为 0 时可自由开合）",
    )
    parser.add_argument(
        "--kd-gripper", type=float, default=0.05,
        help="夹爪 MIT kd（默认 0.05）。保持较小值，方便用手打开夹爪。",
    )
    parser.add_argument(
        "--tau-arm-limit", type=float, default=20.0,
        help="臂前馈力矩上限（N·m，默认 20.0）",
    )
    return parser


def _create_arm_interface(args: argparse.Namespace) -> MockRealRobot | RebotArmClient:
    """根据命令行参数创建机械臂接口（真实或模拟）。"""
    if args.mock:
        print("[mock mode] No real hardware connected.")
        return MockRealRobot(q0=Q0_MOCK, num_joints=7, has_gripper=True)

    arm_interface = create_real_arm(
        hw_yaml=args.hw_yaml,
        channel=args.can,
        fallback_to_mock=True,
        hold=not args.no_hold,
        return_mock=True,
        mock_q0=Q0_MOCK,
        mock_num_joints=7,
        mock_has_gripper=True,
    )
    if getattr(arm_interface, "is_mock", False):
        print("[mock mode] Using mock data because real hardware is unavailable.")
    else:
        print(f"Real robot connected via {args.can}.")
    return arm_interface


def _print_initial_state(
    q_arm: np.ndarray,
    q_gripper: np.ndarray | None,
    args: argparse.Namespace,
) -> None:
    """打印初始状态与控制参数。"""
    print("Initial state synced.")
    print("  arm q (deg):", np.degrees(q_arm).round(2))
    if q_gripper is not None:
        print("  gripper q (deg):", np.degrees(q_gripper).round(2))
    print("Controls:")
    print(f"  arm kp/kd={args.kp_arm}/{args.kd_arm}")
    print(f"  gravity_scale per joint: {args.gravity_scale.round(3).tolist()}")
    if q_gripper is not None:
        print(f"  gripper kp/kd={args.kp_gripper}/{args.kd_gripper}")
    print(f"  tau arm limit={args.tau_arm_limit}")
    if not args.no_hold:
        print("  [HINT] If the arm feels locked, run with --no-hold to disable motor holding torque.")
    print("Close the viewer or press Ctrl+C to stop.")


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    print("[09_gravity_comp] Starting...", flush=True)

    robot = RobotModel()
    arm_interface = _create_arm_interface(args)
    has_gripper = getattr(arm_interface, "has_gripper", True)

    bridge = RealToSimBridge(
        robot,
        arm_interface=arm_interface,
        gripper_scale=args.gripper_scale,
        gripper_offset=args.gripper_offset,
    )
    gravity_comp = GravityCompensator()

    arm_group = arm_interface.arm_group
    gripper_group = arm_interface.gripper_group if has_gripper else None

    # 首次同步：与主循环一致，先读取 arm_group 状态再写入 MuJoCo
    q_arm = arm_group.get_positions()
    q_gripper = gripper_group.get_positions() if gripper_group is not None else None
    bridge.sync(np.concatenate([q_arm, q_gripper]) if q_gripper is not None else q_arm.copy())
    _print_initial_state(q_arm, q_gripper, args)

    dt = 1.0 / max(args.rate, 1.0)

    def step_loop() -> str:
        """单次控制循环：读取 -> 同步 -> 臂重力补偿 -> 仿真步 -> 夹爪位置跟随。"""
        # 1. 读取真实状态（只读一次，避免多次触发硬件通信）
        q_arm = arm_group.get_positions()
        dq_arm = arm_group.get_velocities()
        if gripper_group is not None:
            q_gripper = gripper_group.get_positions()
            dq_gripper = gripper_group.get_velocities()
            q_real = np.concatenate([q_arm, q_gripper])
        else:
            q_real = q_arm.copy()

        # 2. 同步到 MuJoCo
        bridge.sync(q_real)

        # 3. 臂重力补偿
        tau_g = args.gravity_scale * gravity_comp.compute(q_arm)
        tau_arm = tau_g - args.kd_arm * dq_arm
        tau_arm = limit_array(tau_arm, args.tau_arm_limit)
        arm_group.send_mit(
            pos=q_arm,
            vel=np.zeros(arm_group.num_joints),
            kp=np.full(arm_group.num_joints, args.kp_arm),
            kd=np.full(arm_group.num_joints, args.kd_arm),
            tau=tau_arm,
        )

        # 4. 推进 MuJoCo 仿真一步
        robot.step()

        # 5. 夹爪只发送当前位置，无接触力反馈
        if gripper_group is not None:
            q_g = float(q_gripper.item())
            dq_g = float(dq_gripper.item())
            tau_gripper = -args.kd_gripper * dq_g
            tau_gripper = limit_array(tau_gripper, 1.0)
            gripper_group.send_mit(
                pos=q_g,
                vel=np.zeros(gripper_group.num_joints),
                kp=np.full(gripper_group.num_joints, args.kp_gripper),
                kd=np.full(gripper_group.num_joints, args.kd_gripper),
                tau=tau_gripper,
            )

        return f"  tau_g={float(np.linalg.norm(tau_g)):6.3f}N·m"

    try:
        run_passive_loop(
            robot,
            step_loop,
            dt,
            headless=args.headless,
            label="09_gravity_comp",
        )
    finally:
        bridge.close()
        print("[09_gravity_comp] Exited safely.")


if __name__ == "__main__":
    main()
