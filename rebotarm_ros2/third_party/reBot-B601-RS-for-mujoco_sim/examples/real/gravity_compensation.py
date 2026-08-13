#!/usr/bin/env python3
"""真实机械臂纯重力补偿示例。

只连接真实 B601-RS 硬件，不启动 MuJoCo。
机械臂进入 MIT 重力补偿模式后可被手动拖动。

用法:
    # 连接真实机械臂（需先启动 CAN 接口）
    python examples/real/gravity_compensation.py

    # 模拟模式（无硬件，用于离线验证循环）
    python examples/real/gravity_compensation.py --mock

真实机械臂连接前请确保：
    sudo ip link set can0 up type can bitrate 500000
    
如果 CAN 接口已启动但速率不对，可先关闭再重新设置：
    sudo ip link set can0 down 2>/dev/null
    sudo ip link set can0 type can bitrate 1000000 restart-ms 100
    sudo ip link set can0 up
    ip -details link show can0

交互提示：
    - 运行后用手拖动机械臂，松手后关节会悬浮在当前位置。
    - 按 Ctrl+C 安全退出并失能电机。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from rebot_b601_rs_sim.bridge.real_to_sim import (
    MockRealRobot,
    RebotArmClient,
    create_real_arm,
)
from rebot_b601_rs_sim.control.gravity_compensation import GravityCompensator

# 按关节 1~6 的重力补偿缩放系数。
# 如果某个关节下坠，直接把对应位置的数值改大即可，例如：
# GRAVITY_SCALE = np.array([1.0, 1.2, 1.0, 1.0, 1.0, 1.0])
GRAVITY_SCALE = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])


def _limit_array(arr: np.ndarray, limit: float) -> np.ndarray:
    """将数组裁剪到 ±limit 范围内。"""
    return np.clip(np.asarray(arr, dtype=float), -limit, limit)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Real robot gravity compensation only."
    )
    parser.add_argument("--mock", action="store_true", help="Use mock data instead of real hardware")
    parser.add_argument("--can", type=str, default="can0", help="CAN interface name (default: can0)")
    parser.add_argument(
        "--hw-yaml",
        type=str,
        default="rebotarm_rs.yaml",
        help="SDK hardware configuration YAML (default: rebotarm_rs.yaml)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Control rate in Hz (default: 50)",
    )
    parser.add_argument(
        "--no-hold",
        action="store_true",
        help="Disable motor holding torque after enable (allows manual dragging)",
    )
    parser.add_argument("--kp", type=float, default=0.0, help="MIT kp (default: 0.0). Set to 0 for pure gravity compensation so the arm can be dragged.")
    parser.add_argument("--kd", type=float, default=0.2, help="MIT kd (default: 0.2). Small damping for stability while dragging.")
    parser.add_argument(
        "--tau-limit",
        type=float,
        default=20.0,
        help="Feedforward torque limit (N·m, default: 20.0)",
    )
    args = parser.parse_args()

    print("[real/gravity_compensation] Starting...", flush=True)

    if args.mock:
        print("[mock mode] No real hardware connected.")
        arm_interface = MockRealRobot(q0=np.zeros(6), num_joints=6, has_gripper=False)
    else:
        arm_interface = create_real_arm(
            hw_yaml=args.hw_yaml,
            channel=args.can,
            fallback_to_mock=True,
            hold=not args.no_hold,
        )
        if arm_interface is None:
            print("[mock mode] Using mock data because real hardware is unavailable.")
            arm_interface = MockRealRobot(q0=np.zeros(6), num_joints=6, has_gripper=False)
        else:
            print(f"Real robot connected via {args.can}.")

    arm_group = arm_interface.arm_group
    gravity_comp = GravityCompensator()

    dt = 1.0 / max(args.rate, 1.0)

    # 首次读取真实状态
    q_arm = arm_group.get_positions()
    print("Initial state:")
    print(f"  arm q (deg): {np.degrees(q_arm).round(2)}")
    print("Controls:")
    print(f"  kp/kd={args.kp}/{args.kd}")
    print(f"  gravity_scale per joint: {GRAVITY_SCALE.round(3).tolist()}")
    print(f"  tau_limit={args.tau_limit}")
    if not args.no_hold:
        print("  [HINT] If the arm feels locked, run with --no-hold to disable motor holding torque.")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            t_start = time.perf_counter()

            # 读取真实状态
            q_arm = arm_group.get_positions()
            dq_arm = arm_group.get_velocities()

            # 计算重力补偿力矩并按关节缩放
            tau_g = GRAVITY_SCALE * gravity_comp.compute(q_arm)
            tau = tau_g - args.kd * dq_arm
            tau = _limit_array(tau, args.tau_limit)

            # 发送 MIT 命令
            arm_group.send_mit(
                pos=q_arm,
                vel=np.zeros(arm_group.num_joints),
                kp=np.full(arm_group.num_joints, args.kp),
                kd=np.full(arm_group.num_joints, args.kd),
                tau=tau,
            )

            tau_norm = float(np.linalg.norm(tau_g))
            print(
                f"  tau_g={tau_norm:6.3f}N·m  q_deg={np.degrees(q_arm).round(1).tolist()}",
                end="\r",
                flush=True,
            )

            elapsed = time.perf_counter() - t_start
            time.sleep(max(0.0, dt - elapsed))
    except KeyboardInterrupt:
        print("\nCtrl+C received, disabling robot...")
    finally:
        if hasattr(arm_interface, "disconnect"):
            arm_interface.disconnect()
        print("[real/gravity_compensation] Exited safely.")


if __name__ == "__main__":
    main()
