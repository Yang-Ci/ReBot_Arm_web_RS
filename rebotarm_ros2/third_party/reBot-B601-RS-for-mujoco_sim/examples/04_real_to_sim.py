#!/usr/bin/env python3
"""示例 4：将真实机器人状态实时同步到 MuJoCo 仿真（数字孪生）。

用法:
    # 连接真实机械臂（需先启动 CAN 接口）
    python examples/04_real_to_sim.py

    # 模拟模式（无硬件，使用测试数据）
    python examples/04_real_to_sim.py --mock

    # 仅无窗口运行，输出关节角度
    python examples/04_real_to_sim.py --headless

真实机械臂连接前请确保：
    sudo ip link set can0 up type can bitrate 500000
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rebot_b601_rs_sim.bridge.real_to_sim import (
    MockRealRobot,
    RealToSimBridge,
    create_real_arm,
)
from rebot_b601_rs_sim.robot.model import RobotModel


def main() -> None:
    parser = argparse.ArgumentParser(description="Sync real robot state to MuJoCo simulation.")
    parser.add_argument("--mock", action="store_true", help="Use mock data instead of real hardware")
    parser.add_argument("--headless", action="store_true", help="Run without MuJoCo viewer window")
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
        help="Real robot state polling rate in Hz (default: 50)",
    )
    parser.add_argument(
        "--no-hold",
        action="store_true",
        help="Disable motor holding torque after enable (allows manual dragging)",
    )
    parser.add_argument(
        "--gripper-scale",
        type=float,
        default=0.05 / 6.021,
        help="Scale from real gripper motor position (rad) to MuJoCo slide displacement (m). "
             "Calibrated for 345deg motor travel (0deg closed -> 345deg open). "
             "Default: 0.00830 (0.05 m / 6.021 rad)",
    )
    parser.add_argument(
        "--gripper-offset",
        type=float,
        default=0.0,
        help="Real gripper motor angle (rad) that corresponds to fully closed (disp=0). "
             "Default: 0.0 rad (0deg)",
    )
    args = parser.parse_args()

    print("[04_real_to_sim] Starting...", flush=True)

    robot = RobotModel()

    if args.mock:
        print("[mock mode] No real hardware connected, using synthetic joint state.")
        arm_interface = MockRealRobot(q0=np.zeros(7), num_joints=7, has_gripper=True)
    else:
        arm_interface = create_real_arm(
            hw_yaml=args.hw_yaml,
            channel=args.can,
            fallback_to_mock=True,
            hold=not args.no_hold,
        )
        if arm_interface is None:
            print("[mock mode] Using mock data because real hardware is unavailable.")
            arm_interface = MockRealRobot()
        else:
            print(f"Real robot connected via {args.can}.")

    with RealToSimBridge(
        robot,
        arm_interface=arm_interface,
        gripper_scale=args.gripper_scale,
        gripper_offset=args.gripper_offset,
    ) as bridge:
        if bridge.is_mock:
            t0 = time.time()
            # mock 状态包含 6 个臂关节 + 1 个 gripper 关节，便于验证夹爪映射
            q_mock = np.zeros(7)
            has_gripper_mock = True
        else:
            has_gripper_mock = False
            q_real, _, _ = bridge.read_real_state()
            bridge.sync()
            print(
                "Initial real state synced: "
                f"q={np.degrees(q_real[:6]).round(2)} deg",
                flush=True,
            )

        dt = float(robot.model.opt.timestep)
        real_dt = 1.0 / max(args.rate, 1.0) if not bridge.is_mock else dt

        run_viewer = not args.headless

        def step_sync() -> tuple[np.ndarray, float | None, float | None, float | None]:
            nonlocal q_mock
            if bridge.is_mock:
                elapsed = time.time() - t0
                q_mock[:6] = 0.3 * np.sin(2.0 * elapsed + np.arange(6))
                # 让 mock gripper 在 0~345° 之间周期性开合，验证映射
                q_mock[6] = np.deg2rad(172.5 + 172.5 * np.sin(0.5 * elapsed))
                bridge.sync(q_mock)
                gripper_pos = float(q_mock[6])
                left_disp = (
                    float(robot.data.qpos[bridge._gripper_sim_addrs[0]])
                    if len(bridge._gripper_sim_addrs) >= 1
                    else None
                )
                right_disp = (
                    float(robot.data.qpos[bridge._gripper_sim_addrs[1]])
                    if len(bridge._gripper_sim_addrs) >= 2
                    else None
                )
                return q_mock[:6], gripper_pos, left_disp, right_disp

            q_real, _, _ = bridge.read_real_state()
            bridge.sync(q_real)

            gripper_pos = left_disp = right_disp = None
            if bridge._gripper_real_index is not None:
                gripper_pos = float(q_real[bridge._gripper_real_index])
                if len(bridge._gripper_sim_addrs) >= 1:
                    left_disp = float(robot.data.qpos[bridge._gripper_sim_addrs[0]])
                if len(bridge._gripper_sim_addrs) >= 2:
                    right_disp = float(robot.data.qpos[bridge._gripper_sim_addrs[1]])
            return q_real[:6], gripper_pos, left_disp, right_disp

        if run_viewer:
            print("Opening MuJoCo viewer. Close the window to stop.")
            with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
                try:
                    while viewer.is_running():
                        q_now, gripper_pos, left_disp, right_disp = step_sync()
                        viewer.sync()
                        if not bridge.is_mock:
                            q_sim = robot.get_q()
                            gp = f"{np.degrees(gripper_pos):.1f}" if gripper_pos is not None else "N/A"
                            ld = f"{left_disp*1000:.1f}" if left_disp is not None else "N/A"
                            rd = f"{right_disp*1000:.1f}" if right_disp is not None else "N/A"
                            print(
                                f"  arm real(deg): {np.degrees(q_now).round(1)} | "
                                f"arm sim(deg): {np.degrees(q_sim).round(1)} | "
                                f"gripper={gp}deg L={ld}mm R={rd}mm",
                                end="\r",
                                flush=True,
                            )
                        time.sleep(real_dt)
                except KeyboardInterrupt:
                    print("\nCtrl+C received, disabling robot...")
            print("\nViewer closed.")
        else:
            print("Running headless real-to-sim sync. Press Ctrl+C to stop.")
            try:
                while True:
                    q_now, gripper_pos, left_disp, right_disp = step_sync()
                    if bridge.is_mock:
                        q_sim = robot.get_q()
                        gp = f"{np.degrees(q_mock[6]):.1f}" if has_gripper_mock else "N/A"
                        ld = f"{left_disp*1000:.1f}" if left_disp is not None else "N/A"
                        rd = f"{right_disp*1000:.1f}" if right_disp is not None else "N/A"
                        print(
                            f"  Sim q (deg): {np.degrees(q_now).round(2)} | "
                            f"sim q (deg): {np.degrees(q_sim).round(2)} | "
                            f"gripper={gp}deg L={ld}mm R={rd}mm",
                            end="\r",
                            flush=True,
                        )
                    else:
                        q_sim = robot.get_q()
                        gp = f"{np.degrees(gripper_pos):.1f}" if gripper_pos is not None else "N/A"
                        ld = f"{left_disp*1000:.1f}" if left_disp is not None else "N/A"
                        rd = f"{right_disp*1000:.1f}" if right_disp is not None else "N/A"
                        print(
                            f"  arm real(deg): {np.degrees(q_now).round(1)} | "
                            f"arm sim(deg): {np.degrees(q_sim).round(1)} | "
                            f"gripper={gp}deg L={ld}mm R={rd}mm",
                            end="\r",
                            flush=True,
                        )
                    time.sleep(real_dt)
            except KeyboardInterrupt:
                print("\nCtrl+C received, disabling robot...")

    print("[04_real_to_sim] Exited safely.")


if __name__ == "__main__":
    main()
