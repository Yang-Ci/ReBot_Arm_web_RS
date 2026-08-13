#!/usr/bin/env python3
"""示例 1：加载 MuJoCo 模型并运行一个简单仿真，验证模型与基础接口。

用法:
    python examples/01_load_model.py
    python examples/01_load_model.py --viewer   # 打开 MuJoCo 可视化窗口
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

from rebot_b601_rs_sim.robot.model import RobotModel


def run_headless(robot: RobotModel) -> None:
    q0 = np.array([0.0, 0.5, 1.0, 0.0, 0.5, 0.0])
    robot.reset(q0)
    print(f"Initial q: {robot.get_q()}")
    for _ in range(500):
        robot.step()
    print(f"After 1s q: {robot.get_q()}")
    print("Model loaded successfully.")


def run_with_viewer(robot: RobotModel) -> None:
    q0 = np.array([0.0, 0.5, 1.0, 0.0, 0.5, 0.0])
    robot.reset(q0)

    dt = float(robot.model.opt.timestep)
    with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
        while viewer.is_running():
            robot.step()
            viewer.sync()
            time.sleep(dt)
    print("Viewer closed.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Load MuJoCo model and run a simple simulation.")
    parser.add_argument("--viewer", action="store_true", help="Open MuJoCo viewer window")
    args = parser.parse_args()

    print("Loading MuJoCo model...")
    robot = RobotModel()
    print(f"  joints: {robot.joint_ids}")
    print(f"  actuators: {robot.actuator_ids}")
    print(f"  timestep: {robot.model.opt.timestep}")

    if args.viewer:
        run_with_viewer(robot)
    else:
        run_headless(robot)


if __name__ == "__main__":
    main()
