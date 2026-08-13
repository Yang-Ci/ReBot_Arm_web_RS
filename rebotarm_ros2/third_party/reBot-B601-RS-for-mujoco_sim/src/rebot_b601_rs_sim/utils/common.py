"""通用工具函数。"""

from __future__ import annotations

import argparse
from typing import Callable

import mujoco
import mujoco.viewer
import numpy as np

from ..robot.model import RobotModel


def limit_array(arr: np.ndarray, limit: float | np.ndarray) -> np.ndarray:
    """将数组裁剪到 ±limit 范围内，并保证返回至少一维数组。

    Args:
        arr: 输入数组。
        limit: 单个值或各维度对应的限制值。

    Returns:
        裁剪后的数组，且维度至少为 1。
    """
    arr = np.asarray(arr, dtype=float)
    limit = np.asarray(limit, dtype=float)
    return np.atleast_1d(np.clip(arr, -limit, limit))


def parse_gravity_scale(value: str) -> np.ndarray:
    """解析重力补偿缩放系数。

    支持两种形式：
      - 单个值，如 "1.2"，表示 6 个臂关节都用 1.2
      - 逗号分隔的 6 个值，如 "1.0,1.2,1.0,1.0,1.0,1.0"，分别对应 joint1~joint6

    Args:
        value: 命令行传入的字符串。

    Returns:
        长度为 6 的缩放系数数组。

    Raises:
        argparse.ArgumentTypeError: 格式不符合要求时。
    """
    parts = [p.strip() for p in value.split(",")]
    if len(parts) == 1:
        scale = float(parts[0])
        return np.full(6, scale)
    if len(parts) == 6:
        return np.array([float(p) for p in parts], dtype=float)
    raise argparse.ArgumentTypeError(
        f"gravity-scale must be a single float or 6 comma-separated floats, got {value!r}"
    )


def run_passive_loop(
    robot: RobotModel,
    step_callback: Callable[[], str],
    dt: float,
    headless: bool = False,
    label: str = "",
) -> None:
    """运行一个被动控制循环，支持 MuJoCo Viewer 或 headless 模式。

    循环会按固定频率调用 ``step_callback``，并在同一行打印返回的字符串。
    按 Ctrl+C 或关闭 Viewer 时安全退出。

    Args:
        robot: MuJoCo 机器人模型封装。
        step_callback: 每步调用的函数，返回要打印的字符串。
        dt: 每步期望的时间间隔（秒）。
        headless: 为 True 时不打开 Viewer 窗口。
        label: 启动/退出时打印的标识字符串。
    """
    import time

    def _loop_body() -> None:
        t_start = time.perf_counter()
        message = step_callback()
        print(message, end="\r", flush=True)
        elapsed = time.perf_counter() - t_start
        time.sleep(max(0.0, dt - elapsed))

    prefix = f"[{label}] " if label else ""

    if headless:
        print(f"{prefix}Running headless. Press Ctrl+C to stop.")
        try:
            while True:
                _loop_body()
        except KeyboardInterrupt:
            print("\nCtrl+C received, disabling robot...")
    else:
        print(f"{prefix}Opening MuJoCo viewer...")
        with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
            try:
                while viewer.is_running():
                    t_start = time.perf_counter()
                    message = step_callback()
                    viewer.sync()
                    print(message, end="\r", flush=True)
                    elapsed = time.perf_counter() - t_start
                    time.sleep(max(0.0, dt - elapsed))
            except KeyboardInterrupt:
                print("\nCtrl+C received, disabling robot...")
        print("\nViewer closed.")
