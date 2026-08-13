"""测试 RealToSimBridge 模拟模式与硬件回退。"""

from __future__ import annotations

from unittest.mock import patch

import numpy as np

from rebot_b601_rs_sim.bridge.real_to_sim import (
    MockRealRobot,
    RealToSimBridge,
    create_real_arm,
)
from rebot_b601_rs_sim.robot.model import RobotModel


def test_real_to_sim_mock() -> None:
    robot = RobotModel()
    bridge = RealToSimBridge(robot, arm_interface=None)
    assert bridge.is_mock

    q_real = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    bridge.sync(q_real)
    q_sim = robot.get_q()
    print(f"synced q: {q_sim}", flush=True)
    assert np.allclose(q_sim, q_real)


def test_mock_real_robot() -> None:
    robot = RobotModel()
    mock = MockRealRobot(q0=np.array([0.2, -0.1, 0.3, 0.0, 0.1, -0.2]))
    bridge = RealToSimBridge(robot, arm_interface=mock)
    assert bridge.is_mock

    q, dq, tau = bridge.read_real_state()
    assert np.allclose(q, mock.q)
    assert np.allclose(dq, 0.0)
    assert np.allclose(tau, 0.0)

    bridge.sync()
    assert np.allclose(robot.get_q(), mock.q)


def test_create_real_arm_fallback() -> None:
    """CAN 接口未启动时，fallback_to_mock=True 应返回 None。"""
    with patch(
        "rebot_b601_rs_sim.bridge.real_to_sim.RebotArmClient.check_can",
        return_value=False,
    ):
        client = create_real_arm(fallback_to_mock=True)
        assert client is None
