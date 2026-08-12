from types import SimpleNamespace

import numpy as np

from rebotarmcontroller.hardware_manager import HardwareManager


def _manager_with_cache():
    manager = HardwareManager.__new__(HardwareManager)
    manager._arm_group = SimpleNamespace(joint_names=["joint1", "joint2"])
    manager._robot = SimpleNamespace(has_gripper=True)
    manager._cached_arm_position = np.array([1.0, 2.0])
    manager._cached_arm_velocity = np.array([3.0, 4.0])
    manager._cached_arm_torque = np.array([5.0, 6.0])
    manager._cached_gripper_position = 0.0
    manager._cached_gripper_velocity = 0.0
    manager._cached_gripper_torque = 0.0
    return manager


def test_cached_joint_state_does_not_touch_hardware():
    manager = _manager_with_cache()

    def fail_if_called(*args, **kwargs):
        raise AssertionError("cached read must not poll hardware")

    manager._robot_get_state = fail_if_called
    pos, vel, torque = manager._get_arm_state(request_feedback=False)

    np.testing.assert_allclose(pos, [1.0, 2.0])
    np.testing.assert_allclose(vel, [3.0, 4.0])
    np.testing.assert_allclose(torque, [5.0, 6.0])

    # Callers receive snapshots, not mutable references to the real-time cache.
    pos[0] = 99.0
    assert manager._cached_arm_position[0] == 1.0


def test_feedback_poll_refreshes_arm_and_gripper_cache():
    manager = _manager_with_cache()
    manager._robot_get_state = lambda request_feedback=True: (
        np.array([0.1, 0.2, 4.5]),
        np.array([0.3, 0.4, 0.5]),
        np.array([0.6, 0.7, 0.8]),
    )

    pos, vel, torque = manager._get_arm_state(request_feedback=True)

    np.testing.assert_allclose(pos, [0.1, 0.2])
    np.testing.assert_allclose(vel, [0.3, 0.4])
    np.testing.assert_allclose(torque, [0.6, 0.7])
    assert manager._cached_gripper_position == 4.5
    assert manager._cached_gripper_velocity == 0.5
    assert manager._cached_gripper_torque == 0.8


def test_cached_gripper_state_does_not_poll_hardware():
    manager = _manager_with_cache()
    manager._gripper_name = "gripper"
    manager._cached_gripper_position = 2.5
    manager._cached_gripper_velocity = -0.4
    manager._cached_gripper_torque = 0.7
    manager._robot._motor_map = {}

    def fail_if_called(*args, **kwargs):
        raise AssertionError("safe-home cached J7 read must not poll hardware")

    manager._robot_get_state = fail_if_called
    position, velocity, torque, status = manager.get_gripper_state(
        request_feedback=False
    )

    assert position == 2.5
    assert velocity == -0.4
    assert torque == 0.7
    assert status == 0
