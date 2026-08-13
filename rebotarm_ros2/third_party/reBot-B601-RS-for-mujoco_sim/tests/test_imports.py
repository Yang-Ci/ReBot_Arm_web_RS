"""基础导入测试。"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


def test_package_imports() -> None:
    import rebot_b601_rs_sim
    from rebot_b601_rs_sim import config
    from rebot_b601_rs_sim.bridge import RealToSimBridge
    from rebot_b601_rs_sim.control import (
        GravityCompensator,
        IKSolver,
        JointPositionController,
        TorqueController,
    )
    from rebot_b601_rs_sim.robot import RobotModel, RobotState
    from rebot_b601_rs_sim.simulation import Simulator

    assert rebot_b601_rs_sim is not None
    assert config is not None
    assert IKSolver is not None
    assert GravityCompensator is not None
    assert JointPositionController is not None
    assert TorqueController is not None
    assert RealToSimBridge is not None
    assert RobotModel is not None
    assert RobotState is not None
    assert Simulator is not None


def test_paths() -> None:
    from rebot_b601_rs_sim.config import MJCF_PATH, RS_URDF_PATH, SCENE_PATH, SDK_DIR

    assert SDK_DIR.exists(), f"SDK not found at {SDK_DIR}"
    assert RS_URDF_PATH.exists(), f"URDF not found at {RS_URDF_PATH}"
    assert MJCF_PATH.exists(), f"MJCF not found at {MJCF_PATH}"
    assert SCENE_PATH.exists(), f"Scene XML not found at {SCENE_PATH}"
