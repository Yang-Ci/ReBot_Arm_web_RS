"""MuJoCo 机器人模型加载与关节/执行器封装。"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np

from ..config import JOINT_NAMES, MJCF_PATH, NUM_JOINTS, ROBOT_XML_PATH


class RobotModel:
    """封装 MuJoCo 模型，提供关节索引、执行器索引与常用状态读写接口。"""

    def __init__(self, xml_path: str | Path | None = None) -> None:
        """
        Args:
            xml_path: MuJoCo MJCF/XML 文件路径。默认使用 ``assets/00_arm_rs_asm_v3/scene.xml``。
        """
        self.xml_path = Path(xml_path) if xml_path is not None else MJCF_PATH
        if not self.xml_path.exists():
            raise FileNotFoundError(f"MuJoCo model not found: {self.xml_path}")

        # 直接使用用户手动转换的 XML，不再从 URDF 自动生成
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)

        # 关节 ID 与 qpos 地址
        self.joint_ids: list[int] = []
        self.joint_qpos_addrs: list[int] = []
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"Joint '{name}' not found in MuJoCo model")
            self.joint_ids.append(jid)
            self.joint_qpos_addrs.append(self.model.jnt_qposadr[jid])

        # 执行器 ID（命名约定：actuator_jointN）
        self.actuator_ids: list[int] = []
        for name in JOINT_NAMES:
            aname = f"actuator_{name}"
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
            if aid < 0:
                # 允许没有执行器，但会在控制时报错
                aid = -1
            self.actuator_ids.append(aid)

        # 末端执行器 body（用于获取位姿）
        self.ee_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "link6"
        )
        if self.ee_body_id < 0:
            # 回退到最后一个 link
            self.ee_body_id = self.model.nbody - 1

    # ── 关节位置/速度/力矩读写 ───────────────────────────────────────────────────

    def get_q(self) -> np.ndarray:
        """获取当前关节位置 (NUM_JOINTS,)。"""
        return np.array([self.data.qpos[a] for a in self.joint_qpos_addrs])

    def get_dq(self) -> np.ndarray:
        """获取当前关节速度 (NUM_JOINTS,)。"""
        return np.array([self.data.qvel[a] for a in self.joint_qpos_addrs])

    def set_q(self, q: Sequence[float] | np.ndarray, forward: bool = True) -> None:
        """设置关节位置。"""
        q = np.asarray(q, dtype=float)
        if q.shape[0] != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} joint values, got {q.shape[0]}")
        for addr, val in zip(self.joint_qpos_addrs, q):
            self.data.qpos[addr] = val
        if forward:
            mujoco.mj_forward(self.model, self.data)

    def set_ctrl(self, ctrl: Sequence[float] | np.ndarray) -> None:
        """设置执行器控制量 (NUM_JOINTS,)。"""
        ctrl = np.asarray(ctrl, dtype=float)
        if ctrl.shape[0] != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} control values, got {ctrl.shape[0]}")
        for aid, val in zip(self.actuator_ids, ctrl):
            if aid >= 0:
                self.data.ctrl[aid] = val

    def set_qfrc_applied(self, tau: Sequence[float] | np.ndarray) -> None:
        """直接设置关节广义力 (NUM_JOINTS,)，绕过 actuator。"""
        tau = np.asarray(tau, dtype=float)
        if tau.shape[0] != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} force values, got {tau.shape[0]}")
        for addr, val in zip(self.joint_qpos_addrs, tau):
            if addr < self.model.nv:
                self.data.qfrc_applied[addr] = val

    # ── 末端位姿 ─────────────────────────────────────────────────────────────────

    def get_ee_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """获取末端执行器位置与旋转矩阵（世界坐标系）。

        Returns:
            position (3,), rotation (3, 3)
        """
        xmat = self.data.xmat[self.ee_body_id].reshape(3, 3)
        xpos = self.data.xpos[self.ee_body_id].copy()
        return xpos, xmat

    # ── 仿真步进 ─────────────────────────────────────────────────────────────────

    def step(self) -> None:
        """推进一个仿真步。"""
        mujoco.mj_step(self.model, self.data)

    def reset(self, q: Sequence[float] | np.ndarray | None = None) -> None:
        """重置仿真状态。"""
        mujoco.mj_resetData(self.model, self.data)
        if q is not None:
            self.set_q(q, forward=True)
        else:
            mujoco.mj_forward(self.model, self.data)
