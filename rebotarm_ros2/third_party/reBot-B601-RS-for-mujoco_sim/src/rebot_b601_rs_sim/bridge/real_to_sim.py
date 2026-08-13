"""Real-to-Sim：将真实机器人状态同步到 MuJoCo 仿真。"""

from __future__ import annotations

import subprocess
import time
from typing import TYPE_CHECKING

import numpy as np

from ..config import JOINT_NAMES, NUM_JOINTS
from ..robot.model import RobotModel

if TYPE_CHECKING:
    pass


class MockJointGroup:
    """模拟真实机械臂中的单个 JointGroup（arm / gripper）。"""

    is_mock = True

    def __init__(self, name: str, q0: np.ndarray, joint_names: list[str]) -> None:
        self.name = name
        self._joint_names = list(joint_names)
        self.q = np.asarray(q0, dtype=float).reshape(-1).copy()
        self._tau = np.zeros_like(self.q)
        self._mode = "mit"

    @property
    def num_joints(self) -> int:
        return len(self.q)

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    @property
    def mode(self) -> str:
        return self._mode

    def enable(self) -> None:
        pass

    def disable(self) -> None:
        pass

    def mode_mit(self, kp=None, kd=None) -> bool:
        self._mode = "mit"
        return True

    def mode_pos_vel(self, vlim=None) -> bool:
        self._mode = "pos_vel"
        return True

    def send_mit(
        self,
        pos: np.ndarray,
        vel: np.ndarray | None = None,
        kp: np.ndarray | None = None,
        kd: np.ndarray | None = None,
        tau: np.ndarray | None = None,
    ) -> None:
        """模拟 MIT 命令：记录期望位置与前馈力矩，并简单积分位置。

        力矩对位置的影响很小，主要用于反映电机输出，而不让 mock 位置漂移。
        """
        pos = np.asarray(pos, dtype=float).reshape(-1)
        if pos.shape[0] != self.num_joints:
            raise ValueError(
                f"[{self.name}] Expected {self.num_joints} MIT positions, got {pos.shape[0]}"
            )
        tau = np.asarray(tau, dtype=float).reshape(-1) if tau is not None else np.zeros(self.num_joints)
        kp = np.asarray(kp, dtype=float).reshape(-1) if kp is not None else np.zeros(self.num_joints)
        kd = np.asarray(kd, dtype=float).reshape(-1) if kd is not None else np.zeros(self.num_joints)

        # 简化的电机模型：目标位置牵引 + 可忽略的随机前馈力矩位移
        error = pos - self.q
        self.q += 0.001 * (kp * error) + 0.000005 * tau
        self._tau = tau.copy()

    def send_pos_vel(self, pos, vlim=None) -> None:
        pos = np.asarray(pos, dtype=float).reshape(-1)
        if pos.shape[0] == self.num_joints:
            self.q = pos.copy()

    def get_positions(self, request_feedback: bool = True) -> np.ndarray:
        return self.q.copy()

    def get_velocities(self, request_feedback: bool = True) -> np.ndarray:
        return np.zeros(self.num_joints)

    def __repr__(self) -> str:
        return f"MockJointGroup({self.name!r}, joints={self.num_joints})"


class MockRealRobot:
    """无硬件时的模拟真实机器人，便于离线调试。"""

    is_mock = True

    def __init__(
        self,
        q0: np.ndarray | None = None,
        num_joints: int = NUM_JOINTS,
        has_gripper: bool | None = None,
    ) -> None:
        self.num_joints = num_joints
        self.q = np.asarray(q0, dtype=float) if q0 is not None else np.zeros(num_joints)
        if self.q.shape[0] != num_joints:
            raise ValueError(f"Expected {num_joints} joint values, got {self.q.shape[0]}")

        if has_gripper is None:
            has_gripper = num_joints > NUM_JOINTS

        arm_q = self.q[:NUM_JOINTS]
        self.arm = MockJointGroup("arm", arm_q, JOINT_NAMES)

        if has_gripper:
            gripper_q = self.q[NUM_JOINTS:] if len(self.q) > NUM_JOINTS else np.array([0.0])
            self.gripper = MockJointGroup("gripper", gripper_q, ["gripper"])
            self._has_gripper = True
        else:
            self.gripper = MockJointGroup("gripper", np.array([0.0]), ["gripper"])
            self._has_gripper = False

    @property
    def arm_group(self) -> MockJointGroup:
        """与 RebotArmClient 统一接口的分组访问。"""
        return self.arm

    @property
    def gripper_group(self) -> MockJointGroup:
        """与 RebotArmClient 统一接口的分组访问。"""
        return self.gripper

    @property
    def has_gripper(self) -> bool:
        return self._has_gripper

    @property
    def joint_names(self) -> list[str]:
        names = list(JOINT_NAMES)
        if self._has_gripper:
            names.append("gripper")
        return names

    def get_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if self._has_gripper:
            q = np.concatenate([self.arm.q, self.gripper.q])
        else:
            q = self.arm.q.copy()
        dq = np.zeros_like(q)
        tau = np.concatenate([self.arm._tau, self.gripper._tau]) if self._has_gripper else self.arm._tau.copy()
        return q.copy(), dq.copy(), tau.copy()

    def disconnect(self) -> None:
        pass


class RebotArmClient:
    """封装 SDK ``RebotArm``，提供连接检查、状态读取与安全断开。"""

    def __init__(self, hw_yaml: str = "rebotarm_rs.yaml", channel: str = "can0") -> None:
        from reBotArm_control_py.actuator.rebotarm import RebotArm

        self.hw_yaml = hw_yaml
        self.channel = channel
        self.arm = RebotArm(hw_yaml=hw_yaml)
        self._connected = False

    @property
    def arm_group(self):
        """arm 关节组（JointGroup），用于发送 MIT / POS_VEL 命令。"""
        return self.arm.arm

    @property
    def gripper_group(self):
        """gripper 关节组（JointGroup），用于发送 MIT / POS_VEL 命令。"""
        return self.arm.gripper

    def check_can(self) -> bool:
        """检查指定的 CAN 接口是否已启动。"""
        try:
            result = subprocess.run(
                ["ip", "link", "show", self.channel],
                capture_output=True,
                text=True,
                check=True,
            )
            return "state UP" in result.stdout or "UP" in result.stdout
        except Exception:
            return False

    def connect(self, enable: bool = True, hold: bool = True) -> None:
        """连接真实机械臂。

        连接前会检查 CAN 接口是否已启动；若未启动则抛出清晰提示。
        连接成功后默认对所有关节上使能并切换到 MIT 模式，以便持续读取反馈。

        Args:
            enable: 是否对关节上使能。必须使能才能读取到有效反馈。
            hold: 使能后是否保持位置（hold=True）。若只想被动拖动机械臂
                并观察状态变化，可设 ``hold=False``，读取时会持续发送
                零力矩 MIT 命令，电机不产生保持力矩但仍能触发反馈回传。
        """
        from motorbridge.models import Mode

        if not self.check_can():
            raise RuntimeError(
                f"CAN interface '{self.channel}' is not up. "
                f"Run: sudo ip link set {self.channel} up type can bitrate 500000"
            )
        self.arm.connect()
        self._connected = True
        self._hold = hold
        if enable:
            self.arm.enable_all()
            time.sleep(0.3)
            for g in self.arm.groups.values():
                try:
                    g.mode_mit()
                except Exception as exc:
                    print(f"[RebotArmClient] mode_mit failed for group {g.name}: {exc}")
            if not hold:
                # 先发送一次零力矩 MIT，让电机进入零力矩状态
                self._send_zero_mit()

    @property
    def joint_names(self) -> list[str]:
        return list(self.arm.joint_names)

    def _send_zero_mit(self) -> None:
        """发送零力矩 MIT 命令，用于触发反馈回传或解除保持力矩。"""
        for m in self.arm._motor_map.values():
            try:
                m.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
            except Exception:
                pass

    def get_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """读取真实机器人状态，返回 (q, dq, tau)。

        RobStride 电机需要通过发送 MIT 命令触发反馈回传，因此每次读取前
        会先发送零力矩 MIT 命令，再 poll 反馈并读取各电机状态。
        """
        # 发送零力矩 MIT 命令触发反馈回传
        self._send_zero_mit()
        time.sleep(0.002)

        # 轮询反馈
        for ctrl in self.arm._ctrl_map.values():
            try:
                ctrl.poll_feedback_once()
            except Exception:
                pass

        pos, vel, torq = [], [], []
        for jc in self.arm._all_joints:
            st = self.arm._motor_map[jc.name].get_state()
            if st is not None:
                pos.append(st.pos)
                vel.append(st.vel)
                torq.append(st.torq)
            else:
                pos.append(0.0)
                vel.append(0.0)
                torq.append(0.0)
        return (
            np.array(pos, dtype=np.float64),
            np.array(vel, dtype=np.float64),
            np.array(torq, dtype=np.float64),
        )

    def disconnect(self) -> None:
        """安全断开：失能所有电机并标记为未连接。"""
        if self._connected:
            try:
                self.arm.disable_all()
            except Exception as exc:
                print(f"[RebotArmClient] disable_all failed: {exc}")
            self._connected = False

    def estop(self) -> None:
        """紧急停止：立即失能电机。"""
        self.disconnect()


class RealToSimBridge:
    """把真实机械臂的关节状态实时写入 MuJoCo，构建数字孪生。

    支持两种工作模式：
      1. 连接真实硬件：传入 ``RebotArmClient`` 实例。
      2. 模拟模式：不传入或传入 ``MockRealRobot``，使用模拟状态。

    关节映射按照名称进行，确保真实机器人与 MuJoCo 模型的关节顺序一致。
    """

    def __init__(
        self,
        robot: RobotModel,
        arm_interface: RebotArmClient | MockRealRobot | None = None,
        gripper_scale: float = 0.05 / 6.021,
        gripper_offset: float = 0.0,
    ) -> None:
        """
        Args:
            robot: MuJoCo 机器人模型封装。
            arm_interface: 真实机械臂客户端。若为 None，则使用模拟模式。
            gripper_scale: 真实 gripper 电机位置到 MuJoCo 夹爪直线位移的缩放系数。
                当前 XML 中 joint_left/joint_right 范围为 0~0.05 m；
                真实电机 0°~345°（0 rad ~ 6.021 rad）对应该范围，
                因此默认值为 0.05/6.021 ≈ 0.00830。
            gripper_offset: 真实 gripper 电机对应 MuJoCo 夹爪闭合（disp=0）的角度。
                默认 0.0 rad（0° 闭合）。
        """
        import mujoco

        self.robot = robot
        self.arm_interface = arm_interface
        self._mock_mode = arm_interface is None or getattr(arm_interface, "is_mock", False)
        self._gripper_scale = gripper_scale
        self._gripper_offset = gripper_offset

        if arm_interface is None:
            self.arm_interface = MockRealRobot()

        real_joint_names = self.arm_interface.joint_names
        self._real_indices: list[int] = []
        missing: list[str] = []
        for name in JOINT_NAMES:
            try:
                self._real_indices.append(real_joint_names.index(name))
            except ValueError:
                missing.append(name)
        if missing:
            raise ValueError(
                f"Real robot joint list {real_joint_names} is missing "
                f"joints expected by simulation: {missing}"
            )

        # 夹爪映射：真实 gripper -> MuJoCo joint_left / joint_right / joint7
        self._gripper_real_index: int | None = None
        self._gripper_sim_addrs: list[int] = []
        self._gripper7_sim_addr: int | None = None
        if "gripper" in real_joint_names:
            self._gripper_real_index = real_joint_names.index("gripper")
            for sim_name in ("joint_left", "joint_right"):
                jid = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_JOINT, sim_name)
                if jid >= 0:
                    self._gripper_sim_addrs.append(robot.model.jnt_qposadr[jid])
            jid7 = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_JOINT, "joint7")
            if jid7 >= 0:
                self._gripper7_sim_addr = int(robot.model.jnt_qposadr[jid7])

    @property
    def is_mock(self) -> bool:
        """是否为模拟模式（无真实硬件连接）。"""
        return self._mock_mode

    @property
    def gripper_scale(self) -> float:
        """真实 gripper 电机位置到 MuJoCo 夹爪直线位移的缩放系数。"""
        return self._gripper_scale

    @property
    def gripper_offset(self) -> float:
        """真实 gripper 电机对应 MuJoCo 夹爪闭合（disp=0）的角度。"""
        return self._gripper_offset

    @property
    def gripper_sim_addrs(self) -> list[int]:
        """MuJoCo 中左右夹爪滑动关节的 qpos 地址。"""
        return list(self._gripper_sim_addrs)

    @property
    def gripper_real_index(self) -> int | None:
        """真实机器人关节列表中 gripper 电机的索引。"""
        return self._gripper_real_index

    def read_real_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """读取真实机械臂完整状态。

        Returns:
            (q, dq, tau)，维度均为 (num_real_joints,)。
        """
        q_full, dq_full, tau_full = self.arm_interface.get_state()
        return (
            np.asarray(q_full, dtype=float),
            np.asarray(dq_full, dtype=float),
            np.asarray(tau_full, dtype=float),
        )

    def sync(self, q_real: np.ndarray | None = None) -> None:
        """将真实状态写入仿真。

        Args:
            q_real: 若提供，直接使用该关节位置（支持完整真实关节数组或 6-DOF 臂关节数组）；否则从硬件读取。
        """
        import mujoco

        q_full: np.ndarray | None = None
        if q_real is not None:
            q_arr = np.asarray(q_real, dtype=float)
            if q_arr.shape[0] == NUM_JOINTS:
                q_mapped = q_arr
                dq = np.zeros(NUM_JOINTS)
            elif q_arr.shape[0] >= NUM_JOINTS:
                q_full = q_arr
                q_mapped = q_arr[self._real_indices]
                dq = np.zeros(NUM_JOINTS)
            else:
                raise ValueError(
                    f"Expected at least {NUM_JOINTS} joint values, got {q_arr.shape[0]}"
                )
        else:
            q_full, dq_full, _ = self.read_real_state()
            q_mapped = q_full[self._real_indices]
            dq = dq_full[self._real_indices]

        self.robot.set_q(q_mapped, forward=False)

        # 写入关节速度（MuJoCo 中 1-DoF 关节的 qvel 地址与 qpos 地址相同）
        for addr, v in zip(self.robot.joint_qpos_addrs, dq):
            if addr < self.robot.model.nv:
                self.robot.data.qvel[addr] = v

        # 同步夹爪：真实 gripper 电机位置 -> MuJoCo 左右滑动位移
        if self._gripper_real_index is not None and q_full is not None:
            gripper_pos = q_full[self._gripper_real_index]
            disp = self._gripper_scale * (gripper_pos - self._gripper_offset)
            # 裁剪到各关节允许范围，避免负值或超限
            for addr in self._gripper_sim_addrs:
                if addr < self.robot.model.nq:
                    jid = next(
                        (i for i in range(self.robot.model.njnt)
                         if self.robot.model.jnt_qposadr[i] == addr),
                        -1,
                    )
                    if jid >= 0:
                        lo, hi = self.robot.model.jnt_range[jid]
                        disp = float(np.clip(disp, lo, hi))
                    self.robot.data.qpos[addr] = disp

            # 同步 XML 中新增的 joint7 驱动关节，保证 equality 约束一致性
            if self._gripper7_sim_addr is not None:
                self.robot.data.qpos[self._gripper7_sim_addr] = float(
                    np.clip(disp, 0.0, self.robot.model.jnt_range[
                        mujoco.mj_name2id(self.robot.model, mujoco.mjtObj.mjOBJ_JOINT, "joint7")
                    ][1])
                )

        mujoco.mj_forward(self.robot.model, self.robot.data)

    def close(self) -> None:
        """断开真实机械臂连接（模拟模式下无操作）。"""
        if self.arm_interface is not None and hasattr(self.arm_interface, "disconnect"):
            self.arm_interface.disconnect()

    def __enter__(self) -> "RealToSimBridge":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()


def create_real_arm(
    hw_yaml: str = "rebotarm_rs.yaml",
    channel: str = "can0",
    fallback_to_mock: bool = False,
    hold: bool = True,
    return_mock: bool = False,
    mock_q0: np.ndarray | None = None,
    mock_num_joints: int = 7,
    mock_has_gripper: bool = True,
) -> RebotArmClient | MockRealRobot | None:
    """尝试连接真实机械臂，失败时可选回退到模拟模式。

    Args:
        hw_yaml: SDK 硬件配置文件名。
        channel: CAN 接口名，默认 can0。
        fallback_to_mock: 连接失败时是否回退到模拟模式。
        hold: 是否让电机保持位置。设为 False 可让机械臂被手动拖动。
        return_mock: 为 True 时，fallback_to_mock 失败直接返回 ``MockRealRobot``；
            否则返回 ``None``（由调用方自行创建 mock）。
        mock_q0: 创建 ``MockRealRobot`` 时的初始关节角。
        mock_num_joints: ``MockRealRobot`` 的关节数。
        mock_has_gripper: ``MockRealRobot`` 是否包含夹爪。

    Returns:
        已连接的 ``RebotArmClient``；
        或 ``MockRealRobot``（fallback_to_mock=True 且 return_mock=True 且失败时）；
        或 ``None``（fallback_to_mock=True 且 return_mock=False 且失败时）。

    Raises:
        RuntimeError: 真实硬件不可用且 fallback_to_mock=False 时。
    """
    try:
        client = RebotArmClient(hw_yaml=hw_yaml, channel=channel)
        client.connect(hold=hold)
        return client
    except Exception as exc:
        if fallback_to_mock:
            print(f"[WARN] Failed to connect to real robot: {exc}")
            print("[WARN] Falling back to mock mode.")
            if return_mock:
                return MockRealRobot(
                    q0=mock_q0,
                    num_joints=mock_num_joints,
                    has_gripper=mock_has_gripper,
                )
            return None
        raise
