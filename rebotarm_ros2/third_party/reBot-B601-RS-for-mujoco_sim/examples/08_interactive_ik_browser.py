#!/usr/bin/env python3
"""浏览器拖条控制 · IK + MuJoCo 可视化。

用法:
    python examples/08_interactive_ik_browser.py

启动后在浏览器打开 http://localhost:8765，通过拖条设定目标位姿，
MuJoCo viewer 中机械臂将实时跟随。
"""

from __future__ import annotations

import tornado.ioloop
import json
import queue
import signal
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import pinocchio as pin
import tornado.web
import tornado.websocket

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rebot_b601_rs_sim.config import SCENE_PATH
from rebot_b601_rs_sim.control.ik import IKSolver
from rebot_b601_rs_sim.control.qp_velocity_solver import QPVelocitySolver

USE_QP_VELOCITY = True          # 摇杆/速度通道是否使用 QP 求解器
USE_COLLISION_CONSTRAINTS = True  # 速度通道是否启用自碰撞/环境碰撞约束

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
N_ARM_JOINTS = 6
LINEAR_SPEED = 0.15          # m/s，用于估算轨迹时长
HTTP_PORT = 8766
JOY_LINEAR_SPEED = 1.00    # m/s，摇杆推到 100% 对应的最大笛卡尔线速度 (方案 D 速度通道)
VEL_TIKHONOV   = 1e-6       # 阻尼最小二乘 λ,避免雅可比奇异处关节速度爆掉

# 夹爪：真实 7 号电机 0°（闭合）~ 345°（张开）→ MuJoCo 直线位移 0 ~ 0.05 m
GRIPPER_DEG_MAX = 345.0
GRIPPER_DISP_MAX = 0.05               # m
GRIPPER_JOINT7_NAME = "joint7"        # XML 中新增的夹爪驱动关节
GRIPPER_FINGER_JOINTS = ["joint_left", "joint_right"]

# HTML 模板路径
_TEMPLATE_PATH = Path(__file__).resolve().parents[1] / "src" / "rebot_b601_rs_sim" / "templates" / "control_panel.html"

def _load_html() -> str:
    if _TEMPLATE_PATH.exists():
        return _TEMPLATE_PATH.read_text(encoding="utf-8")
    raise FileNotFoundError(f"Template not found: {_TEMPLATE_PATH}")

# 拖条默认范围（与 B601-RS 工作空间大致匹配）
SLIDER_RANGES = {
    "x":     {"min": -0.20, "max": 0.60, "step": 0.001, "default": 0.30},
    "y":     {"min": -0.40, "max": 0.40, "step": 0.001, "default": 0.00},
    "z":     {"min": -0.05, "max": 0.55, "step": 0.001, "default": 0.20},
    "roll":  {"min": -3.14, "max": 3.14, "step": 0.01,  "default": 0.00},
    "pitch": {"min": -3.14, "max": 3.14, "step": 0.01,  "default": 0.00},
    "yaw":   {"min": -3.14, "max": 3.14, "step": 0.01,  "default": 0.00},
}



# ---------------------------------------------------------------------------
# WebSocket 服务（在后台线程中运行）
# ---------------------------------------------------------------------------
# -- tornado handlers (module-level) -----------------------------------------

class _ConstantsHandler(tornado.web.RequestHandler):
    """Serve workspace_constants.json (Pinocchio-derived geometry bounds)."""
    def get(self) -> None:
        path = _TEMPLATE_PATH.parent / "workspace_constants.json"
        if not path.exists():
            self.set_status(404)
            self.write("{}")
            return
        self.set_header("Content-Type", "application/json; charset=utf-8")
        self.set_header("Cache-Control", "no-store")
        self.write(path.read_text(encoding="utf-8"))

class _IndexHandler(tornado.web.RequestHandler):
    def get(self) -> None:
        self.set_header("Content-Type", "text/html; charset=utf-8")
        self.write(_load_html())


class _WsHandler(tornado.websocket.WebSocketHandler):
    def open(self) -> None:
        server: "WebSocketServer" = self.application.settings["server"]
        server._register(self)
        # 发送连接前积压的消息
        with server._clients_lock:
            for msg in server._pending:
                self.write_message(json.dumps(msg))
            server._pending.clear()

    def on_close(self) -> None:
        server: "WebSocketServer" = self.application.settings["server"]
        server._unregister(self)

    def on_message(self, message: str | bytes) -> None:
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            return
        self.application.settings["cmd_queue"].put(data)


class WebSocketServer:
    """轻量 tornado WS 服务器：接收浏览器命令 + 广播日志/位姿到所有客户端。"""

    def __init__(self, cmd_queue: queue.Queue, port: int = HTTP_PORT):
        self.cmd_queue = cmd_queue
        self.port = port
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._ioloop: tornado.ioloop.IOLoop | None = None
        self._clients: set[_WsHandler] = set()
        self._clients_lock = threading.Lock()
        self._pending: list[dict] = []   # 积压消息（浏览器连接前）

    def _register(self, handler: _WsHandler) -> None:
        with self._clients_lock:
            self._clients.add(handler)

    def _unregister(self, handler: _WsHandler) -> None:
        with self._clients_lock:
            self._clients.discard(handler)

    def broadcast(self, data: dict) -> None:
        """从主线程安全地向所有已连接浏览器推送 JSON 消息。"""
        with self._clients_lock:
            if not self._clients:
                self._pending.append(data)
                if len(self._pending) > 200:
                    self._pending = self._pending[-200:]
                return
            clients = list(self._clients)

        if self._ioloop is None:
            return

        msg = json.dumps(data)
        for c in clients:
            self._ioloop.add_callback(c.write_message, msg)

    def _run_loop(self) -> None:
        self._ioloop = tornado.ioloop.IOLoop()
        self._ioloop.make_current()

        app = tornado.web.Application(
            [(r"/", _IndexHandler), (r"/ws", _WsHandler),
                (r"/workspace_constants.json", _ConstantsHandler)],
            cmd_queue=self.cmd_queue,
            server=self,
        )
        app.listen(self.port, "0.0.0.0")
        self._ready.set()
        self._ioloop.start()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self._ready.wait()

    def stop(self) -> None:
        if self._ioloop is not None:
            self._ioloop.add_callback(self._ioloop.stop)


# ---------------------------------------------------------------------------
# 轨迹插值
# ---------------------------------------------------------------------------
def min_jerk_interpolation(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Min-jerk 归一化插值，t ∈ [0, 1]."""
    t = float(np.clip(t, 0.0, 1.0))
    s = 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5
    return q0 + (q1 - q0) * s


# ---------------------------------------------------------------------------
# 主仿真
# ---------------------------------------------------------------------------


def _get_lan_ip() -> str:
    """获取本机局域网 IP，优先返回 RFC1918 私网地址；失败时返回 localhost。"""
    import re
    import socket
    import subprocess

    def _is_private(ip: str) -> bool:
        parts = ip.split(".")
        if len(parts) != 4:
            return False
        try:
            a, b, _, _ = (int(x) for x in parts)
        except ValueError:
            return False
        if a == 10:
            return True
        if a == 172 and 16 <= b <= 31:
            return True
        if a == 192 and b == 168:
            return True
        return False

    def _is_loopback_or_linklocal(ip: str) -> bool:
        return ip.startswith("127.") or ip.startswith("169.254.")

    # 1) 从本机所有网卡解析 IPv4 地址（Linux）
    try:
        out = subprocess.check_output(
            ["ip", "-4", "-o", "addr", "show"], timeout=2, text=True
        )
        ips = re.findall(r"inet\s+([\d.]+)/\d+", out)
        candidates = [ip for ip in ips if not _is_loopback_or_linklocal(ip)]
        private = [ip for ip in candidates if _is_private(ip)]
        if private:
            return private[0]
        if candidates:
            return candidates[0]
    except Exception:
        pass

    # 2) 回退：通过默认路由推断（可能返回 VPN/代理接口，仅作备选）
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.1)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        if not _is_loopback_or_linklocal(ip):
            return ip
    except Exception:
        pass

    return "localhost"

def _log(server: WebSocketServer, text: str, tag: str = "") -> None:
    """同时输出到终端和浏览器日志面板。"""
    print(text)
    server.broadcast({"type": "log", "text": text, "tag": tag})

def main() -> None:
    # ── 加载模型 ──────────────────────────────────────────────────────────
    mj_model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    mj_data = mujoco.MjData(mj_model)
    nq_total = mj_model.nq

    ik = IKSolver()

    # ── 夹爪关节在 mj_data.qpos 中的地址 ──────────────────────────────────
    def _joint_qpos_addr(name: str) -> int:
        jid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"MuJoCo 模型中未找到关节: {name}")
        return int(mj_model.jnt_qposadr[jid])

    gripper7_addr = _joint_qpos_addr(GRIPPER_JOINT7_NAME)
    gripper_finger_addrs = [_joint_qpos_addr(n) for n in GRIPPER_FINGER_JOINTS]
    gripper_finger_ranges = [
        (float(mj_model.jnt_range[mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, n)][0]),
         float(mj_model.jnt_range[mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, n)][1]))
        for n in GRIPPER_FINGER_JOINTS
    ]

    dt = float(mj_model.opt.timestep)
    if dt <= 0.0:
        dt = 0.002
        mj_model.opt.timestep = dt

    qp_solver: QPVelocitySolver | None = None
    if USE_QP_VELOCITY:
        qp_solver = QPVelocitySolver(
            ik=ik,
            mj_model=mj_model,
            dt=dt,
            lambda_reg=VEL_TIKHONOV,
            dq_max=15.0,                # rad/s，摇杆通道最大关节速度
            position_margin=0.005,
            collision_safety_distance=0.001,
            include_obstacles=True,
        )

    # 用模型默认 qpos 初始化，保留 cube 等未被直接控制的自由度初始位姿
    q_current = mj_data.qpos.copy()
    mujoco.mj_forward(mj_model, mj_data)

    # ── WebSocket 服务 ────────────────────────────────────────────────────
    cmd_queue: queue.Queue = queue.Queue()

    ws_server = WebSocketServer(cmd_queue, HTTP_PORT)
    ws_server.start()
    _log(ws_server, f"  浏览器控制面板: http://localhost:{HTTP_PORT}")
    _log(ws_server, f"  局域网访问:    http://{_get_lan_ip()}:{HTTP_PORT}")



    # ── 轨迹状态 ──────────────────────────────────────────────────────────
    trajectory: tuple[np.ndarray, np.ndarray, float, float] | None = None
    trajectory_lock = threading.Lock()
    stop_event = threading.Event()

    # ── 速度通道 (方案 D:摇杆不经过 IK / 不进 start_trajectory) ──────────
    _running_velocity = np.zeros(3)               # (vx, vy, vz) in world frame
    _speed_scale = 1.0                            # 速度通道倍率，由网页滑条控制
    _velocity_lock = threading.Lock()
    _vel_pin_data = ik.model.createData()         # 复用一次,避免与 ik.data 互踩
    _vel_frame_id = ik.frame_id
    _vel_pose_tick = 0                            # 速度模式下周期回传当前位姿

    # ── 速度通道积分后位置可达性检查：若新位置无 IK 解，则回退并停止 ─────

    # ── 速度通道关节速度低通滤波：抑制 QP/DLS 解在步间的小幅跳变 ─────────────
    DQ_FILTER_ALPHA = 0.5                         # 越大越跟手，越小越平滑
    _dq_filtered = np.zeros(N_ARM_JOINTS)

    def _signal_handler(_sig, _frame) -> None:
        stop_event.set()
    signal.signal(signal.SIGINT, _signal_handler)

    def start_trajectory(q_end: np.ndarray, duration: float | None = None) -> None:
        nonlocal trajectory
        q_actual = mj_data.qpos[:N_ARM_JOINTS].copy()
        if duration is None:
            dist = float(np.linalg.norm(q_end - q_actual))
            duration = max(0.05, dist / 0.5)
        with trajectory_lock:
            trajectory = (q_actual.copy(), q_end.copy(), time.time(), duration)

    def send_feedback(ok: bool, text: str) -> None:
        ws_server.broadcast({"type": "feedback", "ok": ok, "text": text})

    def send_pose(pos: np.ndarray, rpy: np.ndarray) -> None:
        ws_server.broadcast({
            "type": "pose",
            "pos": [f"{pos[0]:.3f}", f"{pos[1]:.3f}", f"{pos[2]:.3f}"],
            "rpy": [f"{rpy[0]:.3f}", f"{rpy[1]:.3f}", f"{rpy[2]:.3f}"],
        })

    _log(ws_server, "=" * 60)
    _log(ws_server, "B601-RS 浏览器控制仿真已启动")
    _log(ws_server, f"打开 http://localhost:{HTTP_PORT} 拖条控制机械臂")
    _log(ws_server, "=" * 60)

    # 初始位姿
    pos0, rot0 = ik.forward_kinematics(q_current[:N_ARM_JOINTS])
    rpy0 = pin.rpy.matrixToRpy(rot0)
    send_pose(pos0, rpy0)

    # ── 主仿真循环 ────────────────────────────────────────────────────────
    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        while viewer.is_running() and not stop_event.is_set():
            loop_start = time.perf_counter()          # 用于稳定主循环周期

            # 排空队列，只保留最后一条目标消息
            msg = None
            while True:
                try:
                    msg = cmd_queue.get_nowait()
                except queue.Empty:
                    break

            if msg is not None:
                msg_type = msg.get("type", "")

                if msg_type == "home":
                    _log(ws_server, "  → 归零", "home")
                    start_trajectory(np.zeros(N_ARM_JOINTS))
                    send_feedback(True, "归零")

                elif msg_type == "target":
                    vals = msg.get("values", {})
                    x = float(vals.get("x", 0.3))
                    y = float(vals.get("y", 0.0))
                    z = float(vals.get("z", 0.2))
                    roll = float(vals.get("roll", 0.0))
                    pitch = float(vals.get("pitch", 0.0))
                    yaw = float(vals.get("yaw", 0.0))

                    target_pos = np.array([x, y, z])
                    target_rot = pin.rpy.rpyToMatrix(roll, pitch, yaw)

                    q_actual = mj_data.qpos[:N_ARM_JOINTS].copy()
                    q_target, success = ik.solve(
                        target_pos, target_rot=target_rot, q_init=q_actual,
                    )

                    if not success:
                        # 满 IK 失败 → 回退为仅位置 IK
                        q_pos, pos_ok = ik.solve(target_pos, target_rot=None, q_init=q_actual)
                        if pos_ok:
                            _log(ws_server, f"  △ 方向不可达，仅移动到目标位置", "warn")
                            cur_pos, _ = ik.forward_kinematics(q_actual)
                            duration = max(0.05, float(np.linalg.norm(target_pos - cur_pos)) / LINEAR_SPEED)
                            start_trajectory(q_pos, duration)
                            send_feedback(False, "方向不可达，已移动到目标位置")
                            send_pose(target_pos, np.array([roll, pitch, yaw]))
                        else:
                            _log(ws_server, f"  ✗ 目标超出工作空间 pos={[x,y,z]}", "err")
                            send_feedback(False, "目标超出工作空间")
                            pos_cur, rot_cur = ik.forward_kinematics(q_actual)
                            send_pose(pos_cur, pin.rpy.matrixToRpy(rot_cur))
                        continue

                    _log(
                        ws_server,
                        f"  ✓ pos=[{x:.3f} {y:.3f} {z:.3f}] "
                        f"rpy=[{roll:.2f} {pitch:.2f} {yaw:.2f}] "
                        f"→ joints(deg)={np.degrees(q_target).round(1).tolist()}",
                        "ok",
                    )

                    cur_pos, _ = ik.forward_kinematics(q_actual)
                    duration = max(
                        0.05,
                        float(np.linalg.norm(target_pos - cur_pos)) / LINEAR_SPEED,
                    )
                    start_trajectory(q_target, duration)
                    send_feedback(True, "目标已接受")
                    send_pose(target_pos, np.array([roll, pitch, yaw]))

                elif msg_type == "velocity":
                    # 摇杆速度通道:浏览器 emitVelocity 发的字段在顶层 (与 target/vx 区分,
                    # 因为 vx/vy/vz 本身就是值,无需再套一层 values)。
                    with _velocity_lock:
                        _running_velocity[:] = (
                            float(msg.get("vx", 0.0)),
                            float(msg.get("vy", 0.0)),
                            float(msg.get("vz", 0.0)),
                        )
                    # 速度消息频率很高，不再逐条打印，避免日志/锁竞争导致卡顿

                elif msg_type == "speed_scale":
                    with _velocity_lock:
                        _speed_scale = float(msg.get("scale", 1.0))
                    _log(ws_server, f"  速度倍率改为 {_speed_scale:.2f}x", "ok")

                elif msg_type == "gripper":
                    # 浏览器发来 7 号电机角度（deg），映射到 MuJoCo 直线位移
                    angle_deg = float(msg.get("angle", 0.0))
                    angle_deg = np.clip(angle_deg, 0.0, GRIPPER_DEG_MAX)
                    disp = (angle_deg / GRIPPER_DEG_MAX) * GRIPPER_DISP_MAX
                    q_current[gripper7_addr] = float(np.clip(disp, 0.0, GRIPPER_DISP_MAX))
                    for addr, (lo, hi) in zip(gripper_finger_addrs, gripper_finger_ranges):
                        q_current[addr] = float(np.clip(disp, lo, hi))

                # 注:摇杆速度通道不调用 start_trajectory,由主循环 OSC 块直接积分 q_current

            # ── 速度通道 vs 轨迹:互斥,速度通道优先生效 ──────────────────
            with _velocity_lock:
                _vel_active = bool(np.any(np.abs(_running_velocity) > 1e-6))
                _vel_cmd   = _running_velocity.copy() if _vel_active else np.zeros(3)
                _vel_cmd  *= _speed_scale

            if _vel_active:
                # 方案 D/E: Operational Space 速度控制,绕开 IK 与 trajectory
                # 关键:pinocchio 的 model.nq = 8 (机器人 6+夹爪 2),但 mj_data.qpos 在场景里还会追加
                # cube 的 freejoint (3 位移 + 4 四元数) → 总长 15。所以绝不能直接把 mj_data.qpos 喂给 pinocchio。
                # 解决:按 SDK solve_ik 的相同规约,把机器人 6 维 slice 出来再 pad 到 model.nq=8 (末位夹爪填 0)。
                _PIN_NQ = ik.model.nq                                 # = 8
                _PIN_NV = ik.model.nv                                 # = 8
                q_pin   = np.zeros(_PIN_NQ)                           # 机器人 pinocchio 形态 (夹爪 0)
                q_pin[:N_ARM_JOINTS] = q_current[:N_ARM_JOINTS]       # 只填前 6 个受控关节
                q_arm_prev = q_current[:N_ARM_JOINTS].copy()          # 用于积分后回退

                if USE_QP_VELOCITY and qp_solver is not None:
                    # QP 通道：把关节速度/位置/碰撞约束一次性写进二次规划
                    v6 = np.zeros(6)
                    v6[:3] = _vel_cmd
                    dq_arm, qp_ok, qp_msg = qp_solver.solve(
                        q_current[:N_ARM_JOINTS].copy(), v6,
                        qpos_full=q_current if USE_COLLISION_CONSTRAINTS else None,
                        initvals=_dq_filtered.copy(),
                    )
                    if not qp_ok:
                        _log(ws_server, f"  ⚠ {qp_msg}", "warn")
                    # 低通滤波：抑制 QP/DLS 解在步间的跳变
                    dq_arm = DQ_FILTER_ALPHA * _dq_filtered + (1.0 - DQ_FILTER_ALPHA) * dq_arm
                    _dq_filtered[:] = dq_arm
                    dq_pin = np.zeros(_PIN_NV)
                    dq_pin[:N_ARM_JOINTS] = dq_arm
                    q_new_pin = np.asarray(pin.integrate(ik.model, q_pin, dq_pin * dt)).flatten()
                else:
                    # 原 DLS 通道（无碰撞约束，仅位置硬钳位）
                    pin.framesForwardKinematics(ik.model, _vel_pin_data, q_pin)
                    pin.computeJointJacobians(ik.model, _vel_pin_data, q_pin)
                    J = pin.getFrameJacobian(ik.model, _vel_pin_data, _vel_frame_id, pin.WORLD)[:, :N_ARM_JOINTS]

                    v6 = np.zeros(6)
                    v6[:3] = _vel_cmd
                    JJT = J @ J.T
                    JJT[np.arange(6), np.arange(6)] += VEL_TIKHONOV
                    dq_arm = J.T @ np.linalg.solve(JJT, v6) * dt
                    # 低通滤波：抑制 DLS 解在步间的跳变
                    dq_arm = DQ_FILTER_ALPHA * _dq_filtered + (1.0 - DQ_FILTER_ALPHA) * dq_arm
                    _dq_filtered[:] = dq_arm
                    dq_pin = np.zeros(_PIN_NV)
                    dq_pin[:N_ARM_JOINTS] = dq_arm
                    q_new_pin = np.asarray(pin.integrate(ik.model, q_pin, dq_pin)).flatten()

                    # 关节极限钳位 (仅作用于受控关节,夹爪自由):撞墙即停
                    hit_limit = False
                    for i in range(N_ARM_JOINTS):
                        lo = float(ik.model.lowerPositionLimit[i])
                        hi = float(ik.model.upperPositionLimit[i])
                        if np.isfinite(lo) and q_new_pin[i] < lo:
                            q_new_pin[i] = lo; hit_limit = True
                        elif np.isfinite(hi) and q_new_pin[i] > hi:
                            q_new_pin[i] = hi; hit_limit = True
                    if hit_limit:
                        with _velocity_lock:
                            _running_velocity[:] = 0.0

                # 只写回前 6 维到 q_current,保留 mj_data 中夹爪 / cube / 等的自由度
                q_current[:N_ARM_JOINTS] = q_new_pin[:N_ARM_JOINTS]

                # 积分后位置可达性检查：若新位置不存在位置 IK 解，则回退并停止
                new_pos, _ = ik.forward_kinematics(q_current[:N_ARM_JOINTS])
                _, pos_ok = ik.solve(
                    new_pos, target_rot=None, q_init=q_arm_prev,
                    max_iter=200, tolerance=1e-3,
                )
                if not pos_ok:
                    q_current[:N_ARM_JOINTS] = q_arm_prev
                    with _velocity_lock:
                        _running_velocity[:] = 0.0
                    _log(ws_server, "  △ 目标位置超出工作空间，已停止移动", "warn")

                # 速度模式下周期性回传当前末端位姿,同步浏览器侧滑条
                _vel_pose_tick = (_vel_pose_tick + 1) % 10
                if _vel_pose_tick == 0:
                    pin.framesForwardKinematics(ik.model, _vel_pin_data, q_pin)
                    cur_pos = _vel_pin_data.oMf[_vel_frame_id].translation.copy()
                    cur_rot = _vel_pin_data.oMf[_vel_frame_id].rotation.copy()
                    send_pose(cur_pos, pin.rpy.matrixToRpy(cur_rot))
            else:
                # 原 trajectory 推进 (位置通道)
                _dq_filtered[:] = 0.0                     # 退出速度模式后清空滤波器
                with trajectory_lock:
                    if trajectory is not None:
                        q_start, q_end, t_start, duration = trajectory
                        elapsed = time.time() - t_start
                        if elapsed >= duration:
                            q_current[:N_ARM_JOINTS] = q_end
                            trajectory = None
                        else:
                            q_current[:N_ARM_JOINTS] = min_jerk_interpolation(
                                q_start, q_end, elapsed / duration,
                            )

            mj_data.qpos[:] = q_current
            mujoco.mj_forward(mj_model, mj_data)
            viewer.sync()
            # 稳定周期，避免有时 dt  sleep  0.001  造成不规则停顿
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.0, dt - elapsed))

    ws_server.stop()
    _log(ws_server, "\n退出仿真。")


if __name__ == "__main__":
    main()
