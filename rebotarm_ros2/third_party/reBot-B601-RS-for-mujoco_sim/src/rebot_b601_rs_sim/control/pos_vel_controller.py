"""POS_VEL 串级控制器。

对应真实电机 POS_VEL 控制模式：
    位置环：vel_cmd = pos_kp * (q_target - q)
    速度限制：vel_cmd = clip(vel_cmd, -vlim, vlim)
    速度环：torque = vel_kp * (vel_cmd - qd) + vel_ki * integral
    力矩限制：torque = clip(torque, -tau_max, tau_max)

可选的力矩输出一阶低通滤波：
    torque_filtered = alpha * torque + (1 - alpha) * torque_filtered_prev
    alpha 越小滤波越强（越平滑但响应越慢），alpha=1 等价于无滤波。
"""

from __future__ import annotations

import numpy as np


class POSVELController:
    """位置-速度串级关节控制器。

    参数单位与真实电机 POS_VEL 模式一致：
    - pos_kp: s^-1
    - vel_kp: N·m / (rad/s)
    - vel_ki: N·m / (rad/s) * s
    - vlim: rad/s
    - tau_max: N·m

    输出低通滤波用于抑制仿真中的高频力矩抖动，尤其适合无重力前馈、
    仅靠位置伺服抵抗重力的场景。
    """

    def __init__(
        self,
        pos_kp: np.ndarray,
        vel_kp: np.ndarray,
        vel_ki: np.ndarray,
        vlim: np.ndarray,
        tau_max: np.ndarray,
        dt: float,
        output_filter_alpha: float = 1.0,
    ) -> None:
        self.pos_kp = np.asarray(pos_kp, dtype=float)
        self.vel_kp = np.asarray(vel_kp, dtype=float)
        self.vel_ki = np.asarray(vel_ki, dtype=float)
        self.vlim = np.asarray(vlim, dtype=float)
        self.tau_max = np.asarray(tau_max, dtype=float)
        self.dt = float(dt)
        self.output_filter_alpha = float(output_filter_alpha)

        self._integral = np.zeros_like(self.pos_kp, dtype=float)
        self._tau_filtered = np.zeros_like(self.pos_kp, dtype=float)

    def reset(self) -> None:
        """清空速度环积分项与输出滤波器状态。"""
        self._integral.fill(0.0)
        self._tau_filtered.fill(0.0)

    def compute(self, q_target: np.ndarray, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        """根据目标位置、当前位置、当前速度计算关节力矩。"""
        q_target = np.asarray(q_target, dtype=float)
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)

        # 位置环：位置误差 -> 速度指令
        pos_err = q_target - q
        vel_cmd = self.pos_kp * pos_err

        # 速度限制（真实电机的 vlim）
        vel_cmd = np.clip(vel_cmd, -self.vlim, self.vlim)

        # 速度环：速度误差 -> 力矩
        vel_err = vel_cmd - qd
        self._integral += vel_err * self.dt
        torque = self.vel_kp * vel_err + self.vel_ki * self._integral

        # 力矩限制
        torque = np.clip(torque, -self.tau_max, self.tau_max)

        # 输出低通滤波（可选）
        if self.output_filter_alpha < 1.0:
            self._tau_filtered = (
                self.output_filter_alpha * torque
                + (1.0 - self.output_filter_alpha) * self._tau_filtered
            )
            return self._tau_filtered.copy()

        return torque
