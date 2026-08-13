"""基于 Pinocchio 生成前端可达性预校验所需的几何常数。

算法:
    1. 用 SDK 的 URDF 构建 pinocchio.Model
    2. 在 q = neutral 时取 joint2 / joint3 的世界系原点 -> 计算 shoulder_xyz 与 L_upper
    3. 在关节极限内均匀 / 随机采样 N 份 q,做 FK 取 gripper_end 末端相对 shoulder 的距离
    4. 用 99 / 1 分位估计 L_max / L_min

输出:
    src/rebot_b601_rs_sim/templates/workspace_constants.json

用法:
    python scripts/b601_workspace_constants.py                 # 50000 采样
    python scripts/b601_workspace_constants.py --samples 100000
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / "reBotArm_control_py"))
sys.path.insert(0, str(ROOT / "src"))

from rebot_b601_rs_sim.config import RS_URDF_PATH                # noqa: E402
from reBotArm_control_py.kinematics import load_robot_model       # noqa: E402


def _finite_limits(lo: np.ndarray, hi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    lo = lo.copy()
    hi = hi.copy()
    lo[~np.isfinite(lo)] = -np.pi
    hi[~np.isfinite(hi)] = np.pi
    return lo, hi


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--samples", type=int, default=50_000,
                    help="Monte-Carlo 采样数 (默认 50000)")
    ap.add_argument("--out", type=Path,
                    default=ROOT / "src/rebot_b601_rs_sim/templates/workspace_constants.json",
                    help="输出 JSON 路径")
    args = ap.parse_args()

    # 1. 模型加载
    model: pin.Model = load_robot_model(str(RS_URDF_PATH))     # type: ignore[arg-type]
    data = model.createData()
    fid = model.getFrameId("gripper_end")

    # 2. shoulder / elbow 在世界系下的原点 (joint1 是 R 轴,无平移 -> 从 joint2 起算)
    pin.forwardKinematics(model, data, pin.neutral(model))
    shoulder = data.oMi[2].translation.copy()
    elbow    = data.oMi[3].translation.copy()
    L_upper  = float(np.linalg.norm(elbow - shoulder))

    # 3. 关节限位 + Monte Carlo 采样
    q_lo, q_hi = _finite_limits(model.lowerPositionLimit[:6], model.upperPositionLimit[:6])

    rel_positions = np.empty((args.samples, 3), dtype=float)
    for k in range(args.samples):
        q = np.zeros(model.nq)
        q[:6] = q_lo + (q_hi - q_lo) * np.random.rand(6)
        pin.framesForwardKinematics(model, data, q)
        rel_positions[k] = data.oMf[fid].translation - shoulder

    r = np.linalg.norm(rel_positions, axis=1)
    L_max_m = float(np.percentile(r, 99))
    L_min_m = float(np.percentile(r, 1))

    # 4. "肩下死区" dz 阈值: 在 1% 分位再留一点余量
    dz_floor_m = float(np.percentile(rel_positions[:, 2], 1)) - 0.02

    payload = {
        "_comment": "B601-RS 可达性预校验常数 · 由 Pinocchio FK + Monte-Carlo 生成",
        "source_urdf": str(RS_URDF_PATH.relative_to(ROOT)),
        "shoulder_xyz": [round(float(shoulder[i]), 4) for i in range(3)],
        "L_upper_m":    round(L_upper, 4),
        "L_max_m":      round(L_max_m, 4),
        "L_min_m":      round(max(L_min_m, 0.05), 4),
        "warn_ratio":   0.85,
        "dz_floor_m":   round(dz_floor_m, 4),
        "joint_limits_rad": [
            [round(float(q_lo[i]), 4), round(float(q_hi[i]), 4)] for i in range(6)
        ],
        "samples": args.samples,
    }

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2, ensure_ascii=False))
    print(f"[workspace_constants] saved -> {args.out}")
    print(json.dumps(payload, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
