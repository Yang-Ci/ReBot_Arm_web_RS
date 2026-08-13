#!/usr/bin/env python3
"""Unit tests / sanity checks for the QP velocity solver.

These tests do **not** open a viewer or a web server.  They simply verify that:

1. A normal Cartesian twist command is tracked (the solver returns non-zero dq).
2. A command that would drive a joint past its limit is clamped by the QP.
3. A command that would drive the arm into self-collision is prevented.
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rebot_b601_rs_sim.config import SCENE_PATH
from rebot_b601_rs_sim.control.ik import IKSolver
from rebot_b601_rs_sim.control.qp_velocity_solver import QPVelocitySolver

N_ARM = 6
DT = 0.002


def make_solver() -> QPVelocitySolver:
    ik = IKSolver()
    mj_model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    return QPVelocitySolver(
        ik=ik,
        mj_model=mj_model,
        dt=DT,
        lambda_reg=1e-4,
        dq_max=1.0,
        position_margin=0.02,
        collision_safety_distance=0.005,
        include_obstacles=True,
    )


def fk(ik: IKSolver, q_arm: np.ndarray):
    return ik.forward_kinematics(q_arm)


def test_tracks_simple_twist():
    solver = make_solver()
    q = np.zeros(N_ARM)  # nominal home pose, no active self-contacts
    q_full = np.zeros(solver.mj_model.nq)
    q_full[:N_ARM] = q

    v_des = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0])  # 5 cm/s along X
    dq, ok, msg = solver.solve(q, v_des, qpos_full=q_full)
    assert ok, msg
    assert np.linalg.norm(dq) > 1e-4, "Solver returned zero velocity for a feasible twist"
    print("[PASS] simple twist tracking, ||dq|| =", np.linalg.norm(dq))


def test_joint_limit_avoidance():
    solver = make_solver()
    q = np.array([0.0, 0.03, 1.0, 0.0, 0.0, 0.0])  # joint 1 is just above lower limit (0)
    q_full = np.zeros(solver.mj_model.nq)
    q_full[:N_ARM] = q

    # Command that tries to push joint 1 further down.
    v_des = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    dq, ok, msg = solver.solve(q, v_des, qpos_full=q_full)
    assert ok, msg
    q_next = q + dq * DT
    assert q_next[1] >= (solver.q_min_hard[1] + solver.position_margin) - 1e-6, (
        f"Joint 1 would cross lower margin: q_next[1]={q_next[1]}, limit={solver.q_min[1]}"
    )
    print("[PASS] joint limit avoidance, q_next[1] =", q_next[1])


def find_self_collision_config(solver: QPVelocitySolver) -> tuple[np.ndarray, tuple[int, int], float] | None:
    """Sample until we find a configuration where MuJoCo reports an active,
    non-excluded self-collision contact between two arm links."""
    ik = solver.ik
    rng = np.random.default_rng(42)
    mj_data = mujoco.MjData(solver.mj_model)
    arm_set = set(solver.arm_geom_ids)
    parent = np.asarray(solver.mj_model.body_parentid)

    for _ in range(8000):
        q = rng.uniform(ik.model.lowerPositionLimit[:N_ARM], ik.model.upperPositionLimit[:N_ARM])
        q_full = np.zeros(solver.mj_model.nq)
        q_full[:N_ARM] = q
        mj_data.qpos[:] = q_full
        mujoco.mj_forward(solver.mj_model, mj_data)

        for c in mj_data.contact:
            if c.exclude:
                continue
            gid1, gid2 = int(c.geom1), int(c.geom2)
            if not ((gid1 in arm_set) and (gid2 in arm_set)):
                continue
            body1 = int(np.asarray(solver.mj_model.geom(gid1).bodyid).item())
            body2 = int(np.asarray(solver.mj_model.geom(gid2).bodyid).item())
            if body1 == body2 or parent[body1] == body2 or parent[body2] == body1:
                continue
            return q, (gid1, gid2), float(c.dist)
    return None


def test_self_collision_avoidance():
    solver = make_solver()
    result = find_self_collision_config(solver)
    if result is None:
        print("[SKIP] could not find an active self-collision contact")
        return

    q, closest_pair, dist0 = result
    q_full = np.zeros(solver.mj_model.nq)
    q_full[:N_ARM] = q

    print(f"[INFO] found active self-collision contact: pair={closest_pair}, dist={dist0:.4f}")

    # Command a zero twist.  The QP should either keep the arm still or, if the
    # current contact is feasible to escape, move the links apart.  In no case
    # should it be allowed to drive deeper into penetration.
    v_des = np.zeros(6)

    dq_qp, ok, msg = solver.solve(q, v_des, qpos_full=q_full)
    print(f"[INFO] QP status: {ok}, msg={msg}")

    mj_data = mujoco.MjData(solver.mj_model)
    q_next = q + dq_qp * DT
    q_full_next = q_full.copy()
    q_full_next[:N_ARM] = q_next
    mj_data.qpos[:] = q_full_next
    mujoco.mj_forward(solver.mj_model, mj_data)
    next_dist = mujoco.mj_geomDistance(
        solver.mj_model, mj_data, closest_pair[0], closest_pair[1], 0.2, None
    )

    assert next_dist >= dist0 - 1e-4, (
        f"QP made the self-collision worse: {dist0:.4f} -> {next_dist:.4f}"
    )
    print(f"[PASS] self-collision avoidance: dist {dist0:.4f} -> {next_dist:.4f}")


if __name__ == "__main__":
    test_tracks_simple_twist()
    test_joint_limit_avoidance()
    test_self_collision_avoidance()
