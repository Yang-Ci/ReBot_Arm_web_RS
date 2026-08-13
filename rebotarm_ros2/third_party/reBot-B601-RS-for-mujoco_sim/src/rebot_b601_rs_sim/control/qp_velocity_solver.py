#!/usr/bin/env python3
"""QP-based joint velocity solver with joint limits and collision avoidance.

The solver converts an end-effector Cartesian twist command into feasible joint
velocities by solving a quadratic program:

    min_dq  0.5 * dq^T (J^T J + λ I) dq - (J^T v_des)^T dq
    s.t.    dq_min <= dq <= dq_max
            q_min  <= q + dq*dt <= q_max
            d_i + (∇_q d_i)^T * dq * dt >= 0          (collision)

The collision constraints are generated from MuJoCo's active contacts.  Before
the collision query we temporarily inflate the arm geoms' contact margins to
``collision_safety_distance``; this makes contacts appear slightly before the
real surfaces touch, and the signed distance stored in ``mjData.contact[i].dist``
becomes (true_distance - safety_distance).  Requiring ``dist_next >= 0`` is
therefore equivalent to requiring ``true_distance_next >= safety_distance``.

The distance gradient is computed analytically with MuJoCo's contact Jacobian,
so no finite-difference forward passes are needed per contact.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import mujoco
import numpy as np
import pinocchio as pin
import qpsolvers
import scipy.sparse as sp


@dataclass
class QPVelocitySolver:
    """Solve joint velocities under end-effector twist, joint and collision constraints.

    Parameters
    ----------
    ik:
        The project's IKSolver, which carries the Pinocchio model / data and the
        end-effector frame id.
    mj_model:
        MuJoCo model, used for collision queries.
    dt:
        Controller time step (same unit as the twist command).
    lambda_reg:
        Tikhonov regularisation weight on joint velocities.
    dq_max:
        Max absolute joint velocity (rad/s).  If None, taken from ``ik.model.velocityLimit``.
    position_margin:
        Safety margin subtracted from joint position limits (rad).
    collision_safety_distance:
        Minimum signed distance that must remain between any two arm geoms (m).
    include_obstacles:
        If True, also generate constraints from active contacts with floor, table
        and cube geoms.
    """

    ik: object
    mj_model: mujoco.MjModel
    dt: float
    lambda_reg: float = 1e-4
    dq_max: Sequence[float] | np.ndarray | None = None
    position_margin: float = 0.02
    collision_safety_distance: float = 0.005
    include_obstacles: bool = True

    n_arm: int = field(init=False, default=6)
    frame_id: int = field(init=False)
    q_min_hard: np.ndarray = field(init=False)
    q_max_hard: np.ndarray = field(init=False)
    dq_max_arr: np.ndarray = field(init=False)
    arm_geom_ids: list[int] = field(init=False, default_factory=list)
    obstacle_geom_ids: list[int] = field(init=False, default_factory=list)
    _pin_data: pin.Data = field(init=False, repr=False)
    _mj_data_copy: mujoco.MjData = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.frame_id = self.ik.frame_id
        self._pin_data = self.ik.model.createData()

        # Hard joint limits (actual URDF/Pinocchio bounds).
        lo = np.asarray(self.ik.model.lowerPositionLimit, dtype=float)
        hi = np.asarray(self.ik.model.upperPositionLimit, dtype=float)
        self.q_min_hard = lo[: self.n_arm]
        self.q_max_hard = hi[: self.n_arm]

        # Velocity limits.
        if self.dq_max is None:
            self.dq_max_arr = np.asarray(self.ik.model.velocityLimit, dtype=float)[: self.n_arm]
        else:
            self.dq_max_arr = np.asarray(self.dq_max, dtype=float)
            if self.dq_max_arr.size == 1:
                self.dq_max_arr = np.full(self.n_arm, float(self.dq_max_arr))

        self._mj_data_copy = mujoco.MjData(self.mj_model)
        self._build_collision_sets()

    # ------------------------------------------------------------------
    # Collision set bookkeeping
    # ------------------------------------------------------------------
    def _body_name(self, body_id: int) -> str:
        name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, int(body_id))
        return name or ""

    def _is_collision_geom(self, geom_id: int) -> bool:
        return bool(
            self.mj_model.geom_contype[geom_id] > 0
            and self.mj_model.geom_conaffinity[geom_id] > 0
        )

    def _build_collision_sets(self) -> None:
        m = self.mj_model
        arm_body_names = {
            "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
            "gripper_end", "gripper_left", "gripper_right",
        }
        env_body_names = {"world", "table", "cube"}

        arm_geoms: list[int] = []
        obstacle_geoms: list[int] = []

        for gid in range(m.ngeom):
            if not self._is_collision_geom(gid):
                continue
            body_id = int(np.asarray(m.geom(gid).bodyid).item())
            bname = self._body_name(body_id)
            if bname in arm_body_names:
                arm_geoms.append(gid)
            elif self.include_obstacles and bname in env_body_names:
                obstacle_geoms.append(gid)

        self.arm_geom_ids = arm_geoms
        self.obstacle_geom_ids = obstacle_geoms

    # ------------------------------------------------------------------
    # Public solve interface
    # ------------------------------------------------------------------
    def solve(
        self,
        q_arm: np.ndarray,
        v_des: np.ndarray,
        qpos_full: np.ndarray | None = None,
        initvals: np.ndarray | None = None,
    ) -> tuple[np.ndarray, bool, str]:
        """Compute a feasible joint velocity command.

        Parameters
        ----------
        q_arm:
            Current 6-DOF arm joint angles (rad).
        v_des:
            Desired 6D end-effector twist in world frame [vx, vy, vz, wx, wy, wz].
        qpos_full:
            Full MuJoCo qpos vector.  Needed when collision constraints are used.
        initvals:
            Optional warm-start for the QP solver (rad/s).  Passing the previous
            solution can reduce active-set oscillation between control steps.

        Returns
        -------
        dq:
            Feasible joint velocity (rad/s).  On failure returns zeros.
        ok:
            True if the QP was solved successfully.
        msg:
            Short status message.
        """
        q_arm = np.asarray(q_arm, dtype=float).reshape(self.n_arm)
        v_des = np.asarray(v_des, dtype=float).reshape(6)

        # --- Pinocchio Jacobian in world frame --------------------------------
        q_pin = np.zeros(self.ik.model.nq)
        q_pin[: self.n_arm] = q_arm
        pin.framesForwardKinematics(self.ik.model, self._pin_data, q_pin)
        pin.computeJointJacobians(self.ik.model, self._pin_data, q_pin)
        J = pin.getFrameJacobian(
            self.ik.model, self._pin_data, self.frame_id, pin.ReferenceFrame.WORLD
        )[:, : self.n_arm]

        # --- QP cost: 0.5 dq^T P dq + q^T dq ----------------------------------
        P = J.T @ J + self.lambda_reg * np.eye(self.n_arm)
        q_vec = -(J.T @ v_des)

        # --- Box constraints: velocity + predictive position limits ------------
        lb_vel = -self.dq_max_arr
        ub_vel = self.dq_max_arr

        # Effective position limits: enforce a safety margin when inside the
        # margin region, but never make the current state infeasible.
        eff_min = np.minimum(q_arm, self.q_min_hard + self.position_margin)
        eff_max = np.maximum(q_arm, self.q_max_hard - self.position_margin)
        lb_pos = (eff_min - q_arm) / self.dt
        ub_pos = (eff_max - q_arm) / self.dt

        lb = np.maximum(lb_vel, lb_pos)
        ub = np.minimum(ub_vel, ub_pos)

        # --- Collision constraints --------------------------------------------
        G, h = None, None
        if qpos_full is not None:
            G, h = self._build_collision_constraints(qpos_full)

        # --- Solve ------------------------------------------------------------
        try:
            dq = qpsolvers.solve_qp(
                sp.csc_matrix(P), q_vec,
                G=sp.csc_matrix(G) if G is not None else None, h=h,
                lb=lb, ub=ub,
                solver="osqp",
                verbose=False,
                initvals=initvals,
            )
        except Exception as exc:  # pragma: no cover - solver internal error
            return np.zeros(self.n_arm), False, f"QP solver exception: {exc}"

        if dq is None:
            # Infeasible: fall back to a damped least-squares solution without
            # collision constraints but with joint limits.
            dq = self._fallback_dls(J, v_des, lb, ub)
            if dq is None:
                return np.zeros(self.n_arm), False, "QP infeasible, fallback also failed"
            return dq, False, "QP infeasible, used damped-least-squares fallback"

        return np.asarray(dq, dtype=float), True, "ok"

    # ------------------------------------------------------------------
    # Collision constraint linearisation (contact-based, analytical Jacobian)
    # ------------------------------------------------------------------
    def _build_collision_constraints(
        self,
        qpos_full: np.ndarray,
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Return (G, h) such that G @ dq <= h enforces collision safety.

        Contacts are queried with temporarily inflated geom margins so that the
        constraint becomes active before the real surfaces touch.
        """
        m = self.mj_model
        d_copy = self._mj_data_copy
        arm_set = set(self.arm_geom_ids)
        rows: list[np.ndarray] = []
        rhs: list[float] = []

        # Temporarily inflate margins for the collision copy.  The signed distance
        # returned by MuJoCo becomes (true_distance - safety_distance), so asking
        # ``dist_next >= 0`` keeps ``true_distance_next >= safety_distance``.
        original_margins = m.geom_margin.copy()
        for gid in self.arm_geom_ids + self.obstacle_geom_ids:
            m.geom_margin[gid] = self.collision_safety_distance

        try:
            d_copy.qpos[:] = qpos_full
            mujoco.mj_forward(m, d_copy)

            jacp1 = np.zeros((3, m.nv))
            jacp2 = np.zeros((3, m.nv))
            parent = np.asarray(m.body_parentid)

            for c in d_copy.contact:
                if c.exclude:
                    continue
                gid1, gid2 = int(c.geom1), int(c.geom2)
                in_arm1 = gid1 in arm_set
                in_arm2 = gid2 in arm_set
                if not (in_arm1 or in_arm2):
                    continue

                # Skip parent/child arm pairs; MuJoCo's XML may already exclude
                # them, but keep this as a safety net.
                if in_arm1 and in_arm2:
                    body1 = int(np.asarray(m.geom(gid1).bodyid).item())
                    body2 = int(np.asarray(m.geom(gid2).bodyid).item())
                    if body1 == body2 or parent[body1] == body2 or parent[body2] == body1:
                        continue

                # Normal points from geom1 to geom2 (first row of contact frame).
                normal = np.asarray(c.frame[:3], dtype=float)
                pos = np.asarray(c.pos, dtype=float)

                body1 = int(np.asarray(m.geom(gid1).bodyid).item())
                body2 = int(np.asarray(m.geom(gid2).bodyid).item())
                mujoco.mj_jac(m, d_copy, jacp1, None, pos, body1)
                mujoco.mj_jac(m, d_copy, jacp2, None, pos, body2)

                # Gradient of signed distance w.r.t. the 6 arm joints.
                grad = ((jacp2 - jacp1).T @ normal)[: self.n_arm]
                if np.linalg.norm(grad) < 1e-3:
                    continue

                # dist + grad^T * (dq * dt) >= 0
                # => -(grad * dt)^T dq <= dist
                rows.append(-grad * self.dt)
                rhs.append(float(c.dist))
        finally:
            m.geom_margin[:] = original_margins

        if not rows:
            return None, None
        return np.vstack(rows), np.asarray(rhs, dtype=float)

    # ------------------------------------------------------------------
    # Fallback damped least-squares with joint-limit clipping
    # ------------------------------------------------------------------
    def _fallback_dls(
        self,
        J: np.ndarray,
        v_des: np.ndarray,
        lb: np.ndarray,
        ub: np.ndarray,
    ) -> np.ndarray | None:
        JJT = J @ J.T
        JJT[np.arange(6), np.arange(6)] += self.lambda_reg
        try:
            dq = J.T @ np.linalg.solve(JJT, v_des)
        except np.linalg.LinAlgError:
            return None
        return np.clip(dq, lb, ub)
