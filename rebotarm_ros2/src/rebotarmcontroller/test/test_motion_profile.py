import numpy as np

from rebotarmcontroller.motion_profiles import (
    advance_jerk_limited_reference,
    advance_jerk_limited_reference_over_elapsed,
)


def _run_profile(targets, *, dt=0.002, steps=3000):
    q = np.zeros(1)
    qd = np.zeros(1)
    qdd = np.zeros(1)
    history = []
    for index in range(steps):
        goal = np.array([targets(index * dt)], dtype=np.float64)
        q, qd, qdd = advance_jerk_limited_reference(
            q,
            qd,
            qdd,
            goal,
            np.array([0.6]),
            dt,
        )
        history.append((q[0], qd[0], qdd[0], goal[0]))
    return np.asarray(history)


def test_step_target_has_bounded_motion_and_no_reference_overshoot():
    history = _run_profile(lambda _time: 0.4)
    assert np.max(history[:, 0]) <= 0.4 + 1e-12
    assert np.max(np.abs(history[:, 1])) <= 0.6 + 1e-12
    assert np.max(np.abs(history[:, 2])) <= 4.0 + 1e-12
    jerk = np.diff(history[:, 2]) / 0.002
    assert np.max(np.abs(jerk)) <= 30.0 + 1e-9
    assert abs(history[-1, 0] - 0.4) < 1e-6
    assert abs(history[-1, 1]) < 1e-6


def test_live_retarget_preserves_reference_continuity():
    history = _run_profile(lambda time_s: 0.35 if time_s < 0.45 else -0.15)
    position_steps = np.abs(np.diff(history[:, 0]))
    velocity_steps = np.abs(np.diff(history[:, 1]))
    acceleration_steps = np.abs(np.diff(history[:, 2]))
    assert np.max(position_steps) <= 0.6 * 0.002 + 1e-9
    assert np.max(velocity_steps) <= 4.0 * 0.002 + 1e-9
    assert np.max(acceleration_steps) <= 30.0 * 0.002 + 1e-9
    assert abs(history[-1, 0] + 0.15) < 1e-6
    assert abs(history[-1, 1]) < 1e-6


def test_sparse_callbacks_keep_wall_clock_trajectory_timing():
    q = np.zeros(1)
    qd = np.zeros(1)
    qdd = np.zeros(1)
    for _ in range(20):
        q, qd, qdd = advance_jerk_limited_reference_over_elapsed(
            q,
            qd,
            qdd,
            np.array([0.08]),
            np.array([0.25]),
            0.05,
        )

    dense = _run_profile(lambda _time: 0.08, dt=0.002, steps=500)
    assert abs(q[0] - dense[-1, 0]) < 2e-3
    assert q[0] > 0.075
