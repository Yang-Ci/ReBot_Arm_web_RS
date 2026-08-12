from __future__ import annotations

import numpy as np


def monotone_waypoint_velocities(
    targets: list[np.ndarray],
    point_times: list[float],
    *,
    velocity_limit: float,
) -> list[np.ndarray]:
    """Build shared waypoint velocities without overshooting a segment.

    Interior velocities use the weighted harmonic mean from monotone cubic
    interpolation.  A direction reversal receives zero velocity, while the
    trajectory starts and ends at rest.
    """
    if len(targets) != len(point_times):
        raise ValueError("targets and point_times must have equal lengths")
    if not targets:
        return []

    values = np.asarray(targets, dtype=np.float64)
    velocities = np.zeros_like(values)
    if len(values) < 3:
        return [row.copy() for row in velocities]

    intervals = np.diff(np.asarray(point_times, dtype=np.float64))
    if np.any(intervals <= 0.0):
        raise ValueError("point_times must be strictly increasing")
    slopes = np.diff(values, axis=0) / intervals[:, None]

    for index in range(1, len(values) - 1):
        previous = slopes[index - 1]
        following = slopes[index]
        same_direction = previous * following > 0.0
        previous_dt = intervals[index - 1]
        following_dt = intervals[index]
        weight_previous = 2.0 * following_dt + previous_dt
        weight_following = following_dt + 2.0 * previous_dt
        denominator = np.zeros_like(previous)
        denominator[same_direction] = (
            weight_previous / previous[same_direction]
            + weight_following / following[same_direction]
        )
        valid = same_direction & (np.abs(denominator) > 1e-12)
        velocities[index, valid] = (
            weight_previous + weight_following
        ) / denominator[valid]

    np.clip(velocities, -abs(velocity_limit), abs(velocity_limit), out=velocities)
    return [row.copy() for row in velocities]


def sample_cubic_hermite(
    q0: np.ndarray,
    q1: np.ndarray,
    v0: np.ndarray,
    v1: np.ndarray,
    duration: float,
    ratio: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return continuous position and velocity for one trajectory segment."""
    dt = float(duration)
    if dt <= 1e-9:
        return np.asarray(q1, dtype=np.float64).copy(), np.zeros_like(q1)

    u = float(np.clip(ratio, 0.0, 1.0))
    u2 = u * u
    u3 = u2 * u
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    v0 = np.asarray(v0, dtype=np.float64)
    v1 = np.asarray(v1, dtype=np.float64)

    position = (
        (2.0 * u3 - 3.0 * u2 + 1.0) * q0
        + (u3 - 2.0 * u2 + u) * dt * v0
        + (-2.0 * u3 + 3.0 * u2) * q1
        + (u3 - u2) * dt * v1
    )
    velocity = (
        ((6.0 * u2 - 6.0 * u) / dt) * q0
        + (3.0 * u2 - 4.0 * u + 1.0) * v0
        + ((-6.0 * u2 + 6.0 * u) / dt) * q1
        + (3.0 * u2 - 2.0 * u) * v1
    )
    return position, velocity


def maximum_cubic_hermite_speed(
    targets: list[np.ndarray],
    point_times: list[float],
    waypoint_velocities: list[np.ndarray],
) -> float:
    """Return the exact largest absolute velocity over all cubic segments."""
    maximum = 0.0
    for index in range(1, len(targets)):
        duration = float(point_times[index] - point_times[index - 1])
        if duration <= 1e-9:
            continue
        q0 = np.asarray(targets[index - 1], dtype=np.float64)
        q1 = np.asarray(targets[index], dtype=np.float64)
        v0 = np.asarray(waypoint_velocities[index - 1], dtype=np.float64)
        v1 = np.asarray(waypoint_velocities[index], dtype=np.float64)
        cubic = 2.0 * q0 - 2.0 * q1 + duration * (v0 + v1)
        quadratic = -3.0 * q0 + 3.0 * q1 - duration * (2.0 * v0 + v1)
        candidates = [0.0, 1.0]
        valid = np.abs(cubic) > 1e-12
        extrema = np.zeros_like(cubic)
        extrema[valid] = -quadratic[valid] / (3.0 * cubic[valid])
        candidates.extend(float(value) for value in extrema[valid] if 0.0 < value < 1.0)
        for ratio in candidates:
            _, velocity = sample_cubic_hermite(
                q0, q1, v0, v1, duration, ratio
            )
            maximum = max(maximum, float(np.max(np.abs(velocity))))
    return maximum


def retime_cubic_hermite(
    targets: list[np.ndarray],
    requested_times: list[float],
    *,
    velocity_limit: float,
    max_iterations: int = 12,
) -> tuple[list[float], list[np.ndarray]]:
    """Retimes only unsafe Hermite segments while preserving recorded pauses.

    ``requested_times`` is an absolute playback clock.  If one segment needs
    more time, every following timestamp must shift, but the durations of the
    already-safe following segments must not be multiplied as well.
    """
    if len(targets) != len(requested_times):
        raise ValueError("targets and requested_times must have equal lengths")
    if not targets:
        return [], []
    limit = abs(float(velocity_limit))
    if limit <= 0.0:
        raise ValueError("velocity_limit must be positive")

    times = [max(float(requested_times[0]), 0.0)]
    for index in range(1, len(requested_times)):
        requested_duration = max(
            float(requested_times[index]) - float(requested_times[index - 1]),
            0.001,
        )
        max_delta = float(np.max(np.abs(targets[index] - targets[index - 1])))
        times.append(times[-1] + max(requested_duration, max_delta / limit))

    for _ in range(max(1, int(max_iterations))):
        velocities = monotone_waypoint_velocities(
            targets, times, velocity_limit=limit
        )
        durations = np.diff(np.asarray(times, dtype=np.float64))
        scales = np.ones(len(durations), dtype=np.float64)
        for index, duration in enumerate(durations, start=1):
            peak = maximum_cubic_hermite_speed(
                targets[index - 1:index + 1],
                [0.0, float(duration)],
                velocities[index - 1:index + 1],
            )
            if peak > limit * (1.0 + 1e-9):
                scales[index - 1] = peak / limit

        if float(np.max(scales)) <= 1.0 + 1e-9:
            return times, velocities

        retimed = [times[0]]
        for duration, scale in zip(durations, scales):
            retimed.append(retimed[-1] + float(duration * scale))
        times = retimed

    velocities = monotone_waypoint_velocities(
        targets, times, velocity_limit=limit
    )
    return times, velocities
