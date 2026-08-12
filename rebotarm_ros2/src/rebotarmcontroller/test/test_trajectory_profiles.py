import numpy as np

from rebotarmcontroller.trajectory_profiles import (
    maximum_cubic_hermite_speed,
    monotone_waypoint_velocities,
    retime_cubic_hermite,
    sample_cubic_hermite,
)


def test_monotone_velocities_stop_at_ends_and_direction_reversal():
    targets = [
        np.array([0.0, 0.0]),
        np.array([0.2, 0.2]),
        np.array([0.4, -0.1]),
        np.array([0.6, -0.2]),
    ]
    velocities = monotone_waypoint_velocities(
        targets, [0.0, 0.5, 1.0, 1.5], velocity_limit=0.6
    )
    np.testing.assert_allclose(velocities[0], 0.0)
    np.testing.assert_allclose(velocities[-1], 0.0)
    assert 0.0 < velocities[1][0] <= 0.6
    assert velocities[1][1] == 0.0


def test_hermite_is_position_and_velocity_continuous_at_waypoint():
    q0 = np.array([0.0])
    q1 = np.array([0.2])
    q2 = np.array([0.35])
    velocities = monotone_waypoint_velocities(
        [q0, q1, q2], [0.0, 0.5, 1.0], velocity_limit=0.6
    )
    left_q, left_v = sample_cubic_hermite(
        q0, q1, velocities[0], velocities[1], 0.5, 1.0
    )
    right_q, right_v = sample_cubic_hermite(
        q1, q2, velocities[1], velocities[2], 0.5, 0.0
    )
    np.testing.assert_allclose(left_q, right_q, atol=1e-12)
    np.testing.assert_allclose(left_v, right_v, atol=1e-12)


def test_monotone_hermite_does_not_overshoot_segment_bounds():
    targets = [np.array([0.0]), np.array([0.2]), np.array([0.25])]
    times = [0.0, 0.5, 1.0]
    velocities = monotone_waypoint_velocities(
        targets, times, velocity_limit=0.6
    )
    samples = [
        sample_cubic_hermite(
            targets[0], targets[1], velocities[0], velocities[1], 0.5, ratio
        )[0][0]
        for ratio in np.linspace(0.0, 1.0, 101)
    ]
    assert min(samples) >= targets[0][0] - 1e-12
    assert max(samples) <= targets[1][0] + 1e-12


def test_peak_speed_detects_cubic_start_stop_peak():
    targets = [np.array([0.0]), np.array([0.3])]
    times = [0.0, 0.5]
    velocities = monotone_waypoint_velocities(
        targets, times, velocity_limit=0.6
    )
    peak = maximum_cubic_hermite_speed(targets, times, velocities)
    assert abs(peak - 0.9) < 1e-12

    scale = peak / 0.6
    scaled_times = [value * scale for value in times]
    scaled_velocities = monotone_waypoint_velocities(
        targets, scaled_times, velocity_limit=0.6
    )
    scaled_peak = maximum_cubic_hermite_speed(
        targets, scaled_times, scaled_velocities
    )
    assert scaled_peak <= 0.6 + 1e-12


def test_retime_extends_only_the_unsafe_segment():
    targets = [np.array([0.0]), np.array([0.4]), np.array([0.5])]
    requested = [0.0, 0.25, 1.25]

    times, velocities = retime_cubic_hermite(
        targets, requested, velocity_limit=1.2
    )

    assert times[1] > requested[1]
    assert abs((times[2] - times[1]) - 1.0) < 1e-12
    assert maximum_cubic_hermite_speed(targets, times, velocities) <= 1.2 + 1e-9
