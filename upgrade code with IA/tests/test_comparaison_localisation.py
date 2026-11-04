"""Simulation géométrique, métriques et égalité du protocole entre méthodes."""

import numpy as np
import pytest

from comparaison_localisation import compare_case, simulate_scan, summarize
from geometry import apply_pose


BOX = np.array([[[-2, -2], [2, -2]], [[2, -2], [2, 2]],
                [[2, 2], [-2, 2]], [[-2, 2], [-2, -2]]])


def test_raycasting_uses_sensor_pose_and_first_visible_surface():
    points = simulate_scan(BOX, [0, 0, 0], rays=8, noise_sigma=0)
    np.testing.assert_allclose(points[::2], [[2, 0], [0, 2], [-2, 0], [0, -2]], atol=1e-12)
    pose = [0.5, 0.25, 90]
    moved = simulate_scan(BOX, pose, rays=8, noise_sigma=0)
    np.testing.assert_allclose(moved[0], [1.75, 0], atol=1e-12)
    world = apply_pose(moved, *pose)
    assert np.all(np.isclose(abs(world), 2).any(axis=1))
    obstacles = np.concatenate((BOX, [[[1, -1], [1, 1]]]))
    assert simulate_scan(obstacles, [0, 0, 0], rays=8, noise_sigma=0)[0, 0] == pytest.approx(1)


def test_simulation_noise_is_reproducible_and_no_hit_is_not_an_obstacle():
    first = simulate_scan(BOX, [0, 0, 0], seed=42)
    np.testing.assert_array_equal(first, simulate_scan(BOX, [0, 0, 0], seed=42))
    assert not np.array_equal(first, simulate_scan(BOX, [0, 0, 0], seed=43))
    assert simulate_scan(BOX, [0, 0, 0], rays=8, max_range=0.5).shape == (0, 2)
    with pytest.raises(ValueError):
        simulate_scan(BOX, [0, 0, 0], noise_sigma=-1)


def test_all_methods_receive_identical_query_and_initial_pose():
    seen = []

    class Spy:
        def match(self, query, initial, xy_range, angle_range):
            seen.append((query.copy(), initial.copy(), xy_range, angle_range))
            return {"pose": initial.tolist(), "initial_pose": initial.tolist(),
                    "runtime_s": 0.01, "overlap_20cm": 1, "status": "candidate"}

    query = np.array([[0, 1], [1, 2]], dtype=float)
    initial, truth = np.array([0.2, 0.3, 179]), np.array([0.2, 0.3, -179])
    rows, _ = compare_case({"a": Spy(), "b": Spy()}, query, truth, initial)
    np.testing.assert_array_equal(seen[0][0], seen[1][0])
    np.testing.assert_array_equal(seen[0][1], seen[1][1])
    assert seen[0][2:] == seen[1][2:] == (2, 30)
    assert all(row["angle_error_deg"] == pytest.approx(2) for row in rows)
    assert all(not row["success"] for row in rows)


def test_success_rate_includes_failures_in_denominator():
    rows = [{"method": "a", "success": success, "translation_error_m": error,
             "angle_error_deg": 0, "runtime_s": 0.1}
            for success, error in [(True, 0.01), (False, 1.0)]]
    result = summarize(rows, ["method"])[0]
    assert result["success_rate"] == 0.5
    assert result["trials"] == 2
    assert result["mean_translation_error_m"] == pytest.approx(0.505)
