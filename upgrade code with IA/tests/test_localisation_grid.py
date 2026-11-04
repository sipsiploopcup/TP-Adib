"""Scènes synthétiques pour les tests ; évaluation réelle dans resultats/."""

import numpy as np
import pytest

from geometry import apply_pose
from gridmap import OccupancyGrid, scan_to_map
from localisation_grid import GridMatcher, angle_error, prepare_evaluation


def asymmetric_scene():
    return np.vstack((
        np.column_stack((np.linspace(-2.975, 3.025, 121), np.full(121, 2.025))),
        np.column_stack((np.full(81, 3.025), np.linspace(-1.975, 2.025, 81))),
        np.column_stack((np.linspace(-1.975, 1.025, 61), np.full(61, -1.975))),
    ))


@pytest.mark.parametrize("truth", [(0.6, -0.4, 10), (-0.6, 0.5, -12)])
def test_known_pose_is_recovered_without_modifying_map(truth):
    world = asymmetric_scene()
    grid = scan_to_map(world)
    original = grid.log_odds.copy()
    query = apply_pose(world[::3] - truth[:2], theta_deg=-truth[2])
    result = GridMatcher(grid).match(query, xy_range=1, angle_range=15)
    assert np.linalg.norm(np.array(result["pose"][:2]) - truth[:2]) < 0.08
    assert abs(angle_error(result["pose"][2], truth[2])) < 1
    assert result["overlap_20cm"] > 0.95
    assert result["status"] == "candidate"
    np.testing.assert_array_equal(grid.log_odds, original)


def test_angles_wrap_around_180_degrees():
    assert angle_error(-179, 179) == pytest.approx(2)
    world = asymmetric_scene()
    truth = (0.1, -0.2, -177)
    query = apply_pose(world[::3] - truth[:2], theta_deg=-truth[2])
    result = GridMatcher(scan_to_map(world)).match(query, initial=(0, 0, 179),
                                                 xy_range=0.5, angle_range=10)
    assert abs(angle_error(result["pose"][2], truth[2])) < 1


def test_distances_use_cell_centers_and_penalize_out_of_map_points():
    grid = scan_to_map([[1.025, 1.025]], resolution=0.05)
    matcher = GridMatcher(grid)
    distances = matcher.distances(np.array([[1.025, 1.025], [100, 100]]), np.zeros((1, 3)))
    assert distances[0, 0] == pytest.approx(0, abs=1e-10)
    assert distances[0, 1] == pytest.approx(0.6)


def test_insufficient_overlap_is_reported():
    matcher = GridMatcher(scan_to_map(asymmetric_scene()))
    with pytest.raises(ValueError):
        GridMatcher(matcher.grid, max_distance=0.1)
    result = matcher.match(np.full((6, 2), 100), xy_range=0.1, angle_range=1)
    assert result["status"] == "insufficient_overlap"
    assert result["overlap_20cm"] == 0


def test_empty_map_and_invalid_inputs_are_rejected():
    with pytest.raises(ValueError, match="occupée"):
        GridMatcher(OccupancyGrid())
    matcher = GridMatcher(scan_to_map(asymmetric_scene()))
    for points in (np.empty((0, 2)), np.full((6, 2), np.nan)):
        with pytest.raises(ValueError):
            matcher.match(points)
    for kwargs in ({"xy_range": 0}, {"angle_range": float("nan")}, {"beam_width": 0}):
        with pytest.raises(ValueError):
            matcher.match(asymmetric_scene(), **kwargs)


def test_fusion_evaluation_keeps_origins_and_holds_out_each_group():
    points = np.column_stack((np.linspace(1, 2, 40), np.ones(40)))
    groups = [(points, [0, 0, 0], {"source": "fusion.xlsx", "scan_source": "a"}),
              (points, [10, 3, 90], {"source": "fusion.xlsx", "scan_source": "b"})]
    grid, queries, metadata = prepare_evaluation(groups)
    assert len(queries) == 2
    for index, (world, base_pose, name) in enumerate(queries):
        np.testing.assert_allclose(world, apply_pose(points[::5], *groups[index][1]))
        np.testing.assert_allclose(base_pose, groups[index][1])
        assert metadata[index]["reference_points"] == 32
        assert metadata[index]["reserved_points"] == 8
    # Les rayons du second groupe partent de (10, 3), pas de (0, 0).
    assert grid.states()[grid.cell((10, 3))] == 0
    isolated, _, _ = prepare_evaluation(groups[:1])
    expected = scan_to_map(points[np.arange(len(points)) % 5 != 0])
    np.testing.assert_array_equal(isolated.log_odds, expected.log_odds)


def test_evaluation_exports_one_origin_per_case(tmp_path, monkeypatch):
    import json
    import localisation_grid
    groups = [(asymmetric_scene(), [0, 0, 0], {"source": "a", "scan_source": "a"}),
              (asymmetric_scene(), [5, 3, 90], {"source": "b", "scan_source": "b"})]
    monkeypatch.setattr(localisation_grid, "load_scan_groups", lambda path: groups)
    prefix = tmp_path / "fusion"
    summary = localisation_grid.evaluate(tmp_path / "fusion.xlsx", prefix, "Fusion synthétique")
    assert [case["query_source"] for case in summary["cases"]] == ["a", "b", "a", "b", "a"]
    assert not summary["physical_ground_truth"]
    assert summary["query_points"] is None
    for index, case in enumerate(summary["cases"]):
        np.testing.assert_allclose(case["initial_pose"], groups[index % 2][1])
        assert case["points"] < len(asymmetric_scene())
        assert "reference_coverage_ok" in case
    assert json.loads(prefix.with_suffix(".json").read_text())["groups"]
    assert prefix.with_suffix(".png").stat().st_size > 0
    assert (tmp_path / "fusion_reference.npz").exists()
