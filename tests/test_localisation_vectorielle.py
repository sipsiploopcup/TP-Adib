"""Géométries synthétiques ; les exports utilisateurs utilisent les scans réels."""

import json

import numpy as np
import pytest

from geometry import apply_pose
from localisation_grid import angle_error
from localisation_vectorielle import (VectorMatcher, evaluate, load_vector_map,
                                       prepare_evaluation, save_vector_map)


def scene():
    segments = np.array([[[-3, 2], [3, 2]], [[3, -2], [3, 2]], [[-2, -2], [1, -2]]])
    points = np.vstack([np.linspace(a, b, 35) for a, b in segments])
    return segments, points


def test_distance_is_to_finite_segment_not_infinite_line():
    matcher = VectorMatcher([[[0, 0], [1, 0]]], max_distance=5)
    distances = matcher.distances(np.array([[0.5, 0.3], [2, 0], [-1, 1]]), np.zeros((1, 3)))
    np.testing.assert_allclose(distances, [[0.3, 1, np.sqrt(2)]])
    clipped = VectorMatcher([[[0, 0], [1, 0]]]).distances(np.array([[10, 10]]), np.zeros((1, 3)))
    assert clipped[0, 0] == pytest.approx(0.6)


def test_nearest_of_multiple_segments_and_pose_transform():
    matcher = VectorMatcher([[[0, 0], [1, 0]], [[0, 2], [1, 2]]])
    points = np.array([[0.5, 0]])
    np.testing.assert_allclose(matcher.distances(points, np.array([[0, 0, 0], [0, 1.9, 0]])),
                               [[0], [0.1]])


@pytest.mark.parametrize("truth", [(0.7, -0.5, 12), (-0.8, 0.6, -15)])
def test_recovers_pose_on_asymmetric_vector_map(truth):
    segments, world = scene()
    query = apply_pose(world - truth[:2], theta_deg=-truth[2])
    result = VectorMatcher(segments).match(query, xy_range=1, angle_range=20)
    assert np.linalg.norm(np.array(result["pose"][:2]) - truth[:2]) < 0.06
    assert abs(angle_error(result["pose"][2], truth[2])) < 0.5
    assert result["overlap_20cm"] == 1


def test_map_csv_roundtrip_and_invalid_maps(tmp_path):
    segments, _ = scene()
    path = tmp_path / "map.csv"
    save_vector_map(path, segments)
    np.testing.assert_array_equal(load_vector_map(path), segments)
    path.write_text("x,y\n1,2\n")
    with pytest.raises(ValueError):
        load_vector_map(path)
    for invalid in ([], [[[0, 0], [0, 0]]], [[[0, 0], [float("nan"), 1]]]):
        with pytest.raises(ValueError):
            VectorMatcher(invalid)


def test_reference_extraction_does_not_use_reserved_points(monkeypatch):
    import localisation_vectorielle
    _, points = scene()
    seen = []
    real_extract = localisation_vectorielle.extract_segments

    def capture(reference):
        seen.append(reference.copy())
        return real_extract(reference)

    monkeypatch.setattr(localisation_vectorielle, "extract_segments", capture)
    groups = [(points, [0, 0, 0], {"source": "a"}),
              (points, [5, 3, 90], {"source": "b"})]
    segments, queries, metadata = prepare_evaluation(groups)
    keep = np.arange(len(points)) % 5 != 0
    assert len(segments) > 0
    assert len(queries) == 2
    for index, reference in enumerate(seen):
        np.testing.assert_array_equal(reference, points[keep])
        np.testing.assert_allclose(queries[index][0], apply_pose(points[~keep], *groups[index][1]))
        assert metadata[index]["reference_points"] == int(keep.sum())


def test_evaluation_exports_controlled_results_and_vector_reference(tmp_path, monkeypatch):
    import localisation_vectorielle
    _, points = scene()
    monkeypatch.setattr(localisation_vectorielle, "load_scan_groups",
                        lambda path: [(points, [0, 0, 0], {"source": "synthetic"})])
    prefix = tmp_path / "evaluation"
    result = evaluate(tmp_path / "synthetic.xlsx", prefix, "Scène synthétique")
    assert len(result["cases"]) == 5
    assert not result["physical_ground_truth"]
    assert result["reference_points"] + result["reserved_points_total"] == len(points)
    assert all(r["points"] == result["reserved_points_total"] for r in result["cases"])
    saved = json.loads(prefix.with_suffix(".json").read_text())
    assert saved["reference_segments"] == len(load_vector_map(tmp_path / "evaluation_reference.csv"))
    assert prefix.with_suffix(".png").stat().st_size > 0
    assert prefix.with_suffix(".csv").exists()
