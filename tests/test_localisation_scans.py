"""Tests synthétiques de recalage scan-vers-scan et d'incertitude des données réelles."""

import csv
import json

import numpy as np
import pytest

from geometry import apply_pose
from localisation_grid import angle_error
from localisation_scans import ICPMatcher, ScanMatcher, main, rigid_fit


def asymmetric_points():
    return np.vstack((np.column_stack((np.linspace(-2, 3, 80), np.full(80, 2))),
                      np.column_stack((np.full(60, 3), np.linspace(-1, 2, 60))),
                      np.column_stack((np.linspace(-1, 1, 40), np.full(40, -1)))))


def test_correlative_scan_matcher_recovers_known_transform():
    reference = asymmetric_points()
    truth = (0.6, -0.4, 10)
    query = apply_pose(reference[::3] - truth[:2], theta_deg=-truth[2])
    result = ScanMatcher(reference).match(query, xy_range=1, angle_range=15)
    assert np.linalg.norm(np.array(result["pose"][:2]) - truth[:2]) < 0.06
    assert abs(angle_error(result["pose"][2], truth[2])) < 0.5


def test_icp_refines_nearby_initial_pose_without_modifying_points():
    reference = asymmetric_points()
    truth = (0.3, -0.2, 5)
    query = apply_pose(reference - truth[:2], theta_deg=-truth[2])
    original = query.copy()
    result = ICPMatcher(reference).match(query, initial=(0.31, -0.21, 5.2))
    assert np.linalg.norm(np.array(result["pose"][:2]) - truth[:2]) < 0.03
    assert abs(angle_error(result["pose"][2], truth[2])) < 0.3
    assert result["converged"]
    np.testing.assert_array_equal(query, original)


def test_rigid_fit_never_introduces_a_reflection():
    source = np.array([[0, 0], [1, 0], [0, 2], [2, 3]], dtype=float)
    target = source * [-1, 1]
    rotation, translation = rigid_fit(source, target)
    assert np.linalg.det(rotation) == pytest.approx(1)
    assert np.linalg.norm(source @ rotation.T + translation - target) > 0.1


def test_no_correspondences_and_invalid_input():
    matcher = ICPMatcher(asymmetric_points())
    result = matcher.match(asymmetric_points() + 100)
    assert result["status"] == "insufficient_overlap"
    assert not result["converged"]
    with pytest.raises(ValueError):
        ScanMatcher(np.empty((0, 2)))
    with pytest.raises(ValueError):
        matcher.match(asymmetric_points(), initial=(0, float("nan"), 0))
    with pytest.raises(ValueError):
        matcher.match(asymmetric_points(), max_iterations=0)


def test_real_pair_cli_does_not_invent_ground_truth(tmp_path):
    paths = [tmp_path / "reference.csv", tmp_path / "query.csv"]
    for path in paths:
        with path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["dist_mm", "x_lib_mm", "y_lib_mm"])
            writer.writerows([1000, x * 1000, -y * 1000] for x, y in asymmetric_points())
    prefix = tmp_path / "result"
    result = main([str(paths[0]), str(paths[1]), "--method", "icp", "--output", str(prefix)])
    assert result["translation_error_m"] is None
    assert result["angle_error_deg"] is None
    assert not result["physical_ground_truth"]
    assert "non quantifiée" in result["reference_uncertainty"]
    assert json.loads(prefix.with_suffix(".json").read_text())["methods"]["icp"]["pose"]
    assert prefix.with_suffix(".png").exists()
