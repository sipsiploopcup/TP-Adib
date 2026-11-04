"""Cas géométriques synthétiques ; les exports utilisent le vrai Scan 1."""

import numpy as np
import pytest

from segments import (angle_between, are_close, extract_segments,
                      intersection, merge_segments, trim_segment)


def test_angles_and_proximity():
    horizontal = [[0, 0], [1, 0]]
    assert angle_between(horizontal, [[2, 0], [1, 0]]) == pytest.approx(0)
    assert angle_between(horizontal, [[0, 0], [0, 1]]) == pytest.approx(90)
    assert are_close(horizontal, [[1.1, 0.03], [2, 0.03]])
    assert not are_close(horizontal, [[2, 0], [3, 0]])
    assert not are_close(horizontal, [[0.5, -1], [0.5, 1]])
    np.testing.assert_allclose(merge_segments(horizontal, [[1.1, 0], [2, 0]]),
                               [[0, 0], [2, 0]], atol=1e-10)
    with pytest.raises(ValueError):
        merge_segments(horizontal, [[4, 0], [5, 0]])


@pytest.mark.parametrize("cutter, expected", [
    ([[1, -1], [1, 1]], [[1, 0], [4, 0]]),
    ([[3, -1], [3, 1]], [[0, 0], [3, 0]]),
    ([[2, -1], [2, 1]], [[0, 0], [2, 0]]),
    ([[2 - 1e-8, -1], [2 - 1e-8, 1]], [[0, 0], [2 - 1e-8, 0]]),
    ([[2 + 1e-8, -1], [2 + 1e-8, 1]], [[0, 0], [2 + 1e-8, 0]]),
    ([[0, -1], [0, 1]], [[0, 0], [4, 0]]),
    ([[4, -1], [4, 1]], [[0, 0], [4, 0]]),
    ([[1, 0], [1, 1]], [[1, 0], [4, 0]]),
    ([[1, 1], [1, 2]], [[0, 0], [4, 0]]),
    ([[5, -1], [5, 1]], [[0, 0], [4, 0]]),
    ([[1, 0], [3, 0]], [[0, 0], [4, 0]]),
    ([[0, 1], [4, 1]], [[0, 0], [4, 0]]),
])
def test_trim_rules(cutter, expected):
    np.testing.assert_allclose(trim_segment([[0, 0], [4, 0]], cutter), expected)


def test_intersection_and_degenerate_input():
    np.testing.assert_allclose(intersection([[0, 0], [2, 2]], [[0, 2], [2, 0]]), [1, 1])
    assert intersection([[0, 0], [1, 0]], [[2, -1], [2, 1]]) is None
    assert intersection([[0, 0], [2, 0]], [[1, 0], [3, 0]]) is None
    with pytest.raises(ValueError):
        angle_between([[0, 0], [0, 0]], [[0, 0], [1, 0]])
    with pytest.raises(ValueError):
        extract_segments([[float("nan"), 1]])


def test_extract_corner_and_separate_collinear_runs():
    # Mur en L synthétique, suffisamment échantillonné pour éviter les gaps.
    x = np.linspace(1, 3, 41)
    corner = np.vstack((np.column_stack((x, np.full(41, 3))),
                        np.column_stack((np.full(40, 3), np.linspace(2.95, 1, 40)))))
    segments = extract_segments(corner)
    assert len(segments) == 2
    assert angle_between(*segments) == pytest.approx(90, abs=2)
    separate = np.vstack((np.column_stack((np.linspace(1, 2, 20), np.ones(20))),
                          np.column_stack((np.linspace(4, 5, 20), np.ones(20)))))
    segments = extract_segments(separate)
    assert len(segments) == 2  # Ne pas fermer le trou de deux mètres.
    assert all(np.linalg.norm(s[1] - s[0]) == pytest.approx(1) for s in segments)
    assert extract_segments(np.zeros((10, 2))) == []
    assert extract_segments(np.empty((0, 2))) == []


def test_fused_export_extracts_locally_then_applies_each_pose(tmp_path):
    import csv
    import json
    from openpyxl import Workbook
    from geometry import apply_pose
    from segments import main

    path = tmp_path / "fusion.xlsx"
    book = Workbook()
    sheet = book.active
    sheet.append(["scan_source", "dist_mm", "x_lib_mm", "y_lib_mm", "x_m", "y_m"])
    local = np.column_stack((np.linspace(1, 2, 20), np.ones(20)))
    poses = [(0, 0, 0), (6, 4, 90)]
    for index, pose in enumerate(poses):
        for point, world in zip(local, apply_pose(local, *pose)):
            sheet.append([f"scan{index}", 2000, point[0] * 1000, -point[1] * 1000,
                          float(world[0]), float(world[1])])
    book.save(path)
    prefix = tmp_path / "segments_fusion"
    segments = main([str(path), "--output", str(prefix), "--title", "Fusion", "--no-show"])
    assert len(segments) == 2
    for actual, pose in zip(segments, poses):
        expected = apply_pose(np.array([[1, 1], [2, 1]]), *pose)
        # Le sens des extrémités est sans importance pour cette géométrie.
        assert min(np.linalg.norm(actual - expected), np.linalg.norm(actual[::-1] - expected)) < 1e-8
    with prefix.with_suffix(".csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 2
    metadata = json.loads(prefix.with_suffix(".json").read_text())
    assert metadata["group_count"] == 2
    assert metadata["valid_points"] == 40
    assert not metadata["cross_scan_consolidation"]
    assert prefix.with_suffix(".png").stat().st_size > 0
