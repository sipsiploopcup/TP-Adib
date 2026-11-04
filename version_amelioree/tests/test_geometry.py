"""Tests synthétiques des unités, du repère et de la pose."""

import numpy as np

from geometry import apply_pose, load_scan


def test_library_coordinates_and_missing_return(tmp_path):
    path = tmp_path / "scan.csv"
    path.write_text("dist_mm,x_lib_mm,y_lib_mm\n1000,0,-1000\n0,0,0\n")
    np.testing.assert_allclose(load_scan(path), [[0, 1]])


def test_pose_rotates_before_translating():
    points = np.array([[1, 0], [0, 1]])
    original = points.copy()
    np.testing.assert_allclose(apply_pose(points, 2, 3, 90), [[2, 4], [1, 3]])
    np.testing.assert_array_equal(points, original)
