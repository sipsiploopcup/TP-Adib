"""Scans synthétiques : cellules traversées, impacts et réutilisation d'une carte."""

import numpy as np
import pytest
from openpyxl import Workbook

from geometry import apply_pose
from gridmap import (OccupancyGrid, bresenham, load_measurements, load_scan_groups,
                     recover_pose, scan_to_map, update_map)


def test_ray_free_impact_occupied_and_behind_unknown():
    grid = scan_to_map([[3.2, 0.2]], resolution=1)
    states = grid.states()
    for x in (0.2, 1.2, 2.2):
        assert states[grid.cell((x, 0.2))] == 0
    assert states[grid.cell((3.2, 0.2))] == 1
    assert states[grid.cell((4.2, 0.2))] == -1
    assert states[grid.cell((1.2, 1.2))] == -1


def test_pose_growth_and_save_reload(tmp_path):
    grid = scan_to_map([[2.2, 0.2]], resolution=1)
    first_cell = grid.log_odds[grid.cell((2.2, 0.2))]
    update_map(grid, [[2.2, 0.2]], pose=(-5, -4, 90))
    assert grid.log_odds[grid.cell((2.2, 0.2))] == first_cell
    assert grid.states()[grid.cell((-5.2, -1.8))] == 1
    path = tmp_path / "map.npz"
    grid.save(path)
    loaded = OccupancyGrid.load(path)
    np.testing.assert_array_equal(loaded.states(), grid.states())
    np.testing.assert_array_equal(loaded.observed, grid.observed)
    np.testing.assert_array_equal(loaded.origin, grid.origin)
    update_map(loaded, [[1, 0]], pose=(10, 10, 0))
    assert loaded.states()[loaded.cell((11, 10))] == 1


def test_impacts_win_within_scan_and_duplicate_rays_do_not_overweight():
    grid = scan_to_map([[1.2, 0], [3.2, 0], [3.2, 0]], resolution=1)
    assert grid.states()[grid.cell((1.2, 0))] == 1
    assert grid.log_odds[grid.cell((3.2, 0))] == pytest.approx(0.85)
    for _ in range(30):
        update_map(grid, [[3.2, 0]])
    assert grid.log_odds.max() <= 4
    assert grid.log_odds.min() >= -4
    # Une cellule observée mais exactement contradictoire reste incertaine.
    grid.log_odds[grid.cell((1.2, 0))] = 0
    assert grid.states()[grid.cell((1.2, 0))] == -1


def test_excel_detects_data_sheet_and_ignores_zero_distance(tmp_path):
    path = tmp_path / "scan.xlsx"
    book = Workbook()
    book.active.title = "Vide"
    sheet = book.create_sheet("Mesures")
    sheet.append(["point_index", "angle_deg", "dist_mm", "intensity", "x_lib_mm", "y_lib_mm"])
    sheet.append([0, 90, 1000, 200, 0, -1000])
    sheet.append([1, 0, 0, 0, 0, 0])
    book.save(path)
    points, metadata = load_measurements(path)
    np.testing.assert_array_equal(points, [[0, 1]])
    assert metadata["sheet"] == "Mesures"
    assert metadata["zero_distance_ignored"] == 1


@pytest.mark.parametrize("end", [(5, 2), (-5, 2), (2, 5), (2, -5), (0, 0)])
def test_bresenham_all_directions(end):
    cells = list(bresenham((0, 0), end))
    assert cells[0] == (0, 0)
    assert cells[-1] == end
    assert len(cells) == max(abs(end[0]), abs(end[1])) + 1
    for a, b in zip(cells, cells[1:]):
        assert max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1


def test_invalid_resolution_and_scan():
    with pytest.raises(ValueError):
        scan_to_map([[1, 0]], resolution=0)
    with pytest.raises(ValueError):
        scan_to_map([[float("nan"), 0]])


def test_fused_excel_keeps_distinct_lidar_origins(tmp_path):
    path = tmp_path / "fusion.xlsx"
    book = Workbook()
    sheet = book.active
    sheet.append(["scan_source", "dist_mm", "x_lib_mm", "y_lib_mm", "x_m", "y_m"])
    local = np.array([[2.2, 0], [2.2, 1], [3.2, 1]])
    poses = [(0, 0, 0), (6, 4, 90)]
    for i, pose in enumerate(poses):
        for point, world in zip(local, apply_pose(local, *pose)):
            sheet.append([f"scan{i}.csv", 3000, point[0] * 1000, -point[1] * 1000,
                          round(world[0], 6), round(world[1], 6)])
    book.save(path)
    groups = load_scan_groups(path)
    assert len(groups) == 2
    grid = OccupancyGrid(1)
    for (points, recovered, metadata), pose in zip(groups, poses):
        np.testing.assert_allclose(recovered, pose, atol=1e-6)
        assert metadata["pose_is_estimated"]
        update_map(grid, points, recovered)
    assert grid.states()[grid.cell((6, 4))] == 0
    assert grid.states()[grid.cell((6, 6.2))] == 1
    with pytest.raises(ValueError, match="origines"):
        load_measurements(path)


def test_inconsistent_fusion_coordinates_are_rejected():
    local = np.array([[1, 0], [2, 0], [2, 1]], dtype=float)
    world = local.copy()
    world[2] += 2
    with pytest.raises(ValueError, match="incohérentes"):
        recover_pose(local, world)


def test_vectorised_update_matches_per_ray_bresenham():
    """Version améliorée : mêmes cellules que la boucle rayon par rayon d'origine."""
    from gridmap import bresenham, update_map

    def reference_update(grid, points, pose):
        world = apply_pose(points, *pose)
        grid.ensure_bounds(np.vstack([world, np.asarray(pose[:2])]))
        start = grid.cell(pose[:2])
        free, occupied = set(), set()
        for point in world:
            cells = list(bresenham(start, grid.cell(point)))
            free.update(cells[:-1])
            occupied.add(cells[-1])
        free.difference_update(occupied)
        for cells, increment in ((free, -0.4), (occupied, 0.85)):
            if cells:
                rows, columns = np.array(list(cells)).T
                grid.log_odds[rows, columns] = np.clip(grid.log_odds[rows, columns] + increment, -4, 4)
                grid.observed[rows, columns] = True
        return grid

    rng = np.random.default_rng(3)
    for trial in range(20):
        points = rng.uniform(-6, 6, (300, 2))
        points[:5] = rng.uniform(-.01, .01, (5, 2))  # impacts dans la cellule du robot
        poses = [rng.uniform([-1, -1, -180], [1, 1, 180]) for _ in range(3)]
        fast, slow = OccupancyGrid(0.05), OccupancyGrid(0.05)
        for pose in poses:
            update_map(fast, points, pose)
            reference_update(slow, points, pose)
        np.testing.assert_array_equal(fast.log_odds, slow.log_odds)
        np.testing.assert_array_equal(fast.observed, slow.observed)
