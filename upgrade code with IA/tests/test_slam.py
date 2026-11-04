"""SLAM : repères, rejet avant insertion, boucle et exploration simulée."""

import numpy as np
import pytest

from comparaison_localisation import SIMULATION_SEGMENTS, simulate_scan
from geometry import apply_pose
from gridmap import OccupancyGrid
from slam import IncrementalSLAM, SLAMConfig, compose_pose, relative_pose
from slam_robot import FrontierExplorer
from slam_tp3 import export_results, simulate, LiveView


def test_pose_composition_uses_robot_frame_and_wraps():
    np.testing.assert_allclose(compose_pose([1, 2, 90], [1, 0, 100]), [1, 3, -170], atol=1e-10)
    a, b = np.array([1, 2, 80]), np.array([-.4, 3, -170])
    np.testing.assert_allclose(compose_pose(a, relative_pose(a, b)), b, atol=1e-10)


def test_incremental_known_motion_and_failed_scan_does_not_pollute_map():
    slam = IncrementalSLAM(SLAMConfig(loop_closure=False))
    truths = [[0, 0, 0], [.1, -.05, 3], [.2, -.1, 6]]
    for i, truth in enumerate(truths):
        points = simulate_scan(SIMULATION_SEGMENTS, truth, seed=10+i)
        event = slam.process(points, str(i))
        assert event['accepted']
        assert np.linalg.norm(slam.pose[:2]-truth[:2]) < .12
        assert abs(slam.pose[2]-truth[2]) < 1.5
    old_grid, old_pose = slam.grid.log_odds.copy(), slam.pose.copy()
    old_frames = len(slam.frames)
    bad = np.column_stack((np.linspace(14, 15, 60), np.linspace(-2, 2, 60)))
    assert not slam.process(bad)['accepted']
    np.testing.assert_array_equal(slam.grid.log_odds, old_grid)
    np.testing.assert_array_equal(slam.pose, old_pose)
    assert len(slam.frames) == old_frames
    assert not slam.process(np.array([[np.nan, 0]]))['accepted']
    assert not slam.process(np.zeros((20, 2)))['accepted']


def test_symmetric_wide_relocalisation_refuses_ambiguity():
    wall = np.vstack((np.column_stack((np.linspace(-2, 2, 80), np.full(80, -2))),
                      np.column_stack((np.linspace(-2, 2, 80), np.full(80, 2))),
                      np.column_stack((np.full(80, -2), np.linspace(-2, 2, 80))),
                      np.column_stack((np.full(80, 2), np.linspace(-2, 2, 80)))))
    slam = IncrementalSLAM(SLAMConfig(max_rotation=180, max_step=5))
    slam.process(wall)
    result = slam.process(wall, wide_search=True)
    assert result['reason'] == 'ambiguous_registration'
    assert len(slam.frames) == 1


def test_graph_anchors_first_node_and_rebuilds_map():
    slam = IncrementalSLAM()
    points = simulate_scan(SIMULATION_SEGMENTS, (0, 0, 0), rays=100)
    poses = [np.array(p, dtype=float) for p in [(0, 0, 0), (1, 0, 0), (1, 1, 90),
                                                (0, 1, 180), (.2, .1, -90)]]
    for i, pose in enumerate(poses):
        slam.keyframes.append({'pose': pose.copy(), 'points': points.copy()})
        slam.frames.append({'source': str(i), 'pose': pose.copy(), 'points': points.copy(),
                            'keyframe': i, 'relative': np.zeros(3)})
        if i:
            slam.edges.append({'a': i-1, 'b': i, 'relative': relative_pose(poses[i-1], pose),
                               'type': 'sequential'})
    slam.edges.append({'a': 0, 'b': 4, 'relative': np.array([0, 0, -90]), 'type': 'loop'})
    assert slam.optimize_graph()
    np.testing.assert_allclose(slam.keyframes[0]['pose'], poses[0])
    assert np.linalg.norm(slam.pose[:2]) < .1
    assert slam.grid.observed.any()
    assert slam.optimizations == 1


def test_return_to_start_adds_a_verified_loop_constraint():
    slam = IncrementalSLAM(SLAMConfig(keyframe_distance=.18))
    path = ([[x, 0, 0] for x in np.linspace(0, 1, 5)]
            + [[1, y, 0] for y in np.linspace(.25, 1, 4)]
            + [[x, 1, 0] for x in np.linspace(.75, 0, 4)]
            + [[0, y, 0] for y in np.linspace(.75, 0, 4)])
    for index, pose in enumerate(path):
        points = simulate_scan(SIMULATION_SEGMENTS, pose, seed=200+index)
        assert slam.process(points, str(index), initial=pose)['accepted']
    assert slam.loop_count >= 1
    assert slam.optimizations >= 1
    assert np.linalg.norm(slam.pose[:2]) < .1


def test_near_sensor_points_are_kept_in_map_not_used_to_fit_pose():
    points = simulate_scan(SIMULATION_SEGMENTS, (0, 0, 0))
    nearby = np.array([[.25, 0], [.26, .1], [.3, -.1]])
    slam = IncrementalSLAM()
    event = slam.process(np.vstack((points, nearby)))
    assert event['accepted']
    assert event['matching_points'] < event['points']
    assert slam.grid.states()[slam.grid.cell(nearby[0])] == 1


def test_frontier_navigation_stops_on_failed_localization_and_obstacle():
    planner = FrontierExplorer()
    grid = OccupancyGrid(.1)
    grid.ensure_bounds(np.array([[0, 0], [5, 5]]))
    grid.observed[:] = True
    grid.log_odds[:] = -1
    grid.observed[-8:] = False
    scan = np.array([[2, 0], [0, 2], [-2, 0], [0, -2]])
    pose = [1, 1, 0]
    assert planner.action(grid, pose, scan, False)['reason'] == 'localisation_rejetee'
    assert planner.action(grid, pose, np.array([[.1, 0]]))['kind'] == 'stop'
    assert planner.action(grid, pose, scan)['kind'] in ('forward', 'rotateleft', 'rotateright')
    grid.log_odds[grid.cell(pose[:2])] = 2
    assert planner.action(grid, pose, scan)['reason'] == 'degagement_non_observe'


def test_simulation_evaluation_is_separate_from_robot_accuracy(tmp_path):
    slam = IncrementalSLAM(SLAMConfig(loop_closure=False))
    truths, actions = simulate(slam, 8, LiveView(False))
    assert len(slam.frames) > 1
    assert len(actions) > 1
    result = export_results(slam, tmp_path, 'simulate', truths, actions, vector=False)
    assert not result['physical_ground_truth']
    assert not result['complete_classroom_verified']
    assert result['simulation_evaluation']['mean_position_error_m'] < .15
    assert (tmp_path/'carte_occupation.npz').exists()
    assert (tmp_path/'carte_slam.png').exists()


@pytest.mark.parametrize('kwargs', [{'resolution': 0}, {'min_overlap': 2},
                                    {'max_rotation': 181}, {'max_range': float('nan')}])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        SLAMConfig(**kwargs)
