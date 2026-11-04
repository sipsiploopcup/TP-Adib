"""Transport réseau remplacé par un faux serveur : aucun moteur réel utilisé."""

import struct

import numpy as np
import pytest

from slam_robot import RobotHTTP, StaleScanError, decode_robot_scan


def packet(sequence=1, acquired=100, now=110):
    rays = b''.join(struct.pack('<HHB', angle, 1000, 200) for angle in range(0, 36000, 3000))
    return struct.pack('<4sIIIH', b'SLM1', sequence, acquired, now, 12) + rays


def test_binary_units_orientation_sequence_and_rollover():
    points, metadata = decode_robot_scan(packet())
    np.testing.assert_allclose(points[0], [1, 0], atol=1e-12)
    np.testing.assert_allclose(points[3], [0, 1], atol=1e-12)
    assert metadata['sequence'] == 1
    decode_robot_scan(packet(sequence=0), 0xffffffff)
    decode_robot_scan(packet(acquired=0xfffffff0, now=10))
    with pytest.raises(StaleScanError):
        decode_robot_scan(packet(), 1)
    with pytest.raises(StaleScanError):
        decode_robot_scan(packet(now=2000))
    with pytest.raises(ValueError):
        decode_robot_scan(packet()[:-1])


def test_disabled_motion_never_opens_connection():
    calls = []
    robot = RobotHTTP('http://robot', opener=lambda *a, **kw: calls.append(a))
    with pytest.raises(PermissionError):
        robot.move('forward', .1)
    assert not calls


def test_pulse_is_bounded_rotation_convention_and_failure_stops(monkeypatch):
    monkeypatch.setattr('slam_robot.time.sleep', lambda t: None)
    robot = RobotHTTP('http://robot', motion_enabled=True)
    paths = []
    robot._request = lambda path: paths.append(path)
    robot.move('rotateleft', .2)
    assert paths == ['/rotateleft?time=400', '/stop']
    with pytest.raises(ValueError):
        robot.move('forward', 1)
    def failing_request(path):
        paths.append(path)
        if path.startswith('/forward'):
            raise OSError('network failure')
    robot._request = failing_request
    with pytest.raises(OSError):
        robot.move('forward', .1)
    assert paths[-1] == '/stop'


def test_live_robot_waits_for_two_scans_and_stops_after_last(monkeypatch, tmp_path):
    from types import SimpleNamespace
    import slam_tp3
    from comparaison_localisation import SIMULATION_SEGMENTS, simulate_scan
    from slam import IncrementalSLAM
    calls = []
    class FakeRobot:
        last_payload = b'original packet'
        def __init__(self, *args, **kwargs):
            self.sequence = 0
        def fresh_scan(self):
            self.sequence += 1
            return simulate_scan(SIMULATION_SEGMENTS, (0, 0, 0)), {
                'sequence': self.sequence, 'max_valid_angle_gap_deg': 1}
        def move(self, kind, seconds):
            calls.append((kind, seconds))
        def stop(self):
            calls.append(('stop', 0))
    class FakePlanner:
        def __init__(self, **kwargs):
            pass
        def action(self, *args):
            return {'kind': 'forward', 'distance_m': .08}
    monkeypatch.setattr(slam_tp3, 'RobotHTTP', FakeRobot)
    monkeypatch.setattr(slam_tp3, 'FrontierExplorer', FakePlanner)
    monkeypatch.setattr(slam_tp3.time, 'sleep', lambda seconds: None)
    args = SimpleNamespace(robot_url='http://unused', auto=True, robot_radius=.18,
                           steps=3, speed=.2, turn_rate=60, output=tmp_path)
    actions = []
    slam_tp3.robot_live(IncrementalSLAM(), args, slam_tp3.LiveView(False), actions)
    assert calls == [('forward', .25), ('stop', 0)]
    assert actions[0]['kind'] == 'hold'
    assert actions[-1]['reason'] == 'budget_acquisitions_atteint'
    assert len(list((tmp_path/'scans_locaux').glob('*.bin'))) == 3
