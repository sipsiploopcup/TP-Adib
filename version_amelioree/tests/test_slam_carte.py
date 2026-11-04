"""Tests de la version Claude du SLAM (scan-sur-carte + exploration)."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from comparaison_localisation import SIMULATION_SEGMENTS, simulate_scan  # noqa: E402
from slam_carte import MapSLAM, NearestFrontierExplorer, compose  # noqa: E402


def test_compose_in_robot_frame():
    np.testing.assert_allclose(compose([1, 2, 90], [1, 0, 100]), [1, 3, -170], atol=1e-10)


def test_tracks_small_motions_with_wrong_odometry():
    slam = MapSLAM()
    slam.process(simulate_scan(SIMULATION_SEGMENTS, np.zeros(3), seed=0), "0")
    for i, truth in enumerate([[.1, 0, 2], [.2, .05, 4], [.3, .1, 6]], start=1):
        truth = np.array(truth, float)
        event = slam.process(simulate_scan(SIMULATION_SEGMENTS, truth, seed=i), str(i),
                             initial=truth + [.08, -.06, 3])  # odométrie fausse de 10 cm / 3°
        assert event["accepted"]
        assert np.linalg.norm(slam.pose[:2] - truth[:2]) < .04
        assert abs(slam.pose[2] - truth[2]) < .8


def test_rejected_scan_keeps_map_but_follows_odometry():
    slam = MapSLAM()
    slam.process(simulate_scan(SIMULATION_SEGMENTS, np.zeros(3), seed=0), "0")
    before = slam.grid.log_odds.copy()
    far = np.column_stack((np.linspace(9, 10, 80), np.linspace(-1, 1, 80)))
    event = slam.process(far, "faux", initial=[.2, 0, 0])
    assert not event["accepted"]
    np.testing.assert_array_equal(slam.grid.log_odds, before)
    np.testing.assert_allclose(slam.pose, [.2, 0, 0])  # navigation à l'estime


def test_explorer_moves_towards_unknown_then_stops_when_done():
    slam = MapSLAM()
    points = simulate_scan(SIMULATION_SEGMENTS, np.zeros(3), seed=0)
    slam.process(points, "0")
    action = NearestFrontierExplorer().action(slam.grid, slam.pose, points)
    assert action["kind"] in ("forward", "rotateleft", "rotateright")
    assert NearestFrontierExplorer().action(slam.grid, slam.pose, points, localized=False)["kind"] == "stop"


# ------------------------------------------------------------- robot Wi-Fi
import json  # noqa: E402
import struct  # noqa: E402

import pytest  # noqa: E402

from robot_wifi import Robot, decode_scan  # noqa: E402


def payload_from(points_polar, pose=(0, 0, 0)):
    head = struct.pack("<hhHH", int(pose[0] * 1000), int(pose[1] * 1000), int(pose[2] * 100) % 36000,
                       len(points_polar))
    return head + b"".join(struct.pack("<HHB", int(a * 100), int(d * 1000), 200) for a, d in points_polar)


def test_decode_scan_robotcontrol_format():
    points, rows, pose = decode_scan(payload_from([(0, 1.0), (90, 2.0), (180, 0.0)], (1.5, -0.25, 90)))
    np.testing.assert_allclose(points, [[1, 0], [0, 2]], atol=1e-9)  # distance 0 ignorée
    assert pose == (1.5, -0.25, 90)
    with pytest.raises(ValueError):
        decode_scan(payload_from([(0, 1.0)])[:-1])


def test_robot_refuses_motion_without_auto_and_rejects_stale_scan(monkeypatch):
    robot = Robot("http://192.168.4.1")
    with pytest.raises(PermissionError):
        robot.move("forward", .1)
    payload = payload_from([(a, 2.0) for a in range(0, 360, 10)])
    monkeypatch.setattr(robot, "get", lambda path: payload)
    monkeypatch.setattr("time.sleep", lambda s: None)
    robot.fresh_scan()
    with pytest.raises(TimeoutError):
        robot.fresh_scan()  # même scan qu'avant : refusé


def test_full_loop_with_fake_esp32(monkeypatch, tmp_path):
    """Faux ESP32 : scans simulés à la vraie pose, mouvements avec erreur de 8 %."""
    import robot_wifi
    truth = {"pose": np.zeros(3), "n": 0}

    def fake_get(self, path):
        if path.startswith("/forward"):
            ms = int(path.split("=")[1])
            truth["pose"] = compose(truth["pose"], [0.25 * ms / 1000 * 1.08, 0, 0])
        elif path.startswith("/rotate"):
            ms = int(path.split("=")[1]) / 2
            sign = 1 if "left" in path else -1
            truth["pose"] = compose(truth["pose"], [0, 0, sign * 90 * ms / 1000 * 0.92])
        elif path == "/scan":
            truth["n"] += 1
            pts = simulate_scan(SIMULATION_SEGMENTS, truth["pose"], seed=truth["n"])
            return payload_from([(np.rad2deg(np.arctan2(y, x)) % 360, np.hypot(x, y)) for x, y in pts])
        return b"OK"

    monkeypatch.setattr(Robot, "get", fake_get)
    monkeypatch.setattr("time.sleep", lambda s: None)
    monkeypatch.setattr("builtins.input", lambda *a: "")
    import matplotlib
    matplotlib.use("Agg")
    out = tmp_path
    code = robot_wifi.main(["--auto", "--speed", "0.25", "--turn-rate", "90", "--steps", "25",
                            "--output", str(out)])
    assert code == 0
    events = json.loads((out / "evenements.json").read_text())
    assert sum(e["accepted"] for e in events) >= 20
    last = np.array(events[-1]["pose"])
    assert np.linalg.norm(last[:2] - truth["pose"][:2]) < .10  # le SLAM corrige l'erreur moteur

