"""SLAM en direct sur le robot ESP32, en Wi-Fi, avec le firmware RobotControl d'origine.

Boucle : arrêt → scan frais (/scan) → SLAM scan-sur-carte → pose renvoyée à
RobotControl (/pose) → action de l'explorateur → impulsion moteur courte.

Le firmware d'origine n'horodate pas les scans : on attend après chaque arrêt
que le LiDAR ait fait plusieurs tours, et on exige un scan différent du
précédent. Aucun mouvement sans --auto, et /stop est envoyé à la sortie.

Exemples (PC connecté au réseau RobotESP32) :
  python robot_wifi.py --steps 20                      # cartographie à la main (RobotControl)
  python robot_wifi.py --calibrer --auto               # mesure vitesse et rotation par SLAM
  python robot_wifi.py --auto --speed 0.25 --turn-rate 90 --steps 300
"""

import argparse
import csv
import json
import math
import struct
import sys
import time
from pathlib import Path
from urllib.request import urlopen

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from slam_carte import MapSLAM, NearestFrontierExplorer, compose  # noqa: E402


def decode_scan(payload):
    """Format /scan : int16 x mm, int16 y mm, uint16 theta cdeg, uint16 N, N × (uint16, uint16, uint8)."""
    if len(payload) < 8:
        raise ValueError("Réponse /scan trop courte")
    x, y, theta, n = struct.unpack_from("<hhHH", payload)
    if not 1 <= n <= 460 or len(payload) != 8 + 5 * n:
        raise ValueError(f"Réponse /scan invalide : N={n}, {len(payload)} octets")
    rows = np.array(list(struct.iter_unpack("<HHB", payload[8:])), dtype=float)
    angle, dist = np.deg2rad(rows[:, 0] / 100), rows[:, 1] / 1000
    ok = dist > 0  # distance nulle = pas d'impact, pas un obstacle
    # Même convention que RobotControl : X avant, Y gauche, angle antihoraire.
    points = np.column_stack((dist[ok] * np.cos(angle[ok]), dist[ok] * np.sin(angle[ok])))
    return points, rows, (x / 1000, y / 1000, theta / 100)


class Robot:
    def __init__(self, url, motion=False, timeout=1.5):
        self.url, self.motion, self.timeout = url.rstrip("/"), motion, timeout
        self.last = None

    def get(self, path):
        with urlopen(self.url + path, timeout=self.timeout) as r:
            return r.read(8 + 5 * 460)

    def fresh_scan(self, settle=0.35, tries=10):
        """Attend ~3 tours du LD06 (10 Hz) robot arrêté, puis exige un scan nouveau."""
        time.sleep(settle)
        for _ in range(tries):
            payload = self.get("/scan")
            if payload[8:] != self.last:
                self.last = payload[8:]
                return decode_scan(payload)
            time.sleep(0.1)
        raise TimeoutError("Le scan publié ne change plus : LiDAR arrêté ?")

    def show_pose(self, pose):
        """Affiche la pose SLAM dans RobotControl (champ /pose du firmware)."""
        x, y = (int(np.clip(round(v * 1000), -32768, 32767)) for v in pose[:2])
        self.get(f"/pose?x={x}&y={y}&z={int(round(pose[2] * 100)) % 36000}")

    def move(self, kind, seconds):
        if not self.motion:
            raise PermissionError("Mouvement refusé : ajouter --auto")
        if kind not in ("forward", "rotateleft", "rotateright") or not 0 < seconds <= 0.4:
            raise ValueError("Impulsion refusée")
        ms = int(math.ceil(seconds * 1000))
        # Le firmware divise par deux la durée des rotations.
        self.get(f"/{kind}?time={ms * 2 if kind.startswith('rotate') else ms}")
        time.sleep(ms / 1000 + 0.05)

    def stop(self):
        try:
            self.get("/stop")
        except OSError:
            pass


def save_scan(rows, pose, folder, index):
    """CSV au format RobotControl : angle_deg,distance_m,intensity + pose en commentaire."""
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"scan_{index:04d}.csv"
    with path.open("w", newline="") as f:
        f.write(f"# pose_slam x={pose[0]:.4f} y={pose[1]:.4f} theta={pose[2]:.2f}\n")
        w = csv.writer(f)
        w.writerow(["angle_deg", "distance_m", "intensity"])
        w.writerows([(a / 100, d / 1000, int(i)) for a, d, i in rows])
    return path


def calibrate(robot, slam, pulse=0.3):
    """Mesure la vitesse et la vitesse de rotation avec le SLAM lui-même."""
    points, *_ = robot.fresh_scan()
    slam.process(points, "calib0")
    start = slam.pose.copy()
    robot.move("forward", pulse)
    points, *_ = robot.fresh_scan()
    slam.process(points, "calib1", wide=True)
    moved = np.linalg.norm(slam.pose[:2] - start[:2])
    after = slam.pose.copy()
    robot.move("rotateleft", pulse)
    points, *_ = robot.fresh_scan()
    slam.process(points, "calib2", wide=True)
    turned = abs((slam.pose[2] - after[2] + 180) % 360 - 180)
    return {"speed": moved / pulse, "turn_rate": turned / pulse, "events": slam.events}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default="http://192.168.4.1")
    p.add_argument("--auto", action="store_true", help="autorise les mouvements moteur")
    p.add_argument("--calibrer", action="store_true")
    p.add_argument("--speed", type=float, help="vitesse avant calibrée (m/s)")
    p.add_argument("--turn-rate", type=float, help="rotation calibrée (deg/s)")
    p.add_argument("--steps", type=int, default=200)
    p.add_argument("--output", type=Path, default=Path("resultats/robot"))
    args = p.parse_args(argv)
    if args.auto and not args.calibrer and not (args.speed and args.turn_rate):
        p.error("--auto exige --speed et --turn-rate (lancer d'abord --calibrer --auto)")

    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    robot, slam, explorer = Robot(args.url, motion=args.auto), MapSLAM(), NearestFrontierExplorer()
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        if args.calibrer:
            result = calibrate(robot, slam)
            print(json.dumps({k: v for k, v in result.items() if k != "events"}, indent=2))
            return 0
        plt.ion()
        fig, ax = plt.subplots(figsize=(8, 8))
        nominal, log = None, []
        for step in range(args.steps):
            points, rows, _ = robot.fresh_scan()
            prediction = None if nominal is None else compose(slam.pose, nominal)
            event = slam.process(points, f"scan_{step:04d}", initial=prediction)
            save_scan(rows, slam.pose, args.output / "scans", step)
            robot.show_pose(slam.pose)
            ax.clear()
            ax.imshow(slam.grid.states(), origin="lower", extent=slam.grid.extent,
                      cmap=ListedColormap(["#bbc3cd", "#ffffff", "#243a49"]), vmin=-1, vmax=1)
            traj = np.array([f["pose"] for f in slam.frames])
            ax.plot(traj[:, 0], traj[:, 1], "-", color="#008c78")
            ax.set_title(f"Pas {step} — {event['reason']}")
            ax.set_aspect("equal")
            plt.pause(0.01)
            if not plt.fignum_exists(fig.number):
                break
            if not args.auto:
                input("Déplace le robot avec RobotControl, puis Entrée… ")
                nominal = None
                continue
            action = explorer.action(slam.grid, slam.pose, points, event["accepted"])
            log.append({"step": step, "event": event, "action": action})
            print(step, event["reason"], np.round(slam.pose, 3), action.get("reason"), flush=True)
            if action["kind"] == "stop":
                break
            if action["kind"] == "forward":
                seconds = min(0.4, action["distance_m"] / args.speed)
                nominal = [seconds * args.speed, 0, 0]
            else:
                seconds = min(0.4, action["turn_deg"] / args.turn_rate)
                sign = 1 if action["kind"] == "rotateleft" else -1
                nominal = [0, 0, sign * seconds * args.turn_rate]
            robot.move(action["kind"], seconds)
        return 0
    except KeyboardInterrupt:
        return 1
    finally:
        if args.auto:
            robot.stop()
        if slam.frames:
            slam.grid.save(args.output / "carte_occupation.npz")
            np.savetxt(args.output / "carte_vectorielle.csv", np.array(slam.vector_map(every=5)).reshape(-1, 4),
                       delimiter=",", header="x1,y1,x2,y2", comments="")
            (args.output / "evenements.json").write_text(json.dumps(slam.events, indent=1, default=float))


if __name__ == "__main__":
    raise SystemExit(main())
