"""TP3 : rejeu CSV, exploration simulée et acquisition/localisation en direct."""

import argparse
import csv
import json
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from acquisition import ScanReader, save_scan
from comparaison_localisation import SIMULATION_SEGMENTS, simulate_scan
from geometry import apply_pose
from gridmap import load_measurements
from localisation_grid import angle_error
from segments import extract_segments
from slam import IncrementalSLAM, SLAMConfig, compose_pose
from slam_robot import FrontierExplorer, RobotHTTP
from vectormap import VectorMap


class LiveView:
    def __init__(self, enabled, connect=True):
        self.enabled = enabled
        self.connect = connect
        if enabled:
            import matplotlib.pyplot as plt
            self.plt = plt
            self.figure, self.ax = plt.subplots(figsize=(9, 8))
            plt.ion()

    def update(self, slam):
        if not self.enabled or not slam.frames:
            return
        if not self.plt.fignum_exists(self.figure.number):
            raise KeyboardInterrupt
        self.ax.clear()
        draw_grid(self.ax, slam, self.connect)
        self.figure.canvas.draw_idle()
        self.plt.pause(.001)

    def close(self):
        if self.enabled:
            self.plt.close(self.figure)


def draw_grid(ax, slam, connect=True):
    from matplotlib.colors import ListedColormap
    ax.imshow(slam.grid.states(), origin="lower", extent=slam.grid.extent,
              cmap=ListedColormap(["#bbc3cd", "#ffffff", "#243a49"]), vmin=-1, vmax=1)
    poses = np.array([frame["pose"] for frame in slam.frames])
    if len(poses):
        ax.plot(poses[:, 0], poses[:, 1], "-" if connect else "+", color="#008c78", linewidth=1.6,
                label="Trajectoire estimée" if connect else "Positions estimées (rejeu)")
        ax.plot(*poses[0, :2], "o", color="#008c78", markersize=5)
        p = slam.pose
        a = np.deg2rad(p[2])
        ax.arrow(p[0], p[1], .35*np.cos(a), .35*np.sin(a), width=.025, color="#db7b48")
    ax.set(xlabel="X (m)", ylabel="Y (m)", title="SLAM — carte d’occupation et poses")
    ax.set_aspect("equal")


def summary(slam, mode, truths=None, actions=None):
    accepted = len(slam.frames)
    result = {"mode": mode, "attempted_scans": len(slam.events), "accepted_scans": accepted,
              "rejected_scans": len(slam.events) - accepted,
              "keyframes": len(slam.keyframes), "loop_constraints": slam.loop_count,
              "graph_optimizations": slam.optimizations,
              "mean_processing_time_s": float(np.mean([e["runtime_s"] for e in slam.events])) if slam.events else None,
              "resolution_m": slam.config.resolution,
              "configuration": asdict(slam.config),
              "cell_counts": {str(v): int(np.count_nonzero(slam.grid.states() == v)) for v in (-1, 0, 1)},
              "physical_ground_truth": False,
              "reference_uncertainty": "Poses physiques non mesurées ; incertitude non quantifiée.",
              "complete_classroom_verified": False,
              "note": "Les cellules inconnues restent inconnues. Une carte produite ne prouve pas une couverture complète.",
              "events": slam.events, "actions": actions or []}
    if truths and slam.frames:
        errors = [(float(np.linalg.norm(frame["pose"][:2] - truths[frame["source"]][:2])),
                   abs(angle_error(frame["pose"][2], truths[frame["source"]][2])))
                  for frame in slam.frames]
        result["simulation_evaluation"] = {"known_poses_in_simulation": True,
                 "reference_uncertainty": "Poses simulées exactes ; bruit radial sigma=0,005 m.",
                 "mean_position_error_m": float(np.mean([e[0] for e in errors])),
                 "max_position_error_m": max(e[0] for e in errors),
                 "mean_angle_error_deg": float(np.mean([e[1] for e in errors])),
                 "max_angle_error_deg": max(e[1] for e in errors),
                 "success_thresholds": {"position_m": .15, "angle_deg": 2},
                 "success_rate": float(np.mean([p <= .15 and a <= 2 for p, a in errors]))}
        occupied = slam.grid.states() == 1
        rows, columns = np.nonzero(occupied)
        wall_points = np.vstack([np.linspace(s[0], s[1], max(2, int(np.linalg.norm(s[1]-s[0])/.04)))
                                 for s in SIMULATION_SEGMENTS])
        cells = np.column_stack((columns+.5, rows+.5))*slam.grid.resolution+slam.grid.origin
        result["simulation_evaluation"]["wall_coverage_within_15cm"] = float(
            np.mean(cKDTree(cells).query(wall_points)[0] <= .15)) if len(cells) else 0.0
    return result


def export_results(slam, output, mode, truths=None, actions=None, vector=True):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    result = summary(slam, mode, truths, actions)
    segments = []
    if slam.frames:
        slam.grid.history = [{"source": frame["source"], "pose": frame["pose"].tolist(),
                              "pose_is_estimated": True} for frame in slam.frames]
        slam.grid.save(output / "carte_occupation.npz")
        np.savetxt(output / "carte_occupation.csv", slam.grid.states(), fmt="%d", delimiter=",")
        if vector:
            # Toutes les images clés si <= 16, sinon un échantillon couvrant la trajectoire.
            indices = np.unique(np.linspace(0, len(slam.keyframes)-1, min(16, len(slam.keyframes))).astype(int))
            candidates = []
            for i in indices:
                keyframe = slam.keyframes[i]
                candidates.extend(apply_pose(s, *keyframe["pose"]) for s in extract_segments(keyframe["points"]))
            segments = VectorMap().add_segments(candidates).segments
            result["vector_keyframes_used"] = len(indices)
            result["vector_segments"] = len(segments)
            np.savetxt(output / "carte_vectorielle.csv", np.array(segments).reshape(-1, 4),
                       delimiter=",", header="x1,y1,x2,y2", comments="")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(15, 7))
        draw_grid(axes[0], slam, connect=mode != "replay")
        if truths:
            poses = np.array(list(truths.values()))
            axes[0].plot(poses[:, 0], poses[:, 1], "--", color="#e4a664", label="Trajectoire vraie simulée")
            axes[0].legend(fontsize=8)
        for segment in segments:
            axes[1].plot(*segment.T, color="#008c78", linewidth=1.4)
        axes[1].set(xlabel="X (m)", ylabel="Y (m)", title="Carte vectorielle — images clés")
        axes[1].set_aspect("equal", adjustable="box")
        axes[1].grid(alpha=.2)
        fig.suptitle(f"TP3 — {mode} · {len(slam.frames)} scans intégrés / {len(slam.events)} reçus")
        fig.tight_layout()
        fig.savefig(output / "carte_slam.png", dpi=160)
        plt.close(fig)
    with (output / "trajectoire.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["source", "x_m", "y_m", "theta_deg", "true_x_m", "true_y_m",
                         "true_theta_deg", "position_error_m", "angle_error_deg"])
        for frame in slam.frames:
            truth = truths.get(frame["source"]) if truths else None
            evaluation = ([*truth, float(np.linalg.norm(frame["pose"][:2]-truth[:2])),
                           abs(angle_error(frame["pose"][2], truth[2]))]
                          if truth is not None else [""]*5)
            writer.writerow([frame["source"], *frame["pose"], *evaluation])
    (output / "bilan.json").write_text(json.dumps(result, indent=2, ensure_ascii=False,
                                                 allow_nan=False)+"\n", encoding="utf-8")
    return result


def replay(slam, paths, priors, view, wide=False):
    normalized = {Path(name).name: pose for name, pose in priors.items()}
    for path in paths:
        points, _ = load_measurements(path)
        event = slam.process(points, path.name, initial=normalized.get(path.name), wide_search=wide)
        print(f"{path.name}: {event['reason']} · pose={np.round(event['pose'], 3)}", flush=True)
        view.update(slam)


def collision_free(a, b, radius=.20):
    """Physique de simulation uniquement ; jamais utilisée par le planificateur."""
    starts, ends = SIMULATION_SEGMENTS[:, 0], SIMULATION_SEGMENTS[:, 1]
    directions = ends-starts
    for p in np.linspace(a[:2], b[:2], max(2, int(np.linalg.norm(b[:2]-a[:2])/.025)+1)):
        t = np.clip(np.sum((p-starts)*directions, axis=1)/np.sum(directions**2, axis=1), 0, 1)
        if np.min(np.linalg.norm(p-starts-t[:, None]*directions, axis=1)) < radius:
            return False
    return True


def simulate(slam, steps, view):
    explorer, truth, truths, actions = FrontierExplorer(), np.zeros(3), {}, []
    prediction = None
    for index in range(steps):
        name = f"simulation_{index:04d}"
        points = simulate_scan(SIMULATION_SEGMENTS, truth, noise_sigma=.005, seed=1000+index)
        truths[name] = truth.copy()
        event = slam.process(points, name, initial=prediction)
        view.update(slam)
        action = (explorer.action(slam.grid, slam.pose, points, event["accepted"])
                  if index < steps-1 else {"kind": "stop", "reason": "budget_acquisitions_atteint"})
        action["index"] = index
        actions.append(action)
        if action["kind"] == "stop":
            break
        if action["kind"] == "forward":
            actual = [action["distance_m"], 0, 0]
            nominal = [action["distance_m"]*1.03, 0, 0]
        else:
            turn = action["turn_deg"] * (1 if action["kind"] == "rotateleft" else -1)
            actual, nominal = [0, 0, turn], [0, 0, turn*.97]
        next_truth = compose_pose(truth, actual)
        if not collision_free(truth, next_truth):
            actions.append({"kind": "stop", "reason": "collision_simulee_empechee", "index": index})
            break
        truth = next_truth
        prediction = compose_pose(slam.pose, nominal)  # calibrated command, not true pose
    return truths, actions


def serial_live(slam, port, steps, output, view):
    import serial
    reader, pending, scans = ScanReader(), b"", 0
    with serial.Serial(port, 115200, timeout=.3, exclusive=True) as device:
        last_scan = time.monotonic()
        while scans < steps:
            pending += device.read_until(b"\n")
            if len(pending) > 4096:
                reader, pending = ScanReader(), b""
            if time.monotonic()-last_scan > 15:
                raise TimeoutError("Aucun scan CSV complet depuis 15 s")
            if not pending.endswith(b"\n"):
                continue
            try:
                rows = reader.feed(pending.decode("utf-8"))
            except UnicodeDecodeError:
                reader, rows = ScanReader(), None
            pending = b""
            if rows is None:
                continue
            path = save_scan(rows, output / "scans_bruts")
            points = np.array([[row[4]/1000, -row[5]/1000] for row in rows if row[2] > 0])
            print(slam.process(points, path.name), flush=True)
            last_scan = time.monotonic()
            scans += 1
            view.update(slam)


def robot_live(slam, args, view, actions):
    robot = RobotHTTP(args.robot_url, motion_enabled=args.auto)
    explorer = FrontierExplorer(radius=args.robot_radius)
    nominal = None
    try:
        for index in range(args.steps):
            points, stamp = robot.fresh_scan()
            # Save original local polar samples as an ordinary project CSV.
            distance = np.linalg.norm(points, axis=1)
            angle = np.rad2deg(np.arctan2(points[:, 1], points[:, 0])) % 360
            order = np.argsort(-angle)
            rows = [(i, angle[j], int(round(distance[j]*1000)), 0,
                     int(round(points[j, 0]*1000)), int(round(-points[j, 1]*1000)))
                    for i, j in enumerate(order)]
            path = save_scan(rows, args.output / "scans_locaux")
            path.with_suffix(".bin").write_bytes(robot.last_payload)
            # Prédiction = commande calibrée, pas une mesure ; None au premier scan.
            prediction = None if nominal is None else compose_pose(slam.pose, nominal)
            event = slam.process(points, path.name, initial=prediction)
            nominal = None
            event["device_scan"] = stamp
            if hasattr(robot, "show_pose"):
                try:
                    robot.show_pose(slam.pose)  # pose SLAM visible dans RobotControl
                except OSError:
                    pass
            view.update(slam)
            if not args.auto:
                continue
            if stamp["max_valid_angle_gap_deg"] > 15:
                actions.append({"kind": "stop", "reason": "couverture_angulaire_insuffisante"})
                break
            if index == args.steps-1:
                actions.append({"kind": "stop", "reason": "budget_acquisitions_atteint"})
                break
            if len(slam.frames) < 2 and event["accepted"]:
                actions.append({"kind": "hold", "reason": "validation_deux_scans_initiaux"})
                continue
            action = explorer.action(slam.grid, slam.pose, points, event["accepted"])
            actions.append(action)
            if action["kind"] == "stop":
                break
            seconds = (action["distance_m"]/args.speed if action["kind"] == "forward"
                       else action["turn_deg"]/args.turn_rate)
            action["duration_s"] = min(seconds, .25)
            robot.move(action["kind"], min(seconds, .25))
            if action["kind"] == "forward":
                nominal = [action["duration_s"] * args.speed, 0, 0]
            else:
                sign = 1 if action["kind"] == "rotateleft" else -1
                nominal = [0, 0, sign * action["duration_s"] * args.turn_rate]
            time.sleep(.30)  # stop, then acquire a revolution with reduced motion distortion
    finally:
        if args.auto:
            robot.stop()
    return actions


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scans", nargs="*", type=Path)
    parser.add_argument("--mode", choices=["replay", "simulate", "serial", "robot"], default="replay")
    parser.add_argument("--initial-poses", type=Path, help="Estimations antérieures utilisées comme initialisation, pas vérité terrain")
    parser.add_argument("--anchor", type=Path, help="Rejeu : traiter ce scan de référence en premier")
    parser.add_argument("--wide-search", action="store_true", help="Rejeu discontinu : plusieurs orientations ICP")
    parser.add_argument("--port")
    parser.add_argument("--robot-url", default="http://192.168.4.1")
    parser.add_argument("--auto", action="store_true", help="Active explicitement les impulsions moteur en mode robot")
    parser.add_argument("--speed", type=float, help="Vitesse avant calibrée en m/s")
    parser.add_argument("--turn-rate", type=float, help="Vitesse de rotation calibrée en degrés/s actifs")
    parser.add_argument("--robot-radius", type=float, default=.18)
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--resolution", type=float, default=.05)
    parser.add_argument("--min-overlap", type=float, default=.65)
    parser.add_argument("--max-step", type=float)
    parser.add_argument("--no-loop", action="store_true")
    parser.add_argument("--no-vector", action="store_true")
    parser.add_argument("--show", action="store_true", help="Affichage actualisé pendant le traitement")
    parser.add_argument("--output", type=Path, default=Path("resultats/slam_jour3"))
    args = parser.parse_args(argv)
    if args.steps < 1 or args.steps > 10000:
        parser.error("1 <= --steps <= 10000 requis")
    if args.auto and args.mode != "robot":
        parser.error("--auto est réservé au mode robot")
    if args.auto and (args.speed is None or args.turn_rate is None
                     or not np.isfinite([args.speed, args.turn_rate]).all()
                     or not .02 <= args.speed <= 1 or not 1 <= args.turn_rate <= 180):
        parser.error("--auto exige --speed (0,02 à 1 m/s) et --turn-rate (1 à 180 degrés/s) calibrés")
    if args.mode == "serial" and not args.port:
        parser.error("--port requis")
    if args.mode == "replay" and not args.scans:
        args.scans = sorted(Path("donnees/scans").glob("*.csv"))
    if args.mode == "replay" and not args.scans:
        parser.error("Aucun scan CSV à rejouer")
    if args.anchor:
        if args.mode != "replay":
            parser.error("--anchor est réservé au rejeu de fichiers")
        args.scans = [args.anchor] + [p for p in args.scans if p.resolve() != args.anchor.resolve()]
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        config = SLAMConfig(resolution=args.resolution, min_overlap=args.min_overlap,
                            max_step=args.max_step if args.max_step is not None else (15 if args.mode == "replay" else .7),
                            max_rotation=180 if args.mode == "replay" else 35,
                            loop_closure=not args.no_loop)
        slam, view = IncrementalSLAM(config), LiveView(args.show, connect=args.mode != "replay")
        truths, actions, error = None, [], None
        try:
            if args.mode == "replay":
                priors = json.loads(args.initial_poses.read_text()) if args.initial_poses else {}
                replay(slam, args.scans[:args.steps], priors, view, args.wide_search)
            elif args.mode == "simulate":
                truths, actions = simulate(slam, args.steps, view)
            elif args.mode == "serial":
                serial_live(slam, args.port, args.steps, args.output, view)
            else:
                robot_live(slam, args, view, actions)
        except KeyboardInterrupt:
            error = "Interruption utilisateur ; carte partielle conservée."
        except (OSError, ValueError, TimeoutError) as exc:
            error = str(exc)
        finally:
            view.close()
        result = export_results(slam, args.output, args.mode, truths, actions, not args.no_vector)
        result["initialization"] = ("poses déjà estimées ; pas une vérité terrain indépendante"
                                    if args.initial_poses else "premier scan ancré, puis recalage sur le passé uniquement")
        result["anchor"] = str(args.anchor) if args.anchor else None
        result["acquisition_order"] = ("rejeu réordonné avec référence explicite ; pas une trajectoire temporelle mesurée"
                                      if args.anchor else "ordre des fichiers ou ordre de réception")
        result["error"] = error
        (args.output / "bilan.json").write_text(json.dumps(result, indent=2, ensure_ascii=False,
                                                         allow_nan=False)+"\n", encoding="utf-8")
        print(json.dumps({k: v for k, v in result.items() if k not in ("events", "actions")},
                         indent=2, ensure_ascii=False))
        return 1 if error or not slam.frames else 0
    except (ValueError, OSError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
