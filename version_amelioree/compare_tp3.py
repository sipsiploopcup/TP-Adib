"""Compare le SLAM de Codex (slam.py) et la version Claude (slam_carte.py).

A. Trajet imposé en simulation : mêmes scans, même odométrie bruitée pour les deux.
B. Exploration autonome en simulation : chaque SLAM avec son propre explorateur.
C. Rejeu des 12 scans réels : pas de vérité terrain, comparaison aux poses
   de merge_scans (ICP hors ligne), qui ne sont qu'une référence estimée.

Usage : python compare_tp3.py   (depuis le dossier version_amelioree)
"""

import json
import os
import sys
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from comparaison_localisation import SIMULATION_SEGMENTS, simulate_scan  # noqa: E402
from gridmap import load_measurements  # noqa: E402
from localisation_grid import angle_error  # noqa: E402
from slam import IncrementalSLAM, SLAMConfig  # noqa: E402
from slam_carte import MapSLAM, NearestFrontierExplorer, compose  # noqa: E402
from slam_robot import FrontierExplorer  # noqa: E402

# SORTIE_COMPARAISON : dossier de sortie (pour comparer avant / après amélioration).
OUT = Path(os.environ.get("SORTIE_COMPARAISON", Path(__file__).resolve().parent / "resultats" / "tp3_comparaison"))


def rectangle(x0, y0, x1, y1):
    return [[[x0, y0], [x1, y0]], [[x1, y0], [x1, y1]], [[x1, y1], [x0, y1]], [[x0, y1], [x0, y0]]]


DESKS = [(x, y, x + 1.4, y + 0.6) for y in (2.0, 4.0) for x in (1.5, 3.8, 6.1)] + [(4.0, 6.0, 5.2, 6.6)]
ENVIRONMENTS = {
    "salle_codex": {
        "segments": SIMULATION_SEGMENTS, "start": (0.0, 0.0, 0.0), "box": (-4, -3, 6, 5), "solids": [],
        "route": [(5, 0), (5, 4), (-3, 4), (-3, 0), (0, 0)]},
    "salle_de_classe": {
        "segments": np.array(rectangle(0, 0, 9, 7) + sum((rectangle(*d) for d in DESKS), []), dtype=float),
        "start": (0.8, 1.0, 0.0), "box": (0, 0, 9, 7), "solids": DESKS,
        "route": [(8.2, 1.0), (8.2, 5.4), (0.8, 5.4), (0.8, 3.3), (8.2, 3.3)]},
}


# ---------------------------------------------------------------- simulation
def execute(truth, action, rng):
    """Mouvement réel bruité ; renvoie (nouvelle vérité, odométrie nominale)."""
    if action["kind"] == "forward":
        d = action["distance_m"]
        actual = [d * (1 + rng.normal(0, .05)), rng.normal(0, .01) * d / .15, rng.normal(0, .8)]
        nominal = [d, 0, 0]
    else:
        a = action["turn_deg"] * (1 if action["kind"] == "rotateleft" else -1)
        actual, nominal = [0, 0, a * (1 + rng.normal(0, .05)) + rng.normal(0, .5)], [0, 0, a]
    return compose(truth, actual), nominal


def collides(segments, a, b, radius=.17):
    starts, d = segments[:, 0], segments[:, 1] - segments[:, 0]
    for p in np.linspace(a[:2], b[:2], max(2, int(np.linalg.norm(b[:2] - a[:2]) / .02) + 1)):
        t = np.clip(np.sum((p - starts) * d, axis=1) / np.sum(d ** 2, axis=1), 0, 1)
        if np.min(np.linalg.norm(p - starts - t[:, None] * d, axis=1)) < radius:
            return True
    return False


def scan(env, truth, seed):
    return simulate_scan(env["segments"], truth, noise_sigma=.01, seed=seed, max_range=12)


def scripted_actions(truth, route_index, route):
    """Pilote parfait sur la vérité : le trajet est identique pour les deux SLAM."""
    while route_index < len(route):
        target = np.array(route[route_index])
        delta = target - truth[:2]
        if np.linalg.norm(delta) < .08:
            route_index += 1
            continue
        error = angle_error(np.rad2deg(np.arctan2(delta[1], delta[0])), truth[2])
        if abs(error) > 3:
            return {"kind": "rotateleft" if error > 0 else "rotateright",
                    "turn_deg": min(abs(error), 15)}, route_index
        return {"kind": "forward", "distance_m": min(.15, np.linalg.norm(delta))}, route_index
    return None, route_index


def make(name):
    if name == "codex":
        return IncrementalSLAM(SLAMConfig())
    return MapSLAM()


def feed(slam, points, source, prediction):
    t = perf_counter()
    event = slam.process(points, source, initial=prediction)
    return event, perf_counter() - t


# -------------------------------------------------------------------- mesures
def map_metrics(grid, env):
    states = grid.states()
    rows, cols = np.nonzero(states == 1)
    occupied = np.column_stack((cols + .5, rows + .5)) * grid.resolution + grid.origin
    walls = np.vstack([np.linspace(s[0], s[1], max(2, int(np.linalg.norm(s[1] - s[0]) / .04)))
                       for s in env["segments"]])
    x0, y0, x1, y1 = env["box"]
    gx, gy = np.meshgrid(np.arange(x0 + .025, x1, .05), np.arange(y0 + .025, y1, .05))
    cells = np.column_stack((gx.ravel(), gy.ravel()))
    inside = np.ones(len(cells), bool)
    for a, b, c, d in env["solids"]:
        inside &= ~((cells[:, 0] > a) & (cells[:, 0] < c) & (cells[:, 1] > b) & (cells[:, 1] < d))
    cells = cells[inside]
    idx = np.floor(cells / grid.resolution).astype(int) - grid.origin_cell
    h, w = states.shape
    ok = (idx[:, 0] >= 0) & (idx[:, 0] < w) & (idx[:, 1] >= 0) & (idx[:, 1] < h)
    known = np.zeros(len(cells), bool)
    known[ok] = states[idx[ok, 1], idx[ok, 0]] >= 0
    wall_tree = cKDTree(walls)
    return {
        "couverture_surface": float(known.mean()),
        "murs_retrouves_10cm": float(np.mean(cKDTree(occupied).query(walls)[0] <= .10)) if len(occupied) else 0.0,
        "occupe_correct_10cm": float(np.mean(wall_tree.query(occupied)[0] <= .10)) if len(occupied) else 0.0,
    }


def pose_metrics(estimates, truths):
    e = np.array(estimates)
    t = np.array(truths)
    pos = np.linalg.norm(e[:, :2] - t[:, :2], axis=1)
    ang = np.abs([angle_error(a, b) for a, b in zip(e[:, 2], t[:, 2])])
    return {"erreur_pos_moy_m": float(pos.mean()), "erreur_pos_max_m": float(pos.max()),
            "erreur_pos_finale_m": float(pos[-1]), "erreur_angle_moy_deg": float(ang.mean()),
            "erreur_angle_max_deg": float(ang.max())}


# ---------------------------------------------------------------- bancs d'essai
def bench_scripted(env_name, seed):
    env = ENVIRONMENTS[env_name]
    rng = np.random.default_rng(seed)
    truth, route_index = np.array(env["start"], float), 0
    data = [(scan(env, truth, seed * 1000), truth.copy(), None)]
    while len(data) < 600:
        action, route_index = scripted_actions(truth, route_index, env["route"])
        if action is None:
            break
        truth, nominal = execute(truth, action, rng)
        data.append((scan(env, truth, seed * 1000 + len(data)), truth.copy(), nominal))
    results = {}
    for name in ("codex", "claude"):
        slam, estimates, times, accepted = make(name), [], [], 0
        for i, (points, t, nominal) in enumerate(data):
            prediction = np.array(env["start"], float) if nominal is None else compose(slam.pose, nominal)
            event, dt = feed(slam, points, f"s{i}", prediction)
            accepted += event["accepted"]
            estimates.append(np.array(slam.pose))
            times.append(dt)
        odo = [np.array(env["start"], float)]
        for _, _, nominal in data[1:]:
            odo.append(compose(odo[-1], nominal))
        results[name] = {"scans": len(data), "acceptes": accepted, "temps_moy_s": float(np.mean(times)),
                         **pose_metrics(estimates, [d[1] for d in data]), **map_metrics(slam.grid, env)}
        results[name]["_slam"], results[name]["_est"] = slam, np.array(estimates)
    results["odometrie_seule"] = pose_metrics(odo, [d[1] for d in data])
    results["_truth"] = np.array([d[1] for d in data])
    return results


def bench_exploration(env_name, seed, name, max_steps=250):
    env = ENVIRONMENTS[env_name]
    rng = np.random.default_rng(seed)
    slam = make(name)
    explorer = FrontierExplorer() if name == "codex" else NearestFrontierExplorer()
    truth, prediction = np.array(env["start"], float), np.array(env["start"], float)
    estimates, truths, times, distance, reason = [], [], [], 0.0, "max_pas"
    for step in range(max_steps):
        points = scan(env, truth, seed * 1000 + step)
        event, dt = feed(slam, points, f"e{step}", prediction)
        estimates.append(np.array(slam.pose))
        truths.append(truth.copy())
        times.append(dt)
        action = explorer.action(slam.grid, slam.pose, points, event["accepted"])
        if action["kind"] == "stop":
            reason = action["reason"]
            break
        new_truth, nominal = execute(truth, action, rng)
        if collides(env["segments"], truth, new_truth):
            reason = "COLLISION"
            break
        distance += np.linalg.norm(new_truth[:2] - truth[:2])
        truth = new_truth
        prediction = compose(slam.pose, nominal)
    return {"pas": len(estimates), "arret": reason, "distance_parcourue_m": distance,
            "temps_moy_s": float(np.mean(times)), **pose_metrics(estimates, truths),
            **map_metrics(slam.grid, env), "_slam": slam, "_est": np.array(estimates),
            "_truth": np.array(truths)}


def bench_real():
    reference = json.loads((ROOT / "donnees" / "alignements.json").read_text())
    ref_poses = {name: np.array(v["pose"]) for name, v in reference["scans"].items()}
    anchor = reference["reference"]
    paths = sorted((ROOT / "donnees" / "scans").glob("*.csv"))
    paths = [p for p in paths if p.name == anchor] + [p for p in paths if p.name != anchor]
    out = {}
    for label_, make_slam, kwargs in (
            ("codex", lambda: IncrementalSLAM(SLAMConfig(max_step=15, max_rotation=180)), {"wide_search": True}),
            ("claude", MapSLAM, {"wide": True})):
        slam, rows = make_slam(), []
        for p in paths:
            points, _ = load_measurements(p)
            event = slam.process(points, p.name, **kwargs)
            pose = np.array(event["pose"])
            ref = ref_poses.get(p.name)
            rows.append({"scan": p.name, "accepte": bool(event["accepted"]), "raison": event["reason"],
                         "pose": [round(float(v), 3) for v in pose],
                         "ecart_ref_m": None if ref is None or not event["accepted"]
                         else round(float(np.linalg.norm(pose[:2] - ref[:2])), 3),
                         "ecart_ref_deg": None if ref is None or not event["accepted"]
                         else round(abs(angle_error(pose[2], ref[2])), 2)})
        out[label_] = {"acceptes": sum(r["accepte"] for r in rows), "scans": len(rows), "detail": rows,
                       "_slam": slam}
    return out


# ------------------------------------------------------------------ affichage
def draw(ax, slam, title, truth=None, estimates=None):
    from matplotlib.colors import ListedColormap
    ax.imshow(slam.grid.states(), origin="lower", extent=slam.grid.extent,
              cmap=ListedColormap(["#bbc3cd", "#ffffff", "#243a49"]), vmin=-1, vmax=1)
    if truth is not None:
        ax.plot(truth[:, 0], truth[:, 1], "--", color="#e4a664", lw=1.2, label="vérité")
    if estimates is not None:
        ax.plot(estimates[:, 0], estimates[:, 1], "-", color="#008c78", lw=1.2, label="estimée")
        ax.legend(fontsize=7, loc="lower right")
    ax.set_title(title, fontsize=9)
    ax.set_aspect("equal")


def clean(d):
    if isinstance(d, dict):
        return {k: clean(v) for k, v in d.items() if not k.startswith("_")}
    if isinstance(d, list):
        return [clean(v) for v in d]
    if isinstance(d, (np.floating, np.integer)):
        return d.item()
    return d


def mean_of(runs, key):
    return float(np.mean([r[key] for r in runs]))


def main():
    OUT.mkdir(exist_ok=True)
    report = {"A_trajet_impose": {}, "B_exploration": {}, "C_scans_reels": None}
    seeds = (1, 2, 3)
    fig, axes = plt.subplots(2, 4, figsize=(20, 9))
    for row, env in enumerate(ENVIRONMENTS):
        runs = [bench_scripted(env, s) for s in seeds]
        report["A_trajet_impose"][env] = clean(runs)
        last = runs[0]
        draw(axes[row, 0], last["codex"]["_slam"], f"A {env} — Codex", last["_truth"], last["codex"]["_est"])
        draw(axes[row, 1], last["claude"]["_slam"], f"A {env} — Claude", last["_truth"], last["claude"]["_est"])
        expl = {name: [bench_exploration(env, s, name) for s in seeds[:2]] for name in ("codex", "claude")}
        report["B_exploration"][env] = clean(expl)
        for col, name in ((2, "codex"), (3, "claude")):
            r = expl[name][0]
            draw(axes[row, col], r["_slam"], f"B {env} — {name} ({r['pas']} pas, {r['arret']})",
                 r["_truth"], r["_est"])
        print(env, "terminé", flush=True)
    fig.tight_layout()
    fig.savefig(OUT / "comparaison_simulation.png", dpi=130)
    plt.close(fig)

    real = bench_real()
    report["C_scans_reels"] = clean(real)
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    for ax, name in zip(axes, ("codex", "claude")):
        s = real[name]["_slam"]
        draw(ax, s, f"C scans réels — {name} : {real[name]['acceptes']}/{real[name]['scans']} acceptés",
             estimates=np.array([f["pose"] for f in s.frames]))
    fig.tight_layout()
    fig.savefig(OUT / "comparaison_scans_reels.png", dpi=130)
    plt.close(fig)
    (OUT / "comparaison.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")

    # Résumé lisible : moyennes sur les graines
    lines = []
    for env, runs in report["A_trajet_impose"].items():
        for name in ("codex", "claude"):
            r = [x[name] for x in runs]
            lines.append(f"A {env:16s} {name:7s} pos moy {mean_of(r, 'erreur_pos_moy_m'):.3f} m | "
                         f"max {mean_of(r, 'erreur_pos_max_m'):.3f} m | ang moy {mean_of(r, 'erreur_angle_moy_deg'):.2f}° | "
                         f"acceptés {mean_of(r, 'acceptes'):.0f}/{mean_of(r, 'scans'):.0f} | "
                         f"murs {mean_of(r, 'murs_retrouves_10cm'):.2f} | occ ok {mean_of(r, 'occupe_correct_10cm'):.2f} | "
                         f"{mean_of(r, 'temps_moy_s') * 1000:.0f} ms/scan")
        o = [x["odometrie_seule"] for x in runs]
        lines.append(f"A {env:16s} odo     pos moy {mean_of(o, 'erreur_pos_moy_m'):.3f} m | max {mean_of(o, 'erreur_pos_max_m'):.3f} m")
    for env, by_name in report["B_exploration"].items():
        for name, r in by_name.items():
            lines.append(f"B {env:16s} {name:7s} couverture {mean_of(r, 'couverture_surface'):.2f} | "
                         f"pas {mean_of(r, 'pas'):.0f} | dist {mean_of(r, 'distance_parcourue_m'):.1f} m | "
                         f"pos moy {mean_of(r, 'erreur_pos_moy_m'):.3f} m | arrêts {[x['arret'] for x in r]}")
    for name, r in report["C_scans_reels"].items():
        lines.append(f"C scans réels      {name:7s} acceptés {r['acceptes']}/{r['scans']}")
    summary = "\n".join(lines)
    (OUT / "resume.txt").write_text(summary + "\n")
    print(summary)


if __name__ == "__main__":
    main()
