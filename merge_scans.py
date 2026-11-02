"""Aligne les scans réels par ICP et exporte un nuage fusionné avec provenance.

Les poses sont des estimations géométriques, pas des déplacements mesurés.
Une pièce rectangulaire peut donner plusieurs solutions, notamment à 180°.
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from geometry import apply_pose, load_scan


def voxel_sample(points, resolution=0.07):
    """Un point réel par cellule pour limiter le biais lié à la densité."""
    _, indices = np.unique(np.floor(points / resolution).astype(int), axis=0, return_index=True)
    return points[np.sort(indices)]


def rigid_fit(source, target):
    """Transformation rigide sans réflexion, pour des paires correspondantes."""
    center_s, center_t = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - center_s).T @ (target - center_t))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    return rotation, center_t - rotation @ center_s


def alignment_score(source, target, target_tree):
    """Erreur symétrique bornée : les objets mobiles ne dominent pas le score."""
    forward = target_tree.query(source)[0]
    reverse = cKDTree(source).query(target)[0]
    score = float((np.minimum(forward, 0.6) ** 2).mean()
                  + (np.minimum(reverse, 0.6) ** 2).mean()) / 2
    return score, forward


def refine(source, target, rotation, translation, tree, iterations=70):
    for iteration in range(iterations):
        moved = source @ rotation.T + translation
        distances, indices = tree.query(moved)
        gate = max(0.22, 1.2 * (1 - iteration / iterations))
        cutoff = min(gate, float(np.quantile(distances, 0.80)))
        mask = distances <= cutoff
        if mask.sum() < 12:
            break
        delta_r, delta_t = rigid_fit(moved[mask], target[indices[mask]])
        rotation = delta_r @ rotation
        translation = delta_r @ translation + delta_t
        if np.linalg.norm(delta_t) < 1e-5 and abs(delta_r[1, 0]) < 1e-5:
            break
    moved = source @ rotation.T + translation
    score, distances = alignment_score(moved, target, tree)
    angle = float(np.rad2deg(np.arctan2(rotation[1, 0], rotation[0, 0])))
    return {"pose": [float(translation[0]), float(translation[1]), angle],
            "score": score,
            "overlap_20cm": float(np.mean(distances <= 0.20)),
            "median_error_m": float(np.median(distances))}


def register(source, target):
    """Plusieurs rotations/translations initiales, puis ICP robuste.

    Les quantiles proposent des translations lorsque les scans voient des
    portions différentes de la pièce. Aucun mur artificiel n'est ajouté.
    """
    source = voxel_sample(source)
    target = voxel_sample(target)
    if min(len(source), len(target)) < 12:
        raise ValueError("Au moins 12 points distincts nécessaires à l'alignement")
    tree = cKDTree(target)
    quantiles_t = np.quantile(target, [0.05, 0.5, 0.95], axis=0)
    seeds = []
    for angle in range(-180, 180, 15):
        rad = np.deg2rad(angle)
        rotation = np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])
        turned = source @ rotation.T
        quantiles_s = np.quantile(turned, [0.05, 0.5, 0.95], axis=0)
        offsets = quantiles_t - quantiles_s
        local = []
        for x in offsets[:, 0]:
            for y in offsets[:, 1]:
                translation = np.array([x, y])
                score, _ = alignment_score(turned + translation, target, tree)
                local.append((score, rotation, translation))
        score, _ = alignment_score(turned, target, tree)
        local.append((score, rotation, np.zeros(2)))
        seeds.extend(sorted(local, key=lambda item: item[0])[:2])
    solutions = [refine(source, target, r.copy(), t.copy(), tree) for _, r, t in seeds]
    solutions.sort(key=lambda item: item["score"])
    best = solutions[0]
    alternate = None
    for candidate in solutions[1:]:
        delta = (candidate["pose"][2] - best["pose"][2] + 180) % 360 - 180
        shift = np.linalg.norm(np.array(candidate["pose"][:2]) - best["pose"][:2])
        if abs(delta) > 30 or shift > 0.75:
            alternate = candidate
            break
    best["ambiguous"] = bool(alternate and alternate["score"] <= best["score"] * 1.15 + 0.002)
    best["alternate"] = alternate
    best["review_required"] = best["ambiguous"] or best["overlap_20cm"] < 0.5
    return best


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return [row for row in csv.DictReader(stream) if float(row["dist_mm"]) > 0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scans", nargs="+", type=Path)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("resultats"))
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    paths = sorted(set(path.resolve() for path in args.scans))
    reference = args.reference.resolve()
    if reference not in paths:
        parser.error("Le scan de référence doit figurer dans les entrées")
    clouds = {path: load_scan(path) for path in paths}
    # Exclure le voisinage immédiat du robot pour estimer la pose uniquement.
    # Les points exclus de l'alignement restent dans le CSV fusionné.
    fitting = {p: c[np.linalg.norm(c, axis=1) >= 0.6] for p, c in clouds.items()}
    estimates = {}
    estimates[reference] = {"pose": [0.0, 0.0, 0.0], "score": 0.0,
                            "overlap_20cm": 1.0, "median_error_m": 0.0,
                            "ambiguous": False, "alternate": None, "review_required": False}
    for path in paths:
        if path == reference:
            continue
        estimates[path] = register(fitting[path], fitting[reference])
        result = estimates[path]
        print(f"{path.name} : pose={np.round(result['pose'], 3).tolist()}, "
              f"recouvrement={result['overlap_20cm']:.0%}, "
              f"à vérifier={result['review_required']}", flush=True)

    # Les scans riches en mobilier peuvent avoir peu de murs en commun avec
    # la référence. Une deuxième passe les compare à l'union des scans fiables.
    trusted = [p for p in paths if not estimates[p]["review_required"]]
    stable_map = voxel_sample(np.concatenate([
        apply_pose(fitting[p], *estimates[p]["pose"]) for p in trusted]))
    for path in paths:
        if not estimates[path]["review_required"]:
            continue
        candidate = register(fitting[path], stable_map)
        if candidate["overlap_20cm"] > estimates[path]["overlap_20cm"] + 0.10:
            candidate["target"] = "union des scans initialement fiables"
            estimates[path] = candidate
        result = estimates[path]
        print(f"Deuxième passe {path.name} : recouvrement={result['overlap_20cm']:.0%}, "
              f"à vérifier={result['review_required']}", flush=True)

    args.output.mkdir(parents=True, exist_ok=True)
    csv_path = args.output / "chambre_fusionnee.csv"
    columns = ["scan_source", "point_index", "angle_deg_local", "dist_mm",
               "intensity", "x_lib_mm", "y_lib_mm", "x_m", "y_m", "alignement_a_verifier"]
    count = 0
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(columns)
        for path in paths:
            moved = apply_pose(clouds[path], *estimates[path]["pose"])
            for row, (x, y) in zip(read_rows(path), moved):
                writer.writerow([path.name, row["point_index"], row["angle_deg"], row["dist_mm"],
                                 row["intensity"], row["x_lib_mm"], row["y_lib_mm"],
                                 f"{x:.6f}", f"{y:.6f}", int(estimates[path]["review_required"])])
                count += 1
    metadata = {"reference": reference.name, "method": "ICP multidépart, poses estimées",
                "units": "mètres et degrés antihoraires", "point_count": count,
                "warning": "Ambiguïtés possibles dans une pièce rectangulaire ; aucune pose mesurée.",
                "scans": {p.name: estimates[p] for p in paths}}
    (args.output / "alignements.json").write_text(json.dumps(metadata, indent=2) + "\n")
    (args.output / "poses_estimees.json").write_text(
        json.dumps({str(p): estimates[p]["pose"] for p in paths}, indent=2) + "\n")

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 10))
    colors = plt.get_cmap("turbo")(np.linspace(0.04, 0.96, len(paths)))
    for i, path in enumerate(paths):
        points = apply_pose(clouds[path], *estimates[path]["pose"])
        label = f"{path.stem[14:20]} ({len(points)} pts)"
        if estimates[path]["review_required"]:
            label += " *"
        ax.scatter(points[:, 0], points[:, 1], s=3, alpha=0.55, label=label, color=colors[i])
    ax.set_aspect("equal", adjustable="box")
    ax.set(xlabel="X (m)", ylabel="Y (m)",
           title=f"Chambre — {len(paths)} scans réels fusionnés, {count} points\nPoses estimées automatiquement")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
    fig.text(0.08, 0.025, "* Alignement à vérifier : faible recouvrement ou autre pose de score proche.\n"
             "Aucun mur complété artificiellement ; les zones masquées restent inconnues.", fontsize=9)
    fig.tight_layout(rect=[0, 0.06, 1, 1])
    fig.savefig(args.output / "chambre_fusionnee.png", dpi=180)
    print(f"Export : {csv_path} — {count} points issus de {len(paths)} scans", flush=True)
    if args.show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
