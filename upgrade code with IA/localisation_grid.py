"""Scan matching sur carte d'occupation : recherche de pose grossière puis fine.

Le champ de distance aux cellules occupées sert de fonction de coût.
La recherche est bornée autour d'une pose initiale ; elle n'est pas une
garantie de localisation globale dans une scène symétrique.
"""

import argparse
import csv
import json
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates

from geometry import apply_pose
from gridmap import OccupancyGrid, load_measurements, load_scan_groups, update_map


def angle_error(a, b):
    """Écart angulaire signé minimal, en degrés."""
    return float((a - b + 180) % 360 - 180)


class GridMatcher:
    def __init__(self, grid, max_distance=0.6):
        if not np.isfinite(max_distance) or max_distance <= 0.2:
            raise ValueError("Distance maximale finie et supérieure au seuil de recouvrement de 0,2 m requise")
        occupied = grid.states() == 1
        if not occupied.any():
            raise ValueError("La carte ne contient aucune cellule occupée")
        self.grid = grid
        self.max_distance = max_distance
        self.field = np.minimum(distance_transform_edt(~occupied) * grid.resolution,
                                max_distance)

    def distances(self, points, poses):
        """Distances bilinéaires, une ligne par pose ; hors carte = pénalité maximale."""
        radians = np.deg2rad(poses[:, 2])
        cosine, sine = np.cos(radians)[:, None], np.sin(radians)[:, None]
        x, y = points.T
        world_x = cosine * x - sine * y + poses[:, 0, None]
        world_y = sine * x + cosine * y + poses[:, 1, None]
        # Le champ représente les centres des cellules, d'où le demi-pas.
        columns = (world_x - self.grid.origin[0]) / self.grid.resolution - 0.5
        rows = (world_y - self.grid.origin[1]) / self.grid.resolution - 0.5
        shape = rows.shape
        return map_coordinates(self.field, [rows.ravel(), columns.ravel()],
                               order=1, mode="constant", cval=self.max_distance).reshape(shape)

    def costs(self, points, poses):
        costs = []
        for start in range(0, len(poses), 256):
            distances = self.distances(points, poses[start:start + 256])
            costs.extend(np.mean(distances ** 2, axis=1))
        return np.asarray(costs)

    def match(self, points, initial=(0, 0, 0), xy_range=2.0, angle_range=30.0,
              beam_width=8):
        points = np.asarray(points, dtype=float)
        initial = np.asarray(initial, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 6
                or not np.isfinite(points).all()):
            raise ValueError("Au moins six points 2D finis sont requis")
        if initial.shape != (3,) or not np.isfinite(initial).all():
            raise ValueError("Pose initiale finie X, Y, THETA requise")
        if (not np.isfinite([xy_range, angle_range]).all()
                or not 0 < xy_range <= 10 or not 0 < angle_range <= 180
                or not 1 <= beam_width <= 32):
            raise ValueError("Fenêtre : 0 < XY <= 10 m, 0 < angle <= 180°, 1 <= faisceau <= 32")
        started = perf_counter()
        lower = initial - [xy_range, xy_range, angle_range]
        upper = initial + [xy_range, xy_range, angle_range]
        centers = initial[None, :]
        evaluated = 0
        stages = [(xy_range, angle_range, 0.25, 5.0),
                  (0.25, 5.0, 0.05, 1.0), (0.05, 1.0, 0.01, 0.2)]
        for xy_radius, angular_radius, xy_step, angular_step in stages:
            offsets = np.stack(np.meshgrid(
                np.linspace(-xy_radius, xy_radius, int(np.ceil(2 * xy_radius / xy_step)) + 1),
                np.linspace(-xy_radius, xy_radius, int(np.ceil(2 * xy_radius / xy_step)) + 1),
                np.linspace(-angular_radius, angular_radius,
                            int(np.ceil(2 * angular_radius / angular_step)) + 1),
                indexing="ij"), axis=-1).reshape(-1, 3)
            poses = (centers[:, None, :] + offsets).reshape(-1, 3)
            poses = np.unique(np.round(np.clip(poses, lower, upper), 8), axis=0)
            costs = self.costs(points, poses)
            evaluated += len(poses)
            ranked = np.argsort(costs, kind="stable")
            # Conserver plusieurs bassins de recherche et pas huit voisins
            # quasi identiques du même minimum grossier.
            selected = []
            for index in ranked:
                pose = poses[index]
                if all(np.linalg.norm(pose[:2] - p[:2]) >= xy_step * 1.5
                       or abs(pose[2] - p[2]) >= angular_step * 1.5 for p in selected):
                    selected.append(pose)
                if len(selected) >= beam_width:
                    break
            centers = np.array(selected)
        best = centers[0].copy()
        distances = self.distances(points, best[None, :])[0]
        best[2] = (best[2] + 180) % 360 - 180
        overlap = float(np.mean(distances <= 0.2))
        return {"pose": best.tolist(), "rmse_distance_map_m": float(np.sqrt(np.mean(distances ** 2))),
                "overlap_20cm": overlap,
                "status": "candidate" if overlap >= 0.5 else "insufficient_overlap",
                "points": len(points), "evaluated_poses": evaluated,
                "runtime_s": perf_counter() - started,
                "on_search_boundary": bool(np.any(np.isclose(centers[0], lower, atol=0.011))
                                           or np.any(np.isclose(centers[0], upper, atol=0.011))),
                "initial_pose": initial.tolist(), "xy_range_m": xy_range,
                "angle_range_deg": angle_range}


def plot_match(grid, points, result, output, truth=None, title="Localisation sur grid map"):
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    fig, ax = plt.subplots(figsize=(10, 9))
    ax.imshow(grid.states(), origin="lower", extent=grid.extent,
              cmap=ListedColormap(["#b8bec8", "white", "#252a34"]), vmin=-1, vmax=1)
    initial = apply_pose(points, *result["initial_pose"])
    matched = apply_pose(points, *result["pose"])
    ax.scatter(*initial.T, color="tab:orange", s=12, alpha=0.6, label="Scan à la pose initiale")
    ax.scatter(*matched.T, color="tab:blue", s=12, label="Scan à la pose estimée")
    pose = result["pose"]
    ax.plot(*pose[:2], "b+", markersize=12, label="Position estimée")
    if truth is not None:
        ax.plot(*truth[:2], "r+", markersize=12, label="Position imposée (test contrôlé)")
    ax.set(xlabel="X (m)", ylabel="Y (m)", title=title)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def prepare_evaluation(groups):
    """Sépare les points dans chaque origine et ne cartographie que les références."""
    grid = OccupancyGrid()
    queries, group_info = [], []
    for points, pose, metadata in groups:
        held_out = np.arange(len(points)) % 5 == 0
        reference, query = points[~held_out], points[held_out]
        if min(len(reference), len(query)) < 6:
            raise ValueError("Scan trop court pour une séparation carte/test")
        update_map(grid, reference, pose)
        queries.append((apply_pose(query, *pose), np.asarray(pose, dtype=float),
                        metadata.get("scan_source", Path(metadata.get("source", "scan")).name)))
        group_info.append(dict(metadata, reference_points=len(reference), reserved_points=len(query)))
    if not queries:
        raise ValueError("Au moins une acquisition est requise")
    return grid, queries, group_info


def evaluate(scan, output, title=None):
    """Test contrôlé sur points réels réservés, pas une validation physique.

    80 % des points créent la carte ; les 20 % restants sont exprimés dans
    des repères virtuels connus. Aucun de ces points de test n'est intégré
    comme impact dans la carte de référence.
    """
    groups = load_scan_groups(scan)
    grid, queries, group_info = prepare_evaluation(groups)
    matcher = GridMatcher(grid)
    offsets = [(0, 0, 0), (0.7, -0.5, 12), (-0.8, 0.6, -15),
               (1.1, 0.8, 20), (-1.2, -0.9, -22)]
    rows = []
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    for index, offset in enumerate(offsets):
        # Une seule origine par requête, même pour le classeur Fusion.
        query_world, base_pose, query_source = queries[index % len(queries)]
        truth = base_pose + offset
        local_query = apply_pose(query_world - truth[:2], theta_deg=-truth[2])
        result = matcher.match(local_query, initial=base_pose)
        # Diagnostic réservé à l'évaluation : on ne donne pas cette pose
        # attendue au moteur de recherche. Il permet de vérifier la condition
        # de couverture demandée dans l'énoncé, sans masquer les échecs.
        imposed_distances = matcher.distances(local_query, truth[None, :])[0]
        reference_overlap = float(np.mean(imposed_distances <= 0.2))
        row = {"case": index + 1, "query_source": query_source,
               "true_x_m": truth[0], "true_y_m": truth[1],
               "true_theta_deg": truth[2], "estimated_x_m": result["pose"][0],
               "estimated_y_m": result["pose"][1], "estimated_theta_deg": result["pose"][2],
               "translation_error_m": float(np.linalg.norm(np.array(result["pose"][:2]) - truth[:2])),
               "angle_error_deg": abs(angle_error(result["pose"][2], truth[2])), **result}
        row.update(reference_overlap_20cm=reference_overlap,
                   reference_coverage_ok=reference_overlap >= 0.5,
                   rmse_at_imposed_pose_m=float(np.sqrt(np.mean(imposed_distances ** 2))))
        rows.append(row)
        if index == 1:
            plot_match(grid, local_query, result, output.with_suffix(".png"), truth,
                       f"Localisation — {title or Path(scan).stem}, test contrôlé (points réservés)")
    summary = {"protocol": "Points réels : 80 % carte, 20 % réservés par acquisition, repères virtuels imposés. "
                           "Une acquisition par requête ; pose initiale = pose de base du repère fourni.",
               "source": str(Path(scan).resolve()), "groups": group_info,
               "reference_points": sum(g["reference_points"] for g in group_info),
               "reserved_points_total": sum(g["reserved_points"] for g in group_info),
               "query_points_per_case": [r["points"] for r in rows],
               "query_points": len(queries[0][0]) if len(queries) == 1 else None,
               "cases_with_reference_coverage": sum(r["reference_coverage_ok"] for r in rows),
               "physical_ground_truth": False, "cases": rows,
               "mean_translation_error_m": float(np.mean([r["translation_error_m"] for r in rows])),
               "max_translation_error_m": max(r["translation_error_m"] for r in rows),
               "mean_angle_error_deg": float(np.mean([r["angle_error_deg"] for r in rows])),
               "max_angle_error_deg": max(r["angle_error_deg"] for r in rows),
               "mean_runtime_s": float(np.mean([r["runtime_s"] for r in rows])),
               "success_10cm_1deg": sum(r["translation_error_m"] <= 0.1 and r["angle_error_deg"] <= 1
                                         for r in rows),
               "limits": "Test de récupération géométrique sur des acquisitions partagées en deux. "
                         "Pas de mesure de précision physique ni cinq acquisitions indépendantes pour un scan seul. "
                         "Pour Fusion : référence et poses de base issues du recalage estimé ; "
                         "seules les transformations imposées sont contrôlées. "
                         "Le score utilise les impacts, sans modèle complet des espaces libres."}
    output.with_suffix(".json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    flat_rows = [{k: v for k, v in row.items() if not isinstance(v, (list, dict))} for row in rows]
    with output.with_suffix(".csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(flat_rows)
    grid.save(output.parent / (output.name + "_reference.npz"))
    print(json.dumps({k: v for k, v in summary.items() if k not in ("cases", "source", "groups")}, ensure_ascii=False, indent=2))
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    test = commands.add_parser("evaluate", help="Évaluation contrôlée, sans pose physique mesurée")
    test.add_argument("scan", type=Path)
    test.add_argument("--output", type=Path, default=Path("resultats/localisation_grid_scan_1"))
    test.add_argument("--title", help="Nom de l'acquisition affiché sur l'image")
    match = commands.add_parser("match", help="Localiser un scan dans une grille NPZ existante")
    match.add_argument("map", type=Path)
    match.add_argument("scan", type=Path)
    match.add_argument("--initial", type=float, nargs=3, default=(0, 0, 0))
    match.add_argument("--xy-range", type=float, default=2.0)
    match.add_argument("--angle-range", type=float, default=30.0)
    match.add_argument("--output", type=Path, default=Path("resultats/localisation_grid"))
    args = parser.parse_args(argv)
    if args.command == "evaluate":
        return evaluate(args.scan, args.output, args.title)
    grid = OccupancyGrid.load(args.map)
    points, _ = load_measurements(args.scan)
    result = GridMatcher(grid).match(points, args.initial, args.xy_range, args.angle_range)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    plot_match(grid, points, result, args.output.with_suffix(".png"))
    print(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    main()
