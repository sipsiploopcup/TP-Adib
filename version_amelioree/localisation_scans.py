"""Localisation d'un scan dans le repère d'un autre : recherche corrélative et ICP."""

import argparse
import json
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.spatial import cKDTree

from geometry import apply_pose
from gridmap import load_measurements
from localisation_grid import GridMatcher


def valid_points(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 6 or not np.isfinite(points).all():
        raise ValueError("Au moins six points 2D finis sont requis")
    return points


def local_normals(reference, tree, neighbours=6, radius=0.15, max_flatness=0.15):
    """Normale du mur local par ACP des voisins ; NaN si le voisinage n'est pas rectiligne."""
    distances, indices = tree.query(reference, k=min(neighbours, len(reference)))
    normals = np.full(reference.shape, np.nan)
    for i, (d, idx) in enumerate(zip(distances, indices)):
        group = reference[idx[d <= radius]]
        if len(group) < 3:
            continue
        values, vectors = np.linalg.eigh(np.cov((group - group.mean(axis=0)).T))
        if values[1] > 0 and np.sqrt(values[0] / values[1]) <= max_flatness:
            normals[i] = vectors[:, 0]
    return normals


class ScanMatcher(GridMatcher):
    """Même recherche de poses que les autres références.

    point_to_line=True : distance mesurée le long de la normale du mur local.
    Un LiDAR échantillonne tous les ~0,8° ; avec une distance point-à-point,
    tourner la requête d'un pas d'échantillonnage pose ses points exactement
    sur ceux de la référence et crée un faux minimum. La distance au mur local
    ne dépend plus de l'échantillonnage. Repli point-à-point loin d'un mur
    (> line_radius) ou si le voisinage n'est pas rectiligne.
    """

    def __init__(self, reference, max_distance=0.6, point_to_line=True, line_radius=0.15):
        self.reference = valid_points(reference).copy()
        if not np.isfinite(max_distance) or max_distance <= 0.2:
            raise ValueError("Distance maximale finie et supérieure à 0,2 m requise")
        self.max_distance = max_distance
        self.tree = cKDTree(self.reference)
        self.point_to_line, self.line_radius = point_to_line, line_radius
        self.normals = (local_normals(self.reference, self.tree) if point_to_line
                        else np.full(self.reference.shape, np.nan))

    def nearest(self, world):
        """Distance (point-à-ligne si possible), indice du voisin et normale utilisée."""
        distances, indices = self.tree.query(world)
        normals = self.normals[indices]
        usable = np.isfinite(normals[:, 0]) & (distances <= self.line_radius)
        along = np.abs(np.sum((world - self.reference[indices]) * np.nan_to_num(normals), axis=1))
        return np.where(usable, along, distances), indices, usable

    def distances(self, points, poses):
        angles = np.deg2rad(poses[:, 2])
        cosine, sine = np.cos(angles)[:, None], np.sin(angles)[:, None]
        x, y = points.T
        world_x = cosine * x - sine * y + poses[:, 0, None]
        world_y = sine * x + cosine * y + poses[:, 1, None]
        world = np.column_stack((world_x.ravel(), world_y.ravel()))
        distances = self.nearest(world)[0]
        return np.minimum(distances, self.max_distance).reshape(world_x.shape)


def rigid_fit(source, target):
    """Moindres carrés avec correspondances connues et rotation sans réflexion."""
    center_s, center_t = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - center_s).T @ (target - center_t))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    return rotation, center_t - rotation @ center_s


def point_to_line_step(moved, targets, normals):
    """Pas de Gauss-Newton linéarisé (petit angle) pour sum((n·(R p + t - q))²)."""
    jacobian = np.column_stack((normals[:, 0], normals[:, 1],
                                normals[:, 1] * moved[:, 0] - normals[:, 0] * moved[:, 1]))
    residual = np.sum((moved - targets) * normals, axis=1)
    dx, dy, dtheta = np.linalg.lstsq(jacobian, -residual, rcond=None)[0]
    rotation = np.array([[np.cos(dtheta), -np.sin(dtheta)], [np.sin(dtheta), np.cos(dtheta)]])
    return rotation, np.array([dx, dy])


class ICPMatcher(ScanMatcher):
    """ICP local avec rejet des correspondances les plus éloignées.

    Deux phases : point-à-point jusqu'à convergence (robuste loin de la
    solution), puis point-à-ligne pour la précision finale (insensible au pas
    d'échantillonnage du LiDAR). Le résultat point-à-ligne n'est gardé que
    s'il n'augmente pas l'erreur moyenne au mur local.
    """

    def match(self, points, initial=(0, 0, 0), xy_range=2.0, angle_range=30.0,
              max_iterations=80, correspondence_gate=1.2):
        points = valid_points(points)
        initial = np.asarray(initial, dtype=float)
        if initial.shape != (3,) or not np.isfinite(initial).all():
            raise ValueError("Pose initiale finie X, Y, THETA requise")
        if (not np.isfinite([correspondence_gate, xy_range, angle_range]).all()
                or correspondence_gate <= 0 or xy_range <= 0 or angle_range <= 0
                or not isinstance(max_iterations, int) or max_iterations < 1):
            raise ValueError("Paramètres ICP invalides")
        started = perf_counter()
        theta = np.deg2rad(initial[2])
        rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        translation = initial[:2].copy()
        converged, iterations, correspondences = False, 0, 0
        phase, coarse = 0, None
        for iteration in range(max_iterations + 30):
            iterations = iteration + 1
            moved = points @ rotation.T + translation
            distances, indices, usable = self.nearest(moved)
            cutoff = min(correspondence_gate, float(np.quantile(distances, 0.8)))
            mask = distances <= cutoff + 1e-12
            correspondences = int(mask.sum())
            if correspondences < 6 or np.linalg.norm(moved[mask] - moved[mask].mean(axis=0)) < 1e-9:
                break
            lines = mask & usable
            if phase == 1 and lines.sum() >= 12:
                delta_r, delta_t = point_to_line_step(moved[lines], self.reference[indices[lines]],
                                                      self.normals[indices[lines]])
            else:
                delta_r, delta_t = rigid_fit(moved[mask], self.reference[indices[mask]])
            rotation = delta_r @ rotation
            translation = delta_r @ translation + delta_t
            angle_step = abs(np.rad2deg(np.arctan2(delta_r[1, 0], delta_r[0, 0])))
            if np.linalg.norm(delta_t) < 1e-5 and angle_step < 1e-3:
                if phase == 0 and self.point_to_line and iteration < max_iterations + 29:
                    phase, coarse = 1, (rotation.copy(), translation.copy())
                    continue
                converged = True
                break
            if phase == 0 and iteration >= max_iterations - 1:
                break
        if coarse is not None:
            def score(r, t):
                return np.mean(np.minimum(self.nearest(points @ r.T + t)[0], self.max_distance) ** 2)
            if score(rotation, translation) > score(*coarse) + 1e-12:
                rotation, translation = coarse
        angle = float(np.rad2deg(np.arctan2(rotation[1, 0], rotation[0, 0])))
        pose = [float(translation[0]), float(translation[1]), angle]
        distances = self.distances(points, np.array([pose]))[0]
        overlap = float(np.mean(distances <= 0.2))
        angle_offset = (angle - initial[2] + 180) % 360 - 180
        return {"pose": pose, "initial_pose": initial.tolist(),
                "runtime_s": perf_counter() - started,
                "rmse_distance_map_m": float(np.sqrt(np.mean(distances ** 2))),
                "overlap_20cm": overlap,
                "status": "candidate" if overlap >= 0.5 else "insufficient_overlap",
                "points": len(points), "iterations": iterations,
                "correspondences_last_iteration": correspondences, "converged": converged,
                "outside_comparison_window": bool(np.any(abs(translation - initial[:2]) > xy_range)
                                                  or abs(angle_offset) > angle_range),
                "xy_range_m": xy_range, "angle_range_deg": angle_range}


def plot_match(reference, query, results, output, truth=None, title="Localisation entre scans"):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 9))
    ax.scatter(*reference.T, color="0.3", s=10, label="Scan de référence")
    initial = next(iter(results.values()))["initial_pose"]
    ax.scatter(*apply_pose(query, *initial).T, s=10, color="tab:orange", alpha=0.5, label="Pose initiale")
    for index, (name, result) in enumerate(results.items()):
        color = ["tab:blue", "tab:green", "tab:purple"][index % 3]
        ax.scatter(*apply_pose(query, *result["pose"]).T, s=10, color=color, label=name)
        ax.plot(*result["pose"][:2], "+", markersize=12, color=color)
    if truth is not None:
        ax.plot(*truth[:2], "rx", markersize=12, label="Pose connue en simulation")
    ax.set(xlabel="X (m)", ylabel="Y (m)", title=title)
    ax.set_aspect("equal", adjustable="box")
    ax.legend()
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("query", type=Path)
    parser.add_argument("--method", choices=["search", "icp", "both"], default="both")
    parser.add_argument("--initial", type=float, nargs=3, default=(0, 0, 0))
    parser.add_argument("--xy-range", type=float, default=2)
    parser.add_argument("--angle-range", type=float, default=30)
    parser.add_argument("--output", type=Path, default=Path("resultats/localisation_scans"))
    args = parser.parse_args(argv)
    reference, reference_meta = load_measurements(args.reference)
    query, query_meta = load_measurements(args.query)
    results = {}
    methods = {"search": ScanMatcher, "icp": ICPMatcher}
    for name, cls in methods.items():
        if args.method in (name, "both"):
            results[name] = cls(reference).match(query, args.initial, args.xy_range, args.angle_range)
    metadata = {"reference": reference_meta, "query": query_meta, "methods": results,
                "physical_ground_truth": False, "translation_error_m": None, "angle_error_deg": None,
                "reference_uncertainty": "Poses non mesurées ; incertitude métrique et angulaire non quantifiée.",
                "note": "Transformation requête vers référence. Proximité réelle des acquisitions à confirmer. "
                        "Le résidu de recalage n'est pas une erreur de pose mesurée."}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    plot_match(reference, query, results, args.output.with_suffix(".png"))
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    return metadata


if __name__ == "__main__":
    main()
