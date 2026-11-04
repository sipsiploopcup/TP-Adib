"""Localisation sur des segments finis, par recherche de pose grossière puis fine.

Le moteur de recherche et le critère de recouvrement sont ceux de GridMatcher,
mais le coût est ici une distance géométrique exacte aux segments, sans grille.
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from geometry import apply_pose
from gridmap import load_measurements, load_scan_groups
from localisation_grid import GridMatcher, angle_error
from segments import as_segment, extract_segments
from vectormap import VectorMap


class VectorMatcher(GridMatcher):
    """Réutilise la recherche de poses ; remplace entièrement le calcul des distances."""

    def __init__(self, segments, max_distance=0.6):
        segments = [as_segment(segment).copy() for segment in segments]
        if not segments:
            raise ValueError("La carte doit contenir au moins un segment")
        if not np.isfinite(max_distance) or max_distance <= 0.2:
            raise ValueError("Distance maximale finie et supérieure à 0,2 m requise")
        self.segments = np.array(segments)
        self.max_distance = max_distance
        self.starts = self.segments[:, 0]
        self.directions = self.segments[:, 1] - self.starts
        self.length_squared = np.sum(self.directions ** 2, axis=1)

    def distances(self, points, poses):
        angles = np.deg2rad(poses[:, 2])
        cosine, sine = np.cos(angles)[:, None], np.sin(angles)[:, None]
        x, y = points.T
        world_x = cosine * x - sine * y + poses[:, 0, None]
        world_y = sine * x + cosine * y + poses[:, 1, None]
        world = np.column_stack((world_x.ravel(), world_y.ravel()))
        best = np.full(len(world), self.max_distance ** 2)
        # Limiter la mémoire sans approximation géométrique : toutes les
        # distances aux segments sont examinées, par blocs de huit segments.
        for start in range(0, len(self.segments), 8):
            directions = self.directions[start:start + 8]
            delta = world[:, None, :] - self.starts[None, start:start + 8, :]
            projection = np.sum(delta * directions[None, :, :], axis=2)
            t = np.clip(projection / self.length_squared[None, start:start + 8], 0, 1)
            residual = delta - t[:, :, None] * directions[None, :, :]
            squared = np.sum(residual ** 2, axis=2)
            best = np.minimum(best, squared.min(axis=1))
        return np.sqrt(best).reshape(world_x.shape)


def load_vector_map(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if not {"x1", "y1", "x2", "y2"}.issubset(reader.fieldnames or []):
            raise ValueError("Colonnes x1, y1, x2, y2 requises en mètres")
        segments = [as_segment([[float(row["x1"]), float(row["y1"])],
                                [float(row["x2"]), float(row["y2"])]]) for row in reader]
    if not segments:
        raise ValueError("Carte vectorielle vide")
    return segments


def save_vector_map(path, segments):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["x1", "y1", "x2", "y2"])
        writer.writerows(np.asarray(s).ravel() for s in segments)


def prepare_evaluation(groups):
    """Extraction/consolidation uniquement à partir des 80 % de référence."""
    mapping = VectorMap()
    candidates, queries, metadata = [], [], []
    for points, pose, info in groups:
        held_out = np.arange(len(points)) % 5 == 0
        reference, query = points[~held_out], points[held_out]
        if min(len(reference), len(query)) < 6:
            raise ValueError("Scan trop court pour séparer référence et test")
        extracted = extract_segments(reference)
        candidates.extend(apply_pose(segment, *pose) for segment in extracted)
        queries.append((apply_pose(query, *pose), np.asarray(pose, dtype=float),
                        info.get("scan_source", Path(info.get("source", "scan")).name)))
        metadata.append(dict(info, reference_points=len(reference), reserved_points=len(query),
                             extracted_segments=len(extracted)))
    mapping.add_segments(candidates)
    return mapping.segments, queries, metadata


def plot_match(segments, points, result, path, truth=None, title="Localisation sur carte vectorielle"):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 9))
    for index, segment in enumerate(segments):
        ax.plot(*np.asarray(segment).T, color="0.2", linewidth=2,
                label="Segments de référence" if index == 0 else None)
    ax.scatter(*apply_pose(points, *result["initial_pose"]).T,
               s=13, color="tab:orange", alpha=0.6, label="Scan à la pose initiale")
    ax.scatter(*apply_pose(points, *result["pose"]).T,
               s=13, color="tab:blue", label="Scan à la pose estimée")
    pose = result["pose"]
    ax.plot(*pose[:2], "b+", markersize=12, label="Position estimée")
    angle = np.deg2rad(pose[2])
    ax.arrow(*pose[:2], 0.4 * np.cos(angle), 0.4 * np.sin(angle),
             head_width=0.08, color="tab:blue", length_includes_head=True)
    if truth is not None:
        ax.plot(*truth[:2], "r+", markersize=12, label="Position imposée (test contrôlé)")
    ax.set(xlabel="X (m)", ylabel="Y (m)", title=title)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def evaluate(scan, output, title=None):
    segments, queries, groups = prepare_evaluation(load_scan_groups(scan))
    matcher = VectorMatcher(segments)
    prefix = Path(output)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    offsets = [(0, 0, 0), (0.7, -0.5, 12), (-0.8, 0.6, -15),
               (1.1, 0.8, 20), (-1.2, -0.9, -22)]
    rows = []
    for index, offset in enumerate(offsets):
        world, base_pose, source = queries[index % len(queries)]
        truth = base_pose + offset
        query = apply_pose(world - truth[:2], theta_deg=-truth[2])
        result = matcher.match(query, initial=base_pose)
        expected_distances = matcher.distances(query, truth[None, :])[0]
        overlap = float(np.mean(expected_distances <= 0.2))
        row = {"case": index + 1, "query_source": source,
               "true_x_m": float(truth[0]), "true_y_m": float(truth[1]),
               "true_theta_deg": float(truth[2]),
               "translation_error_m": float(np.linalg.norm(np.array(result["pose"][:2]) - truth[:2])),
               "angle_error_deg": abs(angle_error(result["pose"][2], truth[2])),
               "estimated_x_m": result["pose"][0], "estimated_y_m": result["pose"][1],
               "estimated_theta_deg": result["pose"][2],
               "reference_overlap_20cm": overlap, "reference_coverage_ok": overlap >= 0.5,
               "rmse_at_imposed_pose_m": float(np.sqrt(np.mean(expected_distances ** 2))), **result}
        rows.append(row)
        if index == 1:
            plot_match(segments, query, result, prefix.with_suffix(".png"), truth,
                       f"Localisation vectorielle — {title or Path(scan).stem}, test contrôlé")
    summary = {"method": "Recherche hiérarchique ; distance exacte aux segments finis, bornée à 0,6 m",
               "source": str(Path(scan).resolve()), "groups": groups,
               "reference_segments": len(segments),
               "reference_points": sum(g["reference_points"] for g in groups),
               "reserved_points_total": sum(g["reserved_points"] for g in groups),
               "query_points_per_case": [row["points"] for row in rows],
               "physical_ground_truth": False, "cases": rows,
               "mean_translation_error_m": float(np.mean([r["translation_error_m"] for r in rows])),
               "max_translation_error_m": max(r["translation_error_m"] for r in rows),
               "mean_angle_error_deg": float(np.mean([r["angle_error_deg"] for r in rows])),
               "mean_runtime_s": float(np.mean([r["runtime_s"] for r in rows])),
               "cases_with_reference_coverage": sum(r["reference_coverage_ok"] for r in rows),
               "success_10cm_1deg": sum(r["translation_error_m"] <= 0.1 and r["angle_error_deg"] <= 1
                                         for r in rows),
               "protocol": "80 % référence et 20 % réservés par acquisition ; cinq transformations imposées. "
                           "Une origine par requête ; pose initiale = pose de base, sans perturbation.",
               "limits": "Pas de vérité terrain physique. Poses Fusion estimées. "
                         "La couverture est diagnostiquée, sans exclure les échecs. "
                         "Les objets non représentés par des segments pénalisent le score. "
                         "Temps de préparation de carte exclu du temps de recherche."}
    prefix.with_suffix(".json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    flat = [{k: v for k, v in row.items() if not isinstance(v, (list, dict))} for row in rows]
    with prefix.with_suffix(".csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(flat)
    save_vector_map(prefix.parent / (prefix.name + "_reference.csv"), segments)
    print(json.dumps({k: v for k, v in summary.items() if k not in ("groups", "cases")}, ensure_ascii=False, indent=2))
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    evaluate_parser = commands.add_parser("evaluate")
    evaluate_parser.add_argument("scan", type=Path)
    evaluate_parser.add_argument("--output", type=Path, default=Path("resultats/localisation_vectorielle_scan_1"))
    evaluate_parser.add_argument("--title")
    match_parser = commands.add_parser("match")
    match_parser.add_argument("map", type=Path)
    match_parser.add_argument("scan", type=Path)
    match_parser.add_argument("--initial", type=float, nargs=3, default=(0, 0, 0))
    match_parser.add_argument("--xy-range", type=float, default=2)
    match_parser.add_argument("--angle-range", type=float, default=30)
    match_parser.add_argument("--output", type=Path, default=Path("resultats/localisation_vectorielle"))
    args = parser.parse_args(argv)
    if args.command == "evaluate":
        return evaluate(args.scan, args.output, args.title)
    segments = load_vector_map(args.map)
    points, _ = load_measurements(args.scan)
    result = VectorMatcher(segments).match(points, args.initial, args.xy_range, args.angle_range)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    plot_match(segments, points, result, args.output.with_suffix(".png"))
    print(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    main()
