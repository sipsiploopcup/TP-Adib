"""Carte vectorielle : extraction locale, placement et consolidation de segments.

Méthode proposée en l'absence du document détaillé de construction du TP.
Les intersections sont recensées ; le découpage au plus long côté est
optionnel car un croisement ne prouve pas qu'une branche est fausse.
"""

import argparse
import csv
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from geometry import apply_pose
from gridmap import load_scan_groups
from segments import (are_close, as_segment, extract_segments, fit_line,
                      intersection, trim_segment)


def canonical(segment):
    """Sens stable pour l'export et les égalités lors des découpages."""
    segment = as_segment(segment).copy()
    if tuple(segment[1]) < tuple(segment[0]):
        segment = segment[::-1].copy()
    return segment


def line_offsets(points, segment):
    direction = segment[1] - segment[0]
    normal = np.array([-direction[1], direction[0]]) / np.linalg.norm(direction)
    return np.abs((points - segment[0]) @ normal)


@dataclass
class VectorMap:
    angle_deg: float = 8.0
    distance: float = 0.08
    gap: float = 0.20
    # Chaque groupe conserve toutes ses extrémités d'origine. Réajuster
    # seulement deux extrémités déjà fusionnées entraînerait une dérive.
    supports: list = field(default_factory=list)

    def __post_init__(self):
        if (not np.isfinite([self.angle_deg, self.distance, self.gap]).all()
                or not 0 <= self.angle_deg < 90 or self.distance <= 0 or self.gap < 0):
            raise ValueError("Angle dans [0, 90[, distance positive et gap positif ou nul requis")

    @property
    def segments(self):
        return [canonical(fit_line(points)[0]) for points in self.supports]

    def add_segments(self, segments):
        """Consolide jusqu'à stabilisation ; chaque fusion réduit le nombre de groupes.

        Tri par longueur puis coordonnées pour fixer l'ordre de cette
        association gloutonne. Le résultat peut dépendre des scans déjà
        incorporés lors de mises à jour successives.
        """
        pending = [np.asarray(points).copy() for points in self.supports]
        pending.extend(canonical(segment) for segment in segments)
        pending.sort(key=lambda p: (-np.linalg.norm(fit_line(p)[0][1] - fit_line(p)[0][0]),
                                    tuple(canonical(fit_line(p)[0]).ravel())))
        changed = True
        while changed:
            changed = False
            for i in range(len(pending)):
                a = canonical(fit_line(pending[i])[0])
                for j in range(i + 1, len(pending)):
                    b = canonical(fit_line(pending[j])[0])
                    if not are_close(a, b, self.angle_deg, self.gap):
                        continue
                    # Deux murs parallèles proches ou deux lignes divergentes
                    # ne doivent pas être fusionnés sur le seul critère d'angle.
                    if (line_offsets(b, a).max() > self.distance
                            or line_offsets(a, b).max() > self.distance):
                        continue
                    together = np.vstack((pending[i], pending[j]))
                    _, errors = fit_line(together)
                    if errors.max() > self.distance:
                        continue
                    pending[i] = together
                    pending.pop(j)
                    changed = True
                    break
                if changed:
                    break
        self.supports = pending
        return self

    def update(self, local_points, pose=(0, 0, 0)):
        """Ajoute un scan dans un repère commun ; pose (m, m, degrés CCW)."""
        if len(pose) != 3 or not np.isfinite(pose).all():
            raise ValueError("Pose finie X, Y, THETA requise")
        segments = extract_segments(local_points)
        self.add_segments(apply_pose(segment, *pose) for segment in segments)
        return len(segments)


def find_intersections(segments):
    result = []
    for i, a in enumerate(segments):
        for j in range(i + 1, len(segments)):
            point = intersection(a, segments[j])
            if point is not None:
                result.append({"segment_a": i + 1, "segment_b": j + 1,
                               "x": float(point[0]), "y": float(point[1])})
    return result


def trim_crossings(segments):
    """Découpe chaque segment par les autres, dans l'ordre donné.

    Les coupeurs sont les segments d'entrée, non leurs versions raccourcies.
    Chaque coupe conserve le plus long côté, selon trim_segment.
    """
    originals = [canonical(segment) for segment in segments]
    result = []
    for i, segment in enumerate(originals):
        trimmed = segment.copy()
        for j, cutter in enumerate(originals):
            if i != j:
                trimmed = trim_segment(trimmed, cutter)
        result.append(trimmed)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scan", type=Path)
    parser.add_argument("--output", type=Path, default=Path("resultats/carte_vectorielle_scan_1"))
    parser.add_argument("--title", default="Carte vectorielle — Scan 1")
    parser.add_argument("--angle", type=float, default=8.0)
    parser.add_argument("--distance", type=float, default=0.08)
    parser.add_argument("--gap", type=float, default=0.20)
    parser.add_argument("--pose", type=float, nargs=3, default=(0, 0, 0))
    parser.add_argument("--trim-crossings", action="store_true")
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args(argv)
    groups = load_scan_groups(args.scan, pose=args.pose)
    vector_map = VectorMap(args.angle, args.distance, args.gap)
    group_metadata, clouds, input_segments = [], [], []
    # Consolider tous les segments en une fois assure un résultat indépendant
    # de l'ordre des acquisitions dans le classeur.
    for points, pose, info in groups:
        extracted = extract_segments(points)
        input_segments.extend(apply_pose(segment, *pose) for segment in extracted)
        clouds.append(apply_pose(points, *pose))
        group_metadata.append(dict(info, segment_count=len(extracted)))
    vector_map.add_segments(input_segments)
    output = vector_map.segments
    if args.trim_crossings:
        output = trim_crossings(output)
    crossings = find_intersections(output)
    prefix = args.output
    prefix.parent.mkdir(parents=True, exist_ok=True)
    with prefix.with_suffix(".csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["x1", "y1", "x2", "y2"])
        writer.writerows(segment.ravel() for segment in output)
    metadata = {"source": str(args.scan.resolve()), "units": "m",
                "groups": group_metadata, "input_segments": len(input_segments),
                "output_segments": len(output), "merge_angle_deg": args.angle,
                "merge_distance_m": args.distance, "merge_gap_m": args.gap,
                "extraction": {"max_error_m": 0.05, "max_gap_m": 0.5,
                               "min_points": 6, "min_length_m": 0.25, "merge_angle_deg": 10},
                "trim_crossings": args.trim_crossings, "intersections": crossings,
                "support_segment_counts": [len(p) // 2 for p in vector_map.supports],
                "method": "association gloutonne et ACP sur toutes les extrémités d'origine",
                "limits": "Méthode proposée sans le document détaillé du TP. Poses fusionnées estimées ; "
                          "obstacles non classifiés. Les lacunes <= merge_gap_m peuvent être reliées."}
    prefix.with_suffix(".json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    import matplotlib.pyplot as plt
    cloud = np.vstack(clouds)
    fig, ax = plt.subplots(figsize=(10, 9))
    ax.scatter(*cloud.T, color="0.75", s=7, label=f"Mesures ({len(cloud)})")
    for i, segment in enumerate(output):
        ax.plot(*segment.T, color="tab:blue", linewidth=2,
                label="Carte vectorielle" if i == 0 else None)
    if crossings:
        xy = np.array([[c["x"], c["y"]] for c in crossings])
        ax.scatter(*xy.T, marker="x", color="darkorange", s=35, label="Intersections")
    origins = np.array([pose[:2] for _, pose, _ in groups])
    ax.scatter(*origins.T, marker="+", s=100, color="red", label="Origines LiDAR")
    ax.set(xlabel="X (m)", ylabel="Y (m)",
           title=f"{args.title}\n{len(input_segments)} segments extraits → {len(output)} consolidés")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig(prefix.with_suffix(".png"), dpi=180)
    print(f"{len(input_segments)} segments → {len(output)} ; {len(crossings)} intersections ; {prefix}.*")
    if not args.no_show:
        plt.show()
    plt.close(fig)
    return output


if __name__ == "__main__":
    main()
