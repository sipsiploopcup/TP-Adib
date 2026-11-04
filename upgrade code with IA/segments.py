"""Extraction de segments ordonnés et géométrie 2D, en mètres.

Split-and-merge : couper aux ruptures de distance, ajuster par ACP,
subdiviser si l'écart maximal dépasse le seuil, puis réunir les voisins
compatibles. Les points isolés ne sont pas transformés en murs.
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from geometry import apply_pose
from gridmap import load_scan_groups

EPS = 1e-9
TOL = 1e-6  # mètres ; égalité des longueurs lors d'un découpage


def as_segment(segment):
    result = np.asarray(segment, dtype=float)
    if result.shape != (2, 2) or not np.isfinite(result).all():
        raise ValueError("Un segment doit contenir deux points 2D finis")
    if np.linalg.norm(result[1] - result[0]) <= EPS:
        raise ValueError("Un segment doit avoir une longueur non nulle")
    return result


def cross(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def angle_between(a, b):
    """Angle non orienté entre deux segments, de 0 à 90 degrés."""
    a, b = as_segment(a), as_segment(b)
    u, v = a[1] - a[0], b[1] - b[0]
    cosine = abs(float(u @ v)) / (np.linalg.norm(u) * np.linalg.norm(v))
    return float(np.rad2deg(np.arccos(np.clip(cosine, 0, 1))))


def intersection(a, b, tol=TOL):
    """Intersection sur les deux segments ; None si parallèles/colinéaires."""
    a, b = as_segment(a), as_segment(b)
    u, v = a[1] - a[0], b[1] - b[0]
    length_u, length_v = np.linalg.norm(u), np.linalg.norm(v)
    determinant = cross(u, v)
    if abs(determinant) <= EPS * length_u * length_v:
        return None
    t = cross(b[0] - a[0], v) / determinant
    s = cross(b[0] - a[0], u) / determinant
    if -tol / length_u <= t <= 1 + tol / length_u and -tol / length_v <= s <= 1 + tol / length_v:
        return a[0] + np.clip(t, 0, 1) * u
    return None


def point_distance(point, segment):
    segment = as_segment(segment)
    direction = segment[1] - segment[0]
    t = np.clip((point - segment[0]) @ direction / (direction @ direction), 0, 1)
    return float(np.linalg.norm(point - segment[0] - t * direction))


def are_close(a, b, angle_deg=10.0, distance=0.15):
    """Proches en angle ET en distance entre les segments finis."""
    a, b = as_segment(a), as_segment(b)
    if angle_between(a, b) > angle_deg:
        return False
    if intersection(a, b) is not None:
        return True
    gap = min(*(point_distance(p, b) for p in a),
              *(point_distance(p, a) for p in b))
    return gap <= distance


def fit_line(points):
    """ACP : extrémités projetées, écarts perpendiculaires, segment ajusté."""
    points = np.asarray(points, dtype=float)
    center = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - center, full_matrices=False)
    direction = vt[0]
    if direction @ (points[-1] - points[0]) < 0:
        direction = -direction
    projection = (points - center) @ direction
    fitted = center + np.outer(projection, direction)
    segment = center + np.outer([projection.min(), projection.max()], direction)
    errors = np.linalg.norm(points - fitted, axis=1)
    return segment, errors


def merge_segments(a, b, angle_deg=10.0, distance=0.15):
    """Fusion ACP des quatre extrémités ; refuse les segments éloignés."""
    a, b = as_segment(a), as_segment(b)
    if not are_close(a, b, angle_deg, distance):
        raise ValueError("Les segments ne sont pas proches")
    return fit_line(np.vstack((a, b)))[0]


def trim_segment(segment, cutter, tol=TOL):
    """Coupe à une intersection intérieure et conserve le plus long côté.

    Égalité à tol près : garder le côté du premier point. Une intersection
    à une extrémité, absente ou colinéaire conserve le segment d'origine.
    """
    segment, cutter = as_segment(segment), as_segment(cutter)
    point = intersection(segment, cutter, tol)
    if point is None:
        return segment.copy()
    left, right = np.linalg.norm(point - segment, axis=1)
    if min(left, right) <= tol:
        return segment.copy()
    if left >= right - tol:
        return np.array([segment[0], point])
    return np.array([point, segment[1]])


def extract_segments(points, max_error=0.05, max_gap=0.5,
                     min_points=6, min_length=0.25, merge_angle=10.0):
    """Scan unique : tri angulaire, ruptures, subdivision ACP et fusion.

    Seuils en mètres, angle en degrés. Une rupture de max_gap interdit
    toute fusion entre les deux groupes, même si leurs lignes sont proches.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("Un tableau N × 2 de points finis est requis")
    if max_error <= 0 or max_gap <= 0 or min_length <= 0 or min_points < 2:
        raise ValueError("Seuils positifs et min_points >= 2 requis")
    if len(points) < min_points:
        return []
    ordered = points[np.argsort(np.arctan2(points[:, 1], points[:, 0]))]
    # Ouvrir le cercle à sa plus grande rupture, plutôt qu'à l'axe -X.
    gaps = np.linalg.norm(np.roll(ordered, -1, axis=0) - ordered, axis=1)
    ordered = np.roll(ordered, -int(np.argmax(gaps)) - 1, axis=0)
    breaks = np.flatnonzero(np.linalg.norm(np.diff(ordered, axis=0), axis=1) > max_gap) + 1
    result = []
    for group in np.split(ordered, breaks):
        leaves, pending = [], [group]
        while pending:
            part = pending.pop()
            if len(part) < min_points:
                continue
            segment, errors = fit_line(part)
            if float(errors.max()) > max_error:
                cut = int(np.argmax(errors))
                if cut in (0, len(part) - 1):
                    cut = len(part) // 2
                # Partager le point de coupure pour conserver les angles.
                pending.extend([part[cut:], part[:cut + 1]])
            else:
                leaves.append(part)
        merged = []
        for part in leaves:
            if merged:
                previous, _ = fit_line(merged[-1])
                current, _ = fit_line(part)
                candidate = np.vstack((merged[-1], part))
                _, errors = fit_line(candidate)
                if (np.linalg.norm(previous[1] - previous[0]) > EPS
                        and np.linalg.norm(current[1] - current[0]) > EPS
                        and are_close(previous, current, merge_angle, max_gap)
                        and errors.max() <= max_error):
                    merged[-1] = candidate
                    continue
            merged.append(part)
        for part in merged:
            segment, _ = fit_line(part)
            if np.linalg.norm(segment[1] - segment[0]) >= min_length:
                result.append(segment)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scan", type=Path)
    parser.add_argument("--output", type=Path, default=Path("resultats/segments_scan_1"))
    parser.add_argument("--title", default="Extraction de segments — Scan 1")
    parser.add_argument("--max-error", type=float, default=0.05)
    parser.add_argument("--max-gap", type=float, default=0.5)
    parser.add_argument("--min-points", type=int, default=6)
    parser.add_argument("--min-length", type=float, default=0.25)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args(argv)
    groups = load_scan_groups(args.scan)
    segments, clouds, group_metadata = [], [], []
    for local_points, pose, info in groups:
        local_segments = extract_segments(local_points, args.max_error, args.max_gap,
                                         args.min_points, args.min_length)
        segments.extend(apply_pose(segment, *pose) for segment in local_segments)
        clouds.append(apply_pose(local_points, *pose))
        group_metadata.append(dict(info, segment_count=len(local_segments)))
    points = np.vstack(clouds)
    metadata = {"source": str(args.scan.resolve()), "valid_points": len(points),
                "groups": group_metadata, "group_count": len(groups),
                "cross_scan_consolidation": False}
    prefix = args.output
    prefix.parent.mkdir(parents=True, exist_ok=True)
    with prefix.with_suffix(".csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["x1", "y1", "x2", "y2"])
        writer.writerows(segment.ravel() for segment in segments)
    metadata.update(segment_count=len(segments), units="m", method="split-and-merge avec ajustement ACP",
                    max_error_m=args.max_error, max_gap_m=args.max_gap,
                    min_points=args.min_points, min_length_m=args.min_length,
                    merge_angle_deg=10.0,
                    limits="Segments candidats ; obstacles, mobilier et murs ne sont pas classifiés. "
                           "Les segments issus de scans différents ne sont pas fusionnés ; "
                           "les poses du classeur fusionné restent estimées.")
    prefix.with_suffix(".json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 9))
    ax.scatter(*points.T, s=9, color="0.6", label=f"Mesures LiDAR ({len(points)})")
    for index, segment in enumerate(segments):
        ax.plot(*segment.T, linewidth=2, label="Segments détectés" if index == 0 else None,
                color="tab:blue")
        if len(segments) <= 40:
            ax.text(*segment.mean(axis=0), str(index + 1), fontsize=8, color="darkred")
    origins = np.array([pose[:2] for _, pose, _ in groups])
    ax.scatter(*origins.T, marker="+", s=120, color="red", label="Origines LiDAR")
    ax.set(xlabel="X (m)", ylabel="Y (m)",
           title=f"{args.title}\n{len(segments)} segments candidats, seuil {args.max_error:g} m")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig(prefix.with_suffix(".png"), dpi=180)
    print(f"{len(points)} points valides → {len(segments)} segments ; exports : {prefix}.*")
    if not args.no_show:
        plt.show()
    plt.close(fig)
    return segments


if __name__ == "__main__":
    main()
