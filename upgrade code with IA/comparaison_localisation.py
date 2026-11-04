"""Comparaison commune : grille, segments, scan corrélatif et scan ICP.

Simulation par lancer de rayons depuis des poses distinctes connues, puis
tests contrôlés sur des points réels réservés. Les deux protocoles restent
séparés dans les statistiques. Toutes les méthodes reçoivent les mêmes
points de requête et la même initialisation pour chaque cas.
"""

import argparse
import csv
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from geometry import apply_pose
from gridmap import load_measurements, scan_to_map
from localisation_grid import GridMatcher, angle_error
from localisation_scans import ScanMatcher, ICPMatcher, plot_match
from localisation_vectorielle import VectorMatcher, save_vector_map
from segments import extract_segments
from vectormap import VectorMap


SIMULATION_SEGMENTS = np.array([
    [[-4, -3], [6, -3]], [[6, -3], [6, 5]],
    [[6, 5], [-4, 5]], [[-4, 5], [-4, -3]],
    [[1, -3], [1, -1]], [[2.5, 1.5], [4, 1.5]],
    [[4, 1.5], [4, 3]], [[-2, 2], [-1, 3]],
], dtype=float)
POSES = [(0.7, -0.5, 12), (-0.8, 0.6, -15), (1.1, 0.8, 20)]
INITIAL_OFFSETS = {"proche": (0.1, -0.1, 2), "moyenne": (0.8, -0.6, 15),
                   "difficile": (1.5, -1, 25)}


def cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def simulate_scan(segments, pose, rays=450, noise_sigma=0.005, seed=0, max_range=20):
    """Premier impact de chaque rayon ; distances bruitées, poses exactes par construction."""
    segments = np.asarray(segments, dtype=float)
    pose = np.asarray(pose, dtype=float)
    if (segments.ndim != 3 or segments.shape[1:] != (2, 2)
            or not len(segments) or not np.isfinite(segments).all()
            or pose.shape != (3,) or not np.isfinite(pose).all()
            or rays < 6 or noise_sigma < 0 or max_range <= 0
            or not np.isfinite([noise_sigma, max_range]).all()):
        raise ValueError("Paramètres de simulation invalides")
    local_angles = np.arange(rays) * 2 * np.pi / rays
    world_angles = local_angles + np.deg2rad(pose[2])
    directions = np.column_stack((np.cos(world_angles), np.sin(world_angles)))
    edges = segments[:, 1] - segments[:, 0]
    delta = segments[:, 0] - pose[:2]
    determinant = cross(directions[:, None, :], edges[None, :, :])
    safe = np.where(abs(determinant) > 1e-12, determinant, np.nan)
    distance = cross(delta, edges)[None, :] / safe
    parameter = cross(delta[None, :, :], directions[:, None, :]) / safe
    valid = (np.isfinite(distance) & (distance > 1e-6) & (distance <= max_range)
             & (parameter >= -1e-10) & (parameter <= 1 + 1e-10))
    nearest = np.min(np.where(valid, distance, np.inf), axis=1)
    hit = np.isfinite(nearest)
    noisy = np.maximum(nearest[hit] + np.random.default_rng(seed).normal(0, noise_sigma, hit.sum()), 1e-6)
    return np.column_stack((noisy * np.cos(local_angles[hit]), noisy * np.sin(local_angles[hit])))


def datasets(paths):
    reference = simulate_scan(SIMULATION_SEGMENTS, (0, 0, 0), seed=42)
    queries = [(simulate_scan(SIMULATION_SEGMENTS, pose, seed=100 + i)[::5], np.asarray(pose))
               for i, pose in enumerate(POSES)]
    yield {"name": "simulation", "protocol": "ray_casting_simulation", "reference": reference,
           "queries": queries, "source": "scène asymétrique simulée, graine fixe",
           "uncertainty": "Poses exactes par construction ; bruit radial indépendant sigma=0,005 m."}
    for index, path in enumerate(paths):
        points, info = load_measurements(path)
        held_out = np.arange(len(points)) % 5 == 0
        reference, reserved = points[~held_out], points[held_out]
        if min(len(reference), len(reserved)) < 6:
            raise ValueError("Scan réel trop court pour séparer référence et requête")
        queries = [(apply_pose(reserved - np.asarray(pose[:2]), theta_deg=-pose[2]), np.asarray(pose))
                   for pose in POSES]
        yield {"name": f"scan_{index + 1}", "protocol": "controlled_real_points", "reference": reference,
               "queries": queries, "source": info,
               "uncertainty": "Transformations virtuelles connues ; poses physiques du robot non mesurées, "
                              "incertitudes métrique et angulaire non quantifiées."}


def make_matchers(reference):
    prepared, preparation = {}, {}
    start = perf_counter()
    grid = scan_to_map(reference)
    prepared["grille"] = GridMatcher(grid)
    preparation["grille"] = perf_counter() - start
    start = perf_counter()
    segments = VectorMap().add_segments(extract_segments(reference)).segments
    prepared["vectorielle"] = VectorMatcher(segments)
    preparation["vectorielle"] = perf_counter() - start
    for name, cls in [("scan_recherche", ScanMatcher), ("scan_icp", ICPMatcher)]:
        start = perf_counter()
        prepared[name] = cls(reference)
        preparation[name] = perf_counter() - start
    return prepared, preparation, grid, segments


def compare_case(matchers, query, truth, initial, position_threshold=0.1, angle_threshold=1):
    """Le moteur ne reçoit que la requête et l'initialisation, jamais la pose vraie."""
    rows, outputs = [], {}
    for name, matcher in matchers.items():
        result = matcher.match(query, initial=initial, xy_range=2, angle_range=30)
        translation_error = float(np.linalg.norm(np.asarray(result["pose"][:2]) - truth[:2]))
        rotation_error = abs(angle_error(result["pose"][2], truth[2]))
        rows.append({"method": name, "translation_error_m": translation_error,
                     "angle_error_deg": rotation_error, "runtime_s": result["runtime_s"],
                     "success": translation_error <= position_threshold and rotation_error <= angle_threshold,
                     "overlap_20cm": result["overlap_20cm"], "status": result["status"],
                     "initial_x_m": float(initial[0]), "initial_y_m": float(initial[1]),
                     "initial_theta_deg": float(initial[2]), "true_x_m": float(truth[0]),
                     "true_y_m": float(truth[1]), "true_theta_deg": float(truth[2]),
                     "estimated_x_m": result["pose"][0], "estimated_y_m": result["pose"][1],
                     "estimated_theta_deg": result["pose"][2],
                     "initial_translation_error_m": float(np.linalg.norm(np.array(initial[:2]) - truth[:2])),
                     "initial_angle_error_deg": abs(angle_error(initial[2], truth[2]))})
        outputs[name] = result
    return rows, outputs


def summarize(rows, keys):
    groups = {}
    for row in rows:
        key = tuple(row[k] for k in keys)
        groups.setdefault(key, []).append(row)
    summaries = []
    for key, group in groups.items():
        summaries.append(dict(zip(keys, key), trials=len(group), successes=sum(r["success"] for r in group),
                              success_rate=sum(r["success"] for r in group) / len(group),
                              mean_translation_error_m=float(np.mean([r["translation_error_m"] for r in group])),
                              mean_angle_error_deg=float(np.mean([r["angle_error_deg"] for r in group])),
                              mean_runtime_s=float(np.mean([r["runtime_s"] for r in group]))))
    return summaries


def write_csv(path, rows):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def plot_sensitivity(rows, output):
    import matplotlib.pyplot as plt
    names = list(dict.fromkeys(r["dataset"] for r in rows))
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), squeeze=False)
    aggregated = summarize(rows, ["dataset", "method", "initial_condition"])
    for ax, name in zip(axes.flat, names):
        for method in ["grille", "vectorielle", "scan_recherche", "scan_icp"]:
            selected = [r for r in aggregated if r["dataset"] == name and r["method"] == method]
            ax.plot(range(len(selected)), [max(r["mean_translation_error_m"], 0.001) for r in selected],
                    marker="o", label=method)
        ax.set_xticks(range(3), list(INITIAL_OFFSETS))
        ax.axhline(0.1, color="red", linestyle="--", linewidth=1, label="Seuil position : 10 cm")
        ax.set_yscale("log")
        ax.set(title=name, ylabel="Erreur moyenne de position (m)")
        ax.grid(alpha=0.25)
    for ax in list(axes.flat)[len(names):]:
        ax.set_visible(False)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3)
    fig.suptitle("Sensibilité à l'initialisation — mêmes requêtes et mêmes poses initiales\n"
                 "Simulation et points réels contrôlés ; plancher d'affichage : 1 mm")
    fig.tight_layout(rect=(0, 0.10, 1, 0.93))
    fig.savefig(output, dpi=180)
    plt.close(fig)


def run(paths, output):
    prefix = Path(output)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    rows, metadata = [], []
    for dataset in datasets(paths):
        name = dataset["name"]
        print(f"Comparaison : {name}", flush=True)
        reference, queries = dataset["reference"], dataset["queries"]
        matchers, preparation, grid, segments = make_matchers(reference)
        metadata.append({"name": name, "protocol": dataset["protocol"], "source": dataset["source"],
                         "reference_points": len(reference), "query_points": [len(q) for q, _ in queries],
                         "reference_segments": len(segments), "preparation_time_s": preparation,
                         "reference_uncertainty": dataset["uncertainty"]})
        data_prefix = prefix.parent / (prefix.name + "_" + name)
        np.savez_compressed(str(data_prefix) + "_donnees.npz", reference=reference,
                            truths=np.array([pose for _, pose in queries]),
                            **{f"query_{i}": query for i, (query, _) in enumerate(queries)})
        grid.save(str(data_prefix) + "_grille.npz")
        save_vector_map(str(data_prefix) + "_segments.csv", segments)
        for case, (query, truth) in enumerate(queries, 1):
            for condition, offset in INITIAL_OFFSETS.items():
                initial = truth + offset
                case_rows, outputs = compare_case(matchers, query, truth, initial)
                rows.extend(dict(dataset=name, protocol=dataset["protocol"], case=case,
                                 initial_condition=condition, **row) for row in case_rows)
                if name == "simulation" and case == 1 and condition == "moyenne":
                    plot_match(reference, query, {k: v for k, v in outputs.items() if k.startswith("scan_")},
                               str(prefix) + "_scans_simulation.png", truth,
                               "Localisation entre scans — simulation, pose initiale moyenne")
    summary = summarize(rows, ["dataset", "method"])
    sensitivity = summarize(rows, ["dataset", "method", "initial_condition"])
    report = {"position_success_threshold_m": 0.1, "angle_success_threshold_deg": 1,
              "initial_offsets": INITIAL_OFFSETS, "datasets": metadata,
              "simulation_segments": SIMULATION_SEGMENTS.tolist(), "summary": summary,
              "sensitivity": sensitivity, "trials": rows,
              "limits": "Simulation à poses exactes et bruit radial de 5 mm. Tests réels contrôlés distincts "
                        "d'une validation physique. Chronométrage de recherche, préparation indiquée séparément. "
                        "Recherche bornée ±2 m/±30° ; ICP local non contraint par cette fenêtre, "
                        "avec rejet au-delà de 1,2 m et sélection des 80 % de correspondances les plus proches."}
    prefix.with_suffix(".json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_csv(prefix.with_suffix(".csv"), rows)
    write_csv(str(prefix) + "_resume.csv", summary)
    write_csv(str(prefix) + "_sensibilite.csv", sensitivity)
    markdown = ["# Comparaison des méthodes de localisation", "",
                "Seuils : position ≤ 0,10 m et angle ≤ 1°. Trois poses × trois initialisations par jeu.", "",
                "| Données | Méthode | Erreur position (m) | Erreur angle (°) | Temps (s) | Réussites |",
                "| --- | --- | ---: | ---: | ---: | ---: |"]
    for row in summary:
        markdown.append(f"| {row['dataset']} | {row['method']} | {row['mean_translation_error_m']:.4f} | "
                        f"{row['mean_angle_error_deg']:.3f} | {row['mean_runtime_s']:.3f} | "
                        f"{row['successes']}/{row['trials']} |")
    markdown += ["", "La simulation utilise des poses exactes. Les données réelles utilisent des transformations "
                 "virtuelles sur points réservés ; les poses physiques et leurs incertitudes ne sont pas mesurées."]
    prefix.with_suffix(".md").write_text("\n".join(markdown) + "\n", encoding="utf-8")
    plot_sensitivity(rows, prefix.with_suffix(".png"))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scans", type=Path, nargs="*", default=[])
    parser.add_argument("--output", type=Path, default=Path("resultats/comparaison_localisation"))
    args = parser.parse_args(argv)
    if len(args.scans) > 3:
        parser.error("Trois scans réels uniques au maximum, en plus de la simulation")
    result = run(args.scans, args.output)
    print(f"{len(result['trials'])} essais terminés ; résultats : {args.output}.*")
    return result


if __name__ == "__main__":
    main()
