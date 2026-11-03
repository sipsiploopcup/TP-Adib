"""Recalage réel de toutes les paires de scans fournis, sans vérité terrain inventée."""

import argparse
import csv
from itertools import combinations
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from geometry import apply_pose
from gridmap import load_measurements
from localisation_scans import ScanMatcher, ICPMatcher, plot_match
from merge_scans import register


def match_pairs(paths, output):
    paths = [Path(path).resolve() for path in paths]
    if len(paths) < 2 or len(set(paths)) != len(paths):
        raise ValueError("Au moins deux fichiers de scans distincts sont requis")
    scans = [(path, *load_measurements(path)) for path in paths]
    prefix = Path(output)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    pairs, rows = [], []
    import matplotlib.pyplot as plt
    count = len(paths) * (len(paths) - 1) // 2
    columns = min(count, 3)
    fig, axes = plt.subplots(int(np.ceil(count / columns)), columns,
                             figsize=(6 * columns, 6 * int(np.ceil(count / columns))), squeeze=False)
    for ax, ((ref_path, reference, ref_meta), (query_path, query, query_meta)) in zip(axes.flat, combinations(scans, 2)):
        name = f"{ref_path.stem} / {query_path.stem}"
        print(f"Recalage : {name}", flush=True)
        start = perf_counter()
        # Amorçage géométrique automatique lorsque les poses relatives sont
        # inconnues. Même initialisation pour les deux raffinements suivants.
        ref_far = reference[np.linalg.norm(reference, axis=1) >= 0.6]
        query_far = query[np.linalg.norm(query, axis=1) >= 0.6]
        bootstrap = register(query_far, ref_far)
        bootstrap_time = perf_counter() - start
        results = {"recherche": ScanMatcher(reference).match(query, initial=bootstrap["pose"]),
                   "ICP": ICPMatcher(reference).match(query, initial=bootstrap["pose"])}
        pair = {"reference": ref_meta, "query": query_meta, "bootstrap": bootstrap,
                "bootstrap_time_s": bootstrap_time, "methods": results,
                "physical_ground_truth": False, "translation_error_m": None, "angle_error_deg": None,
                "reference_uncertainty": "Poses réelles non mesurées ; incertitude non quantifiée.",
                "note": "Amorçage ICP multidépart puis même pose initiale pour les deux raffinements. "
                        "Diagnostic visuel, pas comparaison indépendante des algorithmes d'initialisation. "
                        "Proximité physique et cohérence de l'alignement à vérifier."}
        pairs.append(pair)
        index = len(pairs)
        pair_prefix = prefix.parent / (prefix.name + f"_paire_{index}")
        pair_prefix.with_suffix(".json").write_text(json.dumps(pair, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        plot_match(reference, query, results, pair_prefix.with_suffix(".png"),
                   title=f"{query_path.stem} dans le repère de {ref_path.stem}")
        ax.scatter(*reference.T, s=8, color="0.3", label=f"Référence : {ref_path.stem}")
        for method, color in [("recherche", "tab:blue"), ("ICP", "tab:green")]:
            result = results[method]
            ax.scatter(*apply_pose(query, *result["pose"]).T, s=8, color=color, label=f"{query_path.stem} — {method}")
            rows.append({"reference": ref_path.stem, "query": query_path.stem, "method": method,
                         "estimated_x_m": result["pose"][0], "estimated_y_m": result["pose"][1],
                         "estimated_theta_deg": result["pose"][2], "overlap_20cm": result["overlap_20cm"],
                         "bounded_rmse_m": result["rmse_distance_map_m"], "runtime_s": result["runtime_s"],
                         "bootstrap_time_s": bootstrap_time, "bootstrap_ambiguous": bootstrap["ambiguous"],
                         "translation_error_m": None, "angle_error_deg": None})
        suffix = "\nAmorçage ambigu : vérifier visuellement" if bootstrap["ambiguous"] else ""
        ax.set(title=f"{name}{suffix}", xlabel="X (m)", ylabel="Y (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
    for ax in list(axes.flat)[count:]:
        ax.set_visible(False)
    fig.suptitle("Recalage des paires de scans réels — poses estimées")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(prefix.with_suffix(".png"), dpi=180)
    plt.close(fig)
    prefix.with_suffix(".json").write_text(json.dumps({"pairs": pairs, "physical_ground_truth": False},
                                                       ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with prefix.with_suffix(".csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return pairs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scans", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, default=Path("resultats/recalage_scans_reels"))
    args = parser.parse_args(argv)
    pairs = match_pairs(args.scans, args.output)
    print(f"{len(pairs)} paires traitées ; {args.output}.*")
    return pairs


if __name__ == "__main__":
    main()
