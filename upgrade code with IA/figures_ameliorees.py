"""Figures propres à la version améliorée (rapports TP2 et TP3).

1. Coût de recalage autour de la pose vraie, pour un scan réel partagé en
   référence (4 points sur 5) et test (1 sur 5) : met en évidence l'artefact
   d'échantillonnage du protocole contrôlé du TP2.

Usage : python figures_ameliorees.py --output resultats/figures
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from comparaison_localisation import POSES  # noqa: E402
from geometry import apply_pose  # noqa: E402
from gridmap import load_measurements, scan_to_map  # noqa: E402
from localisation_grid import GridMatcher  # noqa: E402
from localisation_scans import ScanMatcher  # noqa: E402


def sampling_cost_curve(path, output):
    points, _ = load_measurements(path)
    held = np.arange(len(points)) % 5 == 0
    reference, reserved = points[~held], points[held]
    truth = np.asarray(POSES[0], dtype=float)
    query = apply_pose(reserved - truth[:2], theta_deg=-truth[2])
    step = np.median(np.abs(np.diff(np.sort(np.rad2deg(np.arctan2(points[:, 1], points[:, 0]))))))
    offsets = np.linspace(-2, 2, 161)
    poses = np.column_stack((np.full_like(offsets, truth[0]), np.full_like(offsets, truth[1]),
                             truth[2] + offsets))
    curves = {"Grille (champ de distance)": GridMatcher(scan_to_map(reference)),
              "Point-à-point": ScanMatcher(reference, point_to_line=False),
              "Point-à-ligne": ScanMatcher(reference)}
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for (label, matcher), color in zip(curves.items(), ("#5b6b7a", "#d9822b", "#138a72")):
        cost = matcher.costs(query, poses)
        ax.plot(offsets, np.sqrt(cost) * 100, color=color, lw=1.8, label=label)
    for k in (-2, -1, 1, 2):
        ax.axvline(k * step, color="#999", ls=":", lw=1)
    ax.axvline(0, color="#c0392b", lw=1.2, label="pose vraie")
    ax.set(xlabel="Écart angulaire à la pose vraie (°)", ylabel="Coût RMS borné (cm)",
           title=f"{Path(path).stem} : coût autour de la pose vraie (pointillés : pas LiDAR {step:.2f}°)")
    ax.legend(fontsize=8)
    ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)
    return step


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan", type=Path, default=Path("donnees/scan 2.xlsx"))
    parser.add_argument("--output", type=Path, default=Path("resultats/figures"))
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)
    step = sampling_cost_curve(args.scan, args.output / "artefact_echantillonnage.png")
    print(f"Pas angulaire médian : {step:.3f}° ; figure dans {args.output}")


if __name__ == "__main__":
    main()
