"""Lecture des x/y embarqués et placement des scans dans un repère commun."""

import csv

import numpy as np


def load_scan(path):
    """Renvoie des points en mètres, X avant, Y gauche.

    La bibliothèque fournie produit y_lib = -r*sin(angle), sans décalage
    de pose dans le programme utilisé. On inverse donc y_lib, sans refaire
    la conversion polaire. Les distances nulles n'indiquent pas un obstacle.
    """
    points = []
    with open(path, newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"dist_mm", "x_lib_mm", "y_lib_mm"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path} : colonnes requises {sorted(required)}")
        for row in reader:
            distance = float(row["dist_mm"])
            x = float(row["x_lib_mm"]) / 1000
            y = -float(row["y_lib_mm"]) / 1000
            if not np.isfinite([distance, x, y]).all() or distance < 0:
                raise ValueError(f"{path} : mesure invalide")
            if distance > 0:
                points.append((x, y))
    if not points:
        raise ValueError(f"{path} : aucun point valide")
    return np.asarray(points)


def apply_pose(points, x=0.0, y=0.0, theta_deg=0.0):
    """Rotation antihoraire en degrés, puis translation en mètres."""
    angle = np.deg2rad(theta_deg)
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]])
    return np.asarray(points) @ rotation.T + np.array([x, y])
