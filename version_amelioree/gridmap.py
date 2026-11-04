"""Carte d'occupation à partir d'un scan Excel/CSV et de sa pose.

Les rayons parcourent les cellules avec Bresenham. Une cellule reçoit au
plus une observation libre ou occupée par scan ; les impacts sont prioritaires.
Les observations successives sont accumulées en log-odds bornés.
"""

import argparse
import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from geometry import apply_pose


def read_input_rows(path, sheet=None):
    """Repère les colonnes utiles sans modifier le classeur d'origine."""
    path = Path(path)
    required = {"dist_mm", "x_lib_mm", "y_lib_mm"}
    workbook = None
    sheet_name = None
    try:
        if path.suffix.lower() == ".xlsx":
            from openpyxl import load_workbook

            workbook = load_workbook(path, read_only=True, data_only=True)
            candidates = [workbook[sheet]] if sheet else workbook.worksheets
            rows = None
            for candidate in candidates:
                iterator = candidate.iter_rows(values_only=True)
                for _ in range(10):
                    header = next(iterator, None)
                    if header is None:
                        break
                    names = [str(value).strip() if value is not None else "" for value in header]
                    if required.issubset(names):
                        rows = [dict(zip(names, row)) for row in iterator]
                        sheet_name = candidate.title
                        break
                if rows is not None:
                    break
            if rows is None:
                raise ValueError("Aucune feuille ne contient dist_mm, x_lib_mm et y_lib_mm")
        elif path.suffix.lower() == ".csv":
            with path.open(newline="", encoding="utf-8-sig") as stream:
                reader = csv.DictReader(stream)
                if not required.issubset(reader.fieldnames or []):
                    raise ValueError("Colonnes dist_mm, x_lib_mm et y_lib_mm requises")
                rows = list(reader)
        else:
            raise ValueError("Utiliser un fichier .xlsx ou .csv")

        rows = [row for row in rows if not all(row.get(k) in (None, "") for k in required)]
        return rows, sheet_name
    finally:
        if workbook is not None:
            workbook.close()


def convert_rows(rows, path, sheet_name):
    points, valid_rows = [], []
    zero_count = 0
    for number, row in enumerate(rows, 2):
        try:
            distance, x, y = (float(row[name]) for name in ("dist_mm", "x_lib_mm", "y_lib_mm"))
        except (TypeError, ValueError) as error:
            raise ValueError(f"Mesure incomplète ou non numérique à la ligne {number}") from error
        if not np.isfinite([distance, x, y]).all() or distance < 0:
            raise ValueError(f"Mesure invalide à la ligne {number}")
        if distance == 0:
            zero_count += 1
            continue
        points.append((x / 1000, -y / 1000))
        valid_rows.append(row)
    if not points:
        raise ValueError("Aucun impact valide dans le fichier")
    metadata = {"source": str(Path(path).resolve()), "sheet": sheet_name,
                "input_points": len(rows), "valid_points": len(points),
                "zero_distance_ignored": zero_count}
    return np.asarray(points), valid_rows, metadata


def load_measurements(path, sheet=None):
    """Lit un scan local unique, en mètres et avec Y inversé."""
    rows, sheet_name = read_input_rows(path, sheet)
    if any("scan_source" in row for row in rows):
        raise ValueError("Utiliser load_scan_groups pour conserver les origines d'un fichier fusionné")
    points, _, metadata = convert_rows(rows, path, sheet_name)
    return points, metadata


def recover_pose(local, world):
    """Retrouve la transformation déjà appliquée dans le classeur fusionné.

    Ce n'est pas un nouveau recalage : les paires local/global sont connues.
    Vérifier les résidus évite d'utiliser des origines incohérentes.
    """
    if len(local) < 2 or not np.isfinite(world).all():
        raise ValueError("Au moins deux points locaux/globaux finis sont nécessaires")
    center_l, center_w = local.mean(axis=0), world.mean(axis=0)
    if np.linalg.norm(local - center_l) < 1e-6:
        raise ValueError("Les points ne permettent pas de retrouver l'orientation du scan")
    u, _, vt = np.linalg.svd((local - center_l).T @ (world - center_w))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = center_w - rotation @ center_l
    residual = float(np.max(np.linalg.norm(local @ rotation.T + translation - world, axis=1)))
    if residual > 0.002:
        raise ValueError(f"Coordonnées locales/globales incohérentes : résidu {residual:g} m")
    pose = [float(translation[0]), float(translation[1]),
            float(np.rad2deg(np.arctan2(rotation[1, 0], rotation[0, 0])))]
    return pose, residual


def load_scan_groups(path, pose=(0, 0, 0), sheet=None):
    """Un groupe par origine LiDAR, même pour un Excel contenant une fusion."""
    rows, sheet_name = read_input_rows(path, sheet)
    if not any("scan_source" in row for row in rows):
        points, _, metadata = convert_rows(rows, path, sheet_name)
        metadata["pose"] = list(pose)
        metadata["pose_method"] = "pose fournie ou origine conventionnelle du scan unique"
        return [(points, list(pose), metadata)]
    if list(pose) != [0, 0, 0]:
        raise ValueError("Le classeur fusionné fournit déjà les poses ; ne pas ajouter --pose")
    grouped = {}
    for row in rows:
        name = row.get("scan_source")
        if not name:
            raise ValueError("Le classeur fusionné contient une mesure sans scan_source")
        grouped.setdefault(str(name), []).append(row)
    groups = []
    for name, source_rows in grouped.items():
        points, valid_rows, metadata = convert_rows(source_rows, path, sheet_name)
        try:
            world = np.array([[float(row["x_m"]), float(row["y_m"])] for row in valid_rows])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("La fusion doit conserver scan_source, x_m et y_m pour retrouver les origines") from error
        recovered, residual = recover_pose(points, world)
        metadata.update(scan_source=name, pose=recovered, pose_is_estimated=True,
                        pose_method="transformation locale/globale du classeur fusionné",
                        pose_recovery_max_error_m=residual)
        groups.append((points, recovered, metadata))
    return groups


def bresenham(start, end):
    """Cellules (ligne, colonne) du départ à l'impact, deux extrémités incluses."""
    y, x = start
    y_end, x_end = end
    dx, dy = abs(x_end - x), -abs(y_end - y)
    sx, sy = (1 if x < x_end else -1), (1 if y < y_end else -1)
    error = dx + dy
    while True:
        yield y, x
        if x == x_end and y == y_end:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x += sx
        if twice <= dx:
            error += dx
            y += sy


def trace_rays(start, ends):
    """Bresenham exécuté sur tous les rayons à la fois (mêmes cellules que bresenham).

    Renvoie (cellules traversées, cellules d'impact) en (ligne, colonne).
    La boucle porte sur la longueur du plus long rayon, pas sur les rayons :
    même arithmétique entière que bresenham(), appliquée en vecteurs.
    """
    ends = np.asarray(ends, dtype=np.int64).reshape(-1, 2)
    y = np.full(len(ends), start[0], dtype=np.int64)
    x = np.full(len(ends), start[1], dtype=np.int64)
    y_end, x_end = ends[:, 0], ends[:, 1]
    dx, dy = np.abs(x_end - x), -np.abs(y_end - y)
    sx, sy = np.where(x < x_end, 1, -1), np.where(y < y_end, 1, -1)
    error = dx + dy
    active = (x != x_end) | (y != y_end)
    visited = [np.empty((0, 2), dtype=np.int64)]
    while active.any():
        i = np.nonzero(active)[0]
        visited.append(np.column_stack((y[i], x[i])))
        twice = 2 * error[i]
        step_x, step_y = twice >= dy[i], twice <= dx[i]
        error[i] += np.where(step_x, dy[i], 0) + np.where(step_y, dx[i], 0)
        x[i] += np.where(step_x, sx[i], 0)
        y[i] += np.where(step_y, sy[i], 0)
        active[i] = (x[i] != x_end[i]) | (y[i] != y_end[i])
    return np.vstack(visited), ends


@dataclass
class OccupancyGrid:
    resolution: float = 0.05
    origin_cell: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.int64))
    log_odds: np.ndarray = field(default_factory=lambda: np.zeros((0, 0), dtype=np.float32))
    observed: np.ndarray = field(default_factory=lambda: np.zeros((0, 0), dtype=bool))
    history: list = field(default_factory=list)

    def __post_init__(self):
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("La résolution doit être un nombre positif fini")

    @property
    def origin(self):
        return self.origin_cell * self.resolution

    @property
    def extent(self):
        x, y = self.origin
        height, width = self.log_odds.shape
        return [float(x), float(x + width * self.resolution),
                float(y), float(y + height * self.resolution)]

    def cell(self, point):
        x, y = np.floor(np.asarray(point) / self.resolution).astype(np.int64) - self.origin_cell
        return int(y), int(x)

    def ensure_bounds(self, points):
        cells = np.floor(points / self.resolution).astype(np.int64)
        low, high = cells.min(axis=0) - 2, cells.max(axis=0) + 3
        if self.log_odds.size:
            low = np.minimum(low, self.origin_cell)
            high = np.maximum(high, self.origin_cell + np.array(self.log_odds.shape[::-1]))
        width, height = map(int, high - low)
        if width * height > 4_000_000:
            raise ValueError("Grille trop grande : augmenter la résolution ou vérifier les unités")
        if self.log_odds.size and np.array_equal(low, self.origin_cell) and (height, width) == self.log_odds.shape:
            return
        values = np.zeros((height, width), dtype=np.float32)
        observed = np.zeros((height, width), dtype=bool)
        if self.log_odds.size:
            x, y = self.origin_cell - low
            h, w = self.log_odds.shape
            values[y:y+h, x:x+w] = self.log_odds
            observed[y:y+h, x:x+w] = self.observed
        self.origin_cell = low
        self.log_odds, self.observed = values, observed

    def states(self):
        """-1 inconnu/incertain, 0 libre (p<=0.45), 1 occupé (p>=0.65)."""
        result = np.full(self.log_odds.shape, -1, dtype=np.int8)
        result[self.observed & (self.log_odds <= math.log(0.45 / 0.55))] = 0
        result[self.observed & (self.log_odds >= math.log(0.65 / 0.35))] = 1
        return result

    def save(self, path):
        np.savez_compressed(path, resolution=self.resolution, origin_cell=self.origin_cell,
                            log_odds=self.log_odds, observed=self.observed,
                            history=np.array(json.dumps(self.history)))

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            grid = cls(float(data["resolution"]))
            grid.origin_cell = data["origin_cell"].copy()
            grid.log_odds = data["log_odds"].copy()
            grid.observed = data["observed"].copy()
            grid.history = json.loads(str(data["history"]))
        return grid


def update_map(grid, local_points, pose=(0, 0, 0)):
    """Met à jour une carte existante et l'agrandit sans perdre ses observations."""
    points = np.asarray(local_points, dtype=float)
    pose = np.asarray(pose, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points) or not np.isfinite(points).all():
        raise ValueError("Le scan doit contenir des points finis de forme (N, 2)")
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError("La pose doit contenir x, y, theta finis")
    world = apply_pose(points, *pose)
    grid.ensure_bounds(np.vstack([world, pose[:2]]))
    start = grid.cell(pose[:2])
    ends = np.floor(world / grid.resolution).astype(np.int64) - grid.origin_cell
    free, occupied = trace_rays(start, ends[:, ::-1])
    width = grid.log_odds.shape[1]
    # Un autre rayon ne doit pas effacer un impact du même scan.
    occupied = np.unique(occupied[:, 0] * width + occupied[:, 1])
    free = np.setdiff1d(free[:, 0] * width + free[:, 1], occupied)
    for cells, increment in ((free, -0.4), (occupied, 0.85)):
        if len(cells):
            rows, columns = np.divmod(cells, width)
            grid.log_odds[rows, columns] = np.clip(grid.log_odds[rows, columns] + increment, -4, 4)
            grid.observed[rows, columns] = True
    return grid


def scan_to_map(points, pose=(0, 0, 0), resolution=0.05):
    """Crée une carte vide puis y insère le premier scan et sa pose."""
    return update_map(OccupancyGrid(resolution), points, pose)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scan", type=Path)
    parser.add_argument("--sheet", help="Nom de la feuille Excel (détection automatique par défaut)")
    parser.add_argument("--pose", type=float, nargs=3, default=[0, 0, 0], metavar=("X", "Y", "THETA"))
    parser.add_argument("--resolution", type=float, default=None, help="Taille des cases en mètres (défaut 0.05)")
    parser.add_argument("--map-in", type=Path, help="Carte NPZ existante à mettre à jour")
    parser.add_argument("--output", type=Path, default=Path("resultats/carte_occupation_2d_scan_1"), help="Préfixe des fichiers exportés")
    parser.add_argument("--title", help="Titre affiché sur l'image")
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()
    try:
        groups = load_scan_groups(args.scan, args.pose, args.sheet)
        if args.map_in:
            grid = OccupancyGrid.load(args.map_in)
            if args.resolution is not None and not math.isclose(args.resolution, grid.resolution):
                raise ValueError("La résolution doit rester celle de la carte existante")
        else:
            grid = OccupancyGrid(args.resolution if args.resolution is not None else 0.05)
        for points, pose, source in groups:
            update_map(grid, points, pose)
            grid.history.append(source)
        point_count = sum(len(points) for points, _, _ in groups)
        prefix = args.output
        prefix.parent.mkdir(parents=True, exist_ok=True)
        grid.save(str(prefix) + ".npz")
        states = grid.states()
        np.savetxt(str(prefix) + ".csv", states, delimiter=",", fmt="%d")
        metadata = {"resolution_m": grid.resolution, "origin_xy_m": grid.origin.tolist(),
                    "shape_rows_columns": list(states.shape), "row_order": "y croissant (bas vers haut)",
                    "states": {"-1": "inconnu ou incertain", "0": "libre", "1": "occupé"},
                    "pose_units": "mètres, degrés antihoraires", "history": grid.history,
                    "log_odds": {"free_increment": -0.4, "occupied_increment": 0.85,
                                 "bounds": [-4, 4], "free_threshold": 0.45, "occupied_threshold": 0.65},
                    "cell_counts": {str(v): int(np.count_nonzero(states == v)) for v in (-1, 0, 1)}}
        Path(str(prefix) + ".json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n")
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))

    import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.patches import Patch

    colors = ["#b7bdc6", "#ffffff", "#202833"]
    fig, ax = plt.subplots(figsize=(10, 9))
    ax.imshow(states, origin="lower", extent=grid.extent, interpolation="nearest",
              cmap=ListedColormap(colors), norm=BoundaryNorm([-1.5, -0.5, 0.5, 1.5], 3))
    for entry in grid.history:
        x, y, theta = entry["pose"]
        ax.plot(x, y, "+", color="#e34935", markersize=12, markeredgewidth=2)
        rad = np.deg2rad(theta)
        ax.quiver(x, y, np.cos(rad), np.sin(rad), color="#e34935", angles="xy", scale_units="xy", scale=2)
    ax.set_aspect("equal")
    ax.set(xlabel="X (m)", ylabel="Y (m)",
           title=f"{args.title or ('Carte d’occupation 2D — ' + args.scan.stem)}\n"
                 f"Cases de {grid.resolution * 100:g} cm · {point_count} impacts · {len(groups)} acquisition(s)")
    ax.legend(handles=[Patch(facecolor=c, edgecolor="#666666", label=l)
                       for c, l in zip(colors, ["Inconnu / incertain", "Libre", "Occupé"])]
              + [plt.Line2D([], [], marker="+", color="#e34935", linestyle="None", label="LiDAR")],
              loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(str(args.output) + ".png", dpi=180)
    print(f"{len(groups)} acquisition(s) — {point_count} impacts, "
          f"{sum(s['zero_distance_ignored'] for _, _, s in groups)} distances nulles ignorées")
    print(f"Grille : {states.shape[1]} × {states.shape[0]} cases, résolution {grid.resolution:g} m")
    print(f"Exports : {args.output}.png / .csv / .npz / .json")
    if args.no_show:
        plt.close(fig)
    else:
        plt.show()


if __name__ == "__main__":
    main()
