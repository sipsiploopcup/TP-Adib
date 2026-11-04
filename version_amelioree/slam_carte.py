"""SLAM 2D scan-sur-carte (variante du TP3).

Chaque scan est recalé sur la grille d'occupation construite avec les scans
précédents (GridMatcher du jour 2), autour d'une prédiction d'odométrie.
Un scan accepté met la carte à jour (update_map du jour 1) ; un scan rejeté
ne touche pas la carte, mais la pose suit l'odométrie (navigation à l'estime)
pour ne pas perdre le déplacement déjà effectué.

Poses : (x m, y m, theta degrés antihoraires), comme dans le reste du projet.
"""

import sys
from collections import deque
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label

sys.path.insert(0, str(Path(__file__).resolve().parent))

from geometry import apply_pose  # noqa: E402
from gridmap import OccupancyGrid, update_map  # noqa: E402
from localisation_grid import GridMatcher, angle_error  # noqa: E402
from merge_scans import voxel_sample  # noqa: E402
from segments import extract_segments  # noqa: E402
from vectormap import VectorMap  # noqa: E402


def compose(a, b):
    """Pose b exprimée dans le repère de a, ramenée dans le repère monde."""
    xy = apply_pose(np.asarray(b[:2], dtype=float)[None, :], *a)[0]
    return np.array([xy[0], xy[1], angle_error(a[2] + b[2], 0)])


class MapSLAM:
    def __init__(self, resolution=0.05, search_xy=0.30, search_deg=10.0,
                 min_overlap=0.60, max_range=11.5, voxel=0.05):
        self.grid = OccupancyGrid(resolution)
        self.search_xy, self.search_deg = search_xy, search_deg
        self.min_overlap, self.max_range, self.voxel = min_overlap, max_range, voxel
        self.pose = np.zeros(3)
        self.frames, self.events = [], []
        self._matcher = None

    def _prepare(self, points):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            return None
        r = np.linalg.norm(points, axis=1)
        return voxel_sample(points[(r >= 0.12) & (r <= self.max_range)], self.voxel)

    def process(self, points, source="scan", initial=None, wide=False):
        """initial : prédiction d'odométrie (pose monde). wide : fenêtre large (rejeu)."""
        started = perf_counter()
        event = {"source": str(source), "index": len(self.events)}
        guess = self.pose.copy() if initial is None else np.asarray(initial, dtype=float)
        points = self._prepare(points)
        if points is None or len(points) < 20:
            self.pose = guess  # pas de mesure exploitable : on garde l'estime
            return self._finish(event, False, "scan_invalide", started)
        if not self.frames:
            return self._accept(event, points, guess, {"overlap_20cm": None}, started)
        if self._matcher is None:
            self._matcher = GridMatcher(self.grid)
        # Fenêtre large : plusieurs bassins à garder. Locale : 3 candidats suffisent.
        xy, deg, beam = (3.0, 180.0, 8) if wide else (self.search_xy, self.search_deg, 3)
        result = self._matcher.match(points, guess, xy_range=xy, angle_range=deg, beam_width=beam)
        event.update(overlap_20cm=result["overlap_20cm"], rmse_m=result["rmse_distance_map_m"])
        if result["overlap_20cm"] < self.min_overlap or (result["on_search_boundary"] and not wide):
            self.pose = guess
            return self._finish(event, False, "recalage_refuse", started)
        return self._accept(event, points, np.asarray(result["pose"]), result, started)

    def _accept(self, event, points, pose, quality, started):
        self.pose = np.array([pose[0], pose[1], angle_error(pose[2], 0)])
        update_map(self.grid, points, self.pose)
        self._matcher = None  # la carte a changé : champ de distance à recalculer
        self.frames.append({"source": event["source"], "points": points, "pose": self.pose.copy()})
        return self._finish(event, True, "accepte", started)

    def _finish(self, event, accepted, reason, started):
        event.update(accepted=accepted, reason=reason, pose=self.pose.tolist(),
                     runtime_s=perf_counter() - started)
        self.events.append(event)
        return event

    def vector_map(self, every=1):
        """Carte vectorielle à partir des scans acceptés et de leurs poses estimées."""
        vmap = VectorMap()
        for frame in self.frames[::every]:
            vmap.add_segments(apply_pose(s, *frame["pose"]) for s in extract_segments(frame["points"]))
        return vmap.segments


class NearestFrontierExplorer:
    """Va vers la frontière accessible la plus proche (chemin BFS dans le libre dégagé)."""

    def __init__(self, radius=0.18, margin=0.08, step=0.15, max_turn=25.0, lookahead=0.40):
        self.clearance = radius + margin
        self.step, self.max_turn, self.lookahead = step, max_turn, lookahead
        self.goal, self.goal_age, self.blacklist = None, 0, []

    def action(self, grid, pose, points, localized=True):
        stop = lambda reason: {"kind": "stop", "reason": reason, "goal": self.goal}  # noqa: E731
        if not localized:
            return stop("localisation_rejetee")
        points = np.asarray(points)
        states = grid.states()
        res = grid.resolution
        free = states == 0
        clear = distance_transform_edt(states != 1) * res >= self.clearance
        traversable = free & clear
        start = self._nearest(traversable, grid.cell(pose[:2]))
        if start is None:
            return stop("aucune_cellule_degagee")
        parents, order = self._bfs(traversable, start)
        frontier = traversable & binary_dilation(states == -1)
        labels, count = label(frontier, structure=np.ones((3, 3)))
        sizes = np.bincount(labels.ravel())
        best = None
        for cell in order:  # ordre BFS = distance croissante
            lab = labels[cell]
            if lab and sizes[lab] >= 3 and not self._blacklisted(grid, cell):
                best = cell
                break
        if best is None:
            return stop("exploration_terminee")
        goal = (np.array(best[::-1]) + 0.5) * res + grid.origin
        if self.goal is None or np.linalg.norm(goal - self.goal) > 0.3:
            self.goal, self.goal_age = goal, 0
        self.goal_age += 1
        if self.goal_age > 60:  # pas de progrès : on abandonne cette frontière
            self.blacklist.append(self.goal)
            self.goal = None
            return {"kind": "rotateleft", "turn_deg": self.max_turn, "reason": "frontiere_abandonnee"}
        path, cell = [], best
        while cell is not None:
            path.append(cell)
            cell = parents[cell]
        path.reverse()
        target = (np.array(path[min(len(path) - 1, int(self.lookahead / res))][::-1]) + 0.5) * res + grid.origin
        heading = np.rad2deg(np.arctan2(target[1] - pose[1], target[0] - pose[0]))
        error = angle_error(heading, pose[2])
        if abs(error) > 15:
            return {"kind": "rotateleft" if error > 0 else "rotateright",
                    "turn_deg": min(abs(error), self.max_turn), "reason": "orientation"}
        ahead = (points[:, 0] > 0) & (points[:, 0] < self.clearance + self.step) \
            & (np.abs(points[:, 1]) < self.clearance)
        if ahead.any():
            self.blacklist.append(self.goal)
            self.goal = None
            return {"kind": "rotateleft", "turn_deg": self.max_turn, "reason": "obstacle_devant"}
        distance = min(self.step, max(0.05, np.linalg.norm(target - pose[:2])))
        return {"kind": "forward", "distance_m": distance, "reason": "vers_frontiere"}

    def _blacklisted(self, grid, cell):
        point = (np.array(cell[::-1]) + 0.5) * grid.resolution + grid.origin
        return any(np.linalg.norm(point - b) < 0.5 for b in self.blacklist)

    @staticmethod
    def _nearest(mask, cell, radius=6):
        h, w = mask.shape
        r0, c0 = cell
        best, best_d = None, None
        for r in range(max(0, r0 - radius), min(h, r0 + radius + 1)):
            for c in range(max(0, c0 - radius), min(w, c0 + radius + 1)):
                d = (r - r0) ** 2 + (c - c0) ** 2
                if mask[r, c] and (best_d is None or d < best_d):
                    best, best_d = (r, c), d
        return best

    @staticmethod
    def _bfs(mask, start):
        h, w = mask.shape
        parents, order, queue = {start: None}, [start], deque([start])
        while queue:
            r, c = queue.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (r + dr, c + dc)
                if 0 <= n[0] < h and 0 <= n[1] < w and mask[n] and n not in parents:
                    parents[n] = (r, c)
                    order.append(n)
                    queue.append(n)
        return parents, order
