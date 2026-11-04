"""SLAM 2D incrémental : scan matching, grille, images clés et fermeture de boucle.

Les poses utilisent (m, m, degrés antihoraires). Un scan rejeté ne modifie
ni la carte, ni la référence de localisation. Les résidus ne sont pas des
incertitudes de pose mesurées.
"""

from dataclasses import dataclass
from time import perf_counter

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

from geometry import apply_pose
from gridmap import OccupancyGrid, update_map
from localisation_grid import angle_error
from localisation_scans import ICPMatcher
from merge_scans import voxel_sample


def compose_pose(a, b):
    """T(a) @ T(b), sans additionner naïvement les translations."""
    xy = apply_pose(np.asarray(b[:2])[None, :], *a)[0]
    return np.array([xy[0], xy[1], angle_error(a[2] + b[2], 0)])


def relative_pose(a, b):
    xy = apply_pose((np.asarray(b[:2]) - a[:2])[None, :], theta_deg=-a[2])[0]
    return np.array([xy[0], xy[1], angle_error(b[2], a[2])])


@dataclass
class SLAMConfig:
    resolution: float = 0.05
    voxel: float = 0.05
    max_range: float = 20.0
    matching_min_range: float = 0.60
    min_overlap: float = 0.65
    max_inlier_rmse: float = 0.10
    max_step: float = 0.70
    max_rotation: float = 35.0
    keyframe_distance: float = 0.25
    keyframe_angle: float = 12.0
    loop_closure: bool = True

    def __post_init__(self):
        values = [self.resolution, self.voxel, self.max_range, self.min_overlap,
                  self.max_inlier_rmse, self.max_step, self.max_rotation,
                  self.keyframe_distance, self.keyframe_angle, self.matching_min_range]
        if (not np.isfinite(values).all() or min(values) <= 0
                or self.min_overlap > 1 or self.max_rotation > 180
                or self.max_range < 0.5 or self.max_step > 20
                or self.matching_min_range >= self.max_range):
            raise ValueError("Configuration SLAM invalide")


def registration_quality(points, target, pose):
    distances = cKDTree(target).query(apply_pose(points, *pose))[0]
    inliers = distances <= 0.20
    return {"overlap": float(inliers.mean()), "inliers": int(inliers.sum()),
            "inlier_rmse_m": float(np.sqrt(np.mean(distances[inliers] ** 2)))
            if inliers.any() else None}


class IncrementalSLAM:
    def __init__(self, config=None):
        self.config = config or SLAMConfig()
        self.grid = OccupancyGrid(self.config.resolution)
        self.pose = np.zeros(3)
        self.frames, self.keyframes, self.edges, self.events = [], [], [], []
        self.loop_count = 0
        self.optimizations = 0

    def _submap(self):
        indices = sorted(range(len(self.keyframes)), key=lambda i:
                         np.linalg.norm(self.keyframes[i]["pose"][:2] - self.pose[:2]))[:8]
        return voxel_sample(np.vstack([apply_pose(self._matching_points(self.keyframes[i]["points"]),
                                                 *self.keyframes[i]["pose"])
                                       for i in indices]), self.config.voxel)

    def _matching_points(self, points):
        # Comme la fusion du TP1 : éviter le châssis et les objets très proches
        # pour estimer la pose. Ces impacts restent présents dans la grille.
        return points[np.linalg.norm(points, axis=1) >= self.config.matching_min_range]

    def _reject(self, event, reason, started):
        event.update(accepted=False, reason=reason, pose=self.pose.tolist(),
                     runtime_s=perf_counter() - started)
        self.events.append(event)
        return event

    def process(self, points, source="scan", initial=None, wide_search=False):
        started = perf_counter()
        event = {"source": str(source), "index": len(self.events)}
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            return self._reject(event, "invalid_scan", started)
        distances = np.linalg.norm(points, axis=1)
        points = voxel_sample(points[(distances >= 0.12) &
                                     (distances <= self.config.max_range)], self.config.voxel)
        event["points"] = len(points)
        if len(points) < 12:
            return self._reject(event, "too_few_points", started)
        fitting = self._matching_points(points)
        event["matching_points"] = len(fitting)
        if len(fitting) < 12:
            return self._reject(event, "too_few_matching_points", started)
        singular = np.linalg.svd(fitting - fitting.mean(axis=0), compute_uv=False)
        if singular[0] < 1e-8 or singular[1] / singular[0] < 0.01:
            return self._reject(event, "degenerate_scan", started)
        guess = self.pose.copy() if initial is None else np.asarray(initial, dtype=float)
        if guess.shape != (3,) or not np.isfinite(guess).all() or abs(guess[:2]).max() > 1e5:
            return self._reject(event, "invalid_initial_pose", started)
        if not self.frames:
            pose = guess.copy()
            quality = {"overlap": None, "inliers": len(points), "inlier_rmse_m": None}
            event["registration"] = "first_scan_anchor"
        else:
            target = self._submap()
            matcher = ICPMatcher(target)
            seeds = [guess.copy()]
            if wide_search:
                for angle in np.arange(-180, 180, 30):
                    seed = guess.copy()
                    seed[2] = angle
                    seeds.append(seed)
                    moved_center = apply_pose(fitting.mean(axis=0)[None, :], theta_deg=angle)[0]
                    seeds.append(np.r_[target.mean(axis=0) - moved_center, angle])
            candidates = []
            for seed in seeds:
                result = matcher.match(fitting, initial=seed, xy_range=self.config.max_step,
                                       angle_range=self.config.max_rotation, max_iterations=50)
                candidate = np.asarray(result["pose"])
                if (np.linalg.norm(candidate[:2] - self.pose[:2]) > self.config.max_step
                        or abs(angle_error(candidate[2], self.pose[2])) > self.config.max_rotation):
                    continue
                q = registration_quality(fitting, target, candidate)
                if (q["inliers"] >= 12 and q["overlap"] >= self.config.min_overlap
                        and q["inlier_rmse_m"] <= self.config.max_inlier_rmse):
                    candidates.append((candidate, q))
            if not candidates:
                return self._reject(event, "insufficient_or_inconsistent_match", started)
            candidates.sort(key=lambda x: (-x[1]["overlap"], x[1]["inlier_rmse_m"]))
            pose, quality = candidates[0]
            if wide_search and any(
                    abs(q["overlap"] - quality["overlap"]) < .025
                    and abs(q["inlier_rmse_m"] - quality["inlier_rmse_m"]) < .015
                    and (np.linalg.norm(p[:2] - pose[:2]) > .5
                         or abs(angle_error(p[2], pose[2])) > 20)
                    for p, q in candidates[1:]):
                return self._reject(event, "ambiguous_registration", started)
            event["registration"] = "ICP_multistart" if wide_search else "ICP_local_submap"
        self.pose = pose.copy()
        is_keyframe = (not self.keyframes or
                       np.linalg.norm(pose[:2] - self.keyframes[-1]["pose"][:2]) >= self.config.keyframe_distance
                       or abs(angle_error(pose[2], self.keyframes[-1]["pose"][2])) >= self.config.keyframe_angle)
        if is_keyframe:
            if self.keyframes:
                self.edges.append({"a": len(self.keyframes) - 1, "b": len(self.keyframes),
                                   "relative": relative_pose(self.keyframes[-1]["pose"], pose),
                                   "type": "sequential"})
            self.keyframes.append({"pose": pose.copy(), "points": points.copy()})
        frame = {"source": str(source), "points": points.copy(), "pose": pose.copy(),
                 "keyframe": len(self.keyframes) - 1,
                 "relative": relative_pose(self.keyframes[-1]["pose"], pose)}
        self.frames.append(frame)
        update_map(self.grid, points, pose)
        event.update(accepted=True, reason="accepted", pose=pose.tolist(), **quality)
        self.events.append(event)
        if is_keyframe and self.config.loop_closure:
            self._try_loop()
        event["pose"] = self.pose.tolist()
        event["runtime_s"] = perf_counter() - started
        return event

    def _try_loop(self):
        j = len(self.keyframes) - 1
        if j < 7:
            return
        current = self.keyframes[j]
        candidates = [i for i in range(j - 5)
                      if np.linalg.norm(self.keyframes[i]["pose"][:2] - current["pose"][:2]) < .9
                      and sum(np.linalg.norm(self.keyframes[k+1]["pose"][:2]
                                             - self.keyframes[k]["pose"][:2])
                              for k in range(i, j)) >= 1.5]
        for i in sorted(candidates, key=lambda i: np.linalg.norm(
                self.keyframes[i]["pose"][:2] - current["pose"][:2]))[:2]:
            old = self.keyframes[i]
            old_points = self._matching_points(old["points"])
            current_points = self._matching_points(current["points"])
            seed = relative_pose(old["pose"], current["pose"])
            result = ICPMatcher(old_points).match(current_points, seed,
                                                     xy_range=.4, angle_range=12)
            measured = np.asarray(result["pose"])
            quality = registration_quality(current_points, old_points, measured)
            if (quality["overlap"] < .8 or quality["inlier_rmse_m"] is None
                    or quality["inlier_rmse_m"] > .065
                    or np.linalg.norm(measured[:2] - seed[:2]) > .35
                    or abs(angle_error(measured[2], seed[2])) > 10):
                continue
            reverse_seed = relative_pose(measured, np.zeros(3))
            reverse = ICPMatcher(current_points).match(old_points, reverse_seed,
                                                         xy_range=.4, angle_range=12)
            identity = compose_pose(measured, reverse["pose"])
            if np.linalg.norm(identity[:2]) > .08 or abs(identity[2]) > 3:
                continue
            self.edges.append({"a": i, "b": j, "relative": measured, "type": "loop"})
            self.loop_count += 1
            self.optimize_graph()
            break

    def optimize_graph(self):
        """Premier nœud fixé ; pondérations heuristiques, pas de covariance certifiée."""
        if len(self.keyframes) < 2:
            return False
        poses = np.array([k["pose"] for k in self.keyframes])
        radians = poses.copy()
        radians[:, 2] = np.deg2rad(radians[:, 2])

        def residual(values):
            proposed = np.vstack((radians[0], values.reshape(-1, 3)))
            result = []
            for edge in self.edges:
                a, b = proposed[edge["a"]], proposed[edge["b"]]
                dx, dy = b[:2] - a[:2]
                predicted = [np.cos(a[2]) * dx + np.sin(a[2]) * dy,
                             -np.sin(a[2]) * dx + np.cos(a[2]) * dy]
                observed = edge["relative"]
                error_angle = b[2] - a[2] - np.deg2rad(observed[2])
                result.extend([(predicted[0] - observed[0]) / .06,
                               (predicted[1] - observed[1]) / .06,
                               np.arctan2(np.sin(error_angle), np.cos(error_angle)) / np.deg2rad(2)])
            return np.asarray(result)

        initial = radians[1:].ravel()
        solved = least_squares(residual, initial, loss="soft_l1", max_nfev=80)
        revised = np.vstack((radians[0], solved.x.reshape(-1, 3)))
        if (np.linalg.norm(residual(solved.x)) > np.linalg.norm(residual(initial)) + 1e-8
                or np.linalg.norm(revised[:, :2] - radians[:, :2], axis=1).max() > 1.5
                or abs(np.rad2deg(np.arctan2(np.sin(revised[:, 2] - radians[:, 2]),
                                             np.cos(revised[:, 2] - radians[:, 2])))).max() > 20):
            return False
        for keyframe, pose in zip(self.keyframes, revised):
            keyframe["pose"] = np.array([pose[0], pose[1], angle_error(np.rad2deg(pose[2]), 0)])
        self.grid = OccupancyGrid(self.config.resolution)
        for frame in self.frames:
            frame["pose"] = compose_pose(self.keyframes[frame["keyframe"]]["pose"], frame["relative"])
            update_map(self.grid, frame["points"], frame["pose"])
        self.pose = self.frames[-1]["pose"].copy()
        self.optimizations += 1
        return True
