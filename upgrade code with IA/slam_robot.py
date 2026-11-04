"""Transport ESP32 et exploration par frontières, sans mouvement à l'import.

La commande physique reste désactivée par défaut. Les durées sont bornées
et le contrôleur travaille par impulsions arrêtées entre deux scans frais.
"""

import math
import struct
import time
from collections import deque
from urllib.error import HTTPError
from urllib.parse import urlencode, urlparse
from urllib.request import urlopen

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label

from localisation_grid import angle_error


class StaleScanError(ValueError):
    pass


def decode_robot_scan(payload, previous_sequence=None, max_age_ms=1000):
    """SLM1, séquence, fin d'acquisition, horloge courante, N, puis N*(angle,dist,I)."""
    header = struct.Struct("<4sIIIH")
    if len(payload) < header.size:
        raise ValueError("En-tête de scan incomplet")
    magic, sequence, acquired, now, count = header.unpack_from(payload)
    if magic != b"SLM1" or not 12 <= count <= 460 or len(payload) != header.size + count * 5:
        raise ValueError("Format ou longueur du scan invalide ; firmware /scan_slam requis")
    if ((now - acquired) & 0xffffffff) > max_age_ms:
        raise StaleScanError("Le scan publié est trop ancien")
    if previous_sequence is not None:
        delta = (sequence - previous_sequence) & 0xffffffff
        if not 0 < delta < 0x80000000:
            raise StaleScanError("Scan déjà lu ou séquence réinitialisée")
    rows = np.array(list(struct.iter_unpack("<HHB", payload[header.size:])), dtype=float)
    if np.any(rows[:, 0] >= 36000):
        raise ValueError("Angle LiDAR invalide")
    angle = np.deg2rad(rows[:, 0] / 100)
    distance = rows[:, 1] / 1000
    valid = distance > 0
    points = np.column_stack((distance[valid] * np.cos(angle[valid]),
                              distance[valid] * np.sin(angle[valid])))
    covered = np.sort(np.unique(rows[valid, 0] / 100))
    max_gap = float(np.max(np.diff(np.r_[covered, covered[0] + 360]))) if len(covered) else 360.0
    return points, {"sequence": sequence, "device_acquired_ms": acquired,
                    "age_ms": (now - acquired) & 0xffffffff, "input_points": count,
                    "max_valid_angle_gap_deg": max_gap}


def decode_legacy_scan(payload):
    """Format /scan du firmware RobotControl d'origine (8 + 5*N octets, sans horodatage)."""
    if len(payload) < 8:
        raise ValueError("Réponse /scan incomplète")
    count = struct.unpack_from("<H", payload, 6)[0]
    if not 1 <= count <= 460 or len(payload) != 8 + count * 5:
        raise ValueError("Format ou longueur du scan /scan invalide")
    rows = np.array(list(struct.iter_unpack("<HHB", payload[8:])), dtype=float)
    if np.any(rows[:, 0] >= 36000):
        raise ValueError("Angle LiDAR invalide")
    angle, distance = np.deg2rad(rows[:, 0] / 100), rows[:, 1] / 1000
    valid = distance > 0
    points = np.column_stack((distance[valid] * np.cos(angle[valid]),
                              distance[valid] * np.sin(angle[valid])))
    covered = np.sort(np.unique(rows[valid, 0] / 100))
    max_gap = float(np.max(np.diff(np.r_[covered, covered[0] + 360]))) if len(covered) else 360.0
    return points, {"sequence": None, "age_ms": None, "input_points": count, "firmware": "legacy",
                    "max_valid_angle_gap_deg": max_gap}


class RobotHTTP:
    def __init__(self, url, motion_enabled=False, timeout=1.0, opener=urlopen):
        parsed = urlparse(url)
        if parsed.scheme != "http" or not parsed.hostname or parsed.username or parsed.password:
            raise ValueError("Adresse http:// du robot requise, sans identifiants intégrés")
        self.url = url.rstrip("/")
        self.motion_enabled = motion_enabled
        self.timeout = timeout
        self.opener = opener
        self.sequence = None
        self.legacy = False  # True si /scan_slam est absent : firmware d'origine
        self.legacy_settle = .35  # ~3 tours du LD06 après l'arrêt, faute d'horodatage

    def _request(self, path):
        with self.opener(self.url + path, timeout=self.timeout) as response:
            return response.read(16384)

    def scan(self):
        if not self.legacy:
            try:
                payload = self._request("/scan_slam")
            except HTTPError as error:
                if error.code != 404:
                    raise
                self.legacy = True
        if self.legacy:
            return self._legacy_scan()
        points, metadata = decode_robot_scan(payload, self.sequence)
        self.last_payload = payload
        self.sequence = metadata["sequence"]
        return points, metadata

    def _legacy_scan(self):
        """Sans séquence : attendre plusieurs tours puis exiger un contenu nouveau."""
        time.sleep(self.legacy_settle)
        payload = self._request("/scan")
        if getattr(self, "last_payload", None) is not None and payload[8:] == self.last_payload[8:]:
            raise StaleScanError("Scan /scan identique au précédent")
        points, metadata = decode_legacy_scan(payload)
        self.last_payload = payload
        return points, metadata

    def show_pose(self, pose):
        """Affiche la pose SLAM dans RobotControl via /pose (mm, centièmes de degré)."""
        x, y = (int(np.clip(round(v * 1000), -32768, 32767)) for v in pose[:2])
        theta = int(round(float(pose[2]) * 100)) % 36000
        self._request("/pose?" + urlencode({"x": x, "y": y, "z": theta}))

    def fresh_scan(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                return self.scan()
            except StaleScanError:
                time.sleep(.04)
        raise TimeoutError("Aucun nouveau scan frais ; arrêt de la navigation")

    def stop(self):
        self._request("/stop")

    def move(self, kind, seconds):
        if not self.motion_enabled:
            raise PermissionError("Mouvement désactivé : --auto doit être explicite")
        if kind not in ("forward", "rotateleft", "rotateright"):
            raise ValueError("Commande de déplacement non autorisée")
        if not math.isfinite(seconds) or not 0 < seconds <= .25:
            raise ValueError("Impulsion attendue entre 0 et 0,25 seconde")
        # Le firmware existant divise par deux le paramètre des rotations.
        duration = int(math.ceil(seconds * 1000))
        argument = duration * (2 if kind.startswith("rotate") else 1)
        try:
            self._request("/" + kind + "?" + urlencode({"time": argument}))
            time.sleep(duration / 1000 + .03)
        finally:
            self.stop()


class FrontierExplorer:
    """Trajets dans les cellules libres, avec gonflement par le rayon du robot."""
    def __init__(self, radius=.18, margin=.10, step=.12, turn_deg=25, patience=40):
        if not np.isfinite([radius, margin, step, turn_deg]).all() or min(radius, margin, step, turn_deg) <= 0:
            raise ValueError("Dimensions de navigation strictement positives requises")
        self.radius, self.margin, self.step, self.turn_deg = radius, margin, step, turn_deg
        self.patience = patience
        self.goal, self.goal_age, self.blacklist = None, 0, []

    def _frontiers(self, states, free, traversable, resolution):
        """Cases libres au bord de l'inconnu, puis zone accessible à côté.

        Le gonflement se fait uniquement à travers le libre : l'inconnu situé
        derrière un mur ne crée plus de frontière à l'intérieur de la pièce.
        """
        raw = free & binary_dilation(states == -1)
        labels, _ = label(raw, structure=np.ones((3, 3)))
        sizes = np.bincount(labels.ravel())
        raw &= sizes[labels] >= 3  # ignorer les trous isolés d'une ou deux cases
        reach = int(np.ceil((self.radius + self.margin) / resolution)) + 4
        return traversable & binary_dilation(raw, iterations=reach, mask=free)

    @staticmethod
    def _line_clear(mask, a, b):
        steps = 2 * max(abs(b[0] - a[0]), abs(b[1] - a[1])) + 1
        rows = np.rint(np.linspace(a[0], b[0], steps)).astype(int)
        cols = np.rint(np.linspace(a[1], b[1], steps)).astype(int)
        return bool(mask[rows, cols].all())

    def _blacklisted(self, grid, cell):
        point = (np.array(cell[::-1]) + .5) * grid.resolution + grid.origin
        return any(np.linalg.norm(point - b) < .5 for b in self.blacklist)

    def _escape(self, grid, pose, points, free, distance, reason):
        """Rejoindre une case libre mieux dégagée, sans traverser de zone inconnue."""
        clearance = self.radius + self.margin
        start = grid.cell(pose[:2])
        current = distance[start]
        candidates = []
        for heading in np.linspace(-180, 180, 24, endpoint=False):
            direction = np.array([np.cos(np.deg2rad(heading)), np.sin(np.deg2rad(heading))])
            cells = [grid.cell(np.array(pose[:2]) + direction*self.step*t) for t in (0, .5, 1)]
            if any(not (0 <= r < free.shape[0] and 0 <= c < free.shape[1])
                   or not free[r, c] or distance[r, c] < clearance for r, c in cells):
                continue
            if distance[cells[-1]] <= current + grid.resolution*.2:
                continue
            local_angle = np.deg2rad(angle_error(heading, pose[2]))
            delta = np.array([np.cos(local_angle), np.sin(local_angle)])*self.step
            t = np.clip(points @ delta / (self.step**2), 0, 1)
            if np.min(np.linalg.norm(points-t[:, None]*delta, axis=1)) < clearance:
                continue
            candidates.append((distance[cells[-1]], heading))
        if not candidates:
            return {"kind": "stop", "reason": reason, "goal": self.goal}
        _, heading = max(candidates)
        error = angle_error(heading, pose[2])
        self.goal = None
        if abs(error) > 8:
            return {"kind": "rotateleft" if error > 0 else "rotateright",
                    "turn_deg": min(abs(error), self.turn_deg), "goal": None,
                    "reason": "degagement_orientation"}
        return {"kind": "forward", "distance_m": self.step, "goal": None,
                "reason": "degagement_zone_libre"}

    def action(self, grid, pose, points, localized=True):
        stop = lambda reason: {"kind": "stop", "reason": reason, "goal": self.goal}
        if not localized:
            return stop("localisation_rejetee")
        points = np.asarray(points)
        if not len(points):
            return stop("scan_vide")
        clearance = self.radius + self.margin
        nearest = np.min(np.linalg.norm(points, axis=1))
        # Arrêt d'urgence au rayon du robot (+3 cm) ; la marge de planification
        # sert ensuite à s'écarter, pas à s'arrêter définitivement.
        if nearest < self.radius + .03:
            return stop("obstacle_dans_le_gabarit")
        states = grid.states()
        if not states.size:
            return stop("carte_vide")
        free = states == 0
        # Le champ mesure les centres des cases : ajouter une marge de discrétisation.
        distance = distance_transform_edt(free) * grid.resolution
        traversable = free & (distance
                              >= clearance + 1.5 * grid.resolution)
        start = grid.cell(pose[:2])
        height, width = states.shape
        if not (0 <= start[0] < height and 0 <= start[1] < width and free[start]):
            return stop("degagement_non_observe")
        if not traversable[start] or nearest < clearance:
            return self._escape(grid, pose, points, free, distance, "degagement_non_observe")
        frontiers = self._frontiers(states, free, traversable, grid.resolution)
        queue, parents, depths = deque([start]), {start: None}, {start: 0}
        while queue:
            cell = queue.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                other = (cell[0] + dr, cell[1] + dc)
                if (0 <= other[0] < height and 0 <= other[1] < width
                        and traversable[other] and other not in parents):
                    parents[other] = cell
                    depths[other] = depths[cell] + 1
                    queue.append(other)
        goal_cell = grid.cell(self.goal) if self.goal is not None else None
        self.goal_age += 1
        if self.goal is not None and (self.goal_age > self.patience or
                                      np.linalg.norm(np.subtract(self.goal, pose[:2])) < .25):
            # Non atteinte après `patience` actions, ou atteinte sans que
            # l'inconnu disparaisse (zone cachée derrière un meuble) : abandon.
            self.blacklist.append(np.array(self.goal))
            goal_cell = None
        if (goal_cell not in parents or depths.get(goal_cell, 0) < 4
                or not frontiers[goal_cell]):
            candidates = [cell for cell in parents if frontiers[cell] and depths[cell] >= 4
                          and not self._blacklisted(grid, cell)]
            if not candidates:
                self.goal = None
                return stop("aucune_frontiere_accessible")
            # La plus proche en chemin : moins de trajet que la plus lointaine.
            goal_cell = min(candidates, key=lambda cell: depths[cell])
            self.goal = ((np.array(goal_cell[::-1]) + .5) * grid.resolution + grid.origin).tolist()
            self.goal_age = 0
        path, cell = [], goal_cell
        while cell is not None:
            path.append(cell)
            cell = parents[cell]
        path.reverse()
        # Viser le point du chemin le plus lointain (<= 40 cm) visible en ligne
        # droite dans la zone dégagée : pas de virage coupé, et pas d'escalier
        # 4-connexe qui ferait alterner les rotations de 90°.
        lookahead = path[min(1, len(path) - 1)]
        for cell in path[1:int(.40 / grid.resolution) + 1]:
            if not self._line_clear(traversable, path[0], cell):
                break
            lookahead = cell
        target = (np.array(lookahead[::-1]) + .5) * grid.resolution + grid.origin
        heading = np.rad2deg(np.arctan2(target[1] - pose[1], target[0] - pose[0]))
        error = angle_error(heading, pose[2])
        if abs(error) > 12:
            return {"kind": "rotateleft" if error > 0 else "rotateright",
                    "turn_deg": min(abs(error), self.turn_deg), "goal": self.goal,
                    "reason": "orientation_vers_frontiere"}
        # Capsule balayée par le gabarit circulaire, pas un rectangle trop conservateur.
        closest_x = np.clip(points[:, 0], 0, self.step)
        ahead = np.hypot(points[:, 0] - closest_x, points[:, 1]) < clearance
        if ahead.any():
            return self._escape(grid, pose, points, free, distance, "obstacle_devant")
        return {"kind": "forward", "distance_m": self.step, "goal": self.goal,
                "reason": "avance_dans_zone_libre"}
