"""Enregistre les scans CSV envoyés par l'ESP32 (pas les trames UART brutes)."""

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports


COLUMNS = ("point_index", "angle_deg", "dist_mm", "intensity", "x_lib_mm", "y_lib_mm")


class ScanReader:
    """Attend un index zéro, des indices consécutifs et la ligne vide finale."""

    def __init__(self):
        self.points = []

    def feed(self, line):
        line = line.strip()
        if not line:
            points, self.points = self.points, []
            # Vérification de la couverture, sans prétendre détecter les pertes
            # de trames survenues en amont dans l'ESP32.
            if (len(points) >= 2 and points[0][1] >= 355
                    and points[-1][1] <= 5
                    and all(a[1] > b[1] for a, b in zip(points, points[1:]))):
                return points
            return None
        try:
            fields = line.split(",")
            if len(fields) != 6:
                raise ValueError("Six colonnes attendues")
            index, angle, distance, intensity, x, y = fields
            point = (int(index), float(angle), int(distance), int(intensity), int(x), int(y))
            if not (math.isfinite(point[1]) and 0 <= point[1] < 360
                    and point[0] >= 0 and 0 <= point[2] <= 65535
                    and 0 <= point[3] <= 255):
                raise ValueError("Valeurs hors limites")
        except ValueError:
            self.points = []
            return None
        if point[0] == 0:
            self.points = [point]
        elif self.points and point[0] == len(self.points):
            self.points.append(point)
        else:
            self.points = []
        return None


def save_scan(points, directory):
    directory.mkdir(parents=True, exist_ok=True)
    name = "scan_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".csv"
    path = directory / name
    # Le mode x interdit d'écraser une acquisition existante.
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(COLUMNS)
        writer.writerows(points)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Port USB de l'ESP32")
    parser.add_argument("--list-ports", action="store_true", help="Liste les ports disponibles")
    parser.add_argument("--count", type=int, default=1, help="Nombre de scans (défaut : 1)")
    parser.add_argument("--timeout", type=float, default=30, help="Délai total en secondes")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "scans")
    args = parser.parse_args()
    if args.list_ports:
        for port in list_ports.comports():
            print(f"{port.device} : {port.description}")
        return 0
    if not args.port:
        parser.error("Préciser --port ou utiliser --list-ports")
    if args.count < 1 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--count et --timeout doivent être strictement positifs")

    reader = ScanReader()
    saved = 0
    try:
        print("Garder le capteur immobile. Attente d'un scan CSV complet à 115200 bauds…")
        with serial.Serial(args.port, 115200, timeout=0.5, exclusive=True) as device:
            deadline = time.monotonic() + args.timeout
            pending = b""
            while saved < args.count and time.monotonic() < deadline:
                # Un timeout de lecture n'est pas un séparateur de scans.
                pending += device.read_until(b"\n")
                if len(pending) > 4096:
                    reader = ScanReader()
                    pending = b""
                    continue
                if not pending.endswith(b"\n"):
                    continue
                try:
                    line = pending.decode("utf-8")
                except UnicodeDecodeError:
                    reader = ScanReader()
                    pending = b""
                    continue
                pending = b""
                points = reader.feed(line)
                if points is not None:
                    path = save_scan(points, args.output)
                    saved += 1
                    print(f"Scan {saved}/{args.count} : {len(points)} points → {path}")
    except (serial.SerialException, OSError) as error:
        print(f"Erreur : {error}\nFermer le Serial Monitor et vérifier le port USB.")
        return 1
    except KeyboardInterrupt:
        print("\nAcquisition interrompue ; les scans déjà enregistrés sont conservés.")
        return 130
    if saved < args.count:
        print(f"Délai écoulé : {saved}/{args.count} scans enregistrés. Vérifier la sortie CSV.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
