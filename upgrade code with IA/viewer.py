"""Affiche les scans CSV et permet d'ajuster indépendamment leurs poses."""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, RadioButtons, Slider

from geometry import apply_pose, load_scan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scans", nargs="+", type=Path)
    parser.add_argument("--poses", type=Path, default=Path(__file__).parent / "poses.json")
    parser.add_argument("--save", type=Path, help="Enregistrer une image PNG")
    parser.add_argument("--no-show", action="store_true", help="Exporter sans ouvrir de fenêtre")
    args = parser.parse_args()
    try:
        clouds = [load_scan(path) for path in args.scans]
        saved = json.loads(args.poses.read_text()) if args.poses.exists() else {}
        poses = []
        for path in args.scans:
            pose = saved.get(str(path.resolve()), [0, 0, 0])
            if len(pose) != 3 or not np.isfinite(pose).all():
                raise ValueError("Une pose doit contenir trois nombres finis")
            poses.append(list(map(float, pose)))
    except (OSError, ValueError, TypeError, AttributeError) as error:
        parser.error(str(error))

    initial_poses = [pose.copy() for pose in poses]
    fig, ax = plt.subplots(figsize=(11, 8))
    fig.subplots_adjust(left=0.09, right=0.72, bottom=0.29)
    artists, origins, arrows = [], [], []
    colors = plt.get_cmap("turbo")(np.linspace(0.08, 0.92, len(clouds)))
    for i, (cloud, pose, color) in enumerate(zip(clouds, poses, colors)):
        points = apply_pose(cloud, *pose)
        artists.append(ax.scatter(points[:, 0], points[:, 1], s=6, color=color,
                                  label=f"Scan {i + 1} ({len(cloud)} points)"))
        origins.append(ax.plot(pose[0], pose[1], "+", color=color, markersize=12)[0])
        angle = np.deg2rad(pose[2])
        arrows.append(ax.quiver(pose[0], pose[1], np.cos(angle), np.sin(angle),
                               angles="xy", scale_units="xy", scale=2, color=color))
    ax.set(xlabel="X (m)", ylabel="Y (m)", title="Scans LD06 — repère du TP (Y vers la gauche)")
    # Garder toute la zone réservée au graphique ; adapter les limites pour
    # obtenir des mètres de même taille sur X et Y, sans décaler le panneau.
    ax.set_aspect("equal", adjustable="datalim", anchor="C")
    ax.grid(alpha=0.3)
    status = fig.text(0.09, 0.025, "X/Y de la bibliothèque : mm → m et inversion de Y. Croix = capteur.", fontsize=9)

    active = [0]
    limit = max(20, max(abs(v) for pose in poses for v in pose[:2]) + 1)
    angle_limit = max(180, max(abs(pose[2]) for pose in poses))
    sliders = [
        Slider(fig.add_axes([0.16, 0.20, 0.5, 0.03]), "X (m)", -limit, limit, valinit=poses[0][0]),
        Slider(fig.add_axes([0.16, 0.14, 0.5, 0.03]), "Y (m)", -limit, limit, valinit=poses[0][1]),
        Slider(fig.add_axes([0.16, 0.08, 0.5, 0.03]), "θ (°)", -angle_limit, angle_limit,
               valinit=poses[0][2]),
    ]
    labels = [f"Scan {i + 1}" for i in range(len(clouds))]
    radio = RadioButtons(fig.add_axes([0.76, 0.43, 0.21, 0.42]), labels)
    for text, color, cloud in zip(radio.labels, colors, clouds):
        text.set_color(color)
        text.set_text(f"{text.get_text()} · {len(cloud)} pts")
    fig.text(0.76, 0.88, "Choisir le scan à déplacer", fontsize=9)

    def fit(_event=None):
        points = np.concatenate([apply_pose(c, *p) for c, p in zip(clouds, poses)]
                                + [np.asarray(poses)[:, :2]])
        low, high = points.min(axis=0), points.max(axis=0)
        margin = max(0.5, float(np.max(high - low)) * 0.05)
        center = (low + high) / 2
        span = high - low + 2 * margin
        box = ax.get_window_extent()
        ratio = box.width / max(box.height, 1)
        width = max(span[0], span[1] * ratio)
        height = width / ratio
        ax.set_xlim(center[0] - width / 2, center[0] + width / 2)
        ax.set_ylim(center[1] - height / 2, center[1] + height / 2)
        fig.canvas.draw_idle()

    def update(_value):
        i = active[0]
        pose = [slider.val for slider in sliders]
        poses[i] = pose
        artists[i].set_offsets(apply_pose(clouds[i], *pose))
        origins[i].set_data([pose[0]], [pose[1]])
        arrows[i].set_offsets([[pose[0], pose[1]]])
        angle = np.deg2rad(pose[2])
        arrows[i].set_UVC(np.cos(angle), np.sin(angle))
        fig.canvas.draw_idle()

    def select(label):
        active[0] = next(i for i, text in enumerate(radio.labels) if text.get_text() == label)
        for slider, value in zip(sliders, poses[active[0]]):
            slider.eventson = False
            slider.set_val(value)
            slider.eventson = True
        fig.canvas.draw_idle()

    def reset_pose(_event=None):
        i = active[0]
        for slider, value in zip(sliders, initial_poses[i]):
            slider.eventson = False
            slider.set_val(value)
            slider.eventson = True
        update(None)
        fit()
        status.set_text(f"Scan {i + 1} : pose d'ouverture restaurée. Enregistrer pour la conserver.")
        fig.canvas.draw_idle()

    def save_poses(_event):
        saved.update({str(path.resolve()): pose for path, pose in zip(args.scans, poses)})
        try:
            args.poses.write_text(json.dumps(saved, indent=2) + "\n", encoding="utf-8")
            status.set_text(f"Poses enregistrées : {args.poses}")
        except OSError as error:
            status.set_text(f"Erreur d'enregistrement : {error}")
        fig.canvas.draw_idle()

    for slider in sliders:
        slider.on_changed(update)
    radio.on_clicked(select)
    save_button = Button(fig.add_axes([0.76, 0.24, 0.21, 0.06]), "Enregistrer poses")
    save_button.on_clicked(save_poses)
    fit_button = Button(fig.add_axes([0.76, 0.15, 0.21, 0.06]), "Recentrer")
    fit_button.on_clicked(fit)
    reset_button = Button(fig.add_axes([0.76, 0.06, 0.21, 0.06]), "Réinitialiser pose")
    reset_button.on_clicked(reset_pose)
    fig.canvas.mpl_connect("resize_event", fit)
    fit()
    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f"Image enregistrée : {args.save}")
    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
