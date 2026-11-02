"""Vérification sans fenêtre des commandes sur deux scans synthétiques."""

import json
import sys

import matplotlib

matplotlib.use("Agg")
import numpy as np

import viewer


def test_reset_selected_pose_and_center_view(tmp_path, monkeypatch):
    paths = [tmp_path / "a.csv", tmp_path / "b.csv"]
    for path in paths:
        path.write_text("dist_mm,x_lib_mm,y_lib_mm\n1000,1000,0\n2000,0,-2000\n")
    poses_path = tmp_path / "poses.json"
    poses_path.write_text(json.dumps({str(paths[0]): [1, 2, 30], str(paths[1]): [-1, 4, -20]}))
    saved_before = poses_path.read_text()
    widgets = {"Slider": [], "Button": [], "RadioButtons": []}
    for name in widgets:
        original = getattr(viewer, name)

        def capture(*args, _name=name, _original=original, **kwargs):
            instance = _original(*args, **kwargs)
            widgets[_name].append(instance)
            return instance

        monkeypatch.setattr(viewer, name, capture)
    monkeypatch.setattr(sys, "argv", ["viewer.py", *map(str, paths), "--poses", str(poses_path)])

    def exercise():
        fig = viewer.plt.gcf()
        ax = fig.axes[0]
        first = ax.collections[0].get_offsets().copy()
        second = ax.collections[2].get_offsets().copy()
        widgets["RadioButtons"][0].set_active(1)
        sliders = widgets["Slider"]
        sliders[0].set_val(10)
        sliders[2].set_val(80)
        assert not np.allclose(ax.collections[2].get_offsets(), second)
        np.testing.assert_allclose(ax.collections[0].get_offsets(), first)
        reset = next(b for b in widgets["Button"] if b.label.get_text() == "Réinitialiser pose")
        reset._observers.process("clicked", None)
        np.testing.assert_allclose(ax.collections[2].get_offsets(), second)
        np.testing.assert_allclose([s.val for s in sliders], [-1, 4, -20])
        assert poses_path.read_text() == saved_before
        fig.canvas.draw()
        all_points = np.concatenate([first, second, [[1, 2], [-1, 4]]])
        center = (all_points.min(axis=0) + all_points.max(axis=0)) / 2
        np.testing.assert_allclose([np.mean(ax.get_xlim()), np.mean(ax.get_ylim())], center)
        screen = ax.transData.transform([[0, 0], [1, 0], [0, 1]])
        np.testing.assert_allclose(np.linalg.norm(screen[1] - screen[0]),
                                   np.linalg.norm(screen[2] - screen[0]))
        assert abs(ax.get_position().width - 0.63) < 1e-6
        viewer.plt.close(fig)

    monkeypatch.setattr(viewer.plt, "show", exercise)
    viewer.main()
