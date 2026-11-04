"""Vérifie la couverture des paires et l'absence de pose physique inventée."""

import numpy as np
import pytest

from localisation_paires import match_pairs


def test_all_pairs_are_processed_with_shared_estimated_initial_pose(tmp_path, monkeypatch):
    import localisation_paires
    calls = []
    points = np.column_stack((np.linspace(1, 3, 30), np.ones(30)))
    monkeypatch.setattr(localisation_paires, "load_measurements",
                        lambda path: (points.copy(), {"source": str(path)}))
    monkeypatch.setattr(localisation_paires, "register",
                        lambda query, ref: {"pose": [0.2, -0.1, 5], "ambiguous": True,
                                            "alternate": {"pose": [1, 2, 95]}})

    class Matcher:
        def __init__(self, reference):
            pass

        def match(self, query, initial):
            calls.append((query.copy(), list(initial)))
            return {"pose": list(initial), "initial_pose": list(initial), "overlap_20cm": 0.8,
                    "rmse_distance_map_m": 0.1, "runtime_s": 0.001}

    monkeypatch.setattr(localisation_paires, "ScanMatcher", Matcher)
    monkeypatch.setattr(localisation_paires, "ICPMatcher", Matcher)
    paths = [tmp_path / f"scan{i}.xlsx" for i in range(3)]
    prefix = tmp_path / "pairs"
    pairs = match_pairs(paths, prefix)
    assert len(pairs) == 3
    assert len(calls) == 6
    assert [(p["reference"]["source"], p["query"]["source"]) for p in pairs] == [
        (str(paths[0]), str(paths[1])), (str(paths[0]), str(paths[2])), (str(paths[1]), str(paths[2]))]
    for i in range(0, 6, 2):
        np.testing.assert_array_equal(calls[i][0], calls[i + 1][0])
        assert calls[i][1] == calls[i + 1][1] == [0.2, -0.1, 5]
    assert all(p["translation_error_m"] is None and not p["physical_ground_truth"] for p in pairs)
    assert all(p["bootstrap"]["ambiguous"] for p in pairs)
    assert prefix.with_suffix(".png").exists()
    assert prefix.with_suffix(".csv").exists()


def test_duplicate_input_files_are_rejected(tmp_path):
    path = tmp_path / "same.xlsx"
    with pytest.raises(ValueError):
        match_pairs([path, path], tmp_path / "output")
