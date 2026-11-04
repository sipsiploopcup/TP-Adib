"""Cas synthétiques de consolidation ; les cartes exportées utilisent les vrais scans."""

import csv
import json

import numpy as np
import pytest

from vectormap import VectorMap, canonical, find_intersections, main, trim_crossings


def test_duplicates_merge_and_keep_original_supports():
    mapping = VectorMap()
    mapping.add_segments([[[0, 0], [2, 0]], [[1, 0.03], [3, 0.03]]])
    assert len(mapping.segments) == 1
    assert len(mapping.supports[0]) == 4
    mapping.add_segments([[[0, 0], [2, 0]]])
    assert len(mapping.segments) == 1
    assert len(mapping.supports[0]) == 6
    assert mapping.segments[0][0, 0] == pytest.approx(0, abs=0.001)
    assert mapping.segments[0][1, 0] == pytest.approx(3, abs=0.001)


@pytest.mark.parametrize("other", [
    [[2.3, 0], [3.3, 0]],  # Ouverture supérieure à 20 cm.
    [[0, 0.2], [2, 0.2]],  # Mur parallèle distinct.
    [[1, -1], [1, 1]],    # Croisement perpendiculaire.
    [[0, 0], [2, 0.2]],   # Petit angle, mais divergence supérieure à 8 cm.
])
def test_incompatible_segments_stay_separate(other):
    mapping = VectorMap().add_segments([[[0, 0], [2, 0]], other])
    assert len(mapping.segments) == 2


def test_small_gap_merges_but_larger_gap_does_not():
    mapping = VectorMap().add_segments([[[0, 0], [1, 0]], [[1.1, 0], [2, 0]]])
    np.testing.assert_allclose(mapping.segments, [[[0, 0], [2, 0]]])


def test_batch_order_independence_and_no_parallel_chain_drift():
    segments = [np.array([[0, y], [2, y]]) for y in [0, 0.06, 0.12, 0.18]]
    first = VectorMap().add_segments(segments)
    second = VectorMap().add_segments(segments[::-1])
    np.testing.assert_allclose(first.segments, second.segments)
    assert len(first.segments) >= 2
    for support in first.supports:
        assert np.ptp(support[:, 1]) <= 0.08


def test_update_uses_scan_pose():
    local = np.column_stack((np.linspace(1, 2, 20), np.ones(20)))
    mapping = VectorMap()
    assert mapping.update(local) == 1
    assert mapping.update(local, (3, 2, 180)) == 1
    assert len(mapping.segments) == 1
    np.testing.assert_allclose(mapping.segments[0], [[1, 1], [2, 1]], atol=1e-10)


def test_intersections_and_optional_longest_side_trimming():
    segments = [np.array([[0, 0], [4, 0]]), np.array([[1, -1], [1, 1]])]
    crossings = find_intersections(segments)
    assert crossings == [{"segment_a": 1, "segment_b": 2, "x": 1.0, "y": 0.0}]
    trimmed = trim_crossings(segments)
    np.testing.assert_allclose(trimmed[0], [[1, 0], [4, 0]])
    np.testing.assert_allclose(trimmed[1], [[1, -1], [1, 0]])
    np.testing.assert_allclose(segments[0], [[0, 0], [4, 0]])


def test_empty_map_and_invalid_parameters():
    assert VectorMap().add_segments([]).segments == []
    for kwargs in ({"distance": 0}, {"gap": -1}, {"angle_deg": 90}, {"gap": float("nan")}):
        with pytest.raises(ValueError):
            VectorMap(**kwargs)
    with pytest.raises(ValueError):
        canonical([[0, 0], [0, 0]])


def test_cli_exports_common_frame_without_destroying_crossings(tmp_path, monkeypatch):
    import vectormap
    local = np.column_stack((np.linspace(1, 2, 20), np.ones(20)))
    monkeypatch.setattr(vectormap, "load_scan_groups", lambda *a, **kw: [
        (local, [0, 0, 0], {"scan_source": "a"}),
        (local, [3, 2, 180], {"scan_source": "b"}),
    ])
    prefix = tmp_path / "carte"
    output = main([str(tmp_path / "synthetic.xlsx"), "--output", str(prefix), "--no-show"])
    assert len(output) == 1
    metadata = json.loads(prefix.with_suffix(".json").read_text())
    assert metadata["input_segments"] == 2
    assert metadata["output_segments"] == 1
    assert metadata["support_segment_counts"] == [2]
    assert not metadata["trim_crossings"]
    with prefix.with_suffix(".csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert set(rows[0]) == {"x1", "y1", "x2", "y2"}
    assert prefix.with_suffix(".png").stat().st_size > 0
