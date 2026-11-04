"""Données synthétiques uniquement : contrôle de la transformation estimée."""

import numpy as np

from geometry import apply_pose
from merge_scans import register, rigid_fit


def test_rigid_fit_preserves_orientation():
    source = np.array([[0, 0], [2, 0], [1, 3], [0, 1]], dtype=float)
    target = apply_pose(source, 1.5, -0.8, 90)
    rotation, translation = rigid_fit(source, target)
    assert np.linalg.det(rotation) > 0
    np.testing.assert_allclose(source @ rotation.T + translation, target, atol=1e-10)


def test_multistart_registration_with_translation_and_rotation():
    rng = np.random.default_rng(42)
    source = rng.uniform([-3, -1], [4, 2], (200, 2))
    target = apply_pose(source, 1.1, -0.7, 90)
    result = register(source, target)
    np.testing.assert_allclose(result["pose"], [1.1, -0.7, 90], atol=0.03)
    assert result["overlap_20cm"] > 0.95


def test_rectangle_symmetry_is_reported():
    x = np.linspace(-3, 3, 40)
    y = np.linspace(-2, 2, 30)
    rectangle = np.vstack([np.column_stack([x, np.full_like(x, -2)]),
                           np.column_stack([x, np.full_like(x, 2)]),
                           np.column_stack([np.full_like(y, -3), y]),
                           np.column_stack([np.full_like(y, 3), y])])
    result = register(rectangle, rectangle)
    assert result["ambiguous"]
    assert result["review_required"]
