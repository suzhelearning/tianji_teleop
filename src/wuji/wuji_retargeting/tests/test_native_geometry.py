"""Native gradient correctness at mixed pinch weights and thumb masking."""
import numpy as np
import pytest

from wuji_retargeting._native import AdaptiveGeometry


def geometry_case():
    rng = np.random.default_rng(724)
    geometry = AdaptiveGeometry([0] * 5, [4, 8, 12, 16, 20],
                                [2, 6, 10, 14, 18], [3, 7, 11, 15, 19])
    positions = rng.normal(size=(21, 3)) * 5.0
    jacobians = rng.normal(size=(21, 3, 20))
    tips = rng.normal(size=(5, 3)) * 3.0
    directions = rng.normal(size=(5, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    full = rng.normal(size=(15, 3)) * 3.0
    alphas = np.array([0.4, 0.0, 1.0, 0.7, 0.2])
    return geometry, positions, jacobians, tips, directions, full, alphas


def test_gradient_matches_independent_central_difference_with_thumb_mask():
    geometry, positions, jacobians, *targets = geometry_case()
    step = 1e-5
    # Both branches matter: the masked thumb PIP must contribute neither cost
    # nor gradient while its remaining DIP/TIP terms use a divisor of two.
    for skip_thumb in (False, True):
        def evaluate(p):
            return geometry.loss_gradient(p, jacobians, *targets,
                                          2.0, 0.5, 1.3, 4.2, 0.8, skip_thumb)
        _, analytical = evaluate(positions)
        numerical = np.empty(20)
        for joint in range(20):
            change = step * jacobians[:, :, joint]
            numerical[joint] = (evaluate(positions + change)[0] -
                                evaluate(positions - change)[0]) / (2 * step)
        # Existing normalized-vector convention uses epsilon=1e-8; retain its
        # small analytical approximation rather than enabling fast-math.
        np.testing.assert_allclose(analytical, numerical, rtol=2e-6, atol=2e-7)


def test_native_rejects_out_of_bounds_model_topology_before_access():
    geometry, positions, jacobians, *targets = geometry_case()
    with pytest.raises(ValueError):
        geometry.loss_gradient(positions[:20].copy(), jacobians[:20].copy(),
                               *targets, 2.0, 0.5, 1.0, 1.0, 1.0, False)
