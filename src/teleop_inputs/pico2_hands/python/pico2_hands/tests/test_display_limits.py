import numpy as np
import pytest

from pico2_hands.display_contract import configure_pico2_limits, validate_display
from pico2_hands.resources import PACKAGE, display_model_path
from simulation.direct_state import DirectStateSimulation


def make_sim():
    return DirectStateSimulation(display_model_path(),
                                 PACKAGE / "config/simulation.yaml")


def test_pico2_j3_ranges_and_cross_zero_command():
    sim = make_sim()
    with pytest.raises(ValueError, match="IK limits exceed"):
        validate_display(sim)
    old_ranges = sim._ranges.copy()
    configure_pico2_limits(sim)
    validate_display(sim)
    for side, index in (("L", 2), ("R", 9)):
        np.testing.assert_array_equal(sim._ranges[index], [-3.1, 3.1])
        np.testing.assert_array_equal(sim.model.joint("Joint3_" + side).range, [-3.1, 3.1])
        np.testing.assert_array_equal(sim.model.actuator("sim_Joint3_" + side).ctrlrange, [-3.1, 3.1])
    q = sim.targets[:14].copy()
    for left, right in ((.0023132363671147776, -.0023132363671147776), (3.1, -3.1), (-3.1, 3.1)):
        q[2], q[9] = left, right
        sim.set_simulation_arm_targets(q)
        sim.step()
        np.testing.assert_allclose(sim.data.qpos[sim.qpos_indices[:14]], q)
    q[2] = 3.11
    with pytest.raises(ValueError, match="Joint3_L"):
        sim.set_simulation_arm_targets(q)
    keep = [i for i in range(54) if i not in (2, 9)]
    np.testing.assert_array_equal(sim._ranges[keep], old_ranges[keep])
    # Independent legacy instances still retain the original one-sided J3 bounds.
    legacy = make_sim()
    np.testing.assert_array_equal(legacy._ranges, old_ranges)
