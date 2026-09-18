"""Validate the viewer's flange TCP against the independent V131 model."""
from pathlib import Path
import mujoco
import numpy as np


def configure_pico2_limits(simulation):
    """Simulation-only J3 override; shared XML and other instances stay unchanged.

    PhysicsSimulation caches ranges in TJRC order, so update that validator as
    well as MuJoCo joint/actuator ranges before any commands are processed.
    """
    for side, index in (("L", 2), ("R", 9)):
        name = "Joint3_" + side
        simulation.model.joint(name).range[:] = (-3.1, 3.1)
        simulation.model.actuator("sim_" + name).ctrlrange[:] = (-3.1, 3.1)
        simulation._ranges[index] = (-3.1, 3.1)


def validate_display(simulation):
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parent /
        "native/models/marvin_m6_qp_pico_fast_kinematics.xml"))
    source = mujoco.MjData(model)
    for side in ("L", "R"):
        for i in range(1, 8):
            name = f"Joint{i}_{side}"
            lower, upper = model.joint(name).range
            bounds = simulation.model.joint(name).range
            if lower < bounds[0] or upper > bounds[1]:
                raise ValueError(f"PICO2 IK limits exceed display limits: {name}")
    display = mujoco.MjData(simulation.model)
    display.qpos[:] = simulation.data.qpos
    home = simulation.targets[:14]
    for probe in (np.zeros(14), .01*np.sin(np.arange(14))):
        for j, name in enumerate(f"Joint{i}_{s}" for s in ("L", "R") for i in range(1, 8)):
            source.qpos[model.joint(name).qposadr[0]] = home[j]+probe[j]
            display.qpos[simulation.model.joint(name).qposadr[0]] = home[j]+probe[j]
        mujoco.mj_forward(model, source)
        mujoco.mj_forward(simulation.model, display)
        for side in ("L", "R"):
            a = model.site("tcp_"+side).id
            b = simulation.model.site("flange_tcp_"+side).id
            if (not np.allclose(source.site_xpos[a], display.site_xpos[b], rtol=0, atol=1e-5) or
                    not np.allclose(source.site_xmat[a], display.site_xmat[b], rtol=0, atol=1e-5)):
                raise ValueError("PICO2 display flange TCP differs from V131 model")
