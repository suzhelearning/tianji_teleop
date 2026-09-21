"""Simulation-only joint-state display, NOT a physical dynamics executor.

Retains the same READY/limit validation as dynamics. Positions are assigned then
forward kinematics runs; no force integration, contact response or gravity sag.
"""
import mujoco
import numpy as np
from .physics import JOINT_NAMES, PhysicsSimulation


class DirectStateSimulation(PhysicsSimulation):
    mode = 'direct'

    def __init__(self, model_path, controller_config, *, object_mesh=None):
        super().__init__(model_path, controller_config, object_mesh=object_mesh)
        self._velocity_indices = np.array([
            self.model.joint(name).dofadr[0]
            for name in JOINT_NAMES
        ], dtype=np.intp)

    def step(self):
        self._check_state()
        dt = float(self.model.opt.timestep)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('direct display timestep must be positive and finite')
        previous = self.data.qpos[self.qpos_indices].copy()
        self.data.qpos[self.qpos_indices] = self.targets
        self.data.qvel[self._velocity_indices] = (self.targets - previous) / dt
        self.data.ctrl[:] = self.targets
        # Display clock only. Do not present this as physical integration time.
        self.data.time += dt
        try:
            mujoco.mj_forward(self.model, self.data)
        except (mujoco.FatalError, ValueError) as error:
            self._fault = 'MuJoCo direct display failed: ' + str(error)
            raise RuntimeError(self._fault) from error
        self._check_state()
