"""Validate the displayed TCP against the selected offline IK model."""
import mujoco
import numpy as np


def validate_dls_display(simulation, ik):
    """Compare the actual worker palm TCP with the display, never flange TCP."""
    data = mujoco.MjData(simulation.model)
    for delta in (0., .01):
        q = simulation.targets[:14].reshape(2,7) + delta*np.sin(np.arange(14)).reshape(2,7)
        expected = ik.forward(q)
        data.qpos[:] = simulation.data.qpos
        for i,side in enumerate(("L","R")):
            for j in range(7):
                data.qpos[simulation.model.joint(f"Joint{j+1}_{side}").qposadr[0]]=q[i,j]
        mujoco.mj_forward(simulation.model,data)
        from scipy.spatial.transform import Rotation
        for name,side in (("left","L"),("right","R")):
            site=simulation.model.site("tcp_"+side).id
            pose=expected[name]["achieved_pose"]
            if (not np.allclose(data.site_xpos[site],pose[:3],atol=1e-5,rtol=0) or
                    not np.allclose(data.site_xmat[site].reshape(3,3),Rotation.from_quat(pose[3:]).as_matrix(),atol=1e-5,rtol=0)):
                raise ValueError("DLS display palm TCP mismatch")


