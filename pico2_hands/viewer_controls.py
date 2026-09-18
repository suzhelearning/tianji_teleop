"""Keep passive-viewer keyboard shortcuts from persisting display changes.

The public passive viewer callback does not consume MuJoCo shortcuts. Sync
imports UI changes first, then explicitly restores the reserved-key flags and
syncs again only if needed. Never call sync while holding the viewer lock.
"""
import mujoco


class TeleopDisplayPolicy:
    def __init__(self, viewer):
        self.viewer = viewer
        self.visual = {
            int(mujoco.mjtVisFlag.mjVIS_CONVEXHULL): 0,
            int(mujoco.mjtVisFlag.mjVIS_CONTACTPOINT): 0,
        }
        self.render = {
            int(mujoco.mjtRndFlag.mjRND_SHADOW): 1,
            int(mujoco.mjtRndFlag.mjRND_REFLECTION): 1,
        }

    def sync(self):
        viewer = self.viewer
        viewer.sync()
        changed = False
        with viewer.lock():
            for index, value in self.visual.items():
                changed |= viewer.opt.flags[index] != value
                viewer.opt.flags[index] = value
            if viewer.user_scn is not None:
                for index, value in self.render.items():
                    changed |= viewer.user_scn.flags[index] != value
                    viewer.user_scn.flags[index] = value
        if changed:
            viewer.sync()
