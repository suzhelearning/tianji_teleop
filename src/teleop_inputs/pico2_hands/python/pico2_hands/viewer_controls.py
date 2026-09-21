"""Keep MuJoCo display shortcuts from competing with teleoperation keys.

The H shortcut is removed from the exported visual table before passive-viewer
UI construction. The remaining display flags are restored after sync. Never
call sync while holding the viewer lock.
"""
import mujoco
import ctypes
from contextlib import contextmanager
import threading


_shortcut_lock = threading.RLock()
# The native viewer closes asynchronously. Retain the replacement string for
# process lifetime, even after restoring the slot, so an in-flight UI rebuild
# can never dereference a freed temporary buffer.
_no_shortcut = ctypes.create_string_buffer(b"")


def _visual_shortcut_table():
    """Access the documented C global in the EXACT library Python is using.

    No file patching, GLFW window discovery, private Simulate memory offsets or
    mutation of string literals. Only an exported const-char* slot is replaced.
    MuJoCo 3.10 MakeRenderingSection copies this slot into each UI shortcut.
    Guard the pinned ABI; a future version must be reviewed, not silently used.
    """
    if mujoco.mj_versionString() != "3.10.0" or mujoco.__version__ != "3.10.0":
        raise RuntimeError("PICO2 H shortcut suppression requires reviewed MuJoCo 3.10.0")
    library = ctypes.CDLL(mujoco._functions.__file__)
    table_type = (ctypes.c_void_p * 3) * int(mujoco.mjtVisFlag.mjNVISFLAG)
    table = table_type.in_dll(library, "mjVISSTRING")
    return library, table


@contextmanager
def suppress_default_home_shortcut():
    """Reserve H for teleop before creating this process's sole passive viewer.

    Keep the empty shortcut alive through viewer lifetime/model UI rebuilds.
    Restore the original pointer on exit, including failed viewer creation.
    Other PICO/VR processes and on-disk SDK files are unaffected.
    """
    with _shortcut_lock:
        library, table = _visual_shortcut_table()
        index = int(mujoco.mjtVisFlag.mjVIS_CONVEXHULL)
        row = table[index]
        if ctypes.string_at(row[0]) != b"Convex Hull" or ctypes.string_at(row[2]) not in (b"H", b""):
            raise RuntimeError("Unexpected MuJoCo convex-hull shortcut contract")
        original = row[2]
        replacement = ctypes.addressof(_no_shortcut)
        row[2] = replacement
        try:
            yield
        finally:
            if row[2] == replacement:
                row[2] = original
            # Keep the loaded library alive until after restoration.
            del library


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
