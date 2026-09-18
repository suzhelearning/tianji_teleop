from contextlib import contextmanager
import mujoco
from pico2_hands.viewer_controls import TeleopDisplayPolicy


class FakeViewer:
    def __init__(self):
        self.opt = mujoco.MjvOption()
        self.user_scn = mujoco.MjvScene()
        self.locked = False
        self.calls = 0
        self.pending = None

    @contextmanager
    def lock(self):
        self.locked = True
        try:
            yield
        finally:
            self.locked = False

    def sync(self):
        assert not self.locked
        self.calls += 1
        if self.pending:
            self.pending()
            self.pending = None


def test_ui_shortcuts_restored_after_sync_without_touching_other_flags():
    viewer = FakeViewer()
    policy = TeleopDisplayPolicy(viewer)
    policy.sync()
    other = int(mujoco.mjtVisFlag.mjVIS_JOINT)
    viewer.opt.flags[other] = 1
    def shortcut_changes():
        for i,v in policy.visual.items():
            viewer.opt.flags[i] = 1-v
        for i,v in policy.render.items():
            viewer.user_scn.flags[i] = 1-v
    viewer.pending = shortcut_changes
    before = viewer.calls
    policy.sync()
    assert viewer.calls == before+2
    assert all(viewer.opt.flags[i] == v for i,v in policy.visual.items())
    assert all(viewer.user_scn.flags[i] == v for i,v in policy.render.items())
    assert viewer.opt.flags[other] == 1
    before = viewer.calls
    policy.sync()
    assert viewer.calls == before+1
