from contextlib import contextmanager
import mujoco
from pico2_hands.viewer_controls import TeleopDisplayPolicy
from pico2_hands.viewer_controls import suppress_default_home_shortcut, _visual_shortcut_table
import ctypes
import pytest


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
    policy.sync()
    assert all(viewer.opt.flags[i] == v for i,v in policy.visual.items())
    assert all(viewer.user_scn.flags[i] == v for i,v in policy.render.items())
    assert viewer.opt.flags[other] == 1


def test_h_shortcut_removed_before_ui_construction_and_restored():
    library, table = _visual_shortcut_table()
    index = int(mujoco.mjtVisFlag.mjVIS_CONVEXHULL)
    before = [tuple(row) for row in table]
    with suppress_default_home_shortcut():
        assert ctypes.string_at(table[index][2]) == b""
        for i, row in enumerate(table):
            for j in range(3):
                if (i, j) != (index, 2):
                    assert row[j] == before[i][j]
        # Nested owners must not restore H while an outer viewer is active.
        with suppress_default_home_shortcut():
            assert ctypes.string_at(table[index][2]) == b""
        assert ctypes.string_at(table[index][2]) == b""
    assert [tuple(row) for row in table] == before


def test_h_shortcut_restored_when_viewer_creation_fails():
    library, table = _visual_shortcut_table()
    index = int(mujoco.mjtVisFlag.mjVIS_CONVEXHULL)
    before = table[index][2]
    with pytest.raises(RuntimeError, match="fake viewer failure"):
        with suppress_default_home_shortcut():
            raise RuntimeError("fake viewer failure")
    assert table[index][2] == before


def test_unreviewed_mujoco_version_fails_closed(monkeypatch):
    monkeypatch.setattr(mujoco, "__version__", "future")
    with pytest.raises(RuntimeError, match="reviewed MuJoCo"):
        with suppress_default_home_shortcut():
            pass
