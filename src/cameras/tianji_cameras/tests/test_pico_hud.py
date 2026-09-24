import unittest
from types import SimpleNamespace
from unittest.mock import patch

from tianji_cameras.pico_hud import ExecutorHud, ExecutorHudState, STATE_TIMEOUT_NS
from tianji_runtime.constants import IMAGE_HEIGHT, IMAGE_WIDTH


def state(*, boot="boot-a", session="session-a", sequence=1, published=1_000_000_000,
          detail="已到初始位置\n按 r 准备", faulted=False):
    return SimpleNamespace(boot_id=boot, session_id=session, sequence=sequence,
                           published_monotonic_ns=published, mode="real",
                           phase="HOME_REACHED", detail=detail, faulted=faulted)


class HudStateTest(unittest.TestCase):
    def test_old_publication_and_duplicate_receipts_cannot_keep_ready_visible(self):
        hud = ExecutorHudState()
        source = 1_000_000_000
        message = state(published=source)
        hud.receive(message, now_ns=source + STATE_TIMEOUT_NS - 1)
        self.assertEqual(hud.snapshot(now_ns=source + STATE_TIMEOUT_NS).detail, message.detail)
        # A retransmission is not a new executor heartbeat, even when received now.
        hud.receive(message, now_ns=source + STATE_TIMEOUT_NS)
        expired = hud.snapshot(now_ns=source + STATE_TIMEOUT_NS + 1)
        self.assertTrue(expired.alert)
        self.assertNotEqual(expired.detail, message.detail)

    def test_restart_and_session_change_replace_state_without_accepting_old_identity(self):
        hud = ExecutorHudState()
        source = 1_000_000_000
        hud.receive(state(sequence=50), now_ns=source)
        hud.receive(state(session="session-b", sequence=1,
                          published=source + 1, detail="新控制器正在检查"), now_ns=source + 1)
        hud.receive(state(sequence=51, published=source + 2), now_ns=source + 2)
        self.assertEqual(hud.snapshot(now_ns=source + 2).detail, "新控制器正在检查")
        # Restarting the executor keeps host boot_id but resets its sequence.
        hud.receive(state(session="session-c", sequence=1,
                          published=source + 3, detail="新会话正在准备"), now_ns=source + 3)
        hud.receive(state(session="session-b", sequence=100,
                          published=source + 4), now_ns=source + 4)
        self.assertEqual(hud.snapshot(now_ns=source + 4).detail, "新会话正在准备")

    def test_future_invalid_identity_and_out_of_order_samples_do_not_renew_state(self):
        hud = ExecutorHudState()
        source = 1_000_000_000
        hud.receive(state(sequence=3), now_ns=source)
        for message in (state(sequence=4, published=source + 2),
                        state(boot="", sequence=4, published=source + 1),
                        state(session="", sequence=4, published=source + 1),
                        state(sequence=2, published=source + 1)):
            hud.receive(message, now_ns=source + 1)
        self.assertTrue(hud.snapshot(now_ns=source + STATE_TIMEOUT_NS + 1).alert)


class HudIsolationTest(unittest.TestCase):
    def test_render_changes_only_video_copy_and_expiration_replaces_ready_banner(self):
        hud = ExecutorHud()
        raw = bytes((25, 75, 125)) * (IMAGE_WIDTH * IMAGE_HEIGHT)
        untouched = bytes(bytearray(raw))
        source = 1_000_000_000
        hud.state.receive(state(), now_ns=source)
        with patch("tianji_cameras.pico_hud.time.monotonic_ns", return_value=source):
            ready_video = hud.render(raw)
        with patch("tianji_cameras.pico_hud.time.monotonic_ns",
                   return_value=source + STATE_TIMEOUT_NS + 1):
            lost_video = hud.render(raw)
        self.assertEqual(raw, untouched)
        self.assertNotEqual(ready_video, raw)
        self.assertNotEqual(lost_video, ready_video)
        # The HUD is at the top; the lower scene is preserved byte-for-byte.
        lower = IMAGE_WIDTH * (IMAGE_HEIGHT // 2) * 3
        self.assertEqual(ready_video[lower:], raw[lower:])
        self.assertEqual(lost_video[lower:], raw[lower:])
