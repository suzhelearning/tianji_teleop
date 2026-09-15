"""官方 Wuji retargeting 外部适配器合同。"""

from __future__ import annotations

import io
import subprocess
import time
import unittest
from unittest.mock import Mock

import numpy as np

from data_glove_wuji_teleop.adapters.retargeting.wuji_official import (
    OfficialWujiRetargetAdapter,
    compute_tip_fit_metrics,
)
from data_glove_wuji_teleop.adapters.retargeting.wuji_process import (
    JsonLineProcessTransport,
)
from data_glove_wuji_teleop.adapters.retargeting.wuji_worker import (
    apply_pinch_d1_override,
    apply_pinch_alpha_max_override,
)


OFFICIAL_JOINT_NAMES = (
    "r_thumb_cmc_flex",
    "r_thumb_cmc_abd",
    "r_thumb_mcp",
    "r_thumb_ip",
    "r_index_finger_mcp_flex",
    "r_index_finger_mcp_abd",
    "r_index_finger_pip",
    "r_index_finger_dip",
    "r_middle_finger_mcp_flex",
    "r_middle_finger_mcp_abd",
    "r_middle_finger_pip",
    "r_middle_finger_dip",
    "r_ring_finger_mcp_flex",
    "r_ring_finger_mcp_abd",
    "r_ring_finger_pip",
    "r_ring_finger_dip",
    "r_pinky_mcp_flex",
    "r_pinky_mcp_abd",
    "r_pinky_pip",
    "r_pinky_dip",
)


class FakeTransport:
    def __init__(self) -> None:
        self.requests: list[dict[str, object]] = []

    def request(self, payload: dict[str, object]) -> dict[str, object]:
        self.requests.append(payload)
        transformed = np.zeros((21, 3), dtype=np.float64)
        robot_dips = np.zeros((5, 3), dtype=np.float64)
        robot_tips = np.zeros((5, 3), dtype=np.float64)
        for finger, (dip_index, tip_index) in enumerate(
            zip((3, 7, 11, 15, 19), (4, 8, 12, 16, 20))
        ):
            transformed[dip_index] = (0.01 * finger, 0.0, 0.05)
            transformed[tip_index] = (0.01 * finger, 0.0, 0.10)
            robot_dips[finger] = (0.01 * finger, 0.0, 0.05)
            robot_tips[finger] = (0.01 * finger, 0.0, 0.11)
        return {
            "qpos": [0.01 * index for index in range(20)],
            "joint_names": list(OFFICIAL_JOINT_NAMES),
            "transformed_keypoints": transformed.tolist(),
            "robot_wrist": [0.0, 0.0, 0.0],
            "robot_dip_positions": robot_dips.tolist(),
            "robot_tip_positions": robot_tips.tolist(),
            "cost": 1.25,
        }

    def close(self) -> None:
        return None


class OfficialWujiRetargetAdapterTest(unittest.TestCase):
    def test_overrides_official_pinch_alpha_max(self) -> None:
        class FakeOptimizer:
            d1 = np.full(4, 3.9)
            d2 = np.full(4, 4.0)
            MP_TIP_INDICES = (4, 8, 12, 16, 20)

            @staticmethod
            def _compute_pinch_alpha(_keypoints):
                return np.full(5, 0.7)

        untouched = FakeOptimizer()
        retargeter = type(
            "FakeRetargeter",
            (),
            {"optimizer": FakeOptimizer()},
        )()
        keypoints = np.zeros((21, 3), dtype=np.float64)
        keypoints[8] = (0.039, 0.0, 0.0)
        keypoints[12] = (0.040, 0.0, 0.0)
        keypoints[16] = (0.041, 0.0, 0.0)
        keypoints[20] = (0.042, 0.0, 0.0)

        apply_pinch_alpha_max_override(retargeter, 0.8)
        alphas = retargeter.optimizer._compute_pinch_alpha(keypoints)

        np.testing.assert_allclose(alphas, (0.8, 0.8, 0.0, 0.0, 0.0))
        np.testing.assert_allclose(
            untouched._compute_pinch_alpha(keypoints),
            np.full(5, 0.7),
        )
        # 满权重边界必须真正关闭全指形状项；不能因分母 epsilon 留下残余权重。
        apply_pinch_alpha_max_override(retargeter, 1.0)
        np.testing.assert_array_equal(
            retargeter.optimizer._compute_pinch_alpha(keypoints),
            (1.0, 1.0, 0.0, 0.0, 0.0),
        )


        retargeter.optimizer.d1 = np.array((3.9, 3.9, np.nan, 3.9))
        with self.assertRaisesRegex(ValueError, "递增有限阈值"):
            apply_pinch_alpha_max_override(retargeter, 0.8)
        retargeter.optimizer.d1 = np.full(4, 3.9)
        retargeter.optimizer.MP_TIP_INDICES = (4, 8, 12, 16, 16)
        with self.assertRaisesRegex(ValueError, "指尖索引无效"):
            apply_pinch_alpha_max_override(retargeter, 0.8)
    def test_overrides_all_pinch_d1_without_changing_d2(self) -> None:
        config = {
            "retarget": {
                "pinch_thresholds": {
                    finger: {"d1": 2.0, "d2": 4.0}
                    for finger in ("index", "middle", "ring", "pinky")
                }
            }
        }

        apply_pinch_d1_override(config, 3.0)

        thresholds = config["retarget"]["pinch_thresholds"]
        for finger in ("index", "middle", "ring", "pinky"):
            self.assertEqual(thresholds[finger], {"d1": 3.0, "d2": 4.0})
        with self.assertRaisesRegex(ValueError, "必须小于.*d2"):
            apply_pinch_d1_override(config, 4.0)

        invalid = {
            "retarget": {
                "pinch_thresholds": {
                    "middle": {"d2": "bad"},
                }
            }
        }
        with self.assertRaisesRegex(ValueError, "middle.d2"):
            apply_pinch_d1_override(invalid, 3.0)
        self.assertEqual(
            invalid,
            {
                "retarget": {
                    "pinch_thresholds": {
                        "middle": {"d2": "bad"},
                    }
                }
            },
        )

    def test_eof_closes_worker_input_and_reaps_process(self) -> None:
        class EofProcess:
            stdin = io.StringIO()
            stdout = io.StringIO("")
            waited = False

            @staticmethod
            def poll():
                return None

            def wait(self, timeout=None):
                self.waited = True
                return 0

            @staticmethod
            def terminate():
                raise AssertionError("clean EOF 不应 terminate")

            @staticmethod
            def kill():
                raise AssertionError("clean EOF 不应 kill")

        process = EofProcess()
        transport = JsonLineProcessTransport(process)

        with self.assertRaisesRegex(EOFError, "未返回结果"):
            transport.request({"keypoints": []})

        self.assertTrue(process.stdin.closed)
        self.assertTrue(process.waited)

    def test_rejects_duplicate_or_missing_official_joint_names(self) -> None:
        class BadNameTransport(FakeTransport):
            def __init__(self, names):
                super().__init__()
                self.names = names

            def request(self, payload):
                response = super().request(payload)
                response["joint_names"] = list(self.names)
                return response

        keypoints = np.zeros((21, 3), dtype=np.float64)
        invalid_names = (
            OFFICIAL_JOINT_NAMES[:-1],
            OFFICIAL_JOINT_NAMES[:-1] + (OFFICIAL_JOINT_NAMES[0],),
        )
        for names in invalid_names:
            with self.subTest(names=names):
                adapter = OfficialWujiRetargetAdapter(BadNameTransport(names))
                with self.assertRaisesRegex(ValueError, "20 个唯一关节名"):
                    adapter.retarget(keypoints, apply_filter=False)

    def test_initialization_budget_does_not_relax_frame_deadline(self) -> None:
        class DelayedStdout:
            def __init__(self):
                self.lines = iter((
                    (0.1, '{"ready":true}\n'),
                    (0.0, '{"frame":1}\n'),
                    (0.1, '{"frame":2}\n'),
                ))

            def readline(self):
                delay, line = next(self.lines)
                time.sleep(delay)
                return line

        process = Mock(stdin=io.StringIO(), stdout=DelayedStdout())
        process.poll.return_value = None
        process.wait.return_value = 0
        transport = JsonLineProcessTransport(process, response_timeout=0.02)
        try:
            transport.wait_ready(timeout=1.0)
            self.assertEqual(transport.request({"keypoints": []}), {"frame": 1})
            with self.assertRaises(TimeoutError):
                transport.request({"keypoints": []})
        finally:
            transport.close()

    def test_invalid_initialization_closes_transport_before_control(self) -> None:
        process = Mock(stdin=io.StringIO(), stdout=io.StringIO('{"frame":1}\n'))
        process.poll.return_value = None
        process.wait.return_value = 0
        transport = JsonLineProcessTransport(process)
        try:
            with self.assertRaises(RuntimeError):
                transport.wait_ready()
            with self.assertRaisesRegex(RuntimeError, "已关闭"):
                transport.request({"keypoints": []})
        finally:
            transport.close()

    def test_process_timeout_escalates_from_terminate_to_kill(self) -> None:
        class SlowStdout:
            @staticmethod
            def readline():
                time.sleep(0.1)
                return ""

        class StuckProcess:
            stdin = io.StringIO()
            stdout = SlowStdout()
            terminated = False
            killed = False

            @staticmethod
            def poll():
                return None

            def wait(self, timeout=None):
                if self.killed:
                    return -9
                raise subprocess.TimeoutExpired("worker", timeout)

            def terminate(self):
                self.terminated = True

            def kill(self):
                self.killed = True

        process = StuckProcess()
        transport = JsonLineProcessTransport(
            process,
            response_timeout=0.01,
            shutdown_timeout=0.01,
        )

        with self.assertRaisesRegex(TimeoutError, "响应超时"):
            transport.request({"keypoints": []})

        self.assertTrue(process.terminated)
        self.assertTrue(process.killed)


    def test_process_transport_sends_and_receives_one_json_line(self) -> None:
        class FakeProcess:
            stdin = io.StringIO()
            stdout = io.StringIO('{"qpos":[1,2,3]}\n')

            @staticmethod
            def poll():
                return None

        process = FakeProcess()
        transport = JsonLineProcessTransport(process)

        response = transport.request({"keypoints": [[0.0, 0.0, 0.0]]})

        self.assertEqual(response, {"qpos": [1, 2, 3]})
        self.assertEqual(
            process.stdin.getvalue(),
            '{"keypoints":[[0.0,0.0,0.0]]}\n',
        )

    def test_returns_named_joint_target_and_transformed_keypoints(self) -> None:
        transport = FakeTransport()
        adapter = OfficialWujiRetargetAdapter(transport)
        keypoints = np.arange(63, dtype=np.float64).reshape(21, 3) / 1000.0

        result = adapter.retarget(keypoints, apply_filter=False)

        self.assertEqual(result.joint_names, OFFICIAL_JOINT_NAMES)
        self.assertEqual(result.qpos.shape, (20,))
        self.assertEqual(result.transformed_keypoints.shape, (21, 3))
        self.assertAlmostEqual(result.cost, 1.25)
        metrics = compute_tip_fit_metrics(result)
        np.testing.assert_allclose(metrics.position_error_m, [0.01] * 5)
        np.testing.assert_allclose(metrics.direction_error_deg, [0.0] * 5)
        self.assertEqual(
            transport.requests,
            [{"keypoints": keypoints.tolist(), "apply_filter": False}],
        )


if __name__ == "__main__":
    unittest.main()
