"""实物数据手套骨架到 MediaPipe 21 点的适配合同。"""

from __future__ import annotations

import unittest
from pathlib import Path

import mujoco
import numpy as np

from data_glove_wuji_teleop.adapters.retargeting.glove_landmarks import (
    GloveLandmarkAdapter,
)
from data_glove_wuji_teleop.adapters.simulation.mujoco_dataglove import (
    load_dataglove_urdf,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
URDF_PATH = (
    PROJECT_ROOT
    / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"
)
EXPECTED_JOINT_BY_MEDIAPIPE_INDEX = {
    1: "thumb_cmc_abd_joint",
    2: "thumb_mcp_joint",
    3: "thumb_ip_joint",
    5: "index_mcp_flex_joint",
    6: "index_pip_joint",
    7: "index_dip_joint",
    9: "middle_mcp_flex_joint",
    10: "middle_pip_joint",
    11: "middle_dip_joint",
    13: "ring_mcp_flex_joint",
    14: "ring_pip_joint",
    15: "ring_dip_joint",
    17: "pink_mcp_flex_joint",
    18: "pink_pip_joint",
    19: "pink_dip_joint",
}
EXPECTED_TIP_JOINTS = {
    4: "thumb_ip_joint",
    8: "index_dip_joint",
    12: "middle_dip_joint",
    16: "ring_dip_joint",
    20: "pink_dip_joint",
}


class GloveLandmarkAdapterTest(unittest.TestCase):
    def test_rejects_degenerate_connected_finger_segment(self) -> None:
        model, data, _qpos = load_dataglove_urdf(URDF_PATH)
        mujoco.mj_forward(model, data)
        adapter = GloveLandmarkAdapter.from_model(model)
        mcp_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "index_mcp_flex_joint",
        )
        pip_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "index_pip_joint",
        )
        data.xanchor[pip_id] = data.xanchor[mcp_id]

        with self.assertRaisesRegex(ValueError, "index.*骨段退化"):
            adapter.to_mediapipe(data)

    def test_exports_wrist_last_three_joint_anchors_and_five_tips(self) -> None:
        model, data, qpos_by_joint = load_dataglove_urdf(URDF_PATH)
        for index, joint_name in enumerate(qpos_by_joint):
            data.qpos[qpos_by_joint[joint_name]] = 0.01 * (index + 1)
        mujoco.mj_forward(model, data)
        adapter = GloveLandmarkAdapter.from_model(model)

        points = adapter.to_mediapipe(data)

        self.assertEqual(points.shape, (21, 3))
        self.assertEqual(points.dtype, np.float64)
        self.assertTrue(np.isfinite(points).all())
        np.testing.assert_allclose(points[0], data.xpos[0], atol=1e-12)
        for point_index, joint_name in EXPECTED_JOINT_BY_MEDIAPIPE_INDEX.items():
            joint_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )
            np.testing.assert_allclose(
                points[point_index],
                data.xanchor[joint_id],
                atol=1e-12,
                err_msg=joint_name,
            )
        for tip_index, terminal_name in EXPECTED_TIP_JOINTS.items():
            terminal_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                terminal_name,
            )
            chain = next(
                chain
                for chain in adapter.skeleton.chains
                if chain.joint_ids[-1] == terminal_id
            )
            expected_tip = adapter.skeleton.fingertip_position(data, chain)
            np.testing.assert_allclose(
                points[tip_index],
                expected_tip,
                atol=1e-12,
                err_msg=terminal_name,
            )


if __name__ == "__main__":
    unittest.main()
