"""实物数据手套 URDF 模型加载与关节帧应用合同。"""

from __future__ import annotations

import unittest
from pathlib import Path

import mujoco

from data_glove_wuji_teleop.adapters.glove.encoder_kinematics import (
    EncoderKinematics,
    GloveJointFrame,
)
from data_glove_wuji_teleop.adapters.glove.encoder_stream import EncoderFrame
from data_glove_wuji_teleop.adapters.simulation.mujoco_dataglove import (
    apply_urdf_joint_frame,
    convert_encoder_frame_for_urdf,
    load_dataglove_urdf,
)
from data_glove_wuji_teleop.profiles.dataglove.urdf_mapping import (
    DataGloveUrdfMapping,
)
from data_glove_wuji_teleop.profiles.dataglove.urdf_zero import UrdfZeroProfile


PROJECT_ROOT = Path(__file__).resolve().parents[2]
URDF_PATH = (
    PROJECT_ROOT
    / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"
)
MAPPING_PATH = PROJECT_ROOT / "config/dataglove/urdf/right.json"


class DataGloveUrdfSimulationTest(unittest.TestCase):


    def test_software_zero_is_applied_before_four_bar_conversion(self) -> None:
        profile = UrdfZeroProfile.from_dict(
            {
                "version": 1,
                "hand": "right",
                "channels": 21,
                "angle_unit": "degree",
                "stream_zeroed": False,
                "stream_range_deg": [0.0, 360.0],
                "expected_cs_by_joint": list(range(21)),
                "zero_offsets_deg": {
                    f"J{index}": 100.0 for index in range(1, 22)
                },
                "captured_groups": [
                    "thumb",
                    "index",
                    "middle",
                    "ring",
                    "pinky",
                    "pinky_cmc",
                ],
            }
        )
        kinematics = EncoderKinematics.load(MAPPING_PATH, commissioning=True)

        converted = convert_encoder_frame_for_urdf(
            EncoderFrame(1, 2, 0, [100.0] * 21),
            kinematics,
            zero_profile=profile,
        )

        self.assertEqual(converted.joint_angles_deg, (0.0,) * 21)

    def test_applies_all_21_physical_joints_to_their_named_urdf_joints(
        self,
    ) -> None:
        mapping = DataGloveUrdfMapping.load(MAPPING_PATH)
        mapped = mapping.map_frame(
            GloveJointFrame(
                sequence=1,
                timestamp_ns=2,
                dropped=0,
                joint_angles_deg=tuple(
                    float(index) for index in range(1, 22)
                ),
            )
        )
        model, data, qpos_by_joint = load_dataglove_urdf(URDF_PATH)

        apply_urdf_joint_frame(model, data, qpos_by_joint, mapped)

        self.assertEqual(model.nq, 21)
        self.assertEqual(len(qpos_by_joint), 21)
        self.assertEqual(set(qpos_by_joint), set(mapped.joint_names))
        for joint_name, expected_position in zip(
            mapped.joint_names,
            mapped.positions_rad,
        ):
            self.assertAlmostEqual(
                data.qpos[qpos_by_joint[joint_name]],
                expected_position,
                msg=joint_name,
            )
        pinky_cmc_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "pinky_cmc_joint",
        )
        self.assertGreaterEqual(pinky_cmc_id, 0)
        self.assertAlmostEqual(
            data.qpos[model.jnt_qposadr[pinky_cmc_id]],
            mapped.positions_rad[mapped.joint_names.index("pinky_cmc_joint")],
        )


if __name__ == "__main__":
    unittest.main()
