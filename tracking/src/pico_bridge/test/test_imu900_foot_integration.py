from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).parents[1]


class Imu900FootIntegrationTest(unittest.TestCase):
    def test_driver_parameters_match_reference_profile(self):
        config_path = ROOT / "config/imu900_feet.yaml"
        self.assertTrue(config_path.exists(), f"missing integration config: {config_path}")
        config = yaml.safe_load(config_path.read_text())
        params = config["im900_foot_multi_node"]["ros__parameters"]
        self.assertEqual(
            params["channel_names"], ["im900/left_foot", "im900/right_foot"]
        )
        self.assertEqual(
            params["frame_ids"], ["left_foot_imu_link", "right_foot_imu_link"]
        )
        self.assertEqual(params["baudrate"], 115200)
        self.assertEqual(params["report_hz"], 110)
        self.assertEqual(params["report_tag"], 46)
        self.assertEqual(params["target_address"], 255)
        self.assertTrue(params["enable_compass"])
        self.assertTrue(params["use_device_timestamp"])
        self.assertTrue(params["use_quaternion_continuity"])
        self.assertFalse(params["clear_world_axes"])
        self.assertFalse(params["restore_world_axes"])

    def test_launch_defaults_and_remaps_are_explicit(self):
        path = ROOT / "launch/start_pico_foot_fusion.launch.py"
        self.assertTrue(path.exists(), f"missing integration launch: {path}")
        spec = spec_from_file_location("start_pico_foot_fusion", path)
        module = module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(module.DEFAULT_LEFT_PORT, "/dev/ttyUSB0")
        self.assertEqual(module.DEFAULT_RIGHT_PORT, "/dev/ttyUSB1")
        self.assertEqual(
            module.IMU_REMAPPINGS,
            [
                ("im900/left_foot/imuData_raw", "/imu/left_feet"),
                ("im900/right_foot/imuData_raw", "/imu/right_feet"),
                ("im900/left_foot/ready", "/imu/left_feet/ready"),
                ("im900/right_foot/ready", "/imu/right_feet/ready"),
            ],
        )



if __name__ == "__main__":
    unittest.main()
