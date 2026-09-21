"""Bare-hand mapping and optional C calibration; no actuator authority.

The session owner must independently enforce idle/Home, input identity and
freshness, then start authorization. calibration_allows_start is only one gate.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

from .reference.height_calibration import HeightCalibration, horizontal_reference
from .reference.pose_mapping import create_arm_pose_mapper

ROOT = Path(__file__).resolve().parent


class OptionalHeightMapping:
    def __init__(self, config=None, *, urdf=None):
        if config is None:
            with (ROOT / "config/hand_tracking_target.yaml").open() as stream:
                config = yaml.safe_load(stream)["head_palm_direct_mapper_config"]
        self.mapper = create_arm_pose_mapper("head_palm_direct", config)
        self.calibration = HeightCalibration(("left", "right"))
        self.urdf = urdf
        self.reference = None

    @property
    def calibration_allows_start(self):
        # Exact source behavior: never requested C permits fixed mapping.
        # A failed first request must be retried, not silently bypassed.
        return (self.calibration.state != "collecting" and
                not (self.calibration.state == "failed" and
                     self.calibration.means is None))

    def request_calibration(self, now_ns, *, idle):
        if not idle or self.calibration.state == "collecting":
            return False
        try:
            self.reference = horizontal_reference(self.urdf)
        except (OSError, ValueError, ET.ParseError) as exc:
            self.calibration.fail(str(exc))
            return False
        self.calibration.begin(now_ns)
        return True

    def add(self, observation, now_ns):
        # Validate frame conventions using the same mapper before sampling.
        try:
            self.mapper.map(observation)
        except (ValueError, TypeError):
            if self.calibration.state == "collecting":
                self.calibration.fail("invalid frame convention during calibration")
            raise
        self.calibration.add(observation, now_ns)

    def tick(self, now_ns):
        means = self.calibration.tick(now_ns)
        if means is None:
            return False
        try:
            self.mapper.calibrate_height(means, self.reference)
        except (ValueError, TypeError) as exc:
            self.calibration.fail(str(exc))
            return False
        self.calibration.accept(means)
        return True

    def map(self, observation):
        return self.mapper.map(observation)

    def status(self):
        return dict(self.calibration.status(),
                    calibration_required=False,
                    calibration_allows_start=self.calibration_allows_start,
                    motion_authorized=False)
