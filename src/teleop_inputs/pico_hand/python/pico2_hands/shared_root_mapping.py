"""Height-template root calibration and bilateral shared-root palm mapping.

No measured shoulders/elbows are claimed. Calibration establishes a gravity-level
heading and a root offset relative to head POSITION. Subsequent head ROTATION does
not rotate the root. Body turns require another stationary calibration.
"""
from pathlib import Path
import math
import hashlib
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

SIDES = ("left", "right")
from tianji_runtime.resources import controller_profile
from .resources import display_model_path


class SharedRootMapping:
    def __init__(self, height_m, forward, *, calibration_key="C"):
        if isinstance(height_m, bool) or not isinstance(height_m, (int, float)) or not math.isfinite(height_m) or not 1 <= height_m <= 2.4:
            raise ValueError("shared-root requires height in metres within [1.0, 2.4]")
        self.height = float(height_m)
        self.calibration_key = calibration_key
        self.width = self.height * .1828
        self.upper = self.height * .155882
        self.forearm = self.height * .152941
        self.palm_distance = self.height * .037037
        self.reach = self.upper + self.forearm + self.palm_distance
        artifact = controller_profile("shared_root_robot_geometry_ceres.yaml")
        geometry = yaml.safe_load(artifact.read_text())["robot_geometry"]
        self.geometry_sha256 = hashlib.sha256(artifact.read_bytes()).hexdigest()
        for path_key, hash_key in (("urdf_path", "urdf_sha256"), ("mujoco_xml_path", "mujoco_xml_sha256")):
            if hashlib.sha256(display_model_path(Path(geometry[path_key]).name).read_bytes()).hexdigest() != geometry[hash_key]:
                raise ValueError("shared-root geometry/model fingerprint mismatch")
        self.origin = np.array(geometry["o_B_m"], float)
        self.robot_rotation = np.array(geometry["R_BCt"], float)
        width = np.linalg.norm(np.array(geometry["left_shoulder_B_m"])-geometry["right_shoulder_B_m"])
        reach = .5 * (sum(geometry["left_segment_lengths_m"]) + sum(geometry["right_segment_lengths_m"]))
        self.scale = np.array([reach/self.reach, width/self.width, reach/self.reach])
        self.forward = forward
        self.state, self.error = "uncalibrated", None
        self.solution = None
        self.samples = []
        self.frame = None
        self._invalidate_cache()

    def _invalidate_cache(self):
        self._extract_frame = self._extracted = None
        self._extract_error = None
        self._target_frame = self._target_solution = self._targets = None

    @property
    def calibration_allows_start(self):
        return self.solution is not None and self.state not in ("collecting", "failed")

    def fail(self, error):
        self.state, self.error = "failed", str(error)
        self._invalidate_cache()

    def request_calibration(self, now, *, idle):
        if not idle or self.state == "collecting":
            return False
        self._invalidate_cache()
        self.samples = []
        self.started = now
        self.state, self.error = "collecting", None
        return True

    @staticmethod
    def _pose(pose):
        p = np.asarray(pose, float)
        if p.shape != (7,) or not np.isfinite(p).all() or not 1e-12 < np.linalg.norm(p[3:]) < 1e12:
            raise ValueError("invalid tracked pose")
        return p, Rotation.from_quat(p[3:])

    def extract(self, frame):
        # Frames are immutable snapshots; association IDs alone are insufficient
        # for distinct snapshots (including invalid replacements).
        if frame is self._extract_frame:
            if self._extract_error is not None:
                raise ValueError(self._extract_error)
            return self._extracted
        self._invalidate_cache()
        self._extract_frame = frame
        try:
            self._extracted = self._extract_geometry(frame)
        except ValueError as error:
            self._extract_error = str(error)
            raise
        return self._extracted

    def _extract_geometry(self, frame):
        if not frame.head_valid:
            raise ValueError("head tracking invalid")
        head, head_rotation = self._pose(frame.head_pose)
        palms, rotations = [], []
        for side in SIDES:
            hand = frame.hands[side]
            if not (hand.valid and hand.wrist_valid and hand.joints[12].valid):
                raise ValueError("wrist/middle proximal tracking invalid")
            wrist, rotation = self._pose(hand.wrist_pose)
            # Protocol point 12: middle proximal. Palm point 0 is NOT consumed,
            # so no second offset can be added to an already translated palm.
            direction = np.asarray(hand.joints[12].pose[:3])-wrist[:3]
            length = np.linalg.norm(direction)
            if not np.isfinite(length) or not .005 < length < .20:
                raise ValueError("invalid wrist-to-middle longitudinal axis")
            palms.append(wrist[:3]+direction/length*self.palm_distance)
            rotations.append(rotation)
        head_position, palm_positions = head[:3].copy(), np.array(palms)
        head_position.setflags(write=False)
        palm_positions.setflags(write=False)
        return head_position, head_rotation, palm_positions, rotations

    def offer_frame(self, frame, now):
        if frame is not self.frame:
            self._invalidate_cache()
        self.frame = frame
        if self.state != "collecting":
            return
        if not 0 <= now-frame.received_timestamp_ns <= 150_000_000:
            self.fail("stale calibration frame")
            return
        if frame.received_timestamp_ns < self.started:
            return
        if self.samples and frame.received_timestamp_ns <= self.samples[-1][0]:
            return
        if frame.received_timestamp_ns-(self.samples[-1][0] if self.samples else self.started) > 250_000_000:
            self.fail("calibration input gap")
            return
        try:
            extracted = self.extract(frame)
        except ValueError as e:
            self.fail(e)
            return
        self.samples.append((frame.received_timestamp_ns, *extracted))


    def tick(self, now):
        if self.state != "collecting":
            return False
        if now-(self.samples[-1][0] if self.samples else self.started) > 250_000_000:
            self.fail("calibration input gap")
            return False
        if now-self.started < 1_000_000_000:
            return False
        try:
            if len(self.samples) < 30 or self.samples[-1][0]-self.samples[0][0] < 750_000_000:
                raise ValueError("insufficient calibration samples")
            heads = np.array([s[1] for s in self.samples])
            palms = np.array([s[3] for s in self.samples])
            if max(np.ptp(heads,axis=0).max(), np.ptp(palms,axis=0).max()) > .06:
                raise ValueError("hold head and both forward arms steady")
            headings = np.array([s[2].apply([1.,0,0]) for s in self.samples])
            norms = np.linalg.norm(headings[:,:2],axis=1)
            if min(norms) < .5:
                raise ValueError("look forward, not vertically")
            headings[:,2] = 0
            headings /= norms[:,None]
            x = headings.mean(axis=0); x /= np.linalg.norm(x)
            if np.min(headings @ x) < math.cos(.10):
                raise ValueError("head heading moved during calibration")
            basis = np.column_stack((x, np.cross([0.,0,1],x), [0.,0,1]))
            centers = palms.mean(axis=0)
            separation = basis.T @ (centers[0]-centers[1])
            if abs(separation[1]-self.width) > max(.08,.35*self.width) or max(abs(separation[[0,2]])) > .08:
                raise ValueError("extend both arms forward at shoulder width and equal height")
            root = centers.mean(axis=0)-x*self.reach
            offset = root-heads.mean(axis=0)
            if np.linalg.norm(offset) > self.height*.6:
                raise ValueError("implausible calibrated head-to-root offset")
            # Orientation calibration is explicit: palms face each other.
            # Only FK is queried; this posture is never commanded.
            q = np.zeros((2,7)); q[:,1] = -np.pi/2
            reference = self.forward(q)
            corrections = []
            for i,side in enumerate(SIDES):
                rotations = Rotation.from_quat([s[4][i].as_quat() for s in self.samples])
                mean = rotations.mean()
                if max((mean.inv()*rotations).magnitude()) > .15:
                    raise ValueError("wrist orientation moved during calibration")
                desired = Rotation.from_quat(reference[side]["achieved_pose"][3:]).as_matrix()
                corrections.append((self.robot_rotation @ basis.T @ mean.as_matrix()).T @ desired)
            self.solution = (basis,offset,np.array(corrections))
            self._invalidate_cache()
            self.state,self.error = "calibrated",None
            return True
        except (ValueError, KeyError) as e:
            self.fail(e)
            return False

    def targets(self, frame):
        if not self.calibration_allows_start:
            raise ValueError(f"{self.calibration_key} calibration required")
        if frame is self._target_frame and self.solution is self._target_solution:
            return self._targets
        head, _, palms, rotations = self.extract(frame)
        basis, offset, corrections = self.solution
        root = head+offset
        # Same shared-root affine equation as SharedRootTargetBuilder:
        # p_B = o_B + R_BCt diag(reach, lateral, reach) p_Ct.
        positions = self.origin + ((palms-root) @ basis * self.scale) @ self.robot_rotation.T
        quats = [Rotation.from_matrix(self.robot_rotation @ basis.T @ rotations[i].as_matrix() @ corrections[i]).as_quat() for i in range(2)]
        if not np.isfinite(positions).all() or np.max(np.linalg.norm(positions-self.origin,axis=1)) > 2:
            raise ValueError("mapped palm exceeds target gate")
        targets = np.column_stack((positions,quats))
        targets.setflags(write=False)
        self._target_frame, self._target_solution, self._targets = frame, self.solution, targets
        return targets

    def status(self):
        solution = self.solution
        return dict(state=self.state,error=self.error,mapping="pico2_shared_root_height_v1",
                    ik_backend="franka_dls_ruckig",root_position_policy="head_translation_with_calibrated_offset",
                    root_rotation_policy=f"{self.calibration_key}_locked_gravity_level_heading",
                    robot_geometry_sha256=self.geometry_sha256,
                    calibration_required=True,calibration_allows_start=self.calibration_allows_start,
                    height_m=self.height, shoulder_width_m=self.width,
                    upper_arm_m=self.upper,forearm_m=self.forearm,wrist_palm_m=self.palm_distance,
                    scale=self.scale.tolist(),samples=len(self.samples),
                    root_basis=None if solution is None else solution[0].tolist(),
                    head_to_root_m=None if solution is None else solution[1].tolist(),
                    wrist_to_robot_palm=None if solution is None else solution[2].tolist(),
                    geometry="estimated_not_measured",motion_authorized=False)
