from typing import Dict, Any
import numpy as np
import cv2


PICO_SMPL_JOINT_NAMES = [
    'Pelvis',
    'LEFT_HIP', 'RIGHT_HIP', 'SPINE1',
    'LEFT_KNEE', 'RIGHT_KNEE', 'SPINE2',
    'LEFT_ANKLE', 'RIGHT_ANKLE', 'SPINE3',
    'LEFT_FOOT', 'RIGHT_FOOT', 'NECK',
    'LEFT_COLLAR', 'RIGHT_COLLAR', 'HEAD',
    'LEFT_SHOULDER', 'RIGHT_SHOULDER',
    'LEFT_ELBOW', 'RIGHT_ELBOW',
    'LEFT_WRIST', 'RIGHT_WRIST', 'LEFT_HAND', 'RIGHT_HAND',
]


class DataProcessor:
    def __init__(self, config: Dict[str, Any]):
        self.config = config

    def process(self, msg) -> Any:
        raise NotImplementedError

    def get_dataset_config(self) -> Dict[str, Any]:
        return {
            'shape': tuple(self.config['output_shape']),
            'dtype': self.config['dtype'],
            'description': self.config['description'],
        }

    def get_message(self) -> str:
        return ''


class CompressedImageProcessor(DataProcessor):
    """Process CompressedImage messages (e.g. from fisheye cameras, PICO cams).
    Returns dict with 'compressed' (raw JPEG bytes), 'decoded_bgr' (BGR numpy array
    for video), and 'ts_ms' (PICO millisecond timestamp decoded from header.stamp).
    """
    def process(self, msg):
        compressed = bytes(msg.data)
        arr = np.frombuffer(compressed, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            # Fallback: keep compressed bytes, use a zero frame for video.
            resize = self.config.get('resize')
            if resize:
                bgr = np.zeros((resize[1], resize[0], 3), dtype=np.uint8)
            else:
                bgr = np.zeros((1, 1, 3), dtype=np.uint8)
        elif self.config.get('resize'):
            bgr = cv2.resize(bgr, tuple(self.config['resize'][:2]))
        return {
            'kind': 'image',
            'compressed': compressed,
            'decoded_bgr': bgr,
            'ts_ms': int(msg.header.stamp.sec) * 1000 + msg.header.stamp.nanosec // 1_000_000,
        }


class PoseStampedProcessor(DataProcessor):
    """Process geometry_msgs/PoseStamped (from PICO bridge).
    Returns pos (xyz), quat (xyzw), and the PICO ts_ms encoded in header.stamp.
    """
    def process(self, msg):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        quat_xyzw = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ], dtype=np.float64)
        ts_ms = int(msg.header.stamp.sec) * 1000 + msg.header.stamp.nanosec // 1_000_000
        return {
            'kind': 'pose',
            'pos': pos,
            'quat_xyzw': quat_xyzw,
            'ts_ms': ts_ms,
        }

    def get_dataset_config(self):
        return {'description': self.config.get('description', 'PoseStamped')}


class SmplPoseArrayProcessor(DataProcessor):
    """Store the complete 24-joint PICO BodyTrackerRole frame.

    The bridge guarantees canonical joint order and publishes only complete
    frames. Each row is [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w].
    """

    def process(self, msg):
        if len(msg.poses) != len(PICO_SMPL_JOINT_NAMES):
            raise ValueError(
                f'Expected {len(PICO_SMPL_JOINT_NAMES)} SMPL joints, got {len(msg.poses)}'
            )
        pose = np.empty((len(PICO_SMPL_JOINT_NAMES), 7), dtype=np.float64)
        for index, item in enumerate(msg.poses):
            pose[index] = (
                item.position.x, item.position.y, item.position.z,
                item.orientation.x, item.orientation.y,
                item.orientation.z, item.orientation.w,
            )
        ts_ms = int(msg.header.stamp.sec) * 1000 + msg.header.stamp.nanosec // 1_000_000
        return {
            'kind': 'smpl',
            'pose': pose,
            'ts_ms': ts_ms,
        }

    def get_dataset_config(self):
        return {
            'description': self.config.get(
                'description', 'PICO 24-joint BodyTrackerRole pose array'
            ),
            'joint_names': list(PICO_SMPL_JOINT_NAMES),
            'layout': '[pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w]',
        }


class OdometryProcessor(DataProcessor):
    """Process nav_msgs/Odometry.

    Stores pose, twist, covariances, and the message header timestamp in ms.
    """
    def process(self, msg):
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], dtype=np.float64)
        quat_xyzw = np.array([
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
        ], dtype=np.float64)
        linear_velocity = np.array([
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
        ], dtype=np.float64)
        angular_velocity = np.array([
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z,
        ], dtype=np.float64)
        ts_ms = int(msg.header.stamp.sec) * 1000 + msg.header.stamp.nanosec // 1_000_000
        return {
            'kind': 'odometry',
            'pos': pos,
            'quat_xyzw': quat_xyzw,
            'linear_velocity': linear_velocity,
            'angular_velocity': angular_velocity,
            'pose_covariance': np.array(msg.pose.covariance, dtype=np.float64),
            'twist_covariance': np.array(msg.twist.covariance, dtype=np.float64),
            'ts_ms': ts_ms,
        }

    def get_dataset_config(self):
        return {'description': self.config.get('description', 'Odometry')}


class BleFrameProcessor(DataProcessor):
    """Process pico_recorder/msg/BleFrame (PICO -> ESP32 BLE passthrough).
    Returns raw sensor bytes (reconstructed to wire format [4B esp32_ts | sensor]),
    plus esp32_ts and PICO ts_ms.
    """
    def process(self, msg):
        import struct
        raw = struct.pack('<I', msg.esp32_ts) + bytes(msg.data)
        ts_ms = int(msg.header.stamp.sec) * 1000 + msg.header.stamp.nanosec // 1_000_000
        return {
            'kind': 'ble',
            'raw': raw,                 # full wire payload: esp32_ts prefix + sensor bytes
            'esp32_ts': int(msg.esp32_ts),
            'ts_ms': ts_ms,
        }

    def get_dataset_config(self):
        return {'description': self.config.get('description', 'BLE frame')}
