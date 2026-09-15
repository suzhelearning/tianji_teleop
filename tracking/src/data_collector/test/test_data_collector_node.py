import csv
import threading
from pathlib import Path

import cv2
import h5py
import numpy as np
import pytest
from geometry_msgs.msg import Pose, PoseArray

from data_collector.data_collector_node import CollectorState, DataCollector
from data_collector.data_processor import PICO_SMPL_JOINT_NAMES, SmplPoseArrayProcessor


class FakeLogger:
    def __init__(self):
        self.messages = []

    def _record(self, level, message):
        self.messages.append((level, message))

    def info(self, message):
        self._record('info', message)

    def warn(self, message):
        self._record('warn', message)

    def error(self, message):
        self._record('error', message)


class SaveHarness:
    save_data = DataCollector.save_data

    def __init__(self, data_dir):
        self.config = {'data_dir': str(data_dir), 'collection_frequency': 30.0}
        self.dataset_cfg_dict = {
            '/observations/images/cam_left': {'description': 'left'},
            '/states/pose_head': {'description': 'head'},
            '/states/smpl': {
                'description': 'smpl',
                'joint_names': PICO_SMPL_JOINT_NAMES,
                'layout': '[pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w]',
            },
        }
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


class FakeTimer:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class ControlHarness:
    start_collecting = DataCollector.start_collecting
    stop_collecting = DataCollector.stop_collecting
    _save_worker = DataCollector._save_worker
    wait_for_save = DataCollector.wait_for_save
    abort_collection = DataCollector.abort_collection

    def __init__(self):
        self.config = {'collection_frequency': 30.0}
        self.state = CollectorState.IDLE
        self.state_lock = threading.RLock()
        self.data_lock = threading.Lock()
        self.is_collecting = False
        self.collect_timer = None
        self.collected_dataset_list = []
        self.collected_timestamps = []
        self.save_thread = None
        self.last_error = None
        self.logger = FakeLogger()
        self.save_started = threading.Event()
        self.allow_save_to_finish = threading.Event()
        self.saved_snapshot = None

    def get_logger(self):
        return self.logger

    def create_timer(self, interval, callback):
        self.timer_interval = interval
        self.timer_callback = callback
        return FakeTimer()

    def destroy_timer(self, timer):
        timer.cancel()

    def collect_data(self):
        pass

    def save_data(self, data_list, timestamps):
        self.saved_snapshot = (list(data_list), list(timestamps))
        self.save_started.set()
        assert self.allow_save_to_finish.wait(timeout=5)


def make_frames(count=4):
    image = np.full((48, 64, 3), 127, dtype=np.uint8)
    ok, encoded = cv2.imencode('.jpg', image)
    assert ok
    jpeg = encoded.tobytes()
    frames = []
    for i in range(count):
        frames.append({
            '/observations/images/cam_left': {
                'kind': 'image',
                'compressed': jpeg,
                'decoded_bgr': image,
                'ts_ms': 1000 + i * 33,
            },
            '/states/pose_head': {
                'kind': 'pose',
                'pos': np.array([i, 0.0, 0.0], dtype=np.float64),
                'quat_xyzw': np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
                'ts_ms': 1000 + i * 33,
            },
            '/states/smpl': {
                'kind': 'smpl',
                'pose': np.tile(
                    np.array([i, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]), (24, 1)
                ),
                'ts_ms': 1000 + i * 33,
            },
        })
    return frames


def test_save_is_published_only_after_all_files_are_closed(tmp_path):
    harness = SaveHarness(tmp_path)
    frames = make_frames()
    timestamps = [1.0 + i / 30.0 for i in range(len(frames))]

    session_dir = Path(harness.save_data(frames, timestamps))

    assert session_dir.is_dir()
    assert not list(tmp_path.glob('.*.inprogress-*'))
    with h5py.File(session_dir / 'data.hdf5', 'r') as hf:
        assert hf['timestamps'].shape == (4,)
        assert hf['observations/images/cam_left/jpeg'].shape == (4,)
        assert hf['states/pose_head/pos'].shape == (4, 3)
        assert hf['states/smpl/pose'].shape == (4, 24, 7)
        assert list(hf['states/smpl'].attrs['joint_names']) == PICO_SMPL_JOINT_NAMES
    with (session_dir / 'timestamps.csv').open(newline='') as stream:
        assert len(list(csv.DictReader(stream))) == 4

    video = cv2.VideoCapture(str(session_dir / 'observations_images_cam_left.mp4'))
    assert video.isOpened()
    assert int(video.get(cv2.CAP_PROP_FRAME_COUNT)) == 4
    video.release()


def test_failed_save_never_appears_as_completed_session(tmp_path):
    harness = SaveHarness(tmp_path)
    frames = make_frames()
    del frames[1]['/states/pose_head']['quat_xyzw']

    with pytest.raises(KeyError):
        harness.save_data(frames, [1.0 + i / 30.0 for i in range(len(frames))])

    assert not [path for path in tmp_path.iterdir() if not path.name.startswith('.')]
    staging = list(tmp_path.glob('.*.inprogress-*'))
    assert len(staging) == 1


def test_controls_are_serialized_while_background_save_runs():
    harness = ControlHarness()
    assert harness.start_collecting()
    harness.collected_dataset_list = [{'frame': 1}]
    harness.collected_timestamps = [1.0]

    assert harness.stop_collecting()
    assert harness.save_started.wait(timeout=2)
    assert harness.state == CollectorState.SAVING
    assert not harness.start_collecting()
    assert not harness.stop_collecting()

    harness.allow_save_to_finish.set()
    assert harness.wait_for_save(timeout=2)
    assert harness.state == CollectorState.IDLE
    assert harness.saved_snapshot == ([{'frame': 1}], [1.0])


def test_input_error_aborts_run_without_raising_and_allows_retry():
    harness = ControlHarness()
    assert harness.start_collecting()
    timer = harness.collect_timer

    harness.abort_collection('No data in /required/topic')

    assert harness.state == CollectorState.IDLE
    assert not harness.is_collecting
    assert timer.cancelled
    assert harness.last_error == 'No data in /required/topic'
    assert harness.start_collecting()
    assert harness.state == CollectorState.RECORDING


def test_smpl_processor_preserves_canonical_joint_order():
    msg = PoseArray()
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 345_000_000
    for index in range(24):
        pose = Pose()
        pose.position.x = float(index)
        pose.orientation.w = 1.0
        msg.poses.append(pose)

    result = SmplPoseArrayProcessor({}).process(msg)

    assert result['kind'] == 'smpl'
    assert result['pose'].shape == (24, 7)
    assert result['pose'][0, 0] == 0.0
    assert result['pose'][23, 0] == 23.0
    assert result['pose'][:, 6].tolist() == [1.0] * 24
    assert result['ts_ms'] == 12_345


def test_smpl_processor_rejects_incomplete_frame():
    msg = PoseArray()
    msg.poses = [Pose()] * 23
    with pytest.raises(ValueError, match='Expected 24 SMPL joints'):
        SmplPoseArrayProcessor({}).process(msg)
