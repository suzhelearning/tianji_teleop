import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
import h5py
import numpy as np
import os
import time
import importlib
from typing import Dict, Any, List
from bisect import bisect_left
import threading
import cv2
from data_collector.data_processor import DataProcessor
import yaml
from enum import Enum


class CollectorState(str, Enum):
    IDLE = 'IDLE'
    RECORDING = 'RECORDING'
    SAVING = 'SAVING'


def load_topics_config(config_path):
    with open(config_path) as f:
        config = yaml.safe_load(f)
    return config


def getch():
    """Read a single character from stdin without echo."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class DataCollector(Node):
    def __init__(self, config_path):
        super().__init__('data_collector')
        self.config = load_topics_config(config_path)

        self.is_collecting = False
        self.collect_timer = None
        self.collected_dataset_list = []
        self.collected_timestamps = []
        self.dataset_cfg_dict = {}
        self.state = CollectorState.IDLE
        self.state_lock = threading.RLock()
        self.data_lock = threading.Lock()
        self.save_thread = None
        self.last_error = None

        self.get_logger().info(f"data_dir: {self.config['data_dir']}")
        self.get_logger().info(f"collection_frequency: {self.config['collection_frequency']}")
        self.get_logger().info(f"max_queue_size: {self.config['max_queue_size']}")
        self.get_logger().info(f"datasets to collect: {list(self.config['datasets'].keys())}")

        self.processors: Dict[str, DataProcessor] = {}
        self.buffers: Dict[str, List] = {}
        self.locks: Dict[str, threading.Lock] = {}
        self.setup_processors()
        self.setup_subscriptions()
        self.setup_record_flag()

    def setup_record_flag(self):
        # Optional: PICO A-button record flag (published by pico_bridge) starts/stops
        # collection remotely; the keyboard s/d keys keep working alongside it.
        topic = self.config.get('record_flag_topic')
        if not topic:
            return
        self.create_subscription(Bool, topic, self.on_record_flag, 10)
        self.get_logger().info(f"record flag sub {topic}")

    def on_record_flag(self, msg):
        if msg.data:
            self.start_collecting()
        else:
            self.stop_collecting()

    def setup_processors(self):
        datasets = self.config['datasets']
        for dataset_name, datset_value in datasets.items():
            try:
                module_name, class_name = datset_value['processor'].rsplit('.', 1)
                module = importlib.import_module(module_name)
                processor_class = getattr(module, class_name)
                self.processors[dataset_name] = processor_class(
                    datset_value.get('processor_config', {})
                )

                topic_name = datset_value['topic']
                self.buffers[topic_name] = []
                if topic_name not in self.locks:
                    self.locks[topic_name] = threading.Lock()

                dataset_config = self.processors[dataset_name].get_dataset_config()
                self.get_logger().info(f"{dataset_name} config: {dataset_config}")
                self.dataset_cfg_dict[dataset_name] = dataset_config

            except Exception as e:
                self.get_logger().error(f"Failed to load processor for {dataset_name}: {str(e)}")
                raise

    def setup_subscriptions(self):
        topic_names = [v['topic'] for _, v in self.config['datasets'].items()]
        topic_configs = {
            topic_name: topic_cfg
            for topic_name, topic_cfg in self.config['topics'].items()
            if topic_name in topic_names
        }
        for topic_name, topic_cfg in topic_configs.items():
            msg_module, msg_type = topic_cfg['msg_type'].rsplit('.', 1)
            module = importlib.import_module(msg_module)
            msg_class = getattr(module, msg_type)

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=5
            )
            self.create_subscription(
                msg_class,
                topic_name,
                self.create_callback(topic_name),
                qos
            )
            self.get_logger().info(f"create sub {topic_name}")

    def create_callback(self, topic_name):
        def callback(msg):
            # Use wall-clock receive time for buffer matching. PICO header.stamp
            # is a device-local boot time and cannot be aligned with ROS wall time.
            # Processors still extract msg.header.stamp -> ts_ms for their own records.
            stamp = time.time()
            with self.locks[topic_name]:
                self.buffers[topic_name].append((stamp, msg))
                max_q = self.config['max_queue_size']
                if len(self.buffers[topic_name]) > max_q:
                    self.buffers[topic_name].pop(0)
        return callback

    def start_collecting(self):
        with self.state_lock:
            if self.state == CollectorState.RECORDING:
                self.get_logger().warn("Already collecting")
                return False
            if self.state == CollectorState.SAVING:
                self.get_logger().warn("Cannot start: save is still in progress")
                return False

            # A timer cancelled after a previous run still belongs to the node.
            # Destroy it before creating the next one so repeated failures do not
            # accumulate dead timers.
            if self.collect_timer is not None:
                try:
                    self.collect_timer.cancel()
                    self.destroy_timer(self.collect_timer)
                except Exception as e:
                    self.get_logger().warn(f"Failed to destroy old collection timer: {e}")
                self.collect_timer = None

            frequency = self.config['collection_frequency']
            interval = 1.0 / frequency
            with self.data_lock:
                self.collected_dataset_list = []
                self.collected_timestamps = []
            self.last_error = None
            self.is_collecting = True
            self.state = CollectorState.RECORDING
            self.collect_timer = self.create_timer(interval, self.collect_data)

        self.get_logger().info(f"Started collecting at {frequency}Hz")
        return True

    def stop_collecting(self):
        with self.state_lock:
            if self.state == CollectorState.SAVING:
                self.get_logger().warn("Save already in progress")
                return False
            if self.state != CollectorState.RECORDING:
                self.get_logger().warn("Not collecting")
                return False

            self.state = CollectorState.SAVING
            self.is_collecting = False
            if self.collect_timer is not None:
                self.collect_timer.cancel()

            # collect_data re-checks state before appending, so after the state
            # transition this snapshot is stable even if a timer callback was
            # already running on the ROS executor thread.
            with self.data_lock:
                data_list = list(self.collected_dataset_list)
                timestamps = list(self.collected_timestamps)
                self.collected_dataset_list = []
                self.collected_timestamps = []

            self.save_thread = threading.Thread(
                target=self._save_worker,
                args=(data_list, timestamps),
                name='data_collector_save',
                daemon=False,
            )
            save_thread = self.save_thread

        self.get_logger().info(
            f"Stop accepted; saving {len(data_list)} frames in background. "
            "Wait for 'Saved ... frames' before restarting or powering off."
        )
        save_thread.start()
        return True

    def _save_worker(self, data_list, timestamps):
        start_time = time.time()
        try:
            self.save_data(data_list, timestamps)
            self.get_logger().info(f"Save time cost: {time.time() - start_time:.2f}s")
        except Exception as e:
            self.last_error = f"Save failed: {e}"
            self.get_logger().error(self.last_error)
        finally:
            with self.state_lock:
                self.state = CollectorState.IDLE
                self.is_collecting = False
                self.save_thread = None

    def wait_for_save(self, timeout=None):
        """Wait for an active background save. Returns False on timeout."""
        with self.state_lock:
            save_thread = self.save_thread
        if save_thread is None or save_thread is threading.current_thread():
            return True
        save_thread.join(timeout=timeout)
        return not save_thread.is_alive()

    def shutdown_collector(self):
        """Stop recording and do not destroy the node until files are closed."""
        with self.state_lock:
            state = self.state
        if state == CollectorState.RECORDING:
            self.stop_collecting()
        with self.state_lock:
            saving = self.state == CollectorState.SAVING
        if saving:
            self.get_logger().info("Waiting for background save to finish before shutdown")
            self.wait_for_save()

    def collect_data(self):
        with self.state_lock:
            if self.state != CollectorState.RECORDING:
                return
        current_time = self.get_clock().now().nanoseconds / 1e9
        processed_data_dict = {}
        max_age = self.config['max_age']

        for dataset_name, dataset_cfg in self.config['datasets'].items():
            required = dataset_cfg.get('required', True)
            topic_name = dataset_cfg['topic']
            with self.locks[topic_name]:
                buffer = self.buffers[topic_name]
                if not buffer:
                    if not required:
                        continue
                    self.abort_collection(f"No data in {topic_name}")
                    return

                stamps = [s for s, _ in buffer]
                idx = bisect_left(stamps, current_time)
                closest = self.find_closest(stamps, idx, current_time)

                if closest is None:
                    if not required:
                        continue
                    self.abort_collection(f"No closest data in {topic_name}")
                    return

                delta_time = abs(buffer[closest][0] - current_time)
                if closest == 0 and delta_time > max_age:
                    if not required:
                        continue
                    self.abort_collection(
                        f"Stale data in {topic_name}, delta time: {delta_time}"
                    )
                    return
                if delta_time > max_age:
                    if not required:
                        continue
                    self.abort_collection(
                        f"Stale data in {topic_name}, delta time: {delta_time}"
                    )
                    return

                _, msg = buffer[closest]
                try:
                    processed_data_dict[dataset_name] = self.processors[dataset_name].process(msg)
                except Exception as e:
                    self.abort_collection(
                        f"Failed to process {topic_name}: {type(e).__name__}: {e}"
                    )
                    return
                self.buffers[topic_name] = buffer[closest:]

        # stop_collecting may have changed state while this callback processed
        # its topic snapshots. Never append to a save that has already started.
        with self.state_lock:
            if self.state != CollectorState.RECORDING:
                return
            with self.data_lock:
                self.collected_dataset_list.append(processed_data_dict)
                self.collected_timestamps.append(current_time)
                n = len(self.collected_dataset_list)

        if n % 100 == 0:
            self.get_logger().info(f"Collected {n} frames")

    def save_data(self, data_list, timestamps):
        data_dir = self.config['data_dir']
        os.makedirs(data_dir, exist_ok=True)
        timestamp_str = time.strftime('%Y%m%d_%H%M%S')

        num_frames = len(data_list)
        if num_frames == 0:
            self.get_logger().warn("No frames collected, skipping save")
            return None

        # Never expose half-written files as a completed session. All outputs
        # are staged in a hidden directory and atomically renamed only after
        # HDF5, MP4 and CSV handles have been closed successfully.
        session_name = timestamp_str
        suffix = 1
        session_dir = os.path.join(data_dir, session_name)
        while os.path.exists(session_dir):
            session_name = f"{timestamp_str}_{suffix:02d}"
            session_dir = os.path.join(data_dir, session_name)
            suffix += 1
        staging_dir = os.path.join(
            data_dir,
            f".{session_name}.inprogress-{os.getpid()}-{threading.get_ident()}",
        )
        os.makedirs(staging_dir, exist_ok=False)

        hdf5_path = os.path.join(staging_dir, 'data.hdf5')
        self.get_logger().info(f"Saving {num_frames} frames to staging directory {staging_dir}")

        fps = self.config['collection_frequency']
        video_writers = {}

        try:
            with h5py.File(hdf5_path, 'w') as hf:
                hf.create_dataset(
                    'timestamps',
                    data=np.array(timestamps, dtype=np.float64),
                )
                hf['timestamps'].attrs['description'] = 'Collection timestamps (seconds since epoch)'

                hf.create_dataset(
                    'collection_frequency',
                    data=self.config['collection_frequency'],
                    dtype='f'
                )

                vlen_type = h5py.special_dtype(vlen=np.uint8)

                # Figure out which kinds are present across all frames. Optional
                # datasets can be absent from the first frame and appear later.
                kinds = {}
                for frame in data_list:
                    for name, result in frame.items():
                        kinds.setdefault(name, result.get('kind', 'image'))

                # Pre-create HDF5 dataset groups per kind
                hdf5_handles = {}
                for name, kind in kinds.items():
                    grp = hf.require_group(name)
                    if kind == 'image':
                        ds = grp.create_dataset('jpeg', (num_frames,), dtype=vlen_type)
                        ds.attrs['encoding'] = 'jpeg'
                        ds.attrs['description'] = self.dataset_cfg_dict[name].get('description', '')
                        grp.create_dataset('ts_ms', (num_frames,), dtype=np.int64)
                        hdf5_handles[name] = grp
                    elif kind == 'pose':
                        grp.create_dataset('pos', (num_frames, 3), dtype=np.float64)
                        grp.create_dataset('quat_xyzw', (num_frames, 4), dtype=np.float64)
                        grp.create_dataset('ts_ms', (num_frames,), dtype=np.int64)
                        grp.attrs['description'] = self.dataset_cfg_dict[name].get('description', '')
                        hdf5_handles[name] = grp
                    elif kind == 'ble':
                        grp.create_dataset('data', (num_frames,), dtype=vlen_type)
                        grp.create_dataset('esp32_ts', (num_frames,), dtype=np.uint32)
                        grp.create_dataset('ts_ms', (num_frames,), dtype=np.int64)
                        grp.attrs['description'] = self.dataset_cfg_dict[name].get('description', '')
                        hdf5_handles[name] = grp
                    elif kind == 'odometry':
                        grp.create_dataset('pos', (num_frames, 3), dtype=np.float64)
                        grp.create_dataset('quat_xyzw', (num_frames, 4), dtype=np.float64)
                        grp.create_dataset('linear_velocity', (num_frames, 3), dtype=np.float64)
                        grp.create_dataset('angular_velocity', (num_frames, 3), dtype=np.float64)
                        grp.create_dataset('pose_covariance', (num_frames, 36), dtype=np.float64)
                        grp.create_dataset('twist_covariance', (num_frames, 36), dtype=np.float64)
                        grp.create_dataset('ts_ms', (num_frames,), dtype=np.int64)
                        grp.create_dataset('valid', (num_frames,), dtype=np.bool_)
                        grp.attrs['description'] = self.dataset_cfg_dict[name].get('description', '')
                        hdf5_handles[name] = grp
                    elif kind == 'smpl':
                        grp.create_dataset(
                            'pose', (num_frames, 24, 7), dtype=np.float64
                        )
                        grp.create_dataset('ts_ms', (num_frames,), dtype=np.int64)
                        cfg = self.dataset_cfg_dict[name]
                        grp.attrs['description'] = cfg.get('description', '')
                        grp.attrs['layout'] = cfg.get(
                            'layout',
                            '[pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w]',
                        )
                        grp.attrs['joint_names'] = np.asarray(
                            cfg.get('joint_names', []), dtype=h5py.string_dtype('utf-8')
                        )
                        hdf5_handles[name] = grp
                    else:
                        self.get_logger().warn(f"Unknown kind {kind} for {name}, skipping")
                    self.get_logger().info(f"{name} ({kind}) save frames: {num_frames}")

                # Fill per-frame
                for i, collected in enumerate(data_list):
                    for name, result in collected.items():
                        grp = hdf5_handles.get(name)
                        if grp is None:
                            continue
                        k = result.get('kind', 'image')
                        if k == 'image':
                            compressed = result['compressed']
                            decoded_bgr = result['decoded_bgr']
                            grp['jpeg'][i] = np.frombuffer(compressed, dtype=np.uint8)
                            grp['ts_ms'][i] = result.get('ts_ms', 0)
                            if name not in video_writers:
                                h, w = decoded_bgr.shape[:2]
                                video_name = name.strip('/').replace('/', '_') + '.mp4'
                                video_path = os.path.join(staging_dir, video_name)
                                writer = cv2.VideoWriter(
                                    video_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h)
                                )
                                if not writer.isOpened():
                                    raise RuntimeError(f"Failed to open video writer: {video_path}")
                                video_writers[name] = writer
                            video_writers[name].write(decoded_bgr)
                        elif k == 'pose':
                            grp['pos'][i] = result['pos']
                            grp['quat_xyzw'][i] = result['quat_xyzw']
                            grp['ts_ms'][i] = result['ts_ms']
                        elif k == 'ble':
                            grp['data'][i] = np.frombuffer(result['raw'], dtype=np.uint8)
                            grp['esp32_ts'][i] = result['esp32_ts']
                            grp['ts_ms'][i] = result['ts_ms']
                        elif k == 'odometry':
                            grp['pos'][i] = result['pos']
                            grp['quat_xyzw'][i] = result['quat_xyzw']
                            grp['linear_velocity'][i] = result['linear_velocity']
                            grp['angular_velocity'][i] = result['angular_velocity']
                            grp['pose_covariance'][i] = result['pose_covariance']
                            grp['twist_covariance'][i] = result['twist_covariance']
                            grp['ts_ms'][i] = result['ts_ms']
                            grp['valid'][i] = True
                        elif k == 'smpl':
                            pose = np.asarray(result['pose'], dtype=np.float64)
                            if pose.shape != (24, 7):
                                raise ValueError(
                                    f"SMPL pose must have shape (24, 7), got {pose.shape}"
                                )
                            grp['pose'][i] = pose
                            grp['ts_ms'][i] = result['ts_ms']

                    if (i + 1) % 100 == 0 or i + 1 == num_frames:
                        self.get_logger().info(f"Saving progress: {i + 1}/{num_frames} frames")
            # Releasing the writers writes each MP4 moov atom. This must happen
            # before the staging directory becomes a completed session.
            for name, writer in video_writers.items():
                writer.release()
                self.get_logger().info(f"Video finalized: {name}")
            video_writers.clear()

            csv_path = os.path.join(staging_dir, 'timestamps.csv')
            with open(csv_path, 'w') as f:
                f.write('frame_index,timestamp\n')
                for i, ts in enumerate(timestamps):
                    f.write(f'{i},{ts:.6f}\n')

            os.replace(staging_dir, session_dir)
            self.get_logger().info(f"Saved {num_frames} frames to {session_dir}")
            return session_dir
        except Exception:
            self.get_logger().error(
                f"Incomplete save retained for diagnosis at {staging_dir}; "
                "it is not a completed session"
            )
            raise
        finally:
            for writer in video_writers.values():
                writer.release()

    def find_closest(self, stamps, idx, target):
        if idx == 0:
            return 0 if stamps else None
        elif idx == len(stamps):
            return idx - 1
        else:
            before = stamps[idx - 1]
            after = stamps[idx]
            return idx - 1 if (target - before) <= (after - target) else idx

    def abort_collection(self, msg):
        """Stop a bad run without terminating the ROS executor thread."""
        with self.state_lock:
            if self.state != CollectorState.RECORDING:
                return
            self.last_error = msg
            self.state = CollectorState.IDLE
            self.is_collecting = False
            if self.collect_timer is not None:
                self.collect_timer.cancel()
        self.get_logger().error(
            f"Collection aborted: {msg}. Fix the input stream and press 's' to retry."
        )


def main(args=None):
    rclpy.init(args=args)

    # Get config path from command line: --config /path/to/config.yaml
    config_path = None
    argv = sys.argv[1:]
    for i, arg in enumerate(argv):
        if arg == '--config' and i + 1 < len(argv):
            config_path = argv[i + 1]
            break

    if config_path is None:
        # Try default location
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', '..', '..', 'config', 'collect_config.yaml')
        if not os.path.exists(config_path):
            print(f"Usage: ros2 run data_collector data_collector_node --config /path/to/collect_config.yaml")
            sys.exit(1)

    node = DataCollector(config_path)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('Keys: [s] start collecting  [d] stop collecting  [q] quit')

    try:
        while True:
            ch = getch()
            if ch == 's':
                node.start_collecting()
            elif ch == 'd':
                node.stop_collecting()
            elif ch in ('q', '\x03'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_collector()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
