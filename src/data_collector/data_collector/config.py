"""Runtime camera identities and schema-v1 dataset metadata; no device I/O.

Imports stay free of ROS: the collector's node layer owns subscriptions, this
module only validates configuration and derives dataset metadata.
"""
from pathlib import Path
import json

from tianji_runtime.constants import (
    CAMERA_ROLES,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    STATE_DIM,
    STATE_RATE_HZ,
)


def load_collection_config(path, *, source_bytes=None):
    config = json.loads(Path(path).read_bytes() if source_bytes is None else source_bytes)
    if not isinstance(config, dict) or set(config) != {'robot_config', 'state_rate_hz', 'cameras'}:
        raise ValueError('collection config requires robot_config, state_rate_hz and cameras')
    if not isinstance(config['robot_config'], str) or not config['robot_config'].strip():
        raise ValueError('robot_config must be a nonempty name')
    if isinstance(config['state_rate_hz'], bool) or config['state_rate_hz'] != STATE_RATE_HZ:
        raise ValueError(f'schema-v1 state acquisition target must be {STATE_RATE_HZ} Hz')
    cameras = config['cameras']
    if not isinstance(cameras, dict) or set(cameras) != set(CAMERA_ROLES):
        raise ValueError('camera mapping must explicitly specify top, left_wrist and right_wrist; 0 disables a camera')
    enabled = {}
    for role in CAMERA_ROLES:
        serial = cameras[role]
        if type(serial) is int and serial == 0:
            continue
        if not isinstance(serial, str) or not serial or not serial.isascii() or not serial.isalnum():
            raise ValueError(f'{role}: expected a camera serial string or integer 0')
        if serial in enabled.values():
            raise ValueError('each enabled view must use a distinct camera serial')
        enabled[role] = serial
    if not enabled:
        raise ValueError('at least one RGB camera must be enabled')
    return config, enabled


def dataset_metadata(config, cameras, model_path):
    # Reuse the actual viewer/model mapping so hand and arm ordering cannot
    # silently diverge from the real executor's TJRC interpretation.
    import mujoco

    from tianji_controller.viewer import _group_qpos

    model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
    groups = _group_qpos(model, mujoco)
    names_by_address = {
        int(model.jnt_qposadr[index]): mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
    }
    names = [names_by_address[address] for group in ('arms', 'left_hand', 'right_hand')
             for address in groups[group]]
    if len(names) != STATE_DIM or len(set(names)) != STATE_DIM or any(not name for name in names):
        raise ValueError(f'controller model must define {STATE_DIM} unique named joints')
    return {
        'schema_version': 1,
        'robot_config': config['robot_config'],
        'joint_names': names,
        'joint_unit': 'rad',
        'policy_rate_hz': 30,
        'camera_names': list(cameras),
        'image_width': IMAGE_WIDTH,
        'image_height': IMAGE_HEIGHT,
        'image_encoding': 'rgb',
        'decoded_color_order': 'RGB',
        'jpeg_quality': None,
    }
