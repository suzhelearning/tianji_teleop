"""Runtime camera identities and schema-v1 dataset metadata; no device I/O."""
from pathlib import Path
import json

CAMERA_ROLES = ('top', 'left_wrist', 'right_wrist')


def load_collection_config(path):
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(config, dict) or set(config) != {'robot_config', 'state_rate_hz', 'cameras'}:
        raise ValueError('collection config requires robot_config, state_rate_hz and cameras')
    if not isinstance(config['robot_config'], str) or not config['robot_config'].strip():
        raise ValueError('robot_config must be a nonempty name')
    if isinstance(config['state_rate_hz'], bool) or config['state_rate_hz'] != 120:
        raise ValueError('schema-v1 state acquisition target must be 120 Hz')
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
    from real_robot.viewer import _group_qpos

    model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
    groups = _group_qpos(model, mujoco)
    names_by_address = {
        int(model.jnt_qposadr[index]): mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
    }
    names = [names_by_address[address] for group in ('arms', 'left_hand', 'right_hand')
             for address in groups[group]]
    if len(names) != 54 or len(set(names)) != 54 or any(not name for name in names):
        raise ValueError('controller model must define 54 unique named joints')
    return {
        'schema_version': 1,
        'robot_config': config['robot_config'],
        'joint_names': names,
        'joint_unit': 'rad',
        'policy_rate_hz': 30,
        'camera_names': list(cameras),
        'image_width': 1280,
        'image_height': 720,
        'image_encoding': 'rgb',
        'decoded_color_order': 'RGB',
        'jpeg_quality': None,
    }
