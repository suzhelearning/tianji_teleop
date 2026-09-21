import json
from pathlib import Path

import pytest
from tianji_runtime import config_path, package_share, workspace
from tianji_runtime.resources import ResourceNotFound

from data_collector.config import dataset_metadata, load_collection_config

CONFIG = config_path('collect_real.json')


def _controller_model() -> Path:
    """The controller model, installed first and source tree second."""
    try:
        return package_share('tianji_description', 'models', 'marvin_m6_wuji2.xml')
    except ResourceNotFound:
        return workspace() / 'src' / 'teleop_outputs' / 'tianji' / 'tianji_description' / 'models' / 'marvin_m6_wuji2.xml'


def test_metadata_uses_real_model_joint_order():
    config, cameras = load_collection_config(CONFIG)
    metadata = dataset_metadata(config, cameras,
        _controller_model())
    assert len(metadata['joint_names']) == 54
    assert metadata['joint_names'][:14] == [f'Joint{i}_{side}' for side in ('L', 'R') for i in range(1, 8)]
    assert all(name.startswith('l_') for name in metadata['joint_names'][14:34])
    assert all(name.startswith('r_') for name in metadata['joint_names'][34:54])
    assert metadata['image_encoding'] == 'rgb'
    assert metadata['jpeg_quality'] is None


def test_two_views_cannot_alias_the_same_camera(tmp_path):
    config = json.loads(CONFIG.read_text())
    config['cameras']['left_wrist'] = config['cameras']['top']
    path = tmp_path / 'collection.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='distinct'):
        load_collection_config(path)


def test_disabled_camera_must_be_explicit_integer_zero(tmp_path):
    config = json.loads(CONFIG.read_text())
    config['cameras']['right_wrist'] = False
    path = tmp_path / 'collection.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        load_collection_config(path)
