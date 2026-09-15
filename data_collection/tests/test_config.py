import json
from pathlib import Path

import pytest

from data_collection.config import dataset_metadata, load_collection_config

ROOT = Path(__file__).resolve().parents[2]


def test_metadata_uses_real_model_joint_order():
    config, cameras = load_collection_config(ROOT / 'collection_config.json')
    metadata = dataset_metadata(config, cameras,
        ROOT / 'control/models/marvin_m6_wuji2.xml')
    assert len(metadata['joint_names']) == 54
    assert metadata['joint_names'][:14] == [f'Joint{i}_{side}' for side in ('L', 'R') for i in range(1, 8)]
    assert all(name.startswith('l_') for name in metadata['joint_names'][14:34])
    assert all(name.startswith('r_') for name in metadata['joint_names'][34:54])
    assert metadata['image_encoding'] == 'rgb'
    assert metadata['jpeg_quality'] is None


def test_two_views_cannot_alias_the_same_camera(tmp_path):
    config = json.loads((ROOT / 'collection_config.json').read_text())
    config['cameras']['left_wrist'] = config['cameras']['top']
    path = tmp_path / 'collection.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='distinct'):
        load_collection_config(path)


def test_disabled_camera_must_be_explicit_integer_zero(tmp_path):
    config = json.loads((ROOT / 'collection_config.json').read_text())
    config['cameras']['right_wrist'] = False
    path = tmp_path / 'collection.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        load_collection_config(path)
