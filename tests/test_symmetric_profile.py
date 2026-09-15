from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.pico_symmetric_profile import generate, resolve, POINTER
from pico_symmetric_geometry import FILES, POLICY_FILE, apply_profile, symmetric_lengths


@pytest.fixture
def original(tmp_path):
    source = ROOT / 'profiles/syz/pico/20260908-01'
    result = tmp_path / '.draft'
    shutil.copytree(source, result)
    return result


def test_preserves_original_and_survives_publication(original):
    before = {name: (original / name).read_bytes() for name in FILES}
    snapshot = generate(original)
    assert resolve(original) == snapshot
    assert before == {name: (original / name).read_bytes() for name in FILES}
    published = original.with_name('cal-published')
    original.rename(published)
    assert resolve(published) == published / snapshot.relative_to(original)
    assert not any(path.is_symlink() for path in published.rglob('*'))


def test_old_pointer_not_updated_on_failure(original):
    generate(original)
    before = (original / POINTER).read_bytes()
    (original / FILES[0]).write_text('invalid: true')
    with pytest.raises((ValueError, KeyError, TypeError)):
        generate(original)
    assert (original / POINTER).read_bytes() == before
    with pytest.raises(ValueError, match='stale'):
        resolve(original)


def test_pending_bilateral_does_not_publish(original):
    (original / FILES[0]).unlink()
    assert generate(original) is None
    assert not (original / POINTER).exists()


def test_runtime_only_changes_lengths(original):
    snapshot = generate(original)
    @dataclass(frozen=True)
    class Geometry:
        upper_arm_length_m: float
        forearm_length_m: float
        shoulder_anchor: tuple = (1, 2, 3)
    geometries = {}
    paths = {}
    for side in ('left', 'right'):
        paths[side] = snapshot / f'pico_{side}_arm_geometry.yaml'
        doc = yaml.safe_load(paths[side].read_text())
        geometries[side] = Geometry(doc['upper_arm_length_m'], doc['forearm_length_m'])
    applied, policy = apply_profile(geometries, paths)
    expected = symmetric_lengths({s: (g.upper_arm_length_m, g.forearm_length_m) for s,g in geometries.items()})
    for g in applied.values():
        assert (g.upper_arm_length_m, g.forearm_length_m) == expected
        assert g.shoulder_anchor == (1,2,3)
    policy['complete'] = False
    (snapshot / POLICY_FILE).write_text(json.dumps(policy))
    with pytest.raises(ValueError):
        apply_profile(geometries, paths)


def test_rejects_escaping_pointer(original):
    generate(original)
    (original / POINTER).write_text(json.dumps({'schema_version':1, 'relative_path':'../other'}))
    with pytest.raises(ValueError):
        resolve(original)


def test_original_runtime_unchanged_and_invalid_lengths_rejected(original):
    geometries = {'left': None, 'right': None}
    paths = {s: original / f'pico_{s}_arm_geometry.yaml' for s in geometries}
    assert apply_profile(geometries, paths) == (geometries, None)
    with pytest.raises(ValueError):
        symmetric_lengths({'left': (.27, .17), 'right': (.28, float('nan'))})
