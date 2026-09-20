import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

spec=importlib.util.spec_from_file_location('trial',Path(__file__).resolve().parents[1]/'scripts/prepare_shared_root_ceres_trial.py')
trial=importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)


def frame(stamp, valid=True, rotation=None):
    if not valid:
        return f'mapping_frame {stamp} 1 0'
    r=np.eye(3) if rotation is None else rotation
    arm=[0.0]*12+[1.0,2.0,3.0]+r.reshape(-1).tolist()
    return f'mapping_frame {stamp} 1 1 '+' '.join(map(str,arm+arm))


def test_preserves_pose_units_axes_and_xyzw():
    rotation=Rotation.from_euler('xyz',[.3,-.4,.8]).as_matrix()
    rows,excluded=trial.mapping_rows('\n'.join([frame(100,rotation=rotation),frame(100000100,rotation=rotation)]))
    assert excluded==0
    assert rows[0][1:4]==[1,2,3]
    assert rows[1][0]==pytest.approx(.1)
    assert np.allclose(Rotation.from_quat(rows[0][4:8]).as_matrix(),rotation)


def test_never_interpolates_across_invalid_frame():
    rows,excluded=trial.mapping_rows('\n'.join([frame(0),frame(10,False),frame(20),frame(30),frame(40)]))
    assert len(rows)==3 and excluded==2
    assert rows[0][0]==pytest.approx(20e-9)


@pytest.mark.parametrize('data',[
    '\n'.join([frame(10),frame(10)]),
    '\n'.join([frame(10),frame(9)]),
    '\n'.join([frame(0,rotation=np.zeros((3,3))),frame(10)]),
    '\n'.join([frame(0),frame(10,False),frame(20)]),
])
def test_rejects_bad_clock_rotation_or_insufficient_segment(data):
    with pytest.raises(ValueError):
        trial.mapping_rows(data)
