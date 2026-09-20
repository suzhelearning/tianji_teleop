"""Summarize completed external Ceres replay trials (stdout JSON; read-only)."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def summarize(path, targets):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows or {r['algorithm'] for r in rows} != {'pico_ee_franka_ceres_lm'}:
        raise ValueError('missing data or algorithm changed')
    times = np.array([float(r['replay_timestamp_s']) for r in rows])
    if times.max() < targets[-1, 0] - .01:
        raise ValueError('replay did not finish the exported segment')
    selected = [r for r, t in zip(rows, times) if 5 <= t <= 47]
    if not selected or any(int(r['paused']) for r in selected):
        raise ValueError('missing or paused action interval')
    result = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  rows=len(rows), action_rows=len(selected),
                  replay_end_s=float(times.max()), interval_s=[5, 47],
                  quantiles=['p50', 'p90', 'p99', 'max'],
                  control_failures=max(int(r['control_failures']) for r in rows))
    ts = np.array([float(r['replay_timestamp_s']) for r in selected])
    for side, offset in [('left', 1), ('right', 8)]:
        arm = {}
        for field in ['position_error_m', 'orientation_error_rad',
                      'pico_ee_dls_final_position_error_m',
                      'pico_ee_dls_final_orientation_error_rad',
                      'pico_ee_dls_solve_time_us']:
            values = np.array([float(r[side+'_'+field]) for r in selected])
            if not np.isfinite(values).all():
                raise ValueError('nonfinite metric')
            arm[field] = np.quantile(values, [.5, .9, .99, 1]).tolist()
        for flag in ['target_stale', 'pico_ee_dls_target_held', 'pico_ee_dls_trajectory_accepted']:
            arm[flag+'_fraction'] = float(np.mean([int(r[side+'_'+flag]) for r in selected]))
        actual_target = np.array([[float(r[side+'_target_p'+axis]) for axis in 'xyz'] for r in selected])
        expected = np.column_stack([np.interp(ts, targets[:, 0], targets[:, offset+k]) for k in range(3)])
        arm['target_position_vs_export_max_m'] = float(np.linalg.norm(actual_target-expected, axis=1).max())
        actual_rotation = Rotation.from_quat([[float(r[side+'_target_q'+axis]) for axis in 'xyzw'] for r in selected])
        expected_rotation = Slerp(targets[:, 0], Rotation.from_quat(targets[:, offset+3:offset+7]))(ts)
        arm['target_orientation_vs_export_max_rad'] = float((expected_rotation.inv()*actual_rotation).magnitude().max())
        result[side] = arm
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial_directory', type=Path)
    args = parser.parse_args()
    folder = args.trial_directory.resolve()
    targets = np.loadtxt(folder/'mapped_targets.csv', delimiter=',', skiprows=1)
    print(json.dumps(dict(manifest=json.loads((folder/'manifest.json').read_text()),
                          trials={name: summarize(folder/(name+'_complete.csv'), targets)
                                  for name in ['matched', 'source']}), indent=2))
