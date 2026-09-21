"""Offline post-M0 synthetic anthropometry experiment; never a replay publisher."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import zlib

import numpy as np

from audit_shared_root_trace import audit, digest
from tianji_runtime import controller_profile, native_executable


def transform_points(points, height_ratio, shoulder_ratio=1., upper_ratio=1.,
                     forearm_ratio=1., wrist_ratio=1.):
    """Preserve joint directions, change shoulder offsets and segment lengths."""
    factors = np.asarray([height_ratio, shoulder_ratio, upper_ratio,
                          forearm_ratio, wrist_ratio])
    if not np.isfinite(factors).all() or np.any(factors <= 0):
        raise ValueError("body ratios must be positive and finite")
    root = np.array([0., 0., 1.121])
    out = np.empty_like(points)
    out[:, 0] = root + (points[:, 0] - root) * height_ratio * shoulder_ratio
    for joint, ratio in enumerate((upper_ratio, forearm_ratio, wrist_ratio), 1):
        out[:, joint] = out[:, joint-1] + (points[:, joint]-points[:, joint-1])*height_ratio*ratio
    return out


def synthesize(data, **ratios):
    out = bytearray(data)
    for offset in range(16, len(out), 664):
        start = offset + 8
        points = np.asarray(struct.unpack_from('<24d', out, start+204)).reshape(2, 4, 3)
        transformed = transform_points(points, **ratios)
        struct.pack_into('<24d', out, start+204, *transformed.ravel())
        struct.pack_into('<I', out, start+652, zlib.crc32(out[start:start+652]))
    return out


def parse_output(output):
    frames = {}
    summaries = []
    coverage = {}
    for line in output.splitlines():
        if line.startswith('mapping_frame '):
            fields = line.split()
            key = (int(fields[1]), int(fields[2]))
            if key in frames:
                raise ValueError('duplicate mapping frame key')
            values = np.asarray([float(x) for x in fields[4:]]) if fields[3] == '1' else None
            if values is not None and (values.size != 48 or not np.isfinite(values).all()):
                raise ValueError('invalid mapping frame payload')
            frames[key] = values
        else:
            summaries.append(line)
        if line.startswith('mapping_coverage '):
            coverage = dict(field.split('=', 1) for field in line.split()[1:] if '=' in field)
    return frames, summaries, coverage


def compare(baseline, candidate):
    if baseline.keys() != candidate.keys():
        raise ValueError('mapping frame identities differ')
    errors = []
    rotation_errors = []
    mask_changes = 0
    for key, reference in baseline.items():
        value = candidate[key]
        mask_changes += (reference is None) != (value is None)
        if reference is None or value is None:
            continue
        for side in (0, 24):
            errors.append(float(np.linalg.norm(value[side+12:side+15]-reference[side+12:side+15])))
            rotation_errors.append(float(np.linalg.norm(value[side+15:side+24]-reference[side+15:side+24])))
    return dict(valid_mask_changed_frames=mask_changes, paired_valid_frames=len(errors)//2,
                palm_target_difference_p90_m=float(np.percentile(errors, 90)) if errors else None,
                palm_target_difference_max_m=max(errors) if errors else None,
                palm_rotation_matrix_difference_max=max(rotation_errors) if rotation_errors else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--calibration-dir', type=Path, required=True)
    parser.add_argument('--profile', type=Path)
    parser.add_argument('--auditor', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.profile is None:
        args.profile = controller_profile('qp_ik_pico_shared_root.yaml')
    if args.auditor is None:
        args.auditor = native_executable('tianji_shared_root_trace_audit')
    source = audit(args.trace, args.calibration_dir)
    if not source['geometry_consistent']:
        raise ValueError('source geometry validation failed')
    data = args.trace.read_bytes()
    import hashlib
    if hashlib.sha256(data).hexdigest() != source['trace_sha256']:
        raise ValueError('source changed after validation')
    height = float(source['calibration']['height_m'])
    profile_hash = digest(args.profile)
    auditor_hash = digest(args.auditor)
    cases = [('baseline', dict(height_ratio=1.))]
    cases += [(f'height_{h:.2f}', dict(height_ratio=h/height)) for h in (1.45, 1.50, 1.75, 1.85, 1.95)]
    cases += [(name, dict(height_ratio=1., **ratios)) for name, ratios in (
        ('arms_short_10pct', dict(upper_ratio=.9, forearm_ratio=.9)),
        ('arms_long_10pct', dict(upper_ratio=1.1, forearm_ratio=1.1)),
        ('shoulders_narrow_15pct', dict(shoulder_ratio=.85)),
        ('shoulders_wide_15pct', dict(shoulder_ratio=1.15)),
        ('upper_long_forearm_short', dict(upper_ratio=1.1, forearm_ratio=.9)),
    )]
    args.output_dir.mkdir(parents=True, exist_ok=False)
    results = dict(scope='synthetic_post_M0_mapping_only', source=source,
        profile=str(args.profile), profile_sha256=profile_hash, auditor_sha256=auditor_hash,
        motion_authorized=False, hardware_acceptance=False, ik_executed=False,
        limitations=['Synthetic input, not measurements from other people.',
            'Only TJVR v4 skeleton positions and CRC are changed; legacy poses are retained and not used by this mapping mode.',
            'Rotations, directions, timestamps and identities are unchanged. Do not replay these traces to a live executor.',
            'Does not test upstream PICO estimation/template clamping or network stream jump gates.',
            'Palm differences are against baseline mapped targets, not IK tracking errors.'], cases=[])
    baseline = None
    failed = False
    for name, ratios in cases:
        trace = args.output_dir / f'{name}.synthetic.tjvr'
        with trace.open('xb') as stream:
            stream.write(data if name == 'baseline' else synthesize(data, **ratios))
        completed = subprocess.run([str(args.auditor.resolve()), str(args.profile.resolve()),
                                   str(trace.resolve()), '--mapping-frames'],
                                  capture_output=True, text=True, timeout=180)
        with (args.output_dir / f'{name}.log').open('x') as stream:
            stream.write(completed.stdout + '\nSTDERR:\n' + completed.stderr)
        frames, summary, coverage = parse_output(completed.stdout)
        ok = completed.returncode == 0 and len(frames) == source['records'] and bool(coverage)
        failed |= not ok
        if name == 'baseline':
            if not ok:
                raise RuntimeError('baseline audit failed; inspect retained log')
            baseline = frames
        result = dict(name=name, ratios=ratios, synthetic_height_m=height*ratios['height_ratio'],
                      success=ok, returncode=completed.returncode, trace_sha256=digest(trace),
                      coverage=coverage, summaries=summary,
                      comparison=compare(baseline, frames) if ok else None)
        results['cases'].append(result)
        print(json.dumps({k: result[k] for k in ('name', 'success', 'coverage', 'comparison') }), flush=True)
    results['inputs_unchanged'] = (digest(args.trace) == source['trace_sha256']
        and digest(args.profile) == profile_hash and digest(args.auditor) == auditor_hash)
    with (args.output_dir / 'results.json').open('x') as stream:
        json.dump(results, stream, indent=2, allow_nan=False)
    return 2 if failed or not results['inputs_unchanged'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
