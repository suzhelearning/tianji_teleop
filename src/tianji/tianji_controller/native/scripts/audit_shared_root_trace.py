"""Read-only simple-model/TJVR geometry check, never a motion or A/B acceptance gate."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import struct
import sys
import zlib

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from teleop_profile import _validate_pico_revision


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def calibration(directory):
    directory = Path(directory).resolve(strict=True)
    _validate_pico_revision(directory)
    manifest = json.loads((directory / "pico_simple_model.json").read_text())
    expected = []
    files = {"pico_simple_model.json": digest(directory / "pico_simple_model.json")}
    for side in ("left", "right"):
        for kind in ("arm_geometry", "palm_tcp", "wrist_pivot"):
            name = f"pico_{side}_{kind}.yaml"
            files[name] = digest(directory / name)
        g = yaml.safe_load((directory / f"pico_{side}_arm_geometry.yaml").read_text())
        w = yaml.safe_load((directory / f"pico_{side}_wrist_pivot.yaml").read_text())
        distance = w.get("wrist_to_palm_distance_m")
        if distance is None:
            v = w["wrist_to_palm_m"]
            distance = float(np.linalg.norm([v[a] for a in "xyz"] if isinstance(v, dict) else v))
        expected.append([g["upper_arm_length_m"], g["forearm_length_m"], distance])
    return dict(directory=str(directory), height_m=manifest["height_m"], model=manifest["model"],
                expected_segments_m=expected, files_sha256=files)


def audit(trace, directory, tolerance_m=.001):
    if not np.isfinite(tolerance_m) or not 0 < tolerance_m <= .01:
        raise ValueError("tolerance_m must be within (0,0.01]")
    config = calibration(directory)
    expected = np.asarray(config["expected_segments_m"])
    errors = Counter()
    seen = set(); epochs = set(); samples = []
    last_source = {}
    initial_digest = digest(trace)
    previous_receive = -1
    max_origin = max_length_error = 0.
    with Path(trace).open("rb") as stream:
        header = stream.read(16)
        if len(header) != 16:
            raise ValueError("truncated TJVT header")
        magic, version, size, count = struct.unpack("<4sHHQ", header)
        if (magic, version, size) != (b"TJVT", 1, 656):
            raise ValueError("requires TJVT v1 containing TJVR v4 (656 bytes)")
        if count == 0 or Path(trace).stat().st_size != 16 + count * (8 + size):
            raise ValueError("empty, incomplete or inconsistent TJVT record count")
        for _ in range(count):
            receive = struct.unpack("<Q", stream.read(8))[0]
            packet = stream.read(size)
            if receive < previous_receive:
                errors["receive_time_regression"] += 1
            previous_receive = receive
            if packet[:8] != struct.pack("<4sHH", b"TJVR", 4, 656):
                errors["wire_header"] += 1; continue
            if zlib.crc32(packet[:-4]) != struct.unpack_from("<I", packet, 652)[0]:
                errors["crc"] += 1; continue
            flags = struct.unpack_from("<I", packet, 40)[0]
            if flags & 0xcf != 0xcf or flags & ~0x1ff:
                errors["flags"] += 1; continue
            seq, epoch, timestamp, send = struct.unpack_from("<QQqq", packet, 8)
            if not seq or not epoch or timestamp <= 0 or send <= 0:
                errors["identity"] += 1; continue
            key = (epoch, seq, timestamp)
            if key in seen:
                errors["duplicate"] += 1; continue
            seen.add(key); epochs.add(epoch)
            if epoch in last_source and (seq <= last_source[epoch][0] or timestamp <= last_source[epoch][1]):
                errors["source_order"] += 1
            last_source[epoch] = (seq, timestamp)
            poses = [np.asarray(struct.unpack_from("<7d", packet, offset)) for offset in (44, 100)]
            if any(not np.isfinite(v).all() or abs(np.linalg.norm(v[3:])-1) > .001 for v in poses):
                errors["header_pose"] += 1; continue
            p = np.asarray(struct.unpack_from("<24d", packet, 204)).reshape(2, 4, 3)
            q = np.asarray(struct.unpack_from("<32d", packet, 396)).reshape(8, 4)
            if not np.isfinite(p).all() or not np.isfinite(q).all() or np.max(abs(np.linalg.norm(q, axis=1)-1)) > .001:
                errors["nonfinite_or_rotation"] += 1; continue
            lengths = np.linalg.norm(np.diff(p, axis=1), axis=2)
            origin = float(np.linalg.norm(.5*(p[0,0]+p[1,0])-[0,0,1.121]))
            length_error = float(np.max(abs(lengths-expected)))
            max_origin = max(max_origin, origin)
            max_length_error = max(max_length_error, length_error)
            samples.append(lengths)
            if origin > tolerance_m:
                errors["origin_mismatch"] += 1
            if length_error > tolerance_m:
                errors["effective_geometry_mismatch"] += 1
    if digest(trace) != initial_digest or calibration(directory) != config:
        raise ValueError("trace or calibration changed during audit")
    return dict(scope="offline_geometry_consistency_only", geometry_consistent=not errors and len(samples)==count,
        phase_a_accepted=False, motion_authorized=False, publisher_attestation=False,
        actual="unavailable", trace_sha256=initial_digest, calibration=config, records=count,
        geometry_samples=len(samples), epochs=sorted(epochs), errors=dict(errors),
        tolerance_m=tolerance_m, max_origin_error_m=max_origin if samples else None,
        max_segment_error_m=max_length_error if samples else None,
        median_segments_m=np.median(samples, axis=0).tolist() if samples else None,
        limitations=["does not verify palm basis semantics or physical mirror axes",
                     "does not establish capture-time configuration provenance",
                     "does not perform IK, controller timing or trace A/B qualification"])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--calibration-dir", type=Path, required=True)
    p.add_argument("--trace", type=Path)
    args = p.parse_args()
    try:
        result = audit(args.trace, args.calibration_dir) if args.trace else dict(
            scope="configuration_only", calibration=calibration(args.calibration_dir),
            trace_available=False, phase_a_accepted=False, motion_authorized=False)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if not args.trace or result["geometry_consistent"] else 2
    except (OSError, ValueError, TypeError, KeyError, struct.error, yaml.YAMLError) as error:
        p.exit(2, str(error)+"\n")


if __name__ == "__main__":
    raise SystemExit(main())
