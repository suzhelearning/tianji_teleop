"""Compare offline V131 traces against an explicitly selected reference binary.

The external path is used for this diagnostic only, never by the live runtime.
The reference binary's identity is reported; a historical binary alone does not
prove that every current source file was used to build it.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-binary", type=Path, required=True)
    args = parser.parse_args()
    reference = args.reference_binary.resolve(strict=True)
    candidate = ROOT.parent / "build/pico2-v131/pico2_v131_model_trace"
    urdf = ROOT / "native/models/marvin_m6_s_ccs_696_v4.urdf"
    model = ROOT / "native/models/marvin_m6_qp_pico_fast_kinematics.xml"
    env = dict(os.environ, TIANJI_V131_MODEL=str(model))
    rows = []
    for combined in (False, True):
        results = []
        for binary in (reference, candidate):
            command = [str(binary), str(urdf), str(model)] + (["combined"] if combined else [])
            child = subprocess.run(command, env=env, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True, timeout=30, check=True)
            data = np.loadtxt(io.StringIO(child.stdout))
            if data.shape != (600, 17) or not np.isfinite(data).all():
                raise AssertionError("invalid offline trace dimensions/numerics")
            if not np.array_equal(data[:, 0], np.arange(600)):
                raise AssertionError("invalid offline trace sequence")
            if not (np.sum(data[:, 1]) > 100 and np.sum(data[:, 2]) > 100):
                raise AssertionError("trace did not establish bilateral accepted output")
            results.append(data)
        np.testing.assert_array_equal(results[0][:, :3], results[1][:, :3])
        difference = float(np.max(np.abs(results[0][:, 3:] - results[1][:, 3:])))
        np.testing.assert_allclose(results[0][:, 3:], results[1][:, 3:], rtol=0, atol=1e-8)
        rows.append(dict(combined=combined, frames=600, max_joint_error_rad=difference))
    print(json.dumps(dict(scope="offline_v131_binary_comparison", traces=rows,
        reference_sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),
        candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),
        hardware_acceptance_complete=False)))


if __name__ == "__main__":
    main()
