"""Synthetic calibration fixtures; never read or publish an operator profile."""
import hashlib
from pathlib import Path
import runpy
import yaml

ROOT = Path(__file__).resolve().parents[1]

def synthetic_bundle(folder):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    chain = runpy.run_path(str(ROOT / "tracking/src/pico_bridge/test/test_pico_calibration_artifact.py"))["_chain"]
    for side in ("left", "right"):
        paths = chain(folder / ("source_" + side))
        hashes = []
        for source, kind in zip(paths, ("palm_tcp", "wrist_pivot", "arm_geometry")):
            # Deliberately artificial, contract-valid data with rebuilt lineage hashes.
            doc = yaml.safe_load(source.read_text().replace("left", side))
            if kind == "arm_geometry":
                doc["tcp_artifact_sha256"], doc["wrist_pivot_sha256"] = hashes
            target = folder / f"pico_{side}_{kind}.yaml"
            target.write_text(yaml.safe_dump(doc, sort_keys=False))
            hashes.append(hashlib.sha256(target.read_bytes()).hexdigest())
    return folder

def synthetic_profile(root):
    person = Path(root) / "profiles/syz"
    synthetic_bundle(person / "pico/20260908-01")
    (person / "profile.yaml").write_text(yaml.safe_dump({
        "schema_version": 1, "user_id": "syz", "pico": {"active_set": "20260908-01"},
        "manus": {"user": "syz"}}))
    return Path(root)
