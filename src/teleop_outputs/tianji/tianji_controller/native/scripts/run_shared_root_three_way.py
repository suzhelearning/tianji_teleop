#!/usr/bin/env python3
"""Three serial online, model-reference-only Viewer runs per raw TJVR trace."""
import argparse
import hashlib
import json
from pathlib import Path
import socket
import subprocess
import sys
import time
import yaml
from tianji_runtime import controller_profile, native_executable
from tianji_runtime.resources import controller_resource

SCRIPTS = Path(__file__).resolve().parent
PROFILES = {
    "spark": "qp_ik_pico_shared_root_reachable.yaml",
    "ceres": "qp_ik_pico_shared_root_ceres.yaml",
    "dls": "qp_ik_pico_shared_root_dls.yaml",
}
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--viewer", type=Path)
    parser.add_argument("--trace", type=Path, action="append", required=True)
    args = parser.parse_args()
    viewer = args.viewer.resolve() if args.viewer is not None else native_executable("tianji_qp_ik_viewer")
    traces = [p.resolve() for p in args.trace]
    from run_pico_trace_algorithm_benchmark import _read_trace
    records = [_read_trace(p)[1] for p in traces]
    if any(not rows for rows in records):
        raise ValueError("empty trace")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = {"scope": "raw UDP replay, online IK and smoothing, MuJoCo model-reference only",
                "viewer": str(viewer), "viewer_sha256": sha(viewer), "runs": []}
    configs = {}
    for name, filename in PROFILES.items():
        profile = controller_profile(filename)
        cfg = yaml.safe_load(profile.read_text())
        assert not cfg["spark_shared_root"]["enabled"]
        cfg["spark_shared_root"]["enabled"] = True
        for key in ("input_contract_artifact", "robot_geometry_artifact"):
            cfg["spark_shared_root"][key] = str(controller_resource(profile, cfg["spark_shared_root"][key]))
        for key in ("home_config", "pico_ee_dls_kinematics_urdf_path"):
            if cfg["controller"].get(key):
                cfg["controller"][key] = str(controller_resource(profile, cfg["controller"][key]))
        configs[name] = cfg
    # Reject accidental geometry, home, mapping, rate or safety-envelope mismatch.
    for name in ("ceres", "dls"):
        for block in ("spark_shared_root", "joint_limits"):
            assert configs[name][block] == configs["spark"][block], (name, block)
        for key in ("rate_hz", "model_state_only", "initial_posture_enabled",
                    "initial_left_q_rad", "initial_right_q_rad"):
            assert configs[name]["controller"][key] == configs["spark"]["controller"][key], key
    for index, (trace, packets) in enumerate(zip(traces, records), 1):
        duration = packets[-1][0]*1e-9 + 8
        folder = output/f"dataset{index}"
        folder.mkdir()
        for name, filename in PROFILES.items():
            cfg_path = folder/f"{name}.yaml"
            with cfg_path.open("x") as f:
                yaml.safe_dump(configs[name], f, sort_keys=False)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind(("127.0.0.1", 0))
                port = sock.getsockname()[1]
            command = [str(viewer), "--config", str(cfg_path), "--pico-teleop",
                       "--pico-bind", "127.0.0.1", "--pico-port", str(port),
                       "--model-state-only", "--headless", "--duration", str(duration),
                       "--telemetry", str(folder/f"{name}.csv"),
                       "--joint-telemetry", str(folder/f"{name}_joints.csv")]
            item = {"dataset": index, "algorithm": name, "trace": str(trace),
                    "trace_sha256": sha(trace), "frames": len(packets),
                    "profile_sha256": sha(controller_profile(filename)),
                    "runtime_config_sha256": sha(cfg_path), "command": command,
                    "complete": False}
            manifest["runs"].append(item)
            (output/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
            print("START", index, name, flush=True)
            with (folder/f"{name}.log").open("x") as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                try:
                    time.sleep(2)
                    if process.poll() is not None:
                        raise RuntimeError(f"Viewer exited before replay: {folder/name}")
                    replay = subprocess.run(
                        [sys.executable, str(SCRIPTS/"replay_pico_udp_trace.py"),
                         "--input", str(trace), "--host", "127.0.0.1", "--port", str(port), "--lead", "2"],
                        capture_output=True, text=True, timeout=duration+10)
                    item["replay_stdout"] = replay.stdout
                    item["replay_stderr"] = replay.stderr
                    replay.check_returncode()
                    item["viewer_exit"] = process.wait(timeout=20)
                    if item["viewer_exit"] not in (0, 2):
                        raise RuntimeError(f"Viewer failed: {item['viewer_exit']}")
                    item["complete"] = True
                finally:
                    if process.poll() is None:
                        process.terminate()
                        process.wait(timeout=5)
                    (output/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
            print("FINISH", index, name, item["replay_stdout"].strip(), flush=True)
    assert sha(viewer) == manifest["viewer_sha256"], "Viewer binary changed during test"

if __name__ == "__main__":
    main()
