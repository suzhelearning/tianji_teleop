import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def checkout(tmp_path):
    root = tmp_path / "checkout with spaces"
    root.mkdir()
    shutil.copyfile(ROOT / "exo.sh", root / "exo.sh")
    shutil.copytree(ROOT / "tianji", root / "tianji", ignore=shutil.ignore_patterns("__pycache__"))
    exoskeleton = root / "exoskeleton"
    exoskeleton.mkdir()
    for name in ("pixi.toml", "pixi.lock"):
        (exoskeleton / name).touch()
    python = exoskeleton / ".pixi/envs/default/bin/python"
    python.parent.mkdir(parents=True)
    python.write_text("#!/bin/sh\nexit 91\n")
    python.chmod(0o755)
    binaries = tmp_path / "bin"
    binaries.mkdir()
    pixi = binaries / "pixi"
    pixi.write_text(
        f"#!{sys.executable}\n"
        "import json, os, sys\n"
        "print(json.dumps({'cwd': os.getcwd(), 'argv': sys.argv[1:]}))\n"
        "sys.exit(int(os.environ.get('FAKE_PIXI_EXIT', '0')))\n"
    )
    pixi.chmod(0o755)
    outside = tmp_path / "outside"
    outside.mkdir()
    environment = dict(os.environ, PATH=f"{binaries}:{os.environ['PATH']}", PYTHONPATH=str(root))
    return root, outside, environment


def test_cli_preserves_sender_failure_from_other_cwd(checkout):
    root, outside, environment = checkout
    environment["FAKE_PIXI_EXIT"] = "73"
    arguments = ["--confirm-send", "--udp-host", "127.0.0.1", "--glove-profile",
                 "config/profile with spaces.json", "--commission-directions"]
    result = subprocess.run(
        [sys.executable, "-m", "tianji", "exoskeleton", *arguments],
        cwd=outside, env=environment, capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 73, result.stderr
    invocation = json.loads(result.stdout)
    assert invocation["cwd"] == str(root / "exoskeleton")

def test_missing_environment_does_not_start_pixi(checkout):
    root, outside, environment = checkout
    (root / "exoskeleton/.pixi/envs/default/bin/python").unlink()
    result = subprocess.run(
        ["/bin/bash", str(root / "exo.sh"), "--confirm-send"],
        cwd=outside, env=environment, capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 1
    assert result.stdout == ""
    assert "install.sh --exoskeleton" in result.stderr

