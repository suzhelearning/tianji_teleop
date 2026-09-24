"""Contract of the shared Hand2 retargeting service client.

The client is the only transport every link uses, so its failure isolation is
tested here: a fake service process exercises the real bounded pipe without
importing the Manus solver or depending on its native ABI.
"""
from __future__ import annotations

import shlex
import subprocess
import sys

import numpy as np
import pytest

from tianji_runtime import hand2


FAKE_SERVICE = (
    "import json, os, sys, time\n"
    "failure = {failure!r}\n"
    "assert not any(key in os.environ for key in "
    "('PYTHONPATH', 'PYTHONHOME', 'LD_PRELOAD', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH'))\n"
    "assert sys.flags.isolated\n"
    "request = json.loads(sys.stdin.readline())\n"
    "side = request['side']\n"
    "kind = 'error' if failure == 'startup' else 'ready'\n"
    "print(json.dumps(dict(kind=kind, sequence=0, side=side, error='solver initialization failed')), flush=True)\n"
    "if failure == 'startup': sys.stdin.read(); sys.exit(0)\n"
    "request = json.loads(sys.stdin.readline())\n"
    "if failure == 'timeout': time.sleep(10)\n"
    "joints = [float('nan')] * 20 if failure == 'nonfinite' else [0.] * (19 if failure == 'wrong_shape' else 20)\n"
    "sequence = request['sequence'] + (failure == 'sequence')\n"
    "print(json.dumps(dict(kind='joints', sequence=sequence, side=side, joints=joints)), flush=True)\n"
    "sys.stdin.read()\n"
)


def _fake_service_workspace(tmp_path, failure: str):
    """Workspace whose `manus` interpreter runs the fake service instead."""
    interpreter = tmp_path / ".pixi/envs/manus/bin/python"
    interpreter.parent.mkdir(parents=True)
    service = tmp_path / "fake_service.py"
    service.write_text(FAKE_SERVICE.format(failure=failure))
    interpreter.write_text(
        f"#!/bin/sh\nexec {shlex.quote(sys.executable)} -I {shlex.quote(str(service))}\n")
    interpreter.chmod(0o755)
    return interpreter


@pytest.mark.parametrize("failure,expected", [
    ("startup", RuntimeError),
    ("wrong_shape", RuntimeError),
    ("nonfinite", RuntimeError),
    ("sequence", RuntimeError),
    ("timeout", TimeoutError),
])
def test_pipe_failure_reaps_the_service_without_restarting(tmp_path, monkeypatch, request, failure, expected):
    _fake_service_workspace(tmp_path, failure)
    monkeypatch.setattr(hand2, "workspace", lambda: tmp_path)
    for key in ("PYTHONPATH", "PYTHONHOME", "LD_PRELOAD", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH"):
        monkeypatch.setenv(key, "/foreign/environment")
    processes = []
    popen = subprocess.Popen

    def launch(*args, **kwargs):
        process = popen(*args, **kwargs)
        processes.append(process)

        def reap():
            if process.poll() is None:
                process.kill()
            process.wait()

        request.addfinalizer(reap)
        return process

    monkeypatch.setattr(hand2.subprocess, "Popen", launch)
    if failure == "startup":
        with pytest.raises(expected):
            hand2.Hand2Retargeter("right", startup_timeout=3.)
    else:
        with hand2.Hand2Retargeter("right", timeout=.25, startup_timeout=3.) as solver:
            with pytest.raises(expected):
                solver.retarget(np.zeros((21, 3)))
            assert processes[0].poll() is not None
            # A failed service must not be recreated nor reuse an older pose.
            with pytest.raises(RuntimeError):
                solver.retarget(np.zeros((21, 3)))
            solver.close()
    assert len(processes) == 1
    assert processes[0].poll() is not None


def test_side_is_rejected_before_any_process_starts(tmp_path, monkeypatch):
    monkeypatch.setattr(hand2, "workspace", lambda: tmp_path)
    with pytest.raises(ValueError):
        hand2.Hand2Retargeter("sideways")
