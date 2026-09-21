"""The workspace stop command must not signal unrelated Python programs."""
import signal
import subprocess
import sys
import time

from tianji_runtime import workspace


def test_stop_only_signals_the_exact_checkout_entrypoint(tmp_path):
    root = tmp_path / 'checkout'
    (root / 'bash').mkdir(parents=True)
    stop = root / 'bash/stop.sh'
    stop.write_bytes((workspace() / 'bash/stop.sh').read_bytes())
    pico_stop = root / 'bash/run_stop_pico.sh'
    pico_stop.write_text('#!/usr/bin/env bash\nexit 0\n')
    pico_stop.chmod(0o755)
    paths = [
        root / 'src/simulation/simulation/run_sim.py',
        root / 'unrelated.py',
        tmp_path / 'other/src/simulation/simulation/run_sim.py',
    ]
    processes = []
    try:
        for path in paths:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                'from pathlib import Path\nimport sys, time\n'
                'Path(sys.argv[0]).with_suffix(".ready").touch()\n'
                'time.sleep(60)\n'
            )
            processes.append(subprocess.Popen([sys.executable, str(path)], cwd=root))
        deadline = time.monotonic() + 5
        while not all(path.with_suffix('.ready').exists() for path in paths):
            assert all(process.poll() is None for process in processes)
            assert time.monotonic() < deadline
            time.sleep(0.01)
        result = subprocess.run(
            ['bash', str(stop)], cwd=root, capture_output=True, text=True, timeout=20,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        assert processes[0].wait(timeout=2) == -signal.SIGTERM
        assert processes[1].poll() is None
        assert processes[2].poll() is None
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=5)
