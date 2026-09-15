"""官方右手仿真与 Hand2 真机一键启动脚本合同。"""

from __future__ import annotations

import subprocess
import unittest
import os
import shutil
import sys
import signal
import time
from pathlib import Path
from tempfile import TemporaryDirectory

from .test_dataglove_network import NetworkSandbox


PROJECT_ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = PROJECT_ROOT / "scripts/launch_official_real_v2_right.sh"


class OfficialRealLauncherTest(unittest.TestCase):
    def test_interrupt_stops_real_before_official_simulation(self) -> None:
        with TemporaryDirectory() as directory:
            temp_dir = Path(directory)
            trace = temp_dir / "trace.log"
            net_root = temp_dir / "sys-class-net"
            glove_interface = net_root / "usb-renamed"
            glove_interface.mkdir(parents=True)
            (glove_interface / "address").write_text(
                "02:33:80:00:00:01\n",
                encoding="utf-8",
            )
            fake_pixi = temp_dir / "fake-pixi"
            fake_pixi.write_text(
                """#!/usr/bin/env python3
import os
import signal
import sys
import time
from pathlib import Path

trace = Path(os.environ["FAKE_LAUNCH_TRACE"])
command = " ".join(sys.argv)

def record(message):
    with trace.open("a", encoding="utf-8") as file:
        file.write(message + "\\n")

if "official-retarget-v2-right" in command:
    if "--publish-targets" not in sys.argv:
        raise SystemExit(8)
    record("sim_started")
    def stop_sim(_signum, _frame):
        record("sim_stopped")
        raise SystemExit(0)
    signal.signal(signal.SIGINT, stop_sim)
    signal.signal(signal.SIGTERM, stop_sim)
    while True:
        time.sleep(0.02)

if "real-v2-right-realtime" in command:
    record("real_started")
    def stop_real(_signum, _frame):
        record("real_stopped")
        raise SystemExit(130)
    signal.signal(signal.SIGINT, stop_real)
    signal.signal(signal.SIGTERM, stop_real)
    while True:
        time.sleep(0.02)

if "run python" in command:
    if len(sys.argv) == 4:
        marker = trace.with_suffix(".python_noarg")
        if not marker.exists():
            marker.write_text("existing lease checked", encoding="utf-8")
            raise SystemExit(1)
    if sys.argv[-1].isdigit():
        raise SystemExit(0 if trace.exists() and "sim_started" in trace.read_text() else 1)
    raise SystemExit(0)
raise SystemExit(9)
""",
                encoding="utf-8",
            )
            fake_pixi.chmod(0o755)
            nc_command = temp_dir / "nc"
            nc_command.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
            nc_command.chmod(0o755)
            ip_command = temp_dir / "ip"
            ip_command.write_text(
                """#!/bin/sh
if [ "$1" = "route" ] && [ "$2" = "get" ]; then
  if [ "$3" = "192.168.7.2" ]; then
    echo "192.168.7.2 dev usb-renamed src 192.168.7.1"
  else
    echo "192.168.1.111 dev enp4s0 src 192.168.1.100"
  fi
fi
case "$*" in
  *"addr show dev usb-renamed"*)
    echo "2: usb-renamed inet 192.168.7.1/24 scope global usb-renamed"
    ;;
esac
exit 0
""",
                encoding="utf-8",
            )
            ip_command.chmod(0o755)
            sudo_command = temp_dir / "sudo"
            sudo_command.write_text(
                "#!/bin/sh\nif [ \"$1\" = \"-n\" ]; then shift; fi\nexec \"$@\"\n",
                encoding="utf-8",
            )
            sudo_command.chmod(0o755)
            for name in ("nmcli", "id"):
                command = temp_dir / name
                command.write_text("#!/bin/sh\nexit 1\n", encoding="utf-8")
                command.chmod(0o755)
            for name in ("bash", "sh", "dirname", "basename", "sleep", "setsid", "cat"):
                executable = shutil.which(name)
                self.assertIsNotNone(executable)
                (temp_dir / name).symlink_to(executable)
            (temp_dir / "python3").symlink_to(sys.executable)
            env = {
                key: value for key, value in os.environ.items()
                if not key.startswith(("DATAGLOVE_", "WUJI_HAND2_"))
            }
            env["PATH"] = str(temp_dir)
            env["DATAGLOVE_PIXI_BIN"] = str(fake_pixi)
            env["FAKE_LAUNCH_TRACE"] = str(trace)
            env["DATAGLOVE_SYS_CLASS_NET"] = str(net_root)
            env["WUJI_HAND2_SN"] = "TEST-HAND2-RIGHT"
            process = subprocess.Popen(
                [str(LAUNCHER), "--confirm-real"],
                cwd=PROJECT_ROOT,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                start_new_session=True,
            )
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                if trace.exists() and "real_started" in trace.read_text(
                    encoding="utf-8"
                ):
                    break
                time.sleep(0.02)
            else:
                process.kill()
                stdout, stderr = process.communicate(timeout=1)
                self.fail(
                    "假真机未启动\n"
                    f"stdout={stdout}\n"
                    f"stderr={stderr}\n"
                    f"trace={trace.read_text(encoding='utf-8') if trace.exists() else ''}"
                )

            os.killpg(process.pid, signal.SIGINT)
            _stdout, _stderr = process.communicate(timeout=5)
            events = trace.read_text(encoding="utf-8").splitlines()

        self.assertIn(process.returncode, (130, -signal.SIGINT))
        self.assertLess(events.index("real_stopped"), events.index("sim_stopped"))

    def test_requires_explicit_real_confirmation_before_startup(self) -> None:
        with TemporaryDirectory() as directory:
            network = NetworkSandbox(Path(directory))
            result = network.run(script=LAUNCHER)
            self.assertEqual(network.state["calls"], [])

        self.assertEqual(result.returncode, 2)

    def test_missing_serial_fails_before_network_or_process_startup(self) -> None:
        with TemporaryDirectory() as directory:
            network = NetworkSandbox(Path(directory))
            network.env.pop("WUJI_HAND2_SN", None)
            result = subprocess.run(
                [str(network.bin / "bash"), str(LAUNCHER), "--confirm-real"],
                cwd=PROJECT_ROOT,
                env=network.env,
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
            )

            self.assertEqual(result.returncode, 2)
            self.assertFalse(network.state_file.exists())



if __name__ == "__main__":
    unittest.main()
