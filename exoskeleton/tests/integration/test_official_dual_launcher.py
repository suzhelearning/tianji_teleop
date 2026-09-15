"""双手编排的真实进程/租约测试；以文件 IPC 和替身 SDK 隔离全部设备边界。"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
import unittest
from contextlib import ExitStack, contextmanager
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from data_glove_wuji_teleop import official_teleop
from data_glove_wuji_teleop.adapters.runtime.simulation_session import SimulationLease, require_active_simulation
from data_glove_wuji_teleop.adapters.hardware import discovery
from data_glove_wuji_teleop.domain.hand_target import HandTarget
from data_glove_wuji_teleop.profiles.dataglove.device import GloveDeviceProfile
from data_glove_wuji_teleop.profiles.dataglove.urdf_zero import URDF_ZERO_GROUPS, UrdfZeroProfile
from data_glove_wuji_teleop.profiles.teleop_task import TeleopTask
from data_glove_wuji_teleop.project import PROJECT_ROOT


FIXTURE = Path(__file__).resolve()


def _event(root, name):
    # 单次 O_APPEND 写入，保留不同进程之间的真实事件顺序。
    with (root / "events").open("a", encoding="utf-8") as stream:
        stream.write(name + "\n")


def _deny_devices(root):
    sys.modules["wuji_sdk"] = None
    (root / f"group-{os.getpgrp()}").touch()

    def guard(event, args):
        if event in ("socket.connect", "socket.bind", "socket.sendto"):
            _event(root, "forbidden_network")
            raise RuntimeError("离线夹具禁止网络")
        if event == "subprocess.Popen":
            command = args[1]
            if not isinstance(command, (list, tuple)) or str(FIXTURE) not in command:
                _event(root, "forbidden_process")
                raise RuntimeError("离线夹具只允许启动自身，不允许 SDK、网卡或真实控制子进程")

    sys.addaudithook(guard)


def _fixture_simulation(root, argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--hand", required=True)
    parser.add_argument("--publish-targets", action="store_true")
    args, _ = parser.parse_known_args(argv)
    hand = args.hand
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    _event(root, f"sim_{hand}_started")
    with ExitStack() as stack:
        if args.publish_targets:
            stack.enter_context(SimulationLease(generation="v2", hand=hand))
        try:
            first = True
            sequence = 0
            while running:
                if (root / f"orphan-{hand}").exists():
                    subprocess.Popen([sys.executable, str(FIXTURE), "--fixture", "worker", str(root)])
                    deadline = time.monotonic() + 5
                    while not (root / "worker-pid").exists() and time.monotonic() < deadline:
                        time.sleep(0.01)
                    return 7
                if (root / f"fail-{hand}").exists():
                    return 7
                if args.publish_targets and (root / f"ready-{hand}").exists():
                    source = "shutdown" if (root / f"shutdown-{hand}").exists() else "official_wuji_retarget"
                    timestamp = time.time_ns()
                    if (root / f"stale-{hand}").exists():
                        timestamp -= 10_000_000_000
                    payload = dict(hand=hand, values=[0.0] * 20, source=source,
                                   sequence=sequence, timestamp_ns=timestamp, dropped=0)
                    pending = root / f"{hand}.pending"
                    pending.write_text(json.dumps(payload), encoding="utf-8")
                    if first:
                        _event(root, f"frame_{hand}")
                        first = False
                    pending.replace(root / f"{hand}.frame")
                    sequence += 1
                time.sleep(0.01)
        finally:
            _event(root, f"sim_{hand}_stopped")
    return 0


def _fixture_hardware(root, argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--hand", required=True)
    parser.add_argument("--hand-sn", default="")
    parser.add_argument("--left-hand-sn", default="")
    parser.add_argument("--right-hand-sn", default="")
    args, _ = parser.parse_known_args(argv)
    hands = ("left", "right") if args.hand == "both" else (args.hand,)
    deadline = time.monotonic() + 3.0
    while True:
        fresh = True
        for hand in hands:
            require_active_simulation(generation="v2", hand=hand)
            target = json.loads((root / f"{hand}.frame").read_text())
            fresh &= target["source"] == "official_wuji_retarget" and 0 <= time.time_ns() - target["timestamp_ns"] < 500_000_000
        if fresh:
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("替身控制器未取得新鲜来源")
        time.sleep(0.01)
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    serials = {hand: args.hand_sn if args.hand != "both" else getattr(args, f"{hand}_hand_sn") for hand in hands}
    (root / "control-output").write_text(json.dumps(serials), encoding="utf-8")
    _event(root, "hardware_started")
    try:
        while running and not (root / "finish-hardware").exists():
            time.sleep(0.01)
    finally:
        time.sleep(0.05)
        _event(root, "hardware_homed")
        _event(root, "hardware_stopped")
    return 0


def _fixture_worker(root):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    (root / "worker-pid").write_text(str(os.getpid()), encoding="utf-8")
    while True:
        time.sleep(0.05)


def _fixture_coordinator(root, argv):
    class FileReceiver:
        def __init__(self, _host, _port, hand, **_kwargs):
            self.path = root / f"{hand}.frame"
            self.sequence = -1

        def recv(self):
            if not self.path.exists():
                return None
            payload = json.loads(self.path.read_text())
            if payload["sequence"] == self.sequence:
                return None
            self.sequence = payload["sequence"]
            return HandTarget(**payload)

        def close(self):
            pass

    class Device:
        def __init__(self, hand):
            self.hand = hand
            self.serial_number = f"SN-{hand}"

        def handedness(self):
            hand = "left" if (root / "wrong-device-side").exists() else self.hand
            return SimpleNamespace(get=lambda: hand)

        def online_joints_count(self):
            count = 19 if self.hand == "right" and (root / "offline-joint").exists() else 20
            return SimpleNamespace(get=lambda: count)

    class Manager:
        def __init__(self):
            self.connections = {}

        def scan(self):
            _event(root, "sdk_scan")
            return [SimpleNamespace(sn=f"SN-{hand}", device_type="Hand2", transport_type="Udp",
                                    address=f"192.168.1.{110 + index}:7447")
                    for index, hand in enumerate(("left", "right"))]

        def connect(self, *, sn, device_name, options):
            _event(root, f"sdk_connect_{sn}")
            self.connections[device_name] = sn
            return Device(sn.removeprefix("SN-"))

        def disconnect(self, name):
            _event(root, f"sdk_disconnect_{self.connections.pop(name)}")

    sdk = SimpleNamespace(SdkManager=SimpleNamespace(instance=Manager),
                          DeviceType=SimpleNamespace(WujiHand2="Hand2"),
                          TransportType=SimpleNamespace(Udp="Udp", Usb="Usb"),
                          WujiHand2=Device,
                          ConnectOptions=lambda **kwargs: kwargs)
    simulation_command = official_teleop.simulation_command
    hardware_command = official_teleop.hardware_command
    stop_process = official_teleop._stop_process
    children = {}

    def simulation(side, args):
        return [sys.executable, str(FIXTURE), "--fixture", "simulation", str(root),
                *simulation_command(side, args)[4:]]

    def hardware(selected):
        return [sys.executable, str(FIXTURE), "--fixture", "hardware", str(root),
                *hardware_command(selected)[4:]]

    def stop(process, timeout):
        command = process.args
        role = command[3]
        if role == "simulation":
            role = command[command.index("--hand") + 1]
        children[role] = process.pid
        _event(root, f"cleanup_{role}")
        stop_process(process, timeout)
        if (root / f"cleanup-error-{role}").exists():
            raise OSError("测试注入的单进程清理错误")

    def resolve_gloves(profiles, **kwargs):
        if (root / "glove-identity-error").exists():
            raise ValueError("请求的左手ID没有匹配端点；实际只读到右手")
        for profile in profiles:
            _event(root, f"glove_network_{profile.hand}")
        return tuple(profiles)

    with patch.dict(sys.modules, {"wuji_sdk": sdk}), \
            patch.object(official_teleop, "resolve_glove_profiles", side_effect=resolve_gloves), \
            patch.object(discovery, "prepare_robot_network",
                         side_effect=lambda *args, **kwargs: _event(root, "robot_network_ready")), \
            patch.object(official_teleop, "ZmqTargetSubscriber", FileReceiver), \
            patch.object(official_teleop, "simulation_command", simulation), \
            patch.object(official_teleop, "hardware_command", hardware), \
            patch.object(official_teleop, "_stop_process", stop), \
            patch.object(sys, "argv", ["official-teleop", *argv]):
        try:
            return official_teleop.main()
        finally:
            (root / "children.json").write_text(json.dumps(children), encoding="utf-8")


def _synthetic_profile(root, hand):
    """仅测试夹具：合成左手资源不是可发布/可用于实物的标定。"""
    directory = root / hand
    directory.mkdir()
    mapping = json.loads((PROJECT_ROOT / "config/dataglove/urdf/right.json").read_text())
    mapping["hand"] = hand
    zero = UrdfZeroProfile.empty(hand, mapping["expected_cs_by_joint"])
    for group in URDF_ZERO_GROUPS:
        zero, _ = zero.capture_group(group.name, [[0.0] * 21], max_motion_deg=1.0)
    profile = GloveDeviceProfile.from_dict({
        "version": 4, "name": f"synthetic-{hand}", "hand": hand,
        "device_id": "0x0000000000000001" if hand == "left" else "0x0000000000000002",
        "network": {"mac": "02:11:22:33:44:55", "interface": "test0" if hand == "left" else "test1", "host": "192.168.7.2",
                    "port": 5580, "http_port": 5570, "mtu": 8000,
                    "host_address": "192.168.7.1/24" if hand == "left" else "192.168.7.3/24",
                    "route_table": 10001 if hand == "left" else 10002},
        "glove_urdf": str(PROJECT_ROOT / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"),
        "mapping": mapping, "zeros": {"initial": zero.to_dict()}, "active_zero": "initial",
    }, path=directory / "profile.json")
    profile.save()
    return profile.path


def _alive(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0] != "Z"
    except FileNotFoundError:
        return False


class OfficialDualLauncherTest(unittest.TestCase):
    @contextmanager
    def launcher(self, *, real=True, markers=(), extra=(), profiles="both", serials=None, task=False, automatic=False):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            paths = {hand: _synthetic_profile(root, hand) for hand in ("left", "right")}
            for marker in markers:
                (root / marker).touch()
            argv = ["--commission-directions", "--ready-timeout", "5"]
            if task:
                binding = TeleopTask.from_dict(
                    {"version": 1, "name": "fixture", "gloves": {hand: str(path) for hand, path in paths.items()}},
                    path=root / "task.json",
                )
                binding.save()
                argv += ["--task", str(binding.path), "--hand", profiles]
            for hand in ("left", "right") if profiles == "both" else (profiles,):
                if not task:
                    argv += [f"--{hand}-profile", str(paths[hand])]
                if real and not automatic:
                    argv += [f"--{hand}-hand-sn", (serials or {}).get(hand, f"SN-{hand}")]
            if real:
                argv += ["--confirm-real"]
            argv += list(extra)
            env = dict(os.environ, DATAGLOVE_RUNTIME_DIR=str(root / "leases"),
                       PYTHONPATH=str(PROJECT_ROOT / "src"))
            output = (root / "output").open("w+", encoding="utf-8")
            process = subprocess.Popen([sys.executable, str(FIXTURE), "--fixture", "coordinator", str(root), *argv],
                                       env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                yield root, process
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGTERM)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)
                # 不依赖被测清理逻辑写回 children：编排挂起也能回收所有夹具会话。
                for group in root.glob("group-*"):
                    try:
                        os.killpg(int(group.name.removeprefix("group-")), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                output.close()

    def events(self, root):
        path = root / "events"
        return path.read_text().splitlines() if path.exists() else []

    def wait_event(self, root, process, event):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if event in self.events(root):
                return
            if process.poll() is not None:
                break
            time.sleep(0.01)
        self.fail(f"未观察到 {event}: {(root / 'output').read_text()}")

    def assert_stopped(self, root, process, expected):
        self.assertEqual(process.wait(timeout=10), expected, (root / "output").read_text())
        children = json.loads((root / "children.json").read_text())
        self.assertTrue(all(not _alive(pid) for pid in children.values()), children)
        self.assertNotIn("forbidden_network", self.events(root))
        self.assertNotIn("forbidden_process", self.events(root))

    def test_glove_identity_failure_never_starts_simulation_or_robot(self):
        with self.launcher(markers=("glove-identity-error",)) as (root, process):
            self.assert_stopped(root, process, 2)
            events = self.events(root)
            self.assertNotIn("sdk_scan", events)
            self.assertNotIn("sim_left_started", events)
            self.assertNotIn("sim_right_started", events)
            self.assertFalse((root / "control-output").exists())

    def test_one_side_failure_before_ready_never_scans_or_controls(self):
        with self.launcher(markers=("ready-left",)) as (root, process):
            self.wait_event(root, process, "frame_left")
            (root / "fail-right").touch()
            self.assert_stopped(root, process, 2)
            self.assertNotIn("sdk_scan", self.events(root))
            self.assertFalse((root / "control-output").exists())
            self.assertIn("sim_left_stopped", self.events(root))

    def test_both_fresh_frames_gate_sdk_and_control_then_signal_homes_first(self):
        with self.launcher(markers=("ready-left",)) as (root, process):
            self.wait_event(root, process, "frame_left")
            self.assertNotIn("sdk_scan", self.events(root))
            (root / "ready-right").touch()
            self.wait_event(root, process, "hardware_started")
            process.send_signal(signal.SIGTERM)
            self.assert_stopped(root, process, 130)
            events = self.events(root)
            for hand in ("left", "right"):
                self.assertLess(events.index(f"frame_{hand}"), events.index("sdk_scan"))
                self.assertLess(events.index(f"sdk_disconnect_SN-{hand}"),
                                events.index("hardware_started"))
                self.assertLess(events.index("hardware_homed"), events.index(f"sim_{hand}_stopped"))

    def test_single_hand_control_owns_only_selected_side(self):
        for hand in ("left", "right"):
            with self.subTest(hand=hand), self.launcher(
                profiles=hand, markers=(f"ready-{hand}",),
            ) as (root, process):
                self.wait_event(root, process, "hardware_started")
                process.send_signal(signal.SIGINT)
                self.assert_stopped(root, process, 130)
                other = "right" if hand == "left" else "left"
                self.assertNotIn(f"sim_{other}_started", self.events(root))
                self.assertFalse((root / f"{other}.frame").exists())
                self.assertTrue((root / "control-output").exists())

    def test_task_automatically_selects_matching_robot_sides(self):
        for hands in ("left", "both"):
            with self.subTest(hands=hands), self.launcher(
                profiles=hands, task=True, automatic=True, markers=("ready-left", "ready-right"),
            ) as (root, process):
                self.wait_event(root, process, "hardware_started")
                expected = {"left": "SN-left"}
                if hands == "both":
                    expected["right"] = "SN-right"
                self.assertEqual(json.loads((root / "control-output").read_text()), expected)
                process.send_signal(signal.SIGTERM)
                self.assert_stopped(root, process, 130)
                if hands == "left":
                    self.assertNotIn("sim_right_started", self.events(root))

    def test_display_only_exits_both_without_leases_or_control(self):
        with self.launcher(real=False, markers=("ready-left", "ready-right")) as (root, process):
            self.wait_event(root, process, "sim_left_started")
            self.wait_event(root, process, "sim_right_started")
            (root / "fail-right").touch()
            self.assert_stopped(root, process, 7)
            self.assertFalse((root / "leases").exists())
            self.assertFalse((root / "left.frame").exists())
            self.assertFalse((root / "right.frame").exists())
            self.assertFalse((root / "control-output").exists())
            self.assertNotIn("sdk_scan", self.events(root))
            self.assertIn("sim_left_stopped", self.events(root))

    def test_normal_hardware_exit_and_cleanup_error_still_stop_every_simulation(self):
        for failing_role in (None, "hardware", "left"):
            with self.subTest(failing_role=failing_role):
                markers = ["ready-left", "ready-right"]
                if failing_role:
                    markers.append(f"cleanup-error-{failing_role}")
                with self.launcher(markers=markers) as (root, process):
                    self.wait_event(root, process, "hardware_started")
                    (root / "finish-hardware").touch()
                    self.assert_stopped(root, process, 2 if failing_role else 0)
                    events = self.events(root)
                    for hand in ("left", "right"):
                        self.assertIn(f"sim_{hand}_stopped", events)
                        self.assertLess(events.index("hardware_homed"), events.index(f"sim_{hand}_stopped"))

    def test_simulation_failure_after_control_stops_hardware_and_other_side(self):
        with self.launcher(markers=("ready-left", "ready-right")) as (root, process):
            self.wait_event(root, process, "hardware_started")
            (root / "fail-left").touch()
            self.assert_stopped(root, process, 2)
            events = self.events(root)
            self.assertLess(events.index("hardware_homed"), events.index("sim_right_stopped"))

    def test_shutdown_and_stale_sources_never_scan_or_control(self):
        for marker in ("shutdown-right", "stale-right"):
            with self.subTest(marker=marker), self.launcher(
                markers=("ready-left", "ready-right", marker),
            ) as (root, process):
                self.assert_stopped(root, process, 2)
                self.assertNotIn("sdk_scan", self.events(root))
                self.assertFalse((root / "control-output").exists())

    def test_sdk_side_or_online_failure_never_starts_control(self):
        for marker in ("wrong-device-side", "offline-joint"):
            with self.subTest(marker=marker), self.launcher(
                markers=("ready-left", "ready-right", marker),
            ) as (root, process):
                self.assert_stopped(root, process, 2)
                self.assertIn("sdk_scan", self.events(root))
                self.assertFalse((root / "control-output").exists())
                self.assertIn("sim_left_stopped", self.events(root))
                self.assertIn("sim_right_stopped", self.events(root))

    def test_duplicate_serial_and_profile_side_mismatch_fail_before_side_effects(self):
        with self.launcher(serials={"left": "same", "right": "same"}) as (root, process):
            self.assert_stopped(root, process, 2)
            self.assertEqual(self.events(root), [])
        with TemporaryDirectory() as directory:
            wrong = _synthetic_profile(Path(directory), "right")
            with self.launcher(extra=("--left-profile", str(wrong))) as (root, process):
                self.assert_stopped(root, process, 2)
                self.assertEqual(self.events(root), [])

    def test_single_side_and_dual_check_config_have_no_side_effects(self):
        for profiles in ("left", "right", "both"):
            with self.subTest(profiles=profiles), self.launcher(
                real=False, profiles=profiles, extra=("--check-config",),
            ) as (root, process):
                self.assert_stopped(root, process, 0)
                self.assertEqual(self.events(root), [])
                self.assertFalse((root / "leases").exists())

    def test_exited_group_leader_does_not_leave_signal_ignoring_worker(self):
        with self.launcher(real=False, markers=("orphan-left",)) as (root, process):
            self.assert_stopped(root, process, 7)
            worker_pid = int((root / "worker-pid").read_text())
            deadline = time.monotonic() + 2
            while _alive(worker_pid) and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertFalse(_alive(worker_pid), f"遗留 worker {worker_pid}")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--fixture":
        role, root = sys.argv[2], Path(sys.argv[3])
        _deny_devices(root)
        roles = {"coordinator": _fixture_coordinator, "simulation": _fixture_simulation,
                 "hardware": _fixture_hardware}
        if role == "worker":
            _fixture_worker(root)
        else:
            raise SystemExit(roles[role](root, sys.argv[4:]))
    else:
        unittest.main()
