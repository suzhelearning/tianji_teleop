"""使用独立网卡状态模型验证手套网络入口，不接触宿主网络。"""

from __future__ import annotations

import argparse
import json
import os
import pty
import shutil
import subprocess
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from data_glove_wuji_teleop.adapters.glove.network import prepare_glove_network


PROJECT_ROOT = Path(__file__).resolve().parents[2]
NETWORK_SCRIPT = PROJECT_ROOT / "scripts/lib/dataglove_network.sh"

# 所有网络命令共享状态；缺少 src 会沿用第一个（链路本地）地址，
# 不带 + 的 nmcli 地址设置会真正覆盖旧地址，而不是回显成功。
FAKE_NETWORK_TOOL = r'''
import json
import os
import re
import shutil
import sys
from pathlib import Path

state_file = Path(os.environ["FAKE_NETWORK_STATE"])
state = json.loads(state_file.read_text())
command = Path(sys.argv[0]).name
args = sys.argv[1:]
state["calls"].append([command, *args])

def finish(code=0, text=""):
    state_file.write_text(json.dumps(state))
    if text:
        print(text, file=sys.stdout if code == 0 else sys.stderr)
    raise SystemExit(code)

def fail(text):
    finish(97, "unsupported fake command: " + text)

def interface_after(token="dev"):
    return args[args.index(token) + 1]

def change(interface, kind):
    if interface not in state["allowed_interfaces"]:
        state["forbidden"].append([command, *args])
        finish(1, "other interfaces must remain untouched")
    if not (Path(os.environ["DATAGLOVE_SYS_CLASS_NET"]) / interface).exists():
        finish(1, "device disappeared")
    state["mutations"].append([command, kind, interface])

def route_line(destination, source=None, table=None):
    if source is not None:
        for rule in sorted(state["rules"], key=lambda rule: rule["priority"]):
            if rule["source"].split("/")[0] != source:
                continue
            candidate = str(rule["table"])
            if destination in state["tables"].get(candidate, {}):
                table = candidate
                break
    routes = state["routes"] if table is None else state["tables"].get(str(table), {})
    route = routes.get(destination, {"dev": "wifi0", "src": "10.0.0.2"})
    line = destination
    if source is not None:
        line += " from " + source
    line += " dev " + route["dev"]
    if table is not None:
        line += " table " + str(table)
    return line + " src " + route["src"]

if command == "id":
    finish(text="1000")
if command == "sudo":
    noninteractive = bool(args and args[0] == "-n")
    if noninteractive:
        args.pop(0)
    if not noninteractive and not sys.stdin.isatty():
        finish(1, "sudo requires a terminal")
    if state["sudo_denied"] or (
        state.get("sudo_needs_password", False) and noninteractive
    ):
        finish(1, "sudo: a password is required")
    # 假授权器在真实 PTY 上模拟用户授权，不读取真实密码或调用宿主 sudo。
    state["sudo_needs_password"] = False
    state_file.write_text(json.dumps(state))
    os.execv(str(Path(sys.argv[0]).parent / args[0]), args)
if command in ("nc", "pixi"):
    state["forbidden"].append([command, *args])
    finish(1, "hardware access is forbidden")
if command == "ip":
    args = [arg for arg in args if arg not in ("-4", "-o", "-br", "-oneline")]
    if args[:2] == ["route", "get"]:
        source = args[args.index("from") + 1] if "from" in args else None
        finish(text=route_line(args[2], source=source))
    if args[:2] == ["rule", "show"]:
        finish(text="\n".join(
            str(rule["priority"]) + ": from " + rule["source"] + " lookup " + str(rule["table"])
            for rule in sorted(state["rules"], key=lambda rule: rule["priority"])
        ))
    if args[:2] == ["rule", "add"]:
        if state["sudo_denied"] or state.get("rule_add_denied"):
            finish(1, "RTNETLINK answers: Operation not permitted")
        rule = {
            "priority": int(args[args.index("priority") + 1]),
            "source": args[args.index("from") + 1],
            "table": int(args[args.index("lookup") + 1]),
        }
        state["rules"].append(rule)
        state["mutations"].append([command, "rule", rule["source"]])
        finish()
    if args[:2] in (["addr", "show"], ["address", "show"]):
        interface = interface_after() if "dev" in args else args[2] if len(args) > 2 else "usb-glove"
        if not (Path(os.environ["DATAGLOVE_SYS_CLASS_NET"]) / interface).exists():
            finish(1, "Cannot find device")
        finish(text="\n".join("2: " + interface + " inet " + address + " scope global " + interface
                               for address in state["addresses"].get(interface, [])))
    if args[:2] in (["route", "show"], ["route", "list"]):
        table = args[args.index("table") + 1] if "table" in args else None
        routes = state["routes"] if table is None else state["tables"].get(table, {})
        finish(text="\n".join(route_line(destination, table=table) for destination in routes))
    if args[:2] == ["link", "show"]:
        interface = interface_after()
        finish(0 if (Path(os.environ["DATAGLOVE_SYS_CLASS_NET"]) / interface).exists() else 1,
               "2: " + interface + ": <UP,LOWER_UP>")
    if len(args) > 1 and args[1] in ("replace", "add", "set", "flush", "del", "delete"):
        if state["sudo_denied"]:
            finish(1, "RTNETLINK answers: Operation not permitted")
        interface = interface_after()
        change(interface, args[0])
        if args[0] in ("addr", "address"):
            if args[1] in ("flush", "del", "delete"):
                state["addresses"][interface] = []
            elif args[2] not in state["addresses"][interface]:
                state["addresses"][interface].append(args[2])
        elif args[0] == "route":
            source = args[args.index("src") + 1] if "src" in args else state["addresses"][interface][0].split("/")[0]
            table = args[args.index("table") + 1] if "table" in args else None
            routes = state["routes"] if table is None else state["tables"].setdefault(table, {})
            routes[args[2].split("/")[0]] = {"dev": interface, "src": source}
        finish()
    fail("ip " + " ".join(args))
if command == "nmcli":
    if "modify" in args:
        if state["nmcli_mode"] == "denied":
            finish(1, "NetworkManager: not authorized to control networking")
        index = args.index("modify")
        interface = args[index + 1]
        values = args[index + 2:]
        if len(values) % 2:
            fail("nmcli modify property/value pairs")
        for prop, value in zip(values[::2], values[1::2]):
            change(interface, prop)
            if prop.lstrip("+") == "ipv4.addresses":
                addresses = [item.strip() for item in value.split(",")]
                if prop.startswith("+"):
                    state["addresses"][interface] += [item for item in addresses if item not in state["addresses"][interface]]
                else:
                    state["addresses"][interface] = addresses
            elif prop.lstrip("+") == "ipv4.routes":
                if not prop.startswith("+"):
                    state["routes"] = {host: route for host, route in state["routes"].items()
                                       if route["dev"] != interface}
                destination = value.split()[0].split("/")[0]
                source = re.search(r"\bsrc[= ]+(\d+\.\d+\.\d+\.\d+)", value)
                if state["nmcli_mode"] == "ignore_source":
                    source = None
                state["routes"][destination] = {
                    "dev": interface,
                    "src": source.group(1) if source else state["addresses"][interface][0].split("/")[0],
                }
            else:
                fail("nmcli property " + prop)
        if state["nmcli_mode"] == "unplug":
            shutil.rmtree(Path(os.environ["DATAGLOVE_SYS_CLASS_NET"]) / interface)
        finish()
    if "show" in args:
        field = next((args[index + 1] for index, arg in enumerate(args[:-1])
                      if arg in ("-g", "--get-values", "-f", "--fields")), "")
        values = {
            "GENERAL.STATE": "20 (unavailable)" if state["nmcli_mode"] == "unavailable" else "100 (connected)",
            "GENERAL.NM-MANAGED": "yes",
            "GENERAL.CONNECTION": "glove-usb",
            "IP4.ADDRESS": "\n".join(state["addresses"]["usb-glove"]),
            "IP4.ROUTE": "\n".join(route_line(destination) for destination in state["routes"]),
        }
        if field not in values:
            fail("nmcli show field " + field)
        finish(text=values[field])
    if "permissions" in args:
        finish(text="org.freedesktop.NetworkManager.network-control:yes")
    fail("nmcli " + " ".join(args))
fail(command)
'''


class NetworkSandbox:
    """真实 shell + 假 sysfs/外部命令；PATH 不包含任何宿主网络工具。"""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.net = root / "net"
        self.net.mkdir()
        self.bin = root / "bin"
        self.bin.mkdir()
        self.state_file = root / "network.json"
        self.initial = {
            "addresses": {"usb-glove": ["169.254.1.2/16"], "wifi0": ["10.0.0.2/24"]},
            "routes": {
                "192.168.7.2": {"dev": "wifi0", "src": "10.0.0.2"},
                "169.254.4.2": {"dev": "usb-glove", "src": "169.254.1.2"},
            },
            "tables": {},
            "rules": [],
            "allowed_interfaces": ["usb-glove"],
            "calls": [],
            "mutations": [],
            "forbidden": [],
            "nmcli_mode": "ok",
            "sudo_denied": False,
        }
        self.add_interface("usb-glove", "02:33:80:00:00:01")
        self.add_interface("wifi0", "00:11:22:33:44:55")
        for name in ("ip", "nmcli", "sudo", "id", "nc", "pixi"):
            path = self.bin / name
            path.write_text(f"#!{sys.executable}\n" + FAKE_NETWORK_TOOL, encoding="utf-8")
            path.chmod(0o755)
        for name in ("bash", "basename", "dirname", "sleep", "cat"):
            executable = shutil.which(name)
            if executable is None:
                raise RuntimeError(f"缺少测试运行所需命令：{name}")
            (self.bin / name).symlink_to(executable)
        self.env = {key: value for key, value in os.environ.items() if not key.startswith("DATAGLOVE_")}
        self.env.update({
            "PATH": str(self.bin),
            "FAKE_NETWORK_STATE": str(self.state_file),
            "DATAGLOVE_SYS_CLASS_NET": str(self.net),
            "DATAGLOVE_USB_WAIT_TIMEOUT_SECONDS": "1",
        })

    def add_interface(self, name: str, mac: str) -> None:
        interface = self.net / name
        interface.mkdir()
        (interface / "address").write_text(mac + "\n", encoding="utf-8")

    def run(
        self, *, source_only: bool = False, script: Path = NETWORK_SCRIPT,
        terminal: bool = False, reset: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        if reset:
            self.state_file.write_text(json.dumps(self.initial), encoding="utf-8")
        command = [str(self.bin / "bash"), str(script)]
        if source_only:
            command = [str(self.bin / "bash"), "-c", 'source "$1"', "source-test", str(script)]
        master, slave = pty.openpty() if terminal else (None, subprocess.DEVNULL)
        try:
            return subprocess.run(
                command, env=self.env, cwd=PROJECT_ROOT, stdin=slave,
                capture_output=True, text=True, timeout=6, check=False,
            )
        finally:
            if master is not None:
                os.close(slave)
                os.close(master)

    @property
    def state(self) -> dict:
        return json.loads(self.state_file.read_text(encoding="utf-8"))


class DatagloveNetworkTest(unittest.TestCase):
    def setUp(self) -> None:
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.network = NetworkSandbox(Path(directory.name))

    def assert_connected(self) -> None:
        state = self.network.state
        self.assertEqual(state["routes"]["192.168.7.2"], {"dev": "usb-glove", "src": "192.168.7.1"})
        self.assertEqual(set(state["addresses"]["usb-glove"]), {"169.254.1.2/16", "192.168.7.1/24"})
        self.assertEqual(state["addresses"]["wifi0"], ["10.0.0.2/24"])
        self.assertEqual(state["routes"]["169.254.4.2"], {"dev": "usb-glove", "src": "169.254.1.2"})
        self.assertEqual(state["forbidden"], [])

    def test_nmcli_repairs_route_source_without_removing_link_local_address(self) -> None:
        self.network.initial["sudo_denied"] = True
        result = self.network.run()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_connected()
        self.assertTrue(self.network.state["mutations"])
        self.assertEqual({change[0] for change in self.network.state["mutations"]}, {"nmcli"})
        self.assertNotIn("sudo", [call[0] for call in self.network.state["calls"]])

    def test_existing_address_does_not_hide_incorrect_source_route(self) -> None:
        self.network.initial["addresses"]["usb-glove"].append("192.168.7.1/24")
        self.network.initial["routes"]["192.168.7.2"] = {"dev": "usb-glove", "src": "169.254.1.2"}
        result = self.network.run()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_connected()
        self.assertEqual(self.network.state["addresses"]["usb-glove"].count("192.168.7.1/24"), 1)

    def test_falls_back_to_noninteractive_ip_when_nmcli_denies_access(self) -> None:
        self.network.initial["nmcli_mode"] = "denied"
        result = self.network.run()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_connected()
        self.assertIn("ip", {change[0] for change in self.network.state["mutations"]})

    def test_terminal_can_authorize_original_ip_binding_when_nmcli_unavailable(self) -> None:
        self.network.initial.update(nmcli_mode="unavailable", sudo_needs_password=True)
        result = self.network.run(terminal=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_connected()

    def test_headless_password_requirement_fails_without_network_changes(self) -> None:
        self.network.initial.update(nmcli_mode="unavailable", sudo_needs_password=True)
        result = self.network.run()
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(self.network.state["mutations"], [])
        self.assertEqual(self.network.state["routes"], self.network.initial["routes"])

    def test_permission_failure_keeps_network_unchanged(self) -> None:
        self.network.initial.update(nmcli_mode="denied", sudo_denied=True)
        result = self.network.run()
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(self.network.state["mutations"], [])
        self.assertEqual(self.network.state["routes"], self.network.initial["routes"])
        self.assertEqual(self.network.state["addresses"], self.network.initial["addresses"])

    def test_successful_tool_exit_is_not_proof_of_correct_source_route(self) -> None:
        self.network.initial["nmcli_mode"] = "ignore_source"
        self.network.initial["sudo_denied"] = True
        result = self.network.run()
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertNotEqual(self.network.state["routes"]["192.168.7.2"]["src"], "192.168.7.1")

    def test_unplug_during_configuration_fails(self) -> None:
        self.network.initial["nmcli_mode"] = "unplug"
        result = self.network.run()
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertFalse((self.network.net / "usb-glove").exists())
        self.assertEqual(self.network.state["forbidden"], [])

    def test_correct_network_and_sourcing_are_read_only(self) -> None:
        self.network.initial["addresses"]["usb-glove"].append("192.168.7.1/24")
        self.network.initial["routes"]["192.168.7.2"] = {"dev": "usb-glove", "src": "192.168.7.1"}
        result = self.network.run()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.network.state["mutations"], [])
        result = self.network.run(source_only=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.network.state["calls"], [])

    def test_explicit_interface_with_wrong_mac_is_never_configured(self) -> None:
        self.network.env["DATAGLOVE_INTERFACE"] = "wifi0"
        result = self.network.run()
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(self.network.state["mutations"], [])
        self.assertEqual(self.network.state["forbidden"], [])

    def test_missing_and_duplicate_devices_never_modify_network(self) -> None:
        for duplicate in (False, True):
            with self.subTest(duplicate=duplicate):
                if duplicate:
                    self.network.add_interface("usb-glove", "02:33:80:00:00:01")
                    self.network.add_interface("usb-other", "02:33:80:00:00:01")
                else:
                    shutil.rmtree(self.network.net / "usb-glove")
                result = self.network.run()
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertEqual(self.network.state["mutations"], [])
                self.assertEqual(self.network.state["forbidden"], [])

    def test_explicit_interface_accepts_duplicate_mac_without_touching_other_device(self) -> None:
        self.network.add_interface("usb-other", "02:33:80:00:00:01")
        self.network.env["DATAGLOVE_INTERFACE"] = "usb-glove"
        result = self.network.run()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_connected()

    def test_two_same_ip_and_mac_gloves_keep_independent_source_routes(self) -> None:
        self.network.add_interface("usb-other", "02:33:80:00:00:01")
        self.network.initial["allowed_interfaces"].append("usb-other")
        self.network.initial["addresses"]["usb-other"] = ["169.254.2.2/16"]
        self.network.env.update(
            DATAGLOVE_INTERFACE="usb-glove",
            DATAGLOVE_HOST_ADDRESS="192.168.7.1/24",
            DATAGLOVE_ROUTE_TABLE="10001",
        )
        first = self.network.run()
        self.assertEqual(first.returncode, 0, first.stderr)
        first_route = self.network.state["tables"]["10001"]["192.168.7.2"]
        self.assertEqual(first_route, {"dev": "usb-glove", "src": "192.168.7.1"})
        self.network.env.update(
            DATAGLOVE_INTERFACE="usb-other",
            DATAGLOVE_HOST_ADDRESS="192.168.7.3/24",
            DATAGLOVE_ROUTE_TABLE="10002",
        )
        second = self.network.run(reset=False)
        self.assertEqual(second.returncode, 0, second.stderr)
        state = self.network.state
        self.assertEqual(state["tables"], {
            "10001": {"192.168.7.2": first_route},
            "10002": {"192.168.7.2": {"dev": "usb-other", "src": "192.168.7.3"}},
        })
        self.assertEqual(state["rules"], [
            {"priority": 10001, "source": "192.168.7.1/32", "table": 10001},
            {"priority": 10002, "source": "192.168.7.3/32", "table": 10002},
        ])
        self.assertEqual(state["routes"], self.network.initial["routes"])
        self.assertEqual(state["addresses"]["usb-glove"], ["169.254.1.2/16", "192.168.7.1/24"])
        self.assertEqual(state["addresses"]["usb-other"], ["169.254.2.2/16", "192.168.7.3/24"])
        self.assertEqual(state["forbidden"], [])
        mutations = state["mutations"]
        self.network.env.update(
            DATAGLOVE_INTERFACE="usb-glove",
            DATAGLOVE_HOST_ADDRESS="192.168.7.1/24",
            DATAGLOVE_ROUTE_TABLE="10001",
        )
        again = self.network.run(reset=False)
        self.assertEqual(again.returncode, 0, again.stderr)
        self.assertEqual(self.network.state["mutations"], mutations)
        self.assertEqual(self.network.state["tables"], state["tables"])

    def test_policy_conflicts_are_rejected_without_modifying_foreign_rules(self) -> None:
        self.network.env["DATAGLOVE_ROUTE_TABLE"] = "10001"
        for rule in (
            {"priority": 10001, "source": "10.0.0.2", "table": 20000},
            {"priority": 9999, "source": "192.168.7.1/32", "table": 20000},
            {"priority": 20000, "source": "10.0.0.2/32", "table": 10001},
        ):
            with self.subTest(rule=rule):
                self.network.initial["rules"] = [rule]
                result = self.network.run()
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertEqual(self.network.state["rules"], [rule])
                self.assertEqual(self.network.state["mutations"], [])

    def test_occupied_policy_table_is_not_overwritten(self) -> None:
        self.network.env["DATAGLOVE_ROUTE_TABLE"] = "10001"
        self.network.initial["tables"]["10001"] = {
            "192.168.7.2": {"dev": "wifi0", "src": "10.0.0.2"},
        }
        result = self.network.run()
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(self.network.state["tables"], self.network.initial["tables"])
        self.assertEqual(self.network.state["mutations"], [])

    def test_failed_policy_rule_does_not_fall_back_to_main_route(self) -> None:
        self.network.env["DATAGLOVE_ROUTE_TABLE"] = "10001"
        self.network.initial["rule_add_denied"] = True
        result = self.network.run()
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(self.network.state["rules"], [])
        self.assertEqual(self.network.state["routes"], self.network.initial["routes"])

    def test_profile_precheck_ignores_stale_network_environment(self) -> None:
        self.network.state_file.write_text(json.dumps(self.network.initial), encoding="utf-8")
        self.network.env.update(
            DATAGLOVE_INTERFACE="wifi0", DATAGLOVE_MAC="00:11:22:33:44:55",
            DATAGLOVE_HOST_ADDRESS="10.0.0.2/24", DATAGLOVE_ROUTE_TABLE="10002",
        )
        args = argparse.Namespace(
            glove_profile="selected", host="192.168.7.2", port=9100,
            glove_interface=None, glove_mac="02:33:80:00:00:01",
            glove_host_address="192.168.7.1/24", glove_route_table=10001,
        )
        with patch.dict(os.environ, self.network.env, clear=True):
            prepare_glove_network(args)
        self.assertEqual(self.network.state["tables"], {
            "10001": {"192.168.7.2": {"dev": "usb-glove", "src": "192.168.7.1"}},
        })
        self.assertEqual(self.network.state["routes"], self.network.initial["routes"])
        self.assertEqual(self.network.state["forbidden"], [])

    def test_precheck_raises_when_policy_setup_fails(self) -> None:
        self.network.initial["rule_add_denied"] = True
        self.network.state_file.write_text(json.dumps(self.network.initial), encoding="utf-8")
        args = argparse.Namespace(
            host="192.168.7.2", port=9100, glove_route_table=10001,
            glove_host_address="192.168.7.1/24",
        )
        with patch.dict(os.environ, self.network.env, clear=True):
            with self.assertRaises(RuntimeError):
                prepare_glove_network(args)
        self.assertEqual(self.network.state["rules"], [])
        self.assertEqual(self.network.state["routes"], self.network.initial["routes"])

    def test_invalid_configuration_fails_before_external_commands(self) -> None:
        for variable, value in (
            ("DATAGLOVE_MAC", "bad-mac"),
            ("DATAGLOVE_USB_WAIT_TIMEOUT_SECONDS", "0"),
            ("DATAGLOVE_HOST", "192.168.8.2"),
            ("DATAGLOVE_HOST_ADDRESS", "192.168.7.999/24"),
            ("DATAGLOVE_PORT", "65536"),
            ("DATAGLOVE_ROUTE_TABLE", "9999"),
            ("DATAGLOVE_ROUTE_TABLE", "30001"),
        ):
            with self.subTest(variable=variable):
                original = self.network.env.get(variable)
                self.network.env[variable] = value
                result = self.network.run()
                if original is None:
                    self.network.env.pop(variable)
                else:
                    self.network.env[variable] = original
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertEqual(self.network.state["calls"], [])


if __name__ == "__main__":
    unittest.main()
