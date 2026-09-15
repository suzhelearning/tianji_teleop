"""网络预检只通过模拟 ip/nmcli 子进程验证，绝不触碰主机网络。"""

from __future__ import annotations

from copy import deepcopy
from hashlib import sha256
import json
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from data_glove_wuji_teleop.adapters.hardware.network import prepare_robot_network


def address(local, prefix=24):
    return {"family": "inet", "local": local, "prefixlen": prefix, "scope": "global"}


def ethernet(name, addresses=()):
    return {"ifname": name, "flags": ["UP", "LOWER_UP"], "addr_info": list(addresses)}


class NetworkMachine:
    def __init__(self):
        self.interfaces = [
            ethernet("eth0", [address("192.168.50.10")]),
            ethernet("glove0", [address("192.168.60.10")]),
            ethernet("wlan0", [address("10.20.0.10")]),
        ]
        self.types = {"eth0": "ethernet", "glove0": "ethernet", "wlan0": "wifi"}
        self.profiles = {"original-uuid": {
            "connection.id": "original", "connection.interface-name": "eth0",
            "ipv4.method": "manual",
        }}
        self.active = "original-uuid"
        self.writes = []
        self.route_verified_after_up = False
        self.fail_up = False
        self.fail_restore = False
        self.repair_routes = True
        self.deny_add = False
        self.route_good = False
        self.defaults = [{"dev": "wlan0", "gateway": "10.20.0.1"}]

    def run(self, command, **kwargs):
        if kwargs.get("stdin") != subprocess.DEVNULL or "--ask" in command:
            raise AssertionError("网络命令不能请求交互密码")
        if command[0] == "ip":
            output = self.ip(command[1:])
        elif command[0] == "nmcli":
            output = self.nm(command[3:])
        else:
            raise AssertionError(command)
        if isinstance(output, tuple):
            return SimpleNamespace(returncode=output[0], stdout="", stderr=output[1])
        return SimpleNamespace(returncode=0, stdout=output, stderr="")

    def ip(self, args):
        if args == ("-j", "address", "show"):
            interfaces = deepcopy(self.interfaces)
            if self.active in self.profiles and self.active != "original-uuid":
                values = self.profiles[self.active].get("ipv4.addresses", "").split(",")
                for interface in interfaces:
                    if interface["ifname"] == "eth0":
                        interface["addr_info"] = [address(value.split("/")[0], int(value.split("/")[1])) for value in values if value]
            return json.dumps(interfaces)
        if args[:4] == ("-j", "-4", "route", "get"):
            good = self.route_good or (self.active not in (None, "original-uuid") and self.repair_routes)
            if self.active not in (None, "original-uuid"):
                self.route_verified_after_up = True
            return json.dumps([{"dev": "eth0", "prefsrc": "192.168.100.254"}] if good else [
                {"dev": "wlan0", "gateway": "10.20.0.1", "prefsrc": "10.20.0.10"},
            ])
        if args == ("-j", "-4", "route", "show", "default"):
            return json.dumps(self.defaults)
        if args == ("-j", "-6", "route", "show", "default"):
            return "[]"
        if args == ("-j", "-4", "route", "show", "table", "all", "dev", "eth0"):
            return json.dumps([{"dst": "192.168.50.0/24", "dev": "eth0", "protocol": "kernel", "scope": "link"}])
        raise AssertionError(args)

    def nm(self, args):
        if args == ("--escape", "no", "-t", "-f", "DEVICE,TYPE", "device", "status"):
            return "\n".join(f"{name}:{kind}" for name, kind in self.types.items())
        if args == ("--escape", "no", "-t", "-f", "UUID,NAME", "connection", "show"):
            return "\n".join(f"{uuid}:{profile['connection.id']}" for uuid, profile in self.profiles.items())
        if args == ("--escape", "no", "-g", "GENERAL.CON-UUID", "device", "show", "eth0"):
            return self.active or "--"
        if args[:3] == ("--escape", "no", "-g") and args[4:7] == ("connection", "show", "uuid"):
            return self.profiles[args[7]].get(args[3], "")
        if args[:2] == ("connection", "add"):
            self.writes.append(args)
            if self.deny_add:
                return 4, "Insufficient privileges"
            props = dict(zip(args[2::2], args[3::2]))
            props["connection.id"] = props.pop("con-name")
            props["connection.interface-name"] = props.pop("ifname")
            props["connection.type"] = "802-3-ethernet"
            self.profiles["dedicated-uuid"] = props
            return "created"
        if args[:2] == ("connection", "up"):
            self.writes.append(args)
            uuid = args[3]
            if uuid == "original-uuid" and self.fail_restore:
                return 4, "restore failed"
            self.active = uuid
            if uuid != "original-uuid" and self.fail_up:
                return 4, "activation failed"
            return "activated"
        if args[:2] == ("connection", "delete"):
            self.writes.append(args)
            del self.profiles[args[3]]
            return "deleted"
        if args == ("device", "disconnect", "eth0"):
            self.writes.append(args)
            self.active = None
            return "disconnected"
        raise AssertionError(args)


class RobotNetworkTest(unittest.TestCase):
    def setUp(self):
        self.machine = NetworkMachine()
        self.patch = patch("data_glove_wuji_teleop.adapters.hardware.network.subprocess.run", side_effect=self.machine.run)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def prepare(self, **kwargs):
        prepare_robot_network(["192.168.100.2:8888"], excluded_interfaces=("glove0",), **kwargs)

    def test_correct_existing_wired_route_is_read_only_even_with_connection_name(self):
        self.machine.interfaces[0]["addr_info"].append(address("192.168.100.254"))
        self.machine.route_good = True
        self.prepare(network_connection="wuji-left")
        self.assertEqual(self.machine.writes, [])
        self.assertEqual(self.machine.active, "original-uuid")

    def test_wifi_gateway_is_repaired_and_actual_route_is_verified(self):
        original = deepcopy(self.machine.profiles["original-uuid"])
        self.prepare()
        self.assertEqual(self.machine.active, "dedicated-uuid")
        self.assertTrue(self.machine.route_verified_after_up)
        self.assertEqual(self.machine.profiles["original-uuid"], original)
        dedicated = self.machine.profiles["dedicated-uuid"]
        self.assertEqual(set(dedicated["ipv4.addresses"].split(",")), {"192.168.50.10/24", "192.168.100.254/24"})
        self.assertEqual(dedicated["connection.autoconnect"], "no")
        self.assertEqual(dedicated["ipv4.never-default"], "yes")
        self.assertGreater(int(dedicated["ipv4.dad-timeout"]), 0)

    def test_successful_nm_exit_with_wrong_route_rolls_back(self):
        self.machine.repair_routes = False
        with self.assertRaises(RuntimeError):
            self.prepare()
        self.assertTrue(self.machine.route_verified_after_up)
        self.assertEqual(self.machine.active, "original-uuid")
        self.assertEqual(set(self.machine.profiles), {"original-uuid"})

    def test_activation_failure_restores_original_connection(self):
        self.machine.fail_up = True
        with self.assertRaisesRegex(RuntimeError, "activation failed"):
            self.prepare()
        self.assertEqual(self.machine.active, "original-uuid")
        self.assertEqual(set(self.machine.profiles), {"original-uuid"})

    def test_restore_failure_is_not_swallowed(self):
        self.machine.fail_up = True
        self.machine.fail_restore = True
        with self.assertRaises(ExceptionGroup) as captured:
            self.prepare()
        self.assertIn("activation failed", str(captured.exception.exceptions[0]))
        self.assertIn("restore failed", str(captured.exception.exceptions[1]))

    def test_wired_ambiguity_refuses_to_guess(self):
        self.machine.interfaces.append(ethernet("eth1"))
        self.machine.types["eth1"] = "ethernet"
        with self.assertRaises(RuntimeError):
            self.prepare()
        self.assertEqual(self.machine.writes, [])

    def test_glove_interface_cannot_be_selected_explicitly(self):
        with self.assertRaises(ValueError):
            self.prepare(network_interface="glove0")
        self.assertEqual(self.machine.writes, [])

    def test_no_candidate_after_glove_exclusion_never_uses_wifi(self):
        self.machine.interfaces = self.machine.interfaces[1:]
        del self.machine.types["eth0"]
        with self.assertRaises(RuntimeError):
            self.prepare()
        self.assertEqual(self.machine.writes, [])

    def test_read_only_option_never_modifies_wrong_route(self):
        with self.assertRaises(RuntimeError):
            self.prepare(prepare_network=False)
        self.assertEqual(self.machine.writes, [])

    def test_permission_failure_is_actionable_and_never_activates(self):
        self.machine.deny_add = True
        with self.assertRaisesRegex(RuntimeError, "管理员.*prepare_network=False"):
            self.prepare()
        self.assertEqual(self.machine.active, "original-uuid")
        self.assertEqual(set(self.machine.profiles), {"original-uuid"})

    def test_foreign_same_name_connection_is_not_overwritten(self):
        plan = "eth0|192.168.50.10/24,192.168.100.254/24"
        name = f"wuji-hand2-eth0-{sha256(plan.encode()).hexdigest()[:12]}"
        self.machine.profiles["foreign-uuid"] = {"connection.id": name}
        with self.assertRaises(RuntimeError):
            self.prepare()
        self.assertEqual(self.machine.writes, [])
        self.assertEqual(self.machine.profiles["foreign-uuid"], {"connection.id": name})

    def test_default_route_interface_is_not_switched(self):
        self.machine.defaults = [{"dev": "eth0", "gateway": "192.168.50.1"}]
        with self.assertRaises(RuntimeError):
            self.prepare()
        self.assertEqual(self.machine.writes, [])

    def test_duplicate_robot_ip_fails_before_network_writes(self):
        with self.assertRaises(ValueError):
            prepare_robot_network(["192.168.100.2:8888", "192.168.100.2:9999"])
        self.assertEqual(self.machine.writes, [])

    def test_unsafe_automatic_subnet_is_not_configured(self):
        with self.assertRaises(ValueError):
            prepare_robot_network(["8.8.8.8:8888"], excluded_interfaces=("glove0",))
        self.assertEqual(self.machine.writes, [])

    def test_other_interface_overlap_refuses_address_addition(self):
        self.machine.interfaces[1]["addr_info"] = [address("192.168.100.10")]
        with self.assertRaises(ValueError):
            self.prepare()
        self.assertEqual(self.machine.writes, [])

    def test_discovered_ip_cannot_equal_host_ip(self):
        with self.assertRaises(ValueError):
            prepare_robot_network(["192.168.50.10:8888"])
        self.assertEqual(self.machine.writes, [])

    def test_dhcp_address_is_not_frozen_into_static_profile(self):
        self.machine.profiles["original-uuid"]["ipv4.method"] = "auto"
        with self.assertRaisesRegex(RuntimeError, "DHCP"):
            self.prepare()
        self.assertEqual(self.machine.writes, [])
