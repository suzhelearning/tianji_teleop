from pathlib import Path
import importlib.util
import sys
import types

import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
READINESS_SCRIPT = SCRIPTS / "pico_m0_readiness.py"


def _load_readiness_module():
    assert READINESS_SCRIPT.is_file()
    specification = importlib.util.spec_from_file_location(
        "pico_m0_readiness", READINESS_SCRIPT
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


def test_readiness_contract_covers_every_m0_runtime_stream():
    module = _load_readiness_module()
    assert module.READINESS_TOPICS == (
        "/pico/smpl_raw",
        "/pico/palm_left",
        "/pico/palm_right",
        "/pico/smpl_palm_corrected",
        "/pico/smpl_palm_corrected_ik",
        "/pico/smpl_palm_corrected/status",
        "/pico/tracking_epoch",
        "/pico/tracking_epoch/status",
    )


def test_readiness_requires_one_message_from_every_stream():
    module = _load_readiness_module()
    tracker = module.ReadinessTracker(module.READINESS_TOPICS)

    for topic in module.READINESS_TOPICS[:-1]:
        tracker.mark_received(topic)

    assert not tracker.ready
    assert tracker.missing == (module.READINESS_TOPICS[-1],)

    tracker.mark_received(module.READINESS_TOPICS[-1])

    assert tracker.ready
    assert tracker.missing == ()


def test_readiness_shuts_rclpy_down_if_node_construction_fails(monkeypatch):
    module = _load_readiness_module()
    state = {"shutdown": False}

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.init = lambda: None
    fake_rclpy.shutdown = lambda: state.__setitem__("shutdown", True)
    fake_rclpy.ok = lambda: not state["shutdown"]

    fake_node = types.ModuleType("rclpy.node")

    class FailingNode:
        def __init__(self, _name):
            raise RuntimeError("node construction failed")

    fake_node.Node = FailingNode

    fake_qos = types.ModuleType("rclpy.qos")

    class Policy:
        RELIABLE = object()
        TRANSIENT_LOCAL = object()

    class QoSProfile:
        def __init__(self, **_kwargs):
            self.durability = None

    fake_qos.DurabilityPolicy = Policy
    fake_qos.QoSProfile = QoSProfile
    fake_qos.ReliabilityPolicy = Policy
    fake_qos.qos_profile_sensor_data = object()

    geometry_messages = types.ModuleType("geometry_msgs.msg")
    geometry_messages.PoseArray = type("PoseArray", (), {})
    geometry_messages.PoseStamped = type("PoseStamped", (), {})
    std_messages = types.ModuleType("std_msgs.msg")
    std_messages.String = type("String", (), {})
    std_messages.UInt64 = type("UInt64", (), {})

    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", fake_node)
    monkeypatch.setitem(sys.modules, "rclpy.qos", fake_qos)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_messages)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_messages)

    with pytest.raises(RuntimeError, match="node construction failed"):
        module.wait_for_m0_streams(0.1)

    assert state["shutdown"]
