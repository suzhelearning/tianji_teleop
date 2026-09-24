#!/usr/bin/env python3
"""Test-only disk backpressure and long status period around the real collector."""
import argparse
import json
from pathlib import Path
import time
from types import SimpleNamespace

from data_collector import node
from data_collector.dataset import EpisodeWriter


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--writer-control", type=Path, required=True)
    parser.add_argument("--status-period", type=float, default=20.0)
    options, arguments = parser.parse_known_args()
    original_handle = EpisodeWriter._handle

    def handle(writer, item):
        deadline = time.monotonic() + 10.0
        while True:
            try:
                blocked = json.loads(options.writer_control.read_text()).get("block_writer", False)
            except FileNotFoundError:
                blocked = False
            if not blocked:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("writer fixture was not released")
            time.sleep(0.01)
        return original_handle(writer, item)

    EpisodeWriter._handle = handle
    original_node = node.CollectorNode

    class ObservedCollector(original_node):
        def __init__(self, **kwargs):
            super().__init__(**kwargs, status_rate_hz=1.0 / options.status_period)

        def get_publishers_info_by_topic(self, topic):
            endpoints = super().get_publishers_info_by_topic(topic)
            try:
                pending = json.loads(options.writer_control.read_text()).get("camera_graph_pending", False)
            except FileNotFoundError:
                pending = False
            if pending and topic.startswith("/cameras/"):
                return [SimpleNamespace(
                    node_name="_NODE_NAME_UNKNOWN_", node_namespace="_NODE_NAMESPACE_UNKNOWN_",
                    endpoint_gid=endpoint.endpoint_gid) for endpoint in endpoints]
            return endpoints

    node.CollectorNode = ObservedCollector
    return node.main(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
