"""Required Manus children fail the launch, but intentional shutdown stays clean."""

import importlib.util
import os
from pathlib import Path
import signal
import sys
import textwrap

from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
import pytest


@pytest.fixture()
def manus_launch():
    path = Path(__file__).resolve().parents[1] / "launch/manus_hand2.launch.py"
    spec = importlib.util.spec_from_file_location("manus_hand2_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run_with_sibling(manus_launch, tmp_path, after_ready):
    stopped = tmp_path / "sibling-stopped"
    sibling = ExecuteProcess(
        cmd=[sys.executable, "-c", textwrap.dedent("""
            from pathlib import Path
            import signal
            import sys

            def stop(signum, frame):
                Path(sys.argv[1]).write_text(str(signum))
                sys.exit(0)

            signal.signal(signal.SIGINT, stop)
            signal.alarm(15)
            print("ready", flush=True)
            while True:
                signal.pause()
        """), str(stopped)],
        sigterm_timeout="1",
        sigkill_timeout="1",
        output="log",
    )
    ready = False
    timed_out = False

    def on_ready(event):
        nonlocal ready
        if not ready:
            ready = True
            return after_ready

    def timeout(context):
        nonlocal timed_out
        timed_out = True
        return [EmitEvent(event=Shutdown(reason="test watchdog"))]

    service = LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnProcessIO(target_action=sibling, on_stdout=on_ready)),
        TimerAction(period=10.0, actions=[OpaqueFunction(function=timeout)]),
        *manus_launch._required_processes([sibling]),
    ]))
    result = service.run()
    assert ready and not timed_out, "launch did not complete the requested lifecycle"
    assert stopped.read_text() == str(signal.SIGINT)
    # LaunchService must reap the sibling, not merely request its shutdown.
    with pytest.raises(ProcessLookupError):
        os.kill(sibling.process_details["pid"], 0)
    return result


@pytest.mark.parametrize("exit_code", [0, 7, -signal.SIGTERM])
def test_unexpected_required_exit_fails_and_reaps_sibling(manus_launch, tmp_path, exit_code):
    failing = ExecuteProcess(
        cmd=[sys.executable, "-c", textwrap.dedent("""
            import os
            import sys

            code = int(sys.argv[1])
            if code < 0:
                os.kill(os.getpid(), -code)
            sys.exit(code)
        """), str(exit_code)],
        output="log",
    )
    assert _run_with_sibling(
        manus_launch, tmp_path, manus_launch._required_processes([failing]),
    ) != 0


def test_intentional_shutdown_does_not_reclassify_required_child_exit(manus_launch, tmp_path):
    assert _run_with_sibling(
        manus_launch, tmp_path, [EmitEvent(event=Shutdown(reason="user requested shutdown"))],
    ) == 0
