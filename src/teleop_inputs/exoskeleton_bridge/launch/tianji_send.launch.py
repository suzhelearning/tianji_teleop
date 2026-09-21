"""Process entry point for the exoskeleton TJH2 sender.

`exoskeleton_bridge` keeps its own Pixi manifest and `pixi.lock` and is
deliberately not part of the workspace colcon build, so this file does not
import the sender: it starts the package's own task as a plain process with
`pixi run --manifest-path <package>/pixi.toml --locked -e default tianji-send`.

    ros2 launch <workspace>/src/teleop_inputs/exoskeleton_bridge/launch/tianji_send.launch.py \
        hand:=left arguments:="--seconds 30"

`--confirm-send` authorises glove acquisition and the loopback TJH2 UDP send
only; `--commission-directions` records the operator's direction commissioning.
Neither grants robot motion, and the TTY is deliberately not emulated: the
sender's authorization is the explicit flag, never a terminal.

`hand` selects left/right/both. `arguments` is appended verbatim, so every other
`tianji-send` option (timeouts, `--seconds`, `--udp-port`, ...) stays reachable
without editing this file.
"""
import shlex
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

PACKAGE_DIR = Path(__file__).resolve().parent.parent
MANIFEST = PACKAGE_DIR / "pixi.toml"


def _sender(context, *args, **kwargs):
    """Build the sender process once the launch arguments have been resolved."""
    command = [
        "pixi", "run", "--manifest-path", str(MANIFEST), "--locked", "-e", "default",
        "tianji-send",
        "--hand", LaunchConfiguration("hand").perform(context),
        "--confirm-send", "--commission-directions",
    ]
    command.extend(shlex.split(LaunchConfiguration("arguments").perform(context)))
    return [ExecuteProcess(cmd=command, cwd=str(PACKAGE_DIR), output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "hand",
            default_value="both",
            choices=["left", "right", "both"],
            description="Hands the exoskeleton sender streams.",
        ),
        DeclareLaunchArgument(
            "arguments",
            default_value="",
            description="Extra `tianji-send` arguments, appended verbatim "
                        "(for example `--seconds 30 --timeout 1`).",
        ),
        OpaqueFunction(function=_sender),
    ])
