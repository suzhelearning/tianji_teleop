"""Franka DLS simulation with Manus ROS or exoskeleton hands; no hardware output."""
from pathlib import Path
import os
import json
import re
import tempfile

from tianji_runtime import controller_profile, native_executable, workspace
from tianji_runtime.resources import ResourceNotFound, controller_resource, config_path


def launch(args):
    import yaml

    # The native viewer and the controller profiles come from the control
    # install prefix, so a missing build reports itself instead of failing on a
    # stale path.
    try:
        executable = native_executable("tianji_arm_ros")
    except ResourceNotFound as error:
        raise RuntimeError(f"Run pixi run build first: {error}") from error
    hands = getattr(args, "hand_teleop", True)
    hand_source = getattr(args, "hand_source", "manus")
    hand_port = getattr(args, "hand_port", None)
    if hand_port is not None and hand_source != "exoskeleton":
        raise ValueError("--hand-port requires --hand-source exoskeleton")
    robot = json.loads(config_path("robot.json").read_text())
    topics = {"pico_input_topic": robot.get("pico_input_topic")}
    if hands:
        if hand_source not in ("manus", "exoskeleton"):
            raise ValueError("hand_source must be manus or exoskeleton")
        if hand_source == "manus":
            hand_topics = robot.get("hand_command_topics")
            if not isinstance(hand_topics, dict):
                raise ValueError("hand_command_topics must map selected hands to ROS topics")
            for hand in ("left_hand", "right_hand"):
                topics[f"hand_command_topics.{hand}"] = hand_topics.get(hand)
        else:
            hand_port = 16000 if hand_port is None else hand_port
            if type(hand_port) is not int or not 1 <= hand_port <= 65535:
                raise ValueError("hand_port must be an integer in [1, 65535]")
    for field, topic in topics.items():
        if not isinstance(topic, str) or re.fullmatch(r"(?:/[A-Za-z_][A-Za-z_0-9]*)+", topic) is None:
            raise ValueError(f"{field} must be an absolute nonempty ROS topic with valid names")
    if len(set(topics.values())) != len(topics):
        raise ValueError("selected input topics must be distinct")
    source = (args.config or controller_profile("qp_ik_pico_shared_root_dls.yaml")).resolve()
    config = yaml.safe_load(source.read_text())
    if config["ik"]["algorithm"] != "pico_ee_franka_dls":
        raise ValueError("Franka DLS simulation requires a DLS controller profile")
    # Relocated runtime copy only: the reviewed profile stays disabled on disk.
    shared = config["shared_root"]
    shared["enabled"] = True
    for key in ("input_contract_artifact", "robot_geometry_artifact"):
        shared[key] = str(controller_resource(source, shared[key]))
    controller = config["controller"]
    for key in ("home_config", "pico_ee_dls_kinematics_urdf_path"):
        if controller.get(key):
            controller[key] = str(controller_resource(source, controller[key]))
    sessions = workspace() / "recordings" / "shared_root"
    sessions.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="dls_interactive_", dir=sessions))
    profile = directory / "runtime.yaml"
    with profile.open("x") as output:
        yaml.safe_dump(config, output, sort_keys=False)
    command = [str(executable), "--config", str(profile), "--simulation-recovery",
               "--pico-teleop", "--model-state-only",
               "--telemetry", str(directory / "telemetry.csv"),
               "--joint-telemetry", str(directory / "joints.csv")]
    command += ["--pico-topic", topics["pico_input_topic"], "--joint-target-topic", ""]
    if hands:
        command += ["--hand-teleop", "--hand-source", hand_source]
        if hand_source == "manus":
            command += ["--left-hand-topic", topics["hand_command_topics.left_hand"],
                        "--right-hand-topic", topics["hand_command_topics.right_hand"]]
        else:
            command += ["--hand-bind", "127.0.0.1", "--hand-port", str(hand_port)]
    else:
        command.append("--no-hand-teleop")
    if args.model:
        command += ["--model", str(args.model.resolve())]
    if getattr(args, "sim_allow_pico_jumps", False):
        command.append("--sim-allow-pico-jumps")
    if args.duration:
        command += ["--duration", str(args.duration)]
    if getattr(args, "user", None):
        print(f"Checking PICO input for {args.user}; robot viewer has not started.", flush=True)
    scope = f"arms + {hand_source} hands" if hands else "arms-only"
    print(f"Franka DLS + Ruckig {scope} simulation; logs: {directory}\n"
          "Click the robot window: S start, H smooth Home then wait, P/Space hold.\n"
          "No joint export, no automatic takeover.", flush=True)
    if hands:
        if hand_source == "manus":
            print("Start the Manus ROS publisher separately; Hand2 command topics: "
                  f"left={topics['hand_command_topics.left_hand']}, "
                  f"right={topics['hand_command_topics.right_hand']}.", flush=True)
        else:
            print("Start the exoskeleton sender separately; TJH2 hand UDP input: "
                  f"127.0.0.1:{hand_port}.", flush=True)
        print("Hands follow only in TELEOP; H/P/fault hold fingers (H homes arms only). "
              "Stale hands hold independently; Ruckig applies to arms, not fingers.", flush=True)
    if getattr(args, "user", None):
        from .pico_owned_session import run_with_owned_pico
        return run_with_owned_pico(command, getattr(args, "user", None), workspace())
    os.execv(command[0], command)
