"""DLS/Ceres simulation with optional TJH2 hands; no export or hardware driver."""
from pathlib import Path
import os
import tempfile

from tianji_runtime import controller_profile, native_executable, workspace
from tianji_runtime.resources import ResourceNotFound, controller_resource


def launch(args):
    import yaml

    # The native viewer and the controller profiles come from the control
    # install prefix, so a missing build reports itself instead of failing on a
    # stale path.
    try:
        executable = native_executable("tianji_qp_ik_viewer")
    except ResourceNotFound as error:
        raise RuntimeError(f"Run pixi run build first: {error}") from error
    dls = getattr(args, "ik_backend", "ceres") == "franka-dls"
    backend = "dls" if dls else "ceres"
    label = "Franka DLS + Ruckig" if dls else "Ceres LM + Ruckig"
    hands = getattr(args, "hand_teleop", True)
    hand_port = getattr(args, "hand_port", 16000)
    source = (args.config or controller_profile(
        f"qp_ik_pico_shared_root_{backend}.yaml")).resolve()
    config = yaml.safe_load(source.read_text())
    expected = "pico_ee_franka_dls" if dls else "pico_ee_franka_ceres_lm"
    if config["ik"]["algorithm"] != expected:
        raise ValueError("Selected backend does not match the supplied profile")
    # Relocated runtime copy only: the reviewed profile stays disabled on disk.
    shared = config["spark_shared_root"]
    shared["enabled"] = True
    for key in ("input_contract_artifact", "robot_geometry_artifact"):
        shared[key] = str(controller_resource(source, shared[key]))
    controller = config["controller"]
    for key in ("home_config", "pico_ee_dls_kinematics_urdf_path"):
        if controller.get(key):
            controller[key] = str(controller_resource(source, controller[key]))
    sessions = workspace() / "recordings" / "shared_root"
    sessions.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix=f"{backend}_interactive_", dir=sessions))
    profile = directory / "runtime.yaml"
    with profile.open("x") as output:
        yaml.safe_dump(config, output, sort_keys=False)
    command = [str(executable), "--config", str(profile), "--simulation-recovery",
               "--pico-teleop", "--pico-bind", "127.0.0.1", "--pico-port", str(args.pico_port),
               "--model-state-only", "--joint-command-port", "0",
               "--telemetry", str(directory / "telemetry.csv"),
               "--joint-telemetry", str(directory / "joints.csv"),
               "--pico-record", str(directory / "input.tjvr")]
    if hands:
        command += ["--hand-teleop", "--hand-bind", "127.0.0.1", "--hand-port", str(hand_port)]
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
    scope = "arms + TJH2 hands" if hands else "arms-only"
    print(f"{label} {scope} simulation; logs: {directory}\n"
          "Click the robot window: S start, H smooth Home then wait, P/Space hold.\n"
          "No joint export, no automatic takeover.", flush=True)
    if hands:
        print(f"Start Manus separately; hand input: 127.0.0.1:{hand_port}. "
              "Hands follow only in TELEOP; H/P/fault hold fingers (H homes arms only). "
              "Stale hands hold independently; Ruckig applies to arms, not fingers.", flush=True)
    if getattr(args, "user", None):
        from .pico_owned_session import run_with_owned_pico
        return run_with_owned_pico(command, getattr(args, "user", None), workspace())
    os.execv(command[0], command)
