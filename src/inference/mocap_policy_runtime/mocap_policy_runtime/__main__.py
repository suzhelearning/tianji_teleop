"""Mocap trajectory replay and policy inference; no hardware access on import."""
from __future__ import annotations

import argparse
import importlib
import sys


COMMANDS = {
    "inspect": ("mocap_policy_runtime.replay.cli", "Inspect trajectory schema and channels without opening devices"),
    "replay": ("mocap_policy_runtime.replay.cli", "Replay a trajectory in the target project's MuJoCo model"),
    "infer": ("mocap_policy_runtime.policies.regrind.benchmark", "Benchmark a policy on reference frame zero"),
    "live": ("mocap_policy_runtime.policies.regrind.live", "Run live Motive shadow inference without control output"),
    "run": ("mocap_policy_runtime.integration.runner", "Execute replay or policy through simulation or guarded real devices"),
    "h5-sim": ("mocap_policy_runtime.integration.h5_simulation", "Motive-aligned H5 simulation with s/hold-Enter/r/q control"),
    "h5-real": ("mocap_policy_runtime.integration.real", "Real H5: Enter enables, hold Enter approaches frame0, new Enter starts replay"),
    "regrind-real": ("mocap_policy_runtime.integration.real", "Real policy: Enter enables, hold Enter approaches frame0, new Enter starts inference"),
    "regrind-hand-sim": ("mocap_policy_runtime.replay.hand_reference", "Display reference hand/object motion; no policy or physics"),
}


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=COMMANDS, help="; ".join(
        f"{name}: {description}" for name, (_, description) in COMMANDS.items()))
    if not arguments or arguments[0] in {"-h", "--help"}:
        parser.print_help()
        return 0 if arguments else 2
    command = parser.parse_args(arguments[:1]).command
    child_arguments = arguments[1:]
    if child_arguments[:1] == ["--"]:
        child_arguments = child_arguments[1:]
    if command in {"infer", "live"}:
        policy_parser = argparse.ArgumentParser(add_help=False)
        policy_parser.add_argument("--policy", choices=("regrind",), default="regrind")
        _, child_arguments = policy_parser.parse_known_args(child_arguments)
    try:
        module = importlib.import_module(COMMANDS[command][0])
        if command in {"inspect", "replay"}:
            child_arguments = [command, *child_arguments]
        elif command == "h5-real":
            child_arguments = ["--h5-replay", *child_arguments]
        elif command == "regrind-real":
            child_arguments = ["--policy", "regrind", *child_arguments]
        return module.main(child_arguments)
    except (ImportError, OSError, ValueError, RuntimeError) as error:
        print(f"MOCAP ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
