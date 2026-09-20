"""PICO2 bare-hand SIM only. No ROS, Zenoh, UDP output or hardware drivers.

Run the hand-tracking APK and configure ADB forwarding separately.
--self-test uses synthetic input, no TCP connection, and scripted simulation keys.
"""
import argparse
from contextlib import ExitStack, contextmanager
from dataclasses import replace
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import select
import signal
import sys
import tempfile
import termios
import threading
import time
import tty
import uuid

import numpy as np
from .ik_worker import NativeIkWorker
from .hand_worker import NativeHandWorker
from .reference.runtime import PicoTcpReceiver
from .simulation_core import SimulationCore
from sim.direct_state import DirectStateSimulation
from real_robot.protocol import CommandFrame

ROOT = Path(__file__).resolve().parent


@contextmanager
def input_lock(port):
    path = Path(tempfile.gettempdir()) / f"tianji-pico2-sim-{os.getuid()}-{port}.lock"
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        if os.fstat(fd).st_uid != os.getuid():
            raise RuntimeError("input lock is owned by another user")
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("another PICO2 simulation owns this input port") from exc
        yield
    finally:
        os.close(fd)


@contextmanager
def keyboard():
    settings = None
    if sys.stdin.isatty():
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    try:
        yield
    finally:
        if settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def synthetic(core, fk, tick, now):
    from .scripts.smoke_pipeline import inverse_fixed_mapping
    from .tests.test_pico_hand_tracking import _packet
    from .reference.pico import parse_pico_packet, PICO_TO_MEDIAPIPE
    frame = parse_pico_packet(_packet(), receiver_instance_id="self-test", connection_generation=1,
                              receiver_frame_sequence=tick, received_timestamp_ns=now)
    hands = {}
    for side, hand in frame.hands.items():
        tcp = fk[side]["achieved_pose"].copy()
        tcp[0] += .003*np.sin(tick*.03)
        wrist = inverse_fixed_mapping(core.mapping.mapper, side, tcp)
        joints = list(hand.joints)
        for i, p in enumerate(PICO_TO_MEDIAPIPE):
            f, j = divmod(max(0, i-1), 4)
            offset = np.zeros(3) if i == 0 else np.array([(f-2)*.02, (j+1)*.025, j*.002])
            joints[p] = replace(joints[p], pose=np.r_[wrist[:3]+offset, wrist[3:]])
        hands[side] = replace(hand, wrist_pose=wrist, joints=tuple(joints))
    # Re-encode the synthetic geometry so raw recording and decoded fields agree.
    import struct
    payload = bytearray(struct.pack("<BBBB", 1, 7, 26, 0))
    payload.extend(struct.pack("<7f", 0, 0, 0, 0, 0, 0, 1))
    for side in ("left", "right"):
        hand = hands[side]
        payload.extend(struct.pack("<BBBB7f", 1, 0, 0, 0, *hand.wrist_pose))
        for joint in hand.joints:
            payload.extend(struct.pack("<BBBB7ff", 1, 0, 0, 0, *joint.pose, joint.radius_m))
    packet = struct.pack("<BBqI", 0xAB, 0x40, tick*10, len(payload)) + payload
    return parse_pico_packet(packet, receiver_instance_id="self-test", connection_generation=1,
                              receiver_frame_sequence=tick, received_timestamp_ns=now)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=10002)
    parser.add_argument("--disable-hands", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration-s", type=float, default=0)
    parser.add_argument("--record", type=Path)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--mapping-mode", choices=("legacy", "shared-root"), default="legacy")
    parser.add_argument("--height-m", type=float)
    args = parser.parse_args(argv)
    shared_root = args.mapping_mode == "shared-root"
    if shared_root and (args.height_m is None or not math.isfinite(args.height_m) or not 1 <= args.height_m <= 2.4):
        parser.error("--mapping-mode shared-root requires --height-m in [1.0, 2.4] metres")
    if not shared_root and args.height_m is not None:
        parser.error("--height-m requires --mapping-mode shared-root; legacy mapping is unchanged")
    if shared_root and args.self_test:
        parser.error("shared-root requires C; use the offline shared-root tests instead of legacy --self-test")
    if not 1 <= args.port <= 65535 or not math.isfinite(args.duration_s) or args.duration_s < 0:
        parser.error("invalid port/duration")
    events = queue.SimpleQueue()
    slot_lock = threading.Lock()
    slot = [None, None, None]
    record = None
    clean = False
    if args.record:
        from .sim_recording import SimRecorder
        record = SimRecorder(args.record)
    try:
        with ExitStack() as stack:
            if not args.self_test:
                stack.enter_context(input_lock(args.port))
            model = "marvin_m6_wuji2_shared_root_ceres.xml" if shared_root else "marvin_m6_wuji2.xml"
            config = ROOT.parent / "control/config/qp_ik_pico_shared_root_dls.yaml" if shared_root else ROOT / "config/simulation.yaml"
            simulation = DirectStateSimulation(ROOT.parent / "control/models" / model, config)
            from .display_contract import configure_pico2_limits, validate_display
            if not shared_root:
                configure_pico2_limits(simulation)
                validate_display(simulation)
            # The shared XML's target markers are not bound to this input route.
            # Hide them in this instance only; never present static markers as IK targets.
            for side in ("L", "R"):
                simulation.model.geom("target_geom_" + side).rgba[3] = 0
                simulation.model.site("target_site_" + side).rgba[3] = 0
            from .dls_worker import DlsWorker
            ik = stack.enter_context((DlsWorker if shared_root else NativeIkWorker)(timeout_s=.5))
            hands = {} if args.disable_hands else {
                side: stack.enter_context(NativeHandWorker(side, timeout_s=.5)) for side in ("left", "right")}
            if shared_root:
                from .shared_root_core import SharedRootCore
                from .display_contract import validate_dls_display
                validate_dls_display(simulation, ik)
                core = SharedRootCore(simulation.targets, ik, hands, height_m=args.height_m)
                print("Shared-root height template + Franka DLS/Ruckig. C: arms forward, shoulder width, palms facing; hold 1s. Then S. H homes arms only.", flush=True)
            else:
                core = SimulationCore(simulation.targets, ik, hands)
            fk = ik.forward(core.q[:14].reshape(2, 7)) if args.self_test else None

            def on_frame(frame):
                if record:
                    try:
                        record.offer("raw", frame)
                    except Exception as error:
                        with slot_lock:
                            slot[2] = repr(error)
                        raise
                with slot_lock:
                    slot[0], slot[1] = frame, None

            def on_error(error):
                with slot_lock:
                    slot[0], slot[1] = None, str(error)

            receiver = None
            if not args.self_test:
                receiver = PicoTcpReceiver(host=args.host, port=args.port,
                    receiver_instance_id=str(uuid.uuid4()), auto_adb_forward=False, on_error=on_error)

                def receive():
                    try:
                        receiver.run(on_frame)
                    except BaseException as error:
                        with slot_lock:
                            slot[2] = repr(error)

                thread = threading.Thread(target=receive, name="pico2-sim-input", daemon=True)
                thread.start()

                def stop_input():
                    receiver.stop(); thread.join(timeout=5)
                    if thread.is_alive():
                        raise RuntimeError("PICO receiver did not stop")
                stack.callback(stop_input)
            viewer = None
            if not args.headless and not args.self_test:
                import mujoco.viewer
                from .viewer_controls import suppress_default_home_shortcut
                # Must precede UI construction: post-sync restoration alone
                # allows a convex-hull frame to appear before H is undone.
                stack.enter_context(suppress_default_home_shortcut())
                viewer = stack.enter_context(mujoco.viewer.launch_passive(
                    simulation.model, simulation.data, show_left_ui=False, show_right_ui=False,
                    key_callback=lambda code: events.put(chr(code).lower()) if 0 <= code < 128 else None))
                from .viewer_controls import TeleopDisplayPolicy
                display_policy = TeleopDisplayPolicy(viewer)
            stack.enter_context(keyboard())
            for sig in (signal.SIGINT, signal.SIGTERM):
                old = signal.signal(sig, lambda *_: events.put("q"))
                stack.callback(signal.signal, sig, old)
            if shared_root:
                print("PICO2 SIM ready. C required: arms forward, palms facing, hold 1s | S follow | H arms Home | Q Home/exit", flush=True)
                print("Arm IK: Franka DLS + online Ruckig", flush=True)
            else:
                print("PICO2 SIM ready. S follow | C optional X/Z: arms forward/horizontal, hold 2s | H Home | R rearm | Q Home/exit", flush=True)
                print("Arm IK: original V131", flush=True)
            start = time.monotonic()
            next_tick = next_render = start
            last_status = None
            seq = 0
            late = 0
            test_phase, test_phase_started = 0, start
            while not core.done:
                now = time.monotonic_ns()
                elapsed = time.monotonic()-start
                if args.self_test:
                    frame = synthetic(core, fk, seq, now)
                    on_frame(frame)
                with slot_lock:
                    frame, error, fatal = slot
                now = time.monotonic_ns()
                if fatal:
                    raise RuntimeError(fatal)
                if error:
                    core.disconnected()
                elif frame is not None:
                    core.offer(frame, now)
                if args.self_test:
                    phase_age = time.monotonic() - test_phase_started
                    key = None
                    if test_phase == 0 and elapsed >= .1:
                        key = "s"
                    elif test_phase == 1 and phase_age >= .5:
                        key = "h"
                    elif test_phase == 2 and core.state == "idle" and frame.received_timestamp_ns > core.armed_at:
                        key = "s"
                    elif test_phase == 3 and phase_age >= .5:
                        key = "q"
                    if key:
                        events.put(key)
                        test_phase += 1
                        test_phase_started = time.monotonic()
                if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                    events.put(os.read(sys.stdin.fileno(), 1).decode(errors="ignore"))
                if ((args.duration_s and elapsed >= args.duration_s) or
                        (viewer is not None and not viewer.is_running())) and not core.exit_requested:
                    events.put("q")
                while not events.empty():
                    key = events.get()
                    now = time.monotonic_ns()
                    accepted = core.action(key, now)
                    print(json.dumps(dict(key=key, accepted=accepted, state=core.state)), flush=True)
                    if record:
                        record.offer("event", dict(timestamp_ns=now, key=key, accepted=accepted, state=core.state))
                    if args.self_test and not accepted:
                        raise RuntimeError("self-test action rejected")
                # Worker reset/FK and terminal output can block. Recheck input
                # freshness at execution time, not with the pre-action clock.
                now = time.monotonic_ns()
                q = core.tick(now)
                seq += 1
                simulation.set_targets(CommandFrame(seq, now, ik.epoch, 7,
                    tuple(q[:7]), tuple(q[7:14]), tuple(q[14:34]), tuple(q[34:])))
                simulation.step()
                if record:
                    association = (-1, -1) if core.frame is None else (core.frame.connection_generation, core.frame.receiver_frame_sequence)
                    record.offer("command", (now, core.state, q, association))
                status = (core.state, core.reason, core.mapping.calibration.state)
                if status != last_status:
                    if record:
                        record.offer("event", dict(timestamp_ns=now, state=core.state, reason=core.reason,
                                                   calibration=core.mapping.status()))
                    print(json.dumps(dict(state=core.state, reason=core.reason,
                                          calibration=core.mapping.status())), flush=True)
                    last_status = status
                if viewer is not None and viewer.is_running() and time.monotonic() >= next_render:
                    viewer.set_texts((None, None, "PICO2 SIM | " + core.state + "\n" + core.reason +
                        ("\nS follow | C root: forward, palms facing 1s | H arms Home | Q exit\nCalibration: " if shared_root else
                         "\nS follow | C X/Z: arms forward/horizontal 2s | H Home | R rearm | Q exit\nCalibration: ") + core.mapping.calibration.state +
                        "\nGestures (observation): " + str(core.gestures), ""))
                    display_policy.sync()
                    next_render = time.monotonic() + 1/30
                next_tick += .005
                wait = next_tick-time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                else:
                    late += 1; next_tick = time.monotonic()
                if args.self_test and elapsed > 45:
                    raise RuntimeError("self-test did not complete Home/exit")
            print(json.dumps(dict(kind="pico2_sim_complete", home=True, ticks=seq,
                                  late_cycles=late, real_time_qualified=False)), flush=True)
        clean = True
    finally:
        if record:
            record.close(clean)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
