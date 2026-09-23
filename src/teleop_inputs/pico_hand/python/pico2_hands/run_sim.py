"""PICO bare-hand targets for SPD simulation over ROS 2; never drives hardware.

The native viewer observes the same DLS/Hand2 targets sent to SPD.
Local controls: R calibrates without following; S explicitly starts live following.
--self-test uses synthetic input and isolated DDS domain 121, with no TCP input.
"""
import argparse
from contextlib import ExitStack, contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import select
import signal
import sys
import struct
import subprocess
import tempfile
import termios
import threading
import time
import tty
import uuid

import numpy as np
from .dls_worker import DlsWorker
from .hand_worker import NativeHandWorker
from .reference.runtime import PicoTcpReceiver
from .resources import display_model_path
from .shared_root_core import SharedRootCore
from simulation.direct_state import DirectStateSimulation
from tianji_controller.protocol import CommandFrame
from tianji_runtime.resources import controller_profile



def ensure_adb_forward(port):
    """Reuse or create one unambiguous forwarding rule; never replace an owner."""
    def adb(*arguments):
        try:
            return subprocess.run(
                ["adb", *arguments], check=True, capture_output=True,
                text=True, timeout=5).stdout
        except FileNotFoundError as error:
            raise RuntimeError("ADB is required for local PICO input; install adb first") from error
        except subprocess.TimeoutExpired as error:
            raise RuntimeError("ADB preflight timed out; check the USB connection") from error
        except subprocess.CalledProcessError as error:
            raise RuntimeError(f"ADB preflight failed: {error.stderr.strip()}") from error

    devices = {}
    for line in adb("devices").splitlines():
        fields = line.split()
        if len(fields) >= 2 and not line.startswith("List of devices"):
            devices[fields[0]] = fields[1]
    serial = os.environ.get("ANDROID_SERIAL")
    if not serial:
        if len(devices) != 1:
            raise RuntimeError("Connect one PICO headset and authorize USB debugging; "
                               "with multiple devices set ANDROID_SERIAL explicitly")
        serial = next(iter(devices))
    if devices.get(serial) != "device":
        raise RuntimeError(f"PICO {serial} is not authorized/online; "
                           "accept USB debugging in the headset")

    endpoint = f"tcp:{port}"
    expected = [serial, endpoint, endpoint]
    forwards = [line.split() for line in adb("forward", "--list").splitlines()]
    owners = [row for row in forwards if len(row) == 3 and row[1] == endpoint]
    if owners:
        if owners != [expected]:
            raise RuntimeError(f"ADB {endpoint} already forwards to another device/port; "
                               "refusing to replace it")
        print(f"ADB ready: {serial} {endpoint} -> {endpoint} (reused)", flush=True)
        return
    adb("-s", serial, "forward", "--no-rebind", endpoint, endpoint)
    forwards = [line.split() for line in adb("forward", "--list").splitlines()]
    if expected not in forwards:
        raise RuntimeError(f"ADB did not retain the requested {endpoint} forwarding rule")
    print(f"ADB ready: {serial} {endpoint} -> {endpoint} (created)", flush=True)


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


def synthetic(height_m, tick, now, *, moving=False):
    """Encode valid bilateral raw geometry, then use the production decoder.

    Calibration sees steady forward arms at shoulder width and facing palms. Following
    frames add a small translation; Hand2 consumes the same recorded joints.
    """
    from .reference.pico import PICO_TO_MEDIAPIPE, parse_pico_packet
    from scipy.spatial.transform import Rotation

    payload = bytearray(struct.pack("<BBBB7f", 1, 7, 26, 0,
                                    0, 0, height_m*.93, 0, 0, 0, 1))
    for side, sign in (("left", 1), ("right", -1)):
        wrist = np.array([height_m*(.155882+.152941), sign*height_m*.1828/2, height_m*.80])
        if moving:
            wrist[0] += .003*np.sin(tick*.03)
        rotation = Rotation.from_euler("x", sign*np.pi/2)
        quaternion = rotation.as_quat()
        wrist_pose = np.r_[wrist, quaternion]
        payload.extend(struct.pack("<BBBB7f", 1, 0, 0, 0, *wrist_pose))
        points = np.tile(wrist, (26, 1))
        for index, pico_index in enumerate(PICO_TO_MEDIAPIPE):
            if index:
                finger, joint = divmod(index-1, 4)
                offset = np.array([.05+joint*.025, (2-finger)*.02, -.002*joint])
                points[pico_index] += rotation.apply(offset)
        for point in points:
            payload.extend(struct.pack("<BBBB7ff", 1, 0, 0, 0, *point, *quaternion, .008))
    packet = struct.pack("<BBqI", 0xAB, 0x40, tick*5, len(payload)) + payload
    return parse_pico_packet(packet, receiver_instance_id="self-test", connection_generation=1,
                             receiver_frame_sequence=tick, received_timestamp_ns=now)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=10002)
    parser.add_argument("--disable-hands", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration-s", type=float, default=0)
    parser.add_argument("--stats-interval-s", type=float, default=5.,
                        help="periodic timing summary interval; 0 disables summaries (default: 5s)")
    parser.add_argument("--record", type=Path)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--height-m", type=float, required=True)
    args = parser.parse_args(argv)
    if not math.isfinite(args.height_m) or not 1 <= args.height_m <= 2.4:
        parser.error("--height-m must be finite and in [1.0, 2.4] metres")
    if args.self_test and args.disable_hands:
        parser.error("--self-test requires both native Hand2 workers")
    if not 1 <= args.port <= 65535 or not math.isfinite(args.duration_s) or args.duration_s < 0:
        parser.error("invalid port/duration")
    if not math.isfinite(args.stats_interval_s) or args.stats_interval_s < 0:
        parser.error("--stats-interval-s must be finite and nonnegative")
    if args.self_test:
        # Synthetic targets must never enter the normal SPD session's domain.
        os.environ["ROS_DOMAIN_ID"] = "121"
    events = queue.SimpleQueue()
    slot_lock = threading.Lock()
    slot = [None, None, None]
    record = None
    stats = None
    clean = False
    if args.record:
        from .sim_recording import SimRecorder
        record = SimRecorder(args.record)
    try:
        with ExitStack() as stack:
            if not args.self_test:
                stack.enter_context(input_lock(args.port))
                if args.host in ("127.0.0.1", "localhost") and args.port == 10002:
                    ensure_adb_forward(args.port)
            model = display_model_path("marvin_m6_wuji2_shared_root_ceres.xml")
            profile = controller_profile("qp_ik_pico_shared_root_dls.yaml")
            simulation = DirectStateSimulation(model, profile)
            from .spd_publisher import SpdPublisher
            publisher = stack.enter_context(SpdPublisher())
            ik = stack.enter_context(DlsWorker(timeout_s=.5, continuous_follow=True))
            hands = {} if args.disable_hands else {
                side: stack.enter_context(NativeHandWorker(side, timeout_s=.5)) for side in ("left", "right")}
            from .display_contract import validate_dls_display
            validate_dls_display(simulation, ik)
            core = SharedRootCore(simulation.targets, ik, hands, height_m=args.height_m,
                                  continuous_follow=True)
            print("Shared-root height template + Franka DLS/Ruckig. "
                  "Calibration pose: arms forward, shoulder width, palms facing; hold 1s.", flush=True)

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
                    if stats is not None:
                        stats.record_input(frame)

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
                from .native_viewer import NativeViewer
                viewer = stack.enter_context(NativeViewer(
                    model, profile, events, continuous_follow=True))
            stack.enter_context(keyboard())
            for sig in (signal.SIGINT, signal.SIGTERM):
                old = signal.signal(sig, lambda *_: events.put("exit"))
                stack.callback(signal.signal, sig, old)
            print("PICO2 SIM ready. R calibrate/recalibrate; S starts continuous follow. "
                  "P/Space hold, H arms Home; Q/Esc/close or Ctrl+C exits without Home.", flush=True)
            print("Arm IK: Franka DLS + online Ruckig", flush=True)
            print("SPD ROS command publisher: /spd/tianji_wuji2/v1/joint_command; "
                  f"DDS domain {os.environ.get('ROS_DOMAIN_ID', '120')}; "
                  "simulation only, SPD local enable still required.", flush=True)
            start = time.monotonic()
            if args.stats_interval_s:
                from .runtime_stats import RuntimeStats
                stats = RuntimeStats(args.stats_interval_s, time.monotonic_ns())
            next_tick = start
            last_status = None
            seq = 0
            late = 0
            test_phase, test_phase_started = 0, start
            test_arm_motion = test_hand_motion = False
            test_ready_q = None
            stop_requested = False
            while not core.done and not stop_requested:
                publisher.check()
                now = time.monotonic_ns()
                cycle_started_ns = now
                elapsed = time.monotonic()-start
                if args.self_test:
                    frame = synthetic(args.height_m, seq, now, moving=core.mapping.state == "calibrated")
                    on_frame(frame)
                with slot_lock:
                    frame, error, fatal = slot
                now = time.monotonic_ns()
                if fatal:
                    raise RuntimeError(fatal)
                if error:
                    core.disconnected()
                elif frame is not None:
                    accepted_input = core.offer(frame, now)
                    if stats is not None:
                        stats.accepted_frames += int(accepted_input)
                        stats.observe("input_processing", time.monotonic_ns() - now)
                if args.self_test:
                    phase_age = time.monotonic() - test_phase_started
                    key = None
                    if test_phase == 0 and elapsed >= .1:
                        key = "r"
                        test_phase = 1
                    elif test_phase in (1, 5) and core.mapping.calibration_allows_start:
                        if core.state != "idle":
                            raise RuntimeError("self-test calibration activated follow without S")
                        test_ready_q = core.q.copy()
                        test_phase += 1
                        test_phase_started = time.monotonic()
                    elif test_phase in (2, 6):
                        if core.state != "idle" or not np.array_equal(core.q, test_ready_q):
                            raise RuntimeError("self-test ready targets moved before S")
                        if phase_age >= .2:
                            key = "s"
                            test_phase += 1
                    elif test_phase in (3, 7) and core.state == "teleop":
                        test_phase += 1
                        test_phase_started = time.monotonic()
                    elif test_phase == 4 and phase_age >= .8:
                        if not test_arm_motion or not test_hand_motion:
                            raise RuntimeError("self-test first R/S did not produce arm/hand motion")
                        test_arm_motion = test_hand_motion = False
                        key = "r"  # Recalibrate while following, without a Home command.
                        test_phase = 5
                    elif test_phase == 8 and phase_age >= .8:
                        key = "exit"
                        test_phase = 9
                    if key:
                        events.put(key)
                if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                    key = os.read(sys.stdin.fileno(), 1).decode(errors="ignore")
                    if key == "\x1b":
                        events.put("exit")
                    elif key.lower() in ("r", "s", "p", " ", "h", "q"):
                        events.put(key.lower())
                if viewer is not None:
                    viewer.poll_events()
                if args.duration_s and elapsed >= args.duration_s and not core.exit_requested:
                    events.put("exit")
                while not events.empty():
                    key = events.get()
                    now = time.monotonic_ns()
                    if key in ("exit", "q"):
                        stop_requested = True
                        break
                    accepted = core.action(key, now)
                    print(json.dumps(dict(key=key, accepted=accepted, state=core.state)), flush=True)
                    if record:
                        record.offer("event", dict(timestamp_ns=now, key=key, accepted=accepted, state=core.state))
                    if args.self_test and not accepted:
                        raise RuntimeError("self-test calibration/start request rejected")
                if stop_requested:
                    break
                # Worker reset/FK and terminal output can block. Recheck input
                # freshness at execution time, not with the pre-action clock.
                now = time.monotonic_ns()
                generated_utc_ns = time.time_ns()
                q = core.tick(now)
                if stats is not None:
                    if core.last_dls_ns:
                        stats.observe("dls", core.last_dls_ns)
                    if core.last_hands_ns:
                        stats.observe("hands", core.last_hands_ns)
                if args.self_test:
                    if test_phase in (4, 8):
                        test_arm_motion |= not np.allclose(q[:14], test_ready_q[:14], atol=1e-6, rtol=0)
                        test_hand_motion |= not np.allclose(q[14:], test_ready_q[14:], atol=1e-6, rtol=0)
                    if not np.isfinite(q).all():
                        raise RuntimeError("continuous follow produced nonfinite targets")
                seq += 1
                model_started_ns = time.monotonic_ns() if stats is not None else 0
                simulation.set_targets(CommandFrame(seq, now, ik.epoch, 7,
                    tuple(q[:7]), tuple(q[7:14]), tuple(q[14:34]), tuple(q[34:])))
                simulation.step()
                if stats is not None:
                    output_started_ns = time.monotonic_ns()
                    stats.observe("model", output_started_ns - model_started_ns)
                # Publish only after the existing whole-model validation has
                # accepted this target. q remains the controller output, not qpos.
                ready_mask, session_key = core.spd_output(time.monotonic_ns())
                publisher.offer(tuple(q), ready_mask, session_key, now, generated_utc_ns)
                if record:
                    association = (-1, -1) if core.frame is None else (core.frame.connection_generation, core.frame.receiver_frame_sequence)
                    record.offer("command", (now, core.state, q, association))
                status = (core.state, core.reason, core.mapping.state)
                if status != last_status:
                    if record:
                        record.offer("event", dict(timestamp_ns=now, state=core.state, reason=core.reason,
                                                   calibration=core.mapping.status()))
                    print(json.dumps(dict(state=core.state, reason=core.reason,
                                          calibration=core.mapping.status())), flush=True)
                    last_status = status
                if viewer is not None:
                    viewer.submit(now, q, "PICO2 SIM | " + core.state + " | " + core.reason +
                        " | Calibration: " + core.mapping.state +
                        " | Gestures (observation): " + str(core.gestures))
                if stats is not None:
                    finished_ns = time.monotonic_ns()
                    stats.observe("output", finished_ns - output_started_ns)
                    stats.record_cycle(cycle_started_ns, finished_ns, core.frame,
                                       core.phase, core.braking_reason)
                    if stats.due(finished_ns):
                        with slot_lock:
                            summary = stats.summary(finished_ns)
                        if summary is not None:
                            print(json.dumps(summary, separators=(",", ":")), flush=True)
                next_tick += .005
                wait = next_tick-time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                else:
                    late += 1; next_tick = time.monotonic()
                if args.self_test and elapsed > 45:
                    raise RuntimeError("self-test did not complete explicit R/S and recalibration")
            if args.self_test and (test_phase != 9 or not test_arm_motion or not test_hand_motion):
                raise RuntimeError("self-test did not exercise explicit DLS/Hand2 follow after both calibrations")
            if stats is not None:
                with slot_lock:
                    summary = stats.summary(time.monotonic_ns(), final=True)
                if summary is not None:
                    print(json.dumps(summary, separators=(",", ":")), flush=True)
            # Flush the last generated reference. Local exit does not command
            # Home; BEST_EFFORT publication is not a remote execution ACK.
            publisher.finish()
            print(json.dumps(dict(kind="pico2_sim_complete",
                                  home=bool(np.allclose(core.q[:14], core.home[:14], atol=1e-6, rtol=0)),
                                  ticks=seq, late_cycles=late, real_time_qualified=False)), flush=True)
        clean = True
    finally:
        if record:
            record.close(clean)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
