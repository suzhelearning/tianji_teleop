#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path
import socket
import struct
import subprocess
import shutil
import tempfile
import sys
import time
import zlib

# The controller project no longer ships the posture helper; it lives with the
# description package. The overlay is what ctest sources, so an unbuilt
# workspace fails with an explicit message rather than a bare ImportError.
try:
    from tianji_description.home_config import load_controller_posture
except ModuleNotFoundError as error:  # pragma: no cover - build prerequisite
    raise SystemExit(
        "tianji_description is not on the path; run 'pixi run build-workspace' "
        "before ctest") from error


PACKET_SIZE = 656
SEND_RATE_HZ = 72.0
SEND_DURATION_SECONDS = 1.10
VIEWER_DURATION_SECONDS = 1.75
# Joint telemetry is sampled asynchronously after simulation/control updates,
# not at the exact instant the configured posture is loaded.  Allow the small
# MuJoCo settling transient while still detecting a wrong startup posture
# (historical wrong postures differ by hundreds of milliradians or more).
INITIAL_POSTURE_TELEMETRY_TOLERANCE_RAD = 1.0e-2


def reserve_udp_port():
    reservation = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reservation.bind(("127.0.0.1", 0))
    port = reservation.getsockname()[1]
    reservation.close()
    return port


def quaternion(axis, angle):
    half = 0.5 * angle
    sine = math.sin(half)
    return (axis[0] * sine, axis[1] * sine, axis[2] * sine, math.cos(half))


def encode_packet(
    sequence,
    source_timestamp_ns,
    phase,
    orientation_amplitude,
    position_offset=0.0,
    button_pressed=False,
):
    left_position = (
        0.35 + position_offset + 0.015 * math.sin(phase),
        0.35 + 0.008 * math.cos(phase),
        1.25 + 0.010 * math.sin(0.5 * phase),
    )
    right_position = (
        0.35 + position_offset + 0.015 * math.sin(phase),
        -0.35 - 0.008 * math.cos(phase),
        1.25 + 0.010 * math.sin(0.5 * phase),
    )
    left_quaternion = quaternion(
        (0.0, 0.0, 1.0), orientation_amplitude * math.sin(phase)
    )
    right_quaternion = quaternion(
        (0.0, 1.0, 0.0), -orientation_amplitude * math.sin(phase)
    )
    bridge_send_ns = time.monotonic_ns()
    flags = 0xFF | (0x100 if button_pressed else 0)
    header = struct.pack(
        "<4sHHQQqqI",
        b"TJVR",
        4,
        PACKET_SIZE,
        sequence,
        9,
        source_timestamp_ns,
        bridge_send_ns,
        flags,
    )
    payload = struct.pack(
        "<14d",
        *left_position,
        *left_quaternion,
        *right_position,
        *right_quaternion,
    )
    # Keep the synthetic elbow directions away from world-down so an epoch
    # resynchronization exposes any accidental reset of the filtered arm-angle
    # reference back to the fallback posture.
    arm_directions = struct.pack("<6d", 0.0, 1.0, 0.0, 0.0, -1.0, 0.0)
    skeleton_points = (
        (0.0, 0.22, 1.121), (0.20, 0.30, 0.90),
        (0.42, 0.34, 1.02), (0.50, 0.35, 1.06),
        (0.0, -0.22, 1.121), (0.20, -0.30, 0.90),
        (0.42, -0.34, 1.02), (0.50, -0.35, 1.06),
    )
    skeleton = struct.pack("<24d", *(value for point in skeleton_points for value in point))
    skeleton_rotations = struct.pack(
        "<32d", *(value for _ in range(8) for value in (0.0, 0.0, 0.0, 1.0))
    )
    without_crc = header + payload + arm_directions + skeleton + skeleton_rotations
    assert len(without_crc) == 652
    packet = without_crc + struct.pack("<I", zlib.crc32(without_crc))
    assert len(packet) == PACKET_SIZE
    return packet


def encode_hand_packet(sequence, left_joint0, right_joint0):
    values = [0.0] * 40
    values[0] = left_joint0
    values[20] = right_joint0
    stamp = time.monotonic_ns()
    body = struct.pack(
        "<4sBBHQqqq40d",
        b"TJH2",
        2,
        (1 << 0) | (1 << 1),
        364,
        sequence,
        stamp,
        stamp,
        stamp,
        *values,
    )
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def parse_summary(stdout):
    summary_lines = [
        line for line in stdout.splitlines() if line.startswith("pico_headless_complete ")
    ]
    if len(summary_lines) != 1:
        raise AssertionError(f"missing PICO summary in stdout:\n{stdout}")
    values = {}
    for item in summary_lines[0].split()[1:]:
        key, value = item.split("=", 1)
        values[key] = value
    return values


def parse_recording_summary(stdout):
    lines = [
        line for line in stdout.splitlines()
        if line.startswith("pico_record_complete ")
    ]
    if len(lines) != 1:
        raise AssertionError(f"missing PICO recording summary in stdout:\n{stdout}")
    values = {}
    for item in lines[0].split()[1:]:
        key, value = item.split("=", 1)
        values[key] = value
    return values


def assert_tjvr_trace(trace_path, stdout, sent_packets):
    data = trace_path.read_bytes()
    if len(data) < 16:
        raise AssertionError("TJVR trace is missing its header")
    magic, version, packet_size, count = struct.unpack_from("<4sHHQ", data)
    assert magic == b"TJVT"
    assert version == 1
    assert packet_size == PACKET_SIZE
    assert count > 10
    assert len(data) == 16 + count * (8 + packet_size)
    recorded_packets = [
        data[
            16 + index * (8 + packet_size) + 8:
            16 + (index + 1) * (8 + packet_size)
        ]
        for index in range(count)
    ]
    for packet in recorded_packets:
        sequence = struct.unpack_from("<Q", packet, 8)[0]
        assert packet == sent_packets[sequence]
    relative_times = [
        struct.unpack_from("<q", data, 16 + index * (8 + packet_size))[0]
        for index in range(count)
    ]
    assert relative_times[0] == 0
    assert all(current >= previous for previous, current in zip(
        relative_times, relative_times[1:]
    ))
    summary = parse_recording_summary(stdout)
    assert summary["pico_record_state"] == "finalized"
    assert int(summary["pico_recorded_packets"]) == count
    assert int(summary["pico_record_packet_size"]) == packet_size


def assert_recording_cli_guards(arguments, directory):
    missing_teleop_path = Path(directory) / "missing_teleop.tjvr"
    result = subprocess.run(
        [
            arguments.viewer,
            "--no-pico-teleop",
            "--pico-record",
            str(missing_teleop_path),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "--pico-record requires --pico-teleop" in result.stdout

    existing_path = Path(directory) / "existing.tjvr"
    existing_path.write_bytes(b"do-not-overwrite")
    result = subprocess.run(
        [
            arguments.viewer,
            "--config", arguments.config,
            "--model", arguments.model,
            "--pico-teleop",
            "--pico-port", str(reserve_udp_port()),
            "--pico-record", str(existing_path),
            "--headless", "--duration", "0.1",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "failed to create PICO trace" in result.stdout
    assert existing_path.read_bytes() == b"do-not-overwrite"


def finite_column(rows, name):
    if name not in rows[0]:
        raise AssertionError(f"missing telemetry column: {name}")
    values = [float(row[name]) for row in rows]
    if not values or not all(math.isfinite(value) for value in values):
        raise AssertionError(f"non-finite telemetry column: {name}")


def configured_initial_posture(config_path):
    posture = load_controller_posture(config_path)
    if posture is None:
        return None
    return dict(zip(("initial_left_q_rad", "initial_right_q_rad"), posture))


def configured_model_state_only(config_path):
    for line in Path(config_path).read_text().splitlines():
        stripped = line.strip().lower()
        if stripped.startswith("model_state_only:"):
            return stripped.split(":", maxsplit=1)[1].strip() == "true"
    return False


def assert_initial_joint_state(joint_telemetry_path, config_path):
    expected = configured_initial_posture(config_path)
    if expected is None:
        return
    with joint_telemetry_path.open(newline="") as telemetry_file:
        rows = list(csv.DictReader(telemetry_file))
    if not rows:
        raise AssertionError("joint telemetry CSV contains no samples")
    first = rows[0]
    for side in ("left", "right"):
        expected_q = expected[f"initial_{side}_q_rad"]
        for joint, expected_value in enumerate(expected_q, start=1):
            actual_q = float(first[f"{side}_j{joint}_reference_q"])
            if (
                abs(actual_q - expected_value)
                > INITIAL_POSTURE_TELEMETRY_TOLERANCE_RAD
            ):
                raise AssertionError(
                    f"{side} J{joint} initial q is {actual_q}, "
                    f"expected {expected_value}"
                )


def quaternion_distance(left, right):
    dot = abs(sum(a * b for a, b in zip(left, right)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def assert_otg_reference_continuity(rows):
    # Translation limits are configured per axis, so the largest possible
    # Euclidean speed is sqrt(3) times the configured 3 m/s per-axis limit.
    max_linear_speed = math.sqrt(3.0) * 3.0
    max_angular_speed = 12.0
    tolerance = 1.0e-6
    for previous, current in zip(rows, rows[1:]):
        dt = float(current["control_time_seconds"]) - float(
            previous["control_time_seconds"]
        )
        if dt <= 0.0:
            continue
        for side in ("left", "right"):
            previous_position = tuple(
                float(previous[f"{side}_reference_p{axis}"])
                for axis in "xyz"
            )
            current_position = tuple(
                float(current[f"{side}_reference_p{axis}"])
                for axis in "xyz"
            )
            linear_speed = math.sqrt(
                sum(
                    (current_value - previous_value) ** 2
                    for previous_value, current_value in zip(
                        previous_position, current_position
                    )
                )
            ) / dt
            if linear_speed > max_linear_speed + tolerance:
                raise AssertionError(
                    f"{side} OTG reference teleported at sequence "
                    f"{current['sequence']}: {linear_speed} m/s"
                )

            previous_quaternion = tuple(
                float(previous[f"{side}_reference_q{axis}"])
                for axis in "xyzw"
            )
            current_quaternion = tuple(
                float(current[f"{side}_reference_q{axis}"])
                for axis in "xyzw"
            )
            angular_speed = (
                quaternion_distance(previous_quaternion, current_quaternion) / dt
            )
            if angular_speed > max_angular_speed + tolerance:
                raise AssertionError(
                    f"{side} OTG orientation teleported at sequence "
                    f"{current['sequence']}: {angular_speed} rad/s"
                )


def assert_arm_angle_continuity_across_resynchronization(rows):
    for previous, current in zip(rows, rows[1:]):
        if int(current["pico_reset_applies"]) <= int(
            previous["pico_reset_applies"]
        ):
            continue
        for side in ("left", "right"):
            previous_error = float(previous[f"{side}_arm_angle_error_rad"])
            current_error = float(current[f"{side}_arm_angle_error_rad"])
            wrapped_change = math.atan2(
                math.sin(current_error - previous_error),
                math.cos(current_error - previous_error),
            )
            if abs(wrapped_change) > 0.10:
                raise AssertionError(
                    f"{side} arm-angle reference jumped at PICO reset, "
                    f"sequence {current['sequence']}: {wrapped_change} rad"
                )


def run_test(arguments):
    target_timeout_seconds = next(
        float(line.split(":", 1)[1].split("#", 1)[0])
        for line in Path(arguments.config).read_text().splitlines()
        if line.strip().startswith("target_timeout_seconds:")
    )
    viewer_duration = max(
        arguments.viewer_duration,
        0.20 + arguments.send_duration + target_timeout_seconds + 0.20,
    )
    port = reserve_udp_port()
    with tempfile.TemporaryDirectory(prefix="tianji_pico_viewer_") as directory:
        if arguments.assert_recording:
            assert_recording_cli_guards(arguments, directory)
        telemetry_path = Path(directory) / "telemetry.csv"
        joint_telemetry_path = Path(directory) / "joint_telemetry.csv"
        trace_path = Path(directory) / "input.tjvr"
        viewer_command = [
            arguments.viewer,
            "--config",
            arguments.config,
            "--model",
            arguments.model,
            "--pico-teleop",
            "--pico-skeleton-overlay",
            "--pico-bind",
            "127.0.0.1",
            "--pico-port",
            str(port),
            "--control-level",
            arguments.control_level,
            "--headless",
            "--duration",
            str(viewer_duration),
            "--telemetry",
            str(telemetry_path),
            "--joint-telemetry",
            str(joint_telemetry_path),
        ]
        if arguments.algorithm:
            viewer_command.extend(["--algorithm", arguments.algorithm])
        if arguments.assert_recording:
            viewer_command.extend(["--pico-record", str(trace_path)])
        if arguments.model_state_only:
            viewer_command.append("--model-state-only")
        if arguments.requested_arm_angle_mode:
            viewer_command.extend(
                ["--arm-angle-mode", arguments.requested_arm_angle_mode]
            )
        process = subprocess.Popen(
            viewer_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        sequence = 1
        sent_packets = {}
        if not arguments.no_send:
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                time.sleep(0.20)
                source_start_ns = time.monotonic_ns()
                send_start = time.monotonic()
                deadline = send_start
                while time.monotonic() - send_start < arguments.send_duration:
                    if process.poll() is not None:
                        break
                    source_timestamp_ns = source_start_ns + round(
                        (sequence - 1) * 1.0e9 / arguments.send_rate
                    )
                    phase = (
                        2.0
                        * math.pi
                        * arguments.motion_frequency
                        * (sequence - 1)
                        / arguments.send_rate
                    )
                    encoded = encode_packet(
                            sequence,
                            source_timestamp_ns,
                            phase,
                            arguments.orientation_amplitude,
                            0.30
                            if arguments.inject_resynchronization
                            and sequence >= 20
                            else 0.0,
                        )
                    sent_packets[sequence] = encoded
                    sender.sendto(encoded, ("127.0.0.1", port))
                    sent_sequence = sequence
                    sequence += 1
                    if arguments.inject_resynchronization and sent_sequence == 22:
                        # Burst ordinary frames immediately after the event
                        # frame so the receiver's latest-only slot supersedes
                        # it before the 200 Hz control loop can rely on it.
                        for _ in range(8):
                            next_timestamp_ns = source_start_ns + round(
                                (sequence - 1) * 1.0e9 / arguments.send_rate
                            )
                            next_phase = (
                                2.0
                                * math.pi
                                * arguments.motion_frequency
                                * (sequence - 1)
                                / arguments.send_rate
                            )
                            encoded = encode_packet(
                                    sequence,
                                    next_timestamp_ns,
                                    next_phase,
                                    arguments.orientation_amplitude,
                                    0.30,
                                )
                            sent_packets[sequence] = encoded
                            sender.sendto(encoded, ("127.0.0.1", port))
                            sequence += 1
                    deadline += 1.0 / arguments.send_rate
                    time.sleep(max(0.0, deadline - time.monotonic()))
            finally:
                sender.close()

        stdout, _ = process.communicate(timeout=5.0)
        if process.returncode != 0:
            raise AssertionError(
                f"Viewer exited with {process.returncode}:\n{stdout}"
            )
        if arguments.assert_recording:
            assert_tjvr_trace(trace_path, stdout, sent_packets)
            if arguments.trace_output:
                shutil.copyfile(trace_path, arguments.trace_output)
        expected_state_source = (
            "control_state_source=model_reference"
            if arguments.model_state_only
            or configured_model_state_only(arguments.config)
            else "control_state_source=actual_feedback_guarded"
        )
        if expected_state_source not in stdout:
            raise AssertionError(
                f"missing {expected_state_source} in viewer output:\n{stdout}"
            )
        assert_initial_joint_state(joint_telemetry_path, arguments.config)
        if arguments.no_send:
            summary = parse_summary(stdout)
            assert summary["pico_configured"] == "1"
            assert summary["pico_enabled"] == "1"
            assert summary["arm_angle_mode"] == "outward_only"
            assert summary["pico_live"] == "0"
            assert summary["pico_stale"] == "0"
            assert summary["pico_sequence"] == "0"
            assert summary["control_failures"] == "0"
            print(" ".join(f"{key}={value}" for key, value in summary.items()))
            return
        if sequence <= int(arguments.send_rate) + 1:
            raise AssertionError(f"sent fewer than one second of frames: {sequence - 1}")

        summary = parse_summary(stdout)
        if arguments.assert_recording:
            recording = parse_recording_summary(stdout)
            assert int(recording["pico_recorded_packets"]) == int(
                summary["pico_datagrams"]
            )
            if arguments.inject_resynchronization:
                assert int(recording["pico_recorded_packets"]) > int(
                    summary["pico_accepted"]
                )
        assert summary["control_level"] == arguments.control_level
        if arguments.algorithm:
            assert summary["algorithm"] == arguments.algorithm
        assert summary["pico_configured"] == "1"
        assert summary["pico_enabled"] == "1"
        assert summary["arm_angle_mode"] == "outward_only"
        assert summary["pico_live"] == "0"
        assert summary["pico_stale"] == "1"
        assert int(summary["pico_tracking_epoch"]) == 9
        assert int(summary["pico_left_source_timestamp_ns"]) > 0
        assert (
            summary["pico_left_source_timestamp_ns"]
            == summary["pico_right_source_timestamp_ns"]
        )
        assert int(summary["pico_sequence"]) > 10
        assert summary["pico_live_seen"] == "1"
        assert summary["pico_stale_seen"] == "1"
        assert summary["pico_skeleton_valid_seen"] == "1"
        assert summary["pico_skeleton_hidden_after_stale"] == "1"
        assert int(summary["joint_plot_drained"]) > 100
        assert summary["joint_plot_both_arms_finite"] == "1"
        assert summary["joint_plot_reference_jerk_valid_seen"] == "1"
        assert summary["joint_plot_actual_jerk_valid_seen"] == "1"
        stale_transition_ms = float(summary["pico_stale_transition_ms"])
        expected_timeout_ms = target_timeout_seconds * 1000.0
        assert expected_timeout_ms <= stale_transition_ms <= expected_timeout_ms + 10.0, summary
        for counter in (
            "pico_malformed",
            "pico_crc_failures",
            "pico_reordered",
            "control_failures",
        ):
            assert int(summary[counter]) == 0, summary
        if arguments.inject_resynchronization:
            assert int(summary["pico_jump_rejections"]) == 2, summary
            assert int(summary["pico_resynchronizations"]) == 1, summary
            assert int(summary["pico_reset_applies"]) == 2, summary
            assert int(summary["pico_superseded"]) >= 1, summary
        else:
            assert int(summary["pico_jump_rejections"]) == 0, summary
            assert int(summary["pico_resynchronizations"]) == 0, summary
            assert int(summary["pico_reset_applies"]) == 1, summary

        with telemetry_path.open(newline="") as telemetry_file:
            rows = list(csv.DictReader(telemetry_file))
        if not rows:
            raise AssertionError("telemetry CSV contains no samples")
        if arguments.control_level == "velocity":
            for side in ("left", "right"):
                finite_column(rows, f"{side}_qddot_max_ratio")
            if arguments.algorithm == (
                "spark_upper_qpoases_headroom_feedforward_velocity_qp"
            ):
                for side in ("left", "right"):
                    assert any(
                        float(row[f"{side}_qddot_max_ratio"]) > 1.0e-4
                        for row in rows
                        if row["pico_live"] == "1"
                    )
                with joint_telemetry_path.open(newline="") as joint_file:
                    joint_rows = list(csv.DictReader(joint_file))
                assert joint_rows
                for side in ("left", "right"):
                    for joint in range(1, 8):
                        expected = 1000.0 if joint <= 3 else 1500.0
                        lower_name = f"{side}_j{joint}_jerk_lower"
                        upper_name = f"{side}_j{joint}_jerk_upper"
                        assert all(
                            math.isfinite(float(row[lower_name]))
                            and math.isfinite(float(row[upper_name]))
                            for row in joint_rows
                        )
                        assert all(
                            abs(float(row[lower_name]) + expected) <= 1.0e-9
                            and abs(float(row[upper_name]) - expected) <= 1.0e-9
                            for row in joint_rows
                        )
        for name in ("left_fallback_applied", "right_fallback_applied"):
            if name not in rows[0]:
                raise AssertionError(f"missing fallback telemetry column: {name}")
        assert any(row["pico_live"] == "1" for row in rows)
        assert any(row["pico_stale"] == "1" for row in rows)
        assert all(row["left_arm_angle_source"] != "pico" for row in rows)
        assert all(row["right_arm_angle_source"] != "pico" for row in rows)
        if arguments.algorithm in (
            "spark_guided_velocity_qp",
            "spark_direct_velocity_qp",
            "spark_upper_qpoases_velocity_qp",
            "spark_upper_qpoases_cartesian_otg_velocity_qp",
            "spark_upper_qpoases_feedforward_velocity_qp",
            "spark_upper_qpoases_headroom_feedforward_velocity_qp",
        ):
            assert any(row["left_spark_posture_active"] == "1" for row in rows)
            assert any(row["right_spark_posture_active"] == "1" for row in rows)
            assert any(row["left_spark_ik_accepted"] == "1" for row in rows)
            assert any(row["right_spark_ik_accepted"] == "1" for row in rows)
            for side in ("left", "right"):
                finite_column(
                    rows, f"{side}_spark_motion_intent_linear_velocity"
                )
                finite_column(
                    rows, f"{side}_spark_motion_intent_angular_velocity"
                )
                assert all(
                    row[f"{side}_spark_stationary_joint_reference_held"]
                    in ("0", "1")
                    for row in rows
                )
                if (
                    arguments.algorithm
                    == "spark_upper_qpoases_headroom_feedforward_velocity_qp"
                ):
                    assert all(
                        row[f"{side}_spark_settled_hold_active"] in ("0", "1")
                        for row in rows
                    )
                    finite_column(
                        rows, f"{side}_spark_settled_hold_dwell_seconds"
                    )
                    assert all(
                        row[f"{side}_spark_settled_hold_reason"]
                        in ("none", "stale", "headroom_exhausted")
                        for row in rows
                    )
                    assert any(
                        row[f"{side}_spark_settled_hold_active"] == "1"
                        for row in rows
                    )
                for name in (
                    "spark_solve_time_us",
                    "spark_palm_position_error_m",
                    "spark_palm_orientation_error_rad",
                    "spark_reference_velocity_ratio",
                    "spark_reference_acceleration_ratio",
                    "spark_reference_jerk_ratio",
                    "spark_q_ik_error_max_abs",
                    "spark_q_ref_error_max_abs",
                    "spark_posture_velocity_max_abs",
                ):
                    finite_column(rows, f"{side}_{name}")
        if (
            arguments.algorithm
            == "spark_upper_qpoases_cartesian_otg_velocity_qp"
        ):
            assert any(row["otg_enabled"] == "1" for row in rows)
            assert any(
                float(row["left_v_ref"]) > 1.0e-5
                or float(row["right_v_ref"]) > 1.0e-5
                or float(row["left_w_ref"]) > 1.0e-5
                or float(row["right_w_ref"]) > 1.0e-5
                for row in rows
            )
        if (
            arguments.algorithm
            == "spark_upper_qpoases_feedforward_velocity_qp"
        ):
            assert any(
                float(row["left_v_ref"]) > 1.0e-5
                or float(row["right_v_ref"]) > 1.0e-5
                or float(row["left_w_ref"]) > 1.0e-5
                or float(row["right_w_ref"]) > 1.0e-5
                for row in rows
                if row["pico_live"] == "1"
            )
        if arguments.algorithm in (
            "spark_upper_qpoases_feedforward_velocity_qp",
            "spark_upper_qpoases_headroom_feedforward_velocity_qp",
        ):
            for side in ("left", "right"):
                for name in (
                    "valid",
                    "state",
                    "dt_valid",
                    "jump_rejected",
                    "epoch_reset",
                    "source_dt_seconds",
                    "median_dt_seconds",
                    "activation",
                    "linear_velocity",
                    "angular_velocity",
                ):
                    finite_column(rows, f"{side}_spark_feedforward_{name}")
                for quantity in ("q_ik", "q", "qdot", "qddot", "jerk"):
                    for joint in range(1, 8):
                        finite_column(
                            rows,
                            f"{side}_spark_feedforward_{quantity}_j{joint}",
                        )
            live_rows = [row for row in rows if row["pico_live"] == "1"]
            assert any(
                abs(float(row[f"{side}_spark_feedforward_qdot_j{joint}"]))
                > 1.0e-5
                for row in live_rows
                for side in ("left", "right")
                for joint in range(1, 8)
            )
            for row in rows:
                for side in ("left", "right"):
                    for joint in range(1, 8):
                        acceleration_limit = 60.0 if joint <= 3 else 90.0
                        jerk_limit = 1000.0 if joint <= 3 else 1500.0
                        assert abs(
                            float(
                                row[
                                    f"{side}_spark_feedforward_qddot_j{joint}"
                                ]
                            )
                        ) <= acceleration_limit + 1.0e-6
                        assert abs(
                            float(
                                row[f"{side}_spark_feedforward_jerk_j{joint}"]
                            )
                        ) <= jerk_limit + 1.0e-6
        if (
            arguments.algorithm
            == "spark_upper_qpoases_headroom_feedforward_velocity_qp"
        ):
            for side in ("left", "right"):
                for name in (
                    "valid",
                    "derivative_history_valid",
                    "velocity",
                    "acceleration",
                    "jerk",
                    "task",
                    "raw",
                    "filtered",
                    "scale",
                    "state",
                    "dominant_source",
                ):
                    finite_column(rows, f"{side}_headroom_{name}")
                assert all(
                    0.0 <= float(row[f"{side}_headroom_scale"]) <= 1.0
                    for row in rows
                )
        if arguments.algorithm == "spark_pose_velocity_qp":
            assert all(row["left_spark_posture_active"] == "0" for row in rows)
            assert all(row["right_spark_posture_active"] == "0" for row in rows)
            assert all(row["left_spark_ik_accepted"] == "0" for row in rows)
            assert all(row["right_spark_ik_accepted"] == "0" for row in rows)
        if arguments.algorithm == "spark_upper_qpoases_direct":
            assert all(row["left_spark_posture_active"] == "0" for row in rows)
            assert all(row["right_spark_posture_active"] == "0" for row in rows)
            assert any(row["left_spark_ik_accepted"] == "1" for row in rows)
            assert any(row["right_spark_ik_accepted"] == "1" for row in rows)
        assert all(
            row["pico_left_source_timestamp_ns"]
            == row["pico_right_source_timestamp_ns"]
            for row in rows
        )
        assert all(
            row["left_target_stale"] == row["right_target_stale"]
            for row in rows
        )
        assert any(
            row["left_target_stale"] == "1" and row["right_target_stale"] == "1"
            for row in rows
        )
        assert any(
            float(row["left_position_error_m"]) > 1.0e-4
            and float(row["right_position_error_m"]) > 1.0e-4
            for row in rows
        )
        assert max(int(row["pico_sequence"]) for row in rows) == int(
            summary["pico_sequence"]
        )
        assert all(row["pico_tracking_epoch"] in ("0", "9") for row in rows)
        if arguments.inject_resynchronization:
            assert max(int(row["pico_resynchronizations"]) for row in rows) == 1
            assert max(int(row["pico_jump_rejections"]) for row in rows) == 2
            assert max(int(row["pico_reset_applies"]) for row in rows) == 2
            assert_otg_reference_continuity(rows)
            assert_arm_angle_continuity_across_resynchronization(rows)
        for name in (
            "pico_bridge_to_control_us",
            "pico_receive_to_control_us",
            "cycle_time_us",
            "left_position_error_m",
            "left_orientation_error_rad",
            "right_position_error_m",
            "right_orientation_error_rad",
            "left_arm_angle_error_rad",
            "left_arm_angle_control_error_rad",
            "left_arm_angle_current_rate_rad_s",
            "left_arm_angle_requested_velocity_rad_s",
            "left_arm_angle_requested_acceleration_rad_s2",
            "left_arm_angle_radius_m",
            "left_arm_angle_reference_projection_norm",
            "left_arm_angle_jacobian_norm",
            "left_arm_angle_projection_held",
            "left_arm_angle_reference_governor_held",
            "left_arm_angle_achieved_acceleration_rad_s2",
            "left_arm_angle_acceleration_residual_rad_s2",
            "left_elbow_world_z",
            "left_shoulder_world_z",
            "left_dls_posture_initial_position_error_m",
            "left_dls_posture_final_position_error_m",
            "left_dls_posture_initial_orientation_error_rad",
            "left_dls_posture_final_orientation_error_rad",
            "left_dls_posture_goal_error_max_abs",
            "left_dls_posture_reference_error_max_abs",
            "left_dls_posture_velocity_target_max_abs",
            "left_dls_posture_qdot_error_max_abs",
            "left_dls_posture_goal_limit_margin_rad",
            "right_arm_angle_error_rad",
            "right_arm_angle_control_error_rad",
            "right_arm_angle_current_rate_rad_s",
            "right_arm_angle_requested_velocity_rad_s",
            "right_arm_angle_requested_acceleration_rad_s2",
            "right_arm_angle_radius_m",
            "right_arm_angle_reference_projection_norm",
            "right_arm_angle_jacobian_norm",
            "right_arm_angle_projection_held",
            "right_arm_angle_reference_governor_held",
            "right_arm_angle_achieved_acceleration_rad_s2",
            "right_arm_angle_acceleration_residual_rad_s2",
            "right_elbow_world_z",
            "right_shoulder_world_z",
            "right_dls_posture_initial_position_error_m",
            "right_dls_posture_final_position_error_m",
            "right_dls_posture_initial_orientation_error_rad",
            "right_dls_posture_final_orientation_error_rad",
            "right_dls_posture_goal_error_max_abs",
            "right_dls_posture_reference_error_max_abs",
            "right_dls_posture_velocity_target_max_abs",
            "right_dls_posture_qdot_error_max_abs",
            "right_dls_posture_goal_limit_margin_rad",
        ):
            finite_column(rows, name)
        pose_roles = ("target", "reference", "actual")
        pose_components = ("px", "py", "pz", "qx", "qy", "qz", "qw")
        for side in ("left", "right"):
            for role in pose_roles:
                for component in pose_components:
                    finite_column(rows, f"{side}_{role}_{component}")
        if arguments.max_steady_orientation_error_mean is not None:
            steady = [
                max(
                    float(row["left_orientation_error_rad"]),
                    float(row["right_orientation_error_rad"]),
                )
                for row in rows
                if row["pico_live"] == "1"
                and int(row["pico_sequence"]) >= 120
            ]
            if not steady:
                raise AssertionError("no live steady-state orientation samples")
            steady_mean = sum(steady) / len(steady)
            steady_peak = max(steady)
            if steady_mean > arguments.max_steady_orientation_error_mean:
                raise AssertionError(
                    f"orientation error mean {steady_mean} exceeds "
                    f"{arguments.max_steady_orientation_error_mean}"
                )
            summary["steady_orientation_error_mean"] = f"{steady_mean:.9f}"
            summary["steady_orientation_error_peak"] = f"{steady_peak:.9f}"
        if arguments.telemetry_output:
            output_path = Path(arguments.telemetry_output)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(telemetry_path, output_path)
        print(" ".join(f"{key}={value}" for key, value in summary.items()))


def run_button_state_test(arguments):
    port = reserve_udp_port()
    with tempfile.TemporaryDirectory(prefix="tianji_pico_button_") as directory:
        telemetry_path = Path(directory) / "telemetry.csv"
        joint_telemetry_path = Path(directory) / "joint_telemetry.csv"
        viewer_command = [
            arguments.viewer,
            "--config", arguments.config,
            "--model", arguments.model,
            "--pico-teleop",
            "--pico-bind", "127.0.0.1",
            "--pico-port", str(port),
            "--control-level", arguments.control_level,
            "--headless",
            "--duration", "2.5",
            "--telemetry", str(telemetry_path),
            "--joint-telemetry", str(joint_telemetry_path),
            "--model-state-only",
        ]
        if arguments.algorithm:
            viewer_command.extend(["--algorithm", arguments.algorithm])
        process = subprocess.Popen(
            viewer_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sequence = 1
        source_start_ns = time.monotonic_ns()

        def send_level(button_pressed, count):
            nonlocal sequence
            for _ in range(count):
                source_timestamp_ns = source_start_ns + round(
                    (sequence - 1) * 1.0e9 / arguments.send_rate
                )
                phase = (
                    2.0 * math.pi * arguments.motion_frequency
                    * (sequence - 1) / arguments.send_rate
                )
                packet = encode_packet(
                    sequence,
                    source_timestamp_ns,
                    phase,
                    arguments.orientation_amplitude,
                    button_pressed=button_pressed,
                )
                sender.sendto(packet, ("127.0.0.1", port))
                sequence += 1
                time.sleep(1.0 / arguments.send_rate)

        try:
            time.sleep(0.20)
            send_level(False, 18)
            send_level(True, 24)
            send_level(False, 24)
        finally:
            sender.close()

        stdout, _ = process.communicate(timeout=6.0)
        if process.returncode != 0:
            raise AssertionError(
                f"Viewer exited with {process.returncode}:\n{stdout}"
            )
        with telemetry_path.open(newline="") as telemetry_file:
            rows = list(csv.DictReader(telemetry_file))
        if not rows:
            raise AssertionError("button test produced no telemetry")
        enabled = [row["pico_enabled"] == "1" for row in rows]
        try:
            pause_index = next(index for index, value in enumerate(enabled) if not value)
            resume_index = next(
                index for index in range(pause_index + 1, len(enabled))
                if enabled[index]
            )
        except StopIteration as error:
            raise AssertionError(
                "telemetry did not show active -> paused -> active"
            ) from error
        assert any(enabled[:pause_index])
        assert any(not value for value in enabled[pause_index:resume_index])
        assert any(enabled[resume_index:])
        assert all(int(row["control_failures"]) == 0 for row in rows)
        assert all(int(row["pico_malformed"]) == 0 for row in rows)
        assert all(int(row["pico_crc_failures"]) == 0 for row in rows)
        assert_otg_reference_continuity(rows)
        print(
            "button_state_test_passed "
            f"pause_index={pause_index} resume_index={resume_index} "
            f"rows={len(rows)}"
        )


def run_button_hand_pause_test(arguments):
    pico_port = reserve_udp_port()
    hand_port = reserve_udp_port()
    command = [
        arguments.viewer,
        "--config", arguments.config,
        "--model", arguments.model,
        "--pico-teleop",
        "--pico-bind", "127.0.0.1",
        "--pico-port", str(pico_port),
        "--hand-teleop",
        "--hand-bind", "127.0.0.1",
        "--hand-port", str(hand_port),
        "--control-level", arguments.control_level,
        "--headless",
        "--duration", "1.8",
        "--algorithm", arguments.algorithm or "hierarchical_qp",
        "--model-state-only",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    pico_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    hand_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    source_start_ns = time.monotonic_ns()
    sequence = 1

    def send_pico(button_pressed):
        nonlocal sequence
        packet = encode_packet(
            sequence,
            source_start_ns + sequence * 10_000_000,
            sequence * 0.02,
            0.02,
            button_pressed=button_pressed,
        )
        pico_sender.sendto(packet, ("127.0.0.1", pico_port))
        sequence += 1

    try:
        time.sleep(0.20)
        for _ in range(18):
            send_pico(False)
            hand_sender.sendto(
                encode_hand_packet(sequence, 0.1, 0.2),
                ("127.0.0.1", hand_port),
            )
            time.sleep(1.0 / arguments.send_rate)
        send_pico(True)
        time.sleep(0.12)
        for _ in range(26):
            send_pico(True)
            hand_sender.sendto(
                encode_hand_packet(sequence, 0.7, 0.8),
                ("127.0.0.1", hand_port),
            )
            time.sleep(1.0 / arguments.send_rate)
    finally:
        pico_sender.close()
        hand_sender.close()

    stdout, _ = process.communicate(timeout=5.0)
    if process.returncode != 0:
        raise AssertionError(
            f"Viewer exited with {process.returncode}:\n{stdout}"
        )
    summary = parse_summary(stdout)
    assert summary["pico_enabled"] == "0", summary
    assert summary["hand_configured"] == "1", summary
    assert int(summary["hand_accepted"]) > 0, summary
    assert math.isclose(float(summary["hand_left_q0"]), 0.1, abs_tol=1.0e-12)
    assert math.isclose(float(summary["hand_right_q0"]), 0.2, abs_tol=1.0e-12)
    assert int(summary["control_failures"]) == 0, summary
    assert int(summary["pico_malformed"]) == 0, summary
    assert int(summary["pico_crc_failures"]) == 0, summary
    print(
        "button_hand_pause_test_passed "
        f"hand_left_q0={summary['hand_left_q0']} "
        f"hand_right_q0={summary['hand_right_q0']}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument(
        "--control-level", choices=("velocity", "acceleration"), required=True
    )
    parser.add_argument(
        "--algorithm",
        choices=(
            "hierarchical_qp",
            "nullspace_dls",
            "spark_guided_velocity_qp",
            "spark_direct_velocity_qp",
            "spark_pose_velocity_qp",
            "spark_upper_qpoases_direct",
            "spark_upper_qpoases_velocity_qp",
            "spark_upper_qpoases_cartesian_otg_velocity_qp",
            "spark_upper_qpoases_feedforward_velocity_qp",
            "spark_upper_qpoases_headroom_feedforward_velocity_qp",
        ),
    )
    parser.add_argument("--viewer-duration", type=float, default=VIEWER_DURATION_SECONDS)
    parser.add_argument("--send-duration", type=float, default=SEND_DURATION_SECONDS)
    parser.add_argument("--send-rate", type=float, default=SEND_RATE_HZ)
    parser.add_argument("--motion-frequency", type=float, default=0.5)
    parser.add_argument("--orientation-amplitude", type=float, default=0.08)
    parser.add_argument("--max-steady-orientation-error-mean", type=float)
    parser.add_argument("--telemetry-output")
    parser.add_argument("--no-send", action="store_true")
    parser.add_argument("--inject-resynchronization", action="store_true")
    parser.add_argument("--model-state-only", action="store_true")
    parser.add_argument("--assert-recording", action="store_true")
    parser.add_argument("--trace-output")
    parser.add_argument(
        "--requested-arm-angle-mode",
        choices=("pico", "default_down", "outward_only", "pico_outward"),
    )
    parser.add_argument("--button-state-test", action="store_true")
    parser.add_argument("--button-hand-pause-test", action="store_true")
    arguments = parser.parse_args()
    if arguments.button_hand_pause_test:
        run_button_hand_pause_test(arguments)
    elif arguments.button_state_test:
        run_button_state_test(arguments)
    else:
        run_test(arguments)


if __name__ == "__main__":
    main()
