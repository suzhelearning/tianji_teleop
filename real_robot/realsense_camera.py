"""Read-only RealSense RGB publisher at 1280x720 / 30 FPS with OpenCV preview."""

import argparse
from array import array
import os
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image


WIDTH, HEIGHT, FPS = 1280, 720, 30
WINDOW = 'RealSense - RGB (Q / Esc to quit)'


def camera_info(profile, frame_id):
    intr = profile.as_video_stream_profile().get_intrinsics()
    models = {
        rs.distortion.none: 'plumb_bob',
        rs.distortion.brown_conrady: 'plumb_bob',
        rs.distortion.inverse_brown_conrady: 'realsense_inverse_brown_conrady',
    }
    if intr.model not in models:
        raise ValueError(f'Unsupported RealSense distortion model: {intr.model}')
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width, msg.height = intr.width, intr.height
    # Inverse Brown coefficients are NOT standard ROS/OpenCV plumb_bob coefficients.
    # Preserve their actual model; consumers must use RealSense deprojection.
    msg.distortion_model = models[intr.model]
    msg.d = list(intr.coeffs)
    msg.k = [intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def image_message(pixels, encoding, header):
    msg = Image()
    msg.header = header
    msg.height, msg.width = pixels.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = int(sys.byteorder == 'big')
    msg.step = pixels.strides[0]
    data = array('B')
    data.frombytes(memoryview(pixels).cast('B'))
    msg.data = data
    return msg


def run(args):
    if not args.no_preview and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise RuntimeError('No desktop display. Run in a desktop terminal or use --no-preview.')
    context = rs.context()
    devices = context.query_devices()
    if not args.serial and len(devices) != 1:
        raise RuntimeError(f'Found {len(devices)} RealSense devices; connect one or select --serial.')
    serial = args.serial or devices[0].get_info(rs.camera_info.serial_number)
    pipeline = rs.pipeline(context)
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, FPS)
    device = config.resolve(rs.pipeline_wrapper(pipeline)).get_device()
    for sensor in device.query_sensors():
        if sensor.supports(rs.option.global_time_enabled):
            sensor.set_option(rs.option.global_time_enabled, 1)

    node = rclpy.create_node('realsense_camera')
    started = False
    try:
        profile = pipeline.start(config)
        started = True
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        image_publisher = node.create_publisher(Image, '/realsense/color/image_raw', qos)
        info_publisher = node.create_publisher(CameraInfo, '/realsense/color/camera_info', qos)
        info = camera_info(profile.get_stream(rs.stream.color), 'realsense_color_optical_frame')
        node.get_logger().info(
            f'{device.get_info(rs.camera_info.name)} serial={serial}, {WIDTH}x{HEIGHT}@{FPS}; '
            'RGB only; no depth stream or camera-to-robot TF.'
        )
        node.get_logger().info(
            'Color calibration model: ' + info.distortion_model
            + '; inverse Brown requires RealSense deprojection, not OpenCV plumb_bob.'
        )
        preview = None
        if not args.no_preview:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, WIDTH, HEIGHT + 40)
            preview = np.zeros((HEIGHT + 40, WIDTH, 3), dtype=np.uint8)
        count = 0
        last_number = None
        last_stamp = 0
        begin = time.monotonic()
        logged = begin
        ready = False
        while rclpy.ok():
            frames = pipeline.wait_for_frames(1000)
            color = frames.get_color_frame()
            if not color or color.get_frame_number() == last_number:
                continue
            domain = color.get_frame_timestamp_domain()
            if domain not in (rs.timestamp_domain.global_time, rs.timestamp_domain.system_time):
                if time.monotonic() - begin > 10:
                    raise RuntimeError('Camera timestamps did not synchronize to the host clock.')
                continue
            stamp_ns = round(color.get_timestamp() * 1_000_000)
            if stamp_ns <= last_stamp:
                raise RuntimeError('color acquisition timestamp moved backwards.')
            info.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
            pixels = np.asanyarray(color.get_data())
            image_publisher.publish(image_message(pixels, 'rgb8', info.header))
            info_publisher.publish(info)
            last_number = color.get_frame_number()
            last_stamp = stamp_ns
            count += 1
            now = time.monotonic()
            if preview is not None:
                preview[:40] = 0
                cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR, dst=preview[40:])
                fps = count / (now - begin)
                cv2.putText(preview, f'RGB | {fps:.1f} FPS | Q / Esc: quit', (10, 27), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
                cv2.imshow(WINDOW, preview)
                key = cv2.waitKey(1) & 0xff
                if key in (27, ord('q'), ord('Q')) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
            if (not ready and count >= 30) or now - logged >= 5:
                visible = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) if preview is not None else 'disabled'
                node.get_logger().info(f'LIVE color={count} window={visible}')
                ready = count >= 30
                logged = now
    finally:
        try:
            if started:
                pipeline.stop()
        finally:
            if not args.no_preview:
                cv2.destroyAllWindows()
            node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-preview', action='store_true', help='Publish without an OpenCV desktop window')
    parser.add_argument('--serial', help='RealSense serial number; required if multiple devices are connected')
    args = parser.parse_args()
    rclpy.init()
    try:
        run(args)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
