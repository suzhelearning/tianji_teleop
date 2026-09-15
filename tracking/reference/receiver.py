#!/usr/bin/env python3
"""
PICO Streaming Receiver
接收来自 PicoStreamingServer 的数据流。

使用方法：
  1. adb forward tcp:9999 tcp:9999
  2. python receiver.py

帧协议：[0xAB][type:1B][ts_ms:8B][payload_len:4B][payload]
"""

import socket
import struct
import threading
import time
import numpy as np
import cv2

HOST = 'localhost'
PORT = 9999

# 帧类型
TYPE_CAM_LEFT   = 0x01
TYPE_CAM_RIGHT  = 0x02
TYPE_POSE_LEFT  = 0x03
TYPE_POSE_RIGHT = 0x04
TYPE_POSE_HEAD  = 0x05
TYPE_BLE1       = 0x10
TYPE_BLE2       = 0x11

HEADER_SIZE = 1 + 1 + 8 + 4  # magic + type + ts + len

latest_left  = None
latest_right = None
lock = threading.Lock()

# FPS 统计
_fps_counts    = {'left': 0, 'right': 0}
_fps_values    = {'left': 0.0, 'right': 0.0}
_fps_last_time = time.monotonic()
_fps_lock      = threading.Lock()


def recv_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Connection closed")
        buf += chunk
    return buf


def _tick_fps(key):
    global _fps_last_time
    with _fps_lock:
        _fps_counts[key] += 1
        now = time.monotonic()
        dt  = now - _fps_last_time
        if dt >= 1.0:
            _fps_values['left']  = _fps_counts['left']  / dt
            _fps_values['right'] = _fps_counts['right'] / dt
            _fps_counts['left']  = 0
            _fps_counts['right'] = 0
            _fps_last_time = now


def parse_pose(payload):
    """解析 6DOF 位姿：7×float = pos.xyz + rot.xyzw"""
    if len(payload) < 28:
        return None
    floats = struct.unpack('<7f', payload[:28])
    pos = floats[:3]
    rot = floats[3:]
    return {'pos': pos, 'rot': rot}


def receive_loop(sock):
    global latest_left, latest_right
    while True:
        try:
            header = recv_exact(sock, HEADER_SIZE)
        except Exception as e:
            print(f"[Recv] Disconnected: {e}")
            break

        magic, frame_type = struct.unpack('BB', header[:2])
        if magic != 0xAB:
            print(f"[Recv] Bad magic: 0x{magic:02X}, resyncing...")
            continue

        ts_ms = struct.unpack('<q', header[2:10])[0]
        payload_len = struct.unpack('<I', header[10:14])[0]

        if payload_len > 10 * 1024 * 1024:  # 超过 10MB 认为异常
            print(f"[Recv] Payload too large: {payload_len}, skipping")
            continue

        try:
            payload = recv_exact(sock, payload_len)
        except Exception as e:
            print(f"[Recv] Failed to read payload: {e}")
            break

        if frame_type == TYPE_CAM_LEFT:
            img = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                with lock:
                    latest_left = (ts_ms, img)
                _tick_fps('left')

        elif frame_type == TYPE_CAM_RIGHT:
            img = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                with lock:
                    latest_right = (ts_ms, img)
                _tick_fps('right')

        elif frame_type == TYPE_POSE_LEFT:
            pose = parse_pose(payload)
            if pose:
                p, r = pose['pos'], pose['rot']
                print(f"[{ts_ms:8d}ms] LeftHand  pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})  quat=({r[0]:+.3f},{r[1]:+.3f},{r[2]:+.3f},{r[3]:+.3f})")

        elif frame_type == TYPE_POSE_RIGHT:
            pose = parse_pose(payload)
            if pose:
                p, r = pose['pos'], pose['rot']
                print(f"[{ts_ms:8d}ms] RightHand pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})  quat=({r[0]:+.3f},{r[1]:+.3f},{r[2]:+.3f},{r[3]:+.3f})")

        elif frame_type == TYPE_POSE_HEAD:
            pose = parse_pose(payload)
            if pose:
                p, r = pose['pos'], pose['rot']
                print(f"[{ts_ms:8d}ms] Head      pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})  quat=({r[0]:+.3f},{r[1]:+.3f},{r[2]:+.3f},{r[3]:+.3f})")

        elif frame_type in (TYPE_BLE1, TYPE_BLE2):
            dev = "BLE1" if frame_type == TYPE_BLE1 else "BLE2"
            # payload: [4B ts_esp32][data]
            if len(payload) >= 4:
                esp_ts = struct.unpack('<I', payload[:4])[0]
                data   = payload[4:]
                print(f"[{ts_ms:8d}ms] {dev} esp_ts={esp_ts}ms data={data.hex()}")


def display_loop():
    while True:
        with lock:
            left  = latest_left
            right = latest_right
        with _fps_lock:
            fps_l = _fps_values['left']
            fps_r = _fps_values['right']

        if left is not None and right is not None:
            ts_l, img_l = left
            ts_r, img_r = right
            combined = np.hstack([img_l, img_r])
            h = img_l.shape[0]
            w = img_l.shape[1]
            cv2.putText(combined, f"L {ts_l}ms", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(combined, f"R {ts_r}ms", (w + 10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(combined, f"L {fps_l:.1f} fps", (10, h - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(combined, f"R {fps_r:.1f} fps", (w + 10, h - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("PICO Stereo", combined)
        elif left is not None:
            ts_l, img_l = left
            h = img_l.shape[0]
            cv2.putText(img_l, f"L {fps_l:.1f} fps", (10, h - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("PICO Stereo", img_l)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.01)

    cv2.destroyAllWindows()


def main():
    print(f"[Main] Connecting to {HOST}:{PORT} ...")
    print("[Main] Make sure you ran: adb forward tcp:9999 tcp:9999")

    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((HOST, PORT))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[Main] Connected!")

            recv_thread = threading.Thread(target=receive_loop, args=(sock,), daemon=True)
            recv_thread.start()

            display_loop()
            recv_thread.join(timeout=1)
            sock.close()
        except Exception as e:
            print(f"[Main] Error: {e}, retrying in 2s...")
            time.sleep(2)


if __name__ == '__main__':
    main()
