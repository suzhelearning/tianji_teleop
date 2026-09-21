#!/usr/bin/env python3
"""
PICO 实时可视化 —— Rerun 版

用法：
  1. adb forward tcp:9999 tcp:9999
  2. pip install rerun-sdk numpy opencv-python scipy
  3. python visualizer_rerun.py

Rerun Viewer 内操作：
  鼠标左键拖动     旋转视角
  鼠标右键/滚轮    平移 / 缩放
  空格            时间轴播放/暂停
  左侧面板        切换实体显隐
"""

import socket
import struct
import time
import collections
import numpy as np
import cv2
import rerun as rr
import rerun.blueprint as rrb
from scipy.spatial.transform import Rotation

# ─────────────────────────────────────────────────────────────────────────────
#  配置
# ─────────────────────────────────────────────────────────────────────────────
HOST       = 'localhost'
PORT       = 9999
TRAIL_LEN  = 200
AXIS_LEN   = 0.07
GRID_HALF  = 1.0
GRID_STEP  = 0.25

DEVICE_CFG = {
    'left':  ('Left Hand',  [ 80, 160, 255]),
    'right': ('Right Hand', [255,  80,  80]),
    'head':  ('Head',       [ 80, 220, 140]),
}

TYPE_CAM_LEFT    = 0x01
TYPE_CAM_RIGHT   = 0x02
TYPE_POSE_LEFT   = 0x03
TYPE_POSE_RIGHT  = 0x04
TYPE_POSE_HEAD   = 0x05
TYPE_WORLD_RESET = 0x06
HEADER_SIZE      = 14

# 轨迹缓存（单线程内使用）
trails = {k: collections.deque(maxlen=TRAIL_LEN) for k in DEVICE_CFG}

# ─────────────────────────────────────────────────────────────────────────────
#  姿态标定（三轴全修正）：按 A 后首个 pose 帧记录 Q_first，缓存 Q_first⁻¹；
#  后续帧 Q_new = Q_received · Q_first⁻¹（右乘本地系），将整个安装偏置抹掉。
#  标定瞬间手柄三轴与世界三轴重合。
# ─────────────────────────────────────────────────────────────────────────────
_rot_calib = {'left': None, 'right': None}   # 存 Q_first⁻¹


def reset_rot_calib():
    for k in _rot_calib:
        _rot_calib[k] = None


def apply_rot_calib(key: str, quat: np.ndarray) -> np.ndarray:
    rot = Rotation.from_quat(quat)
    if _rot_calib[key] is None:
        _rot_calib[key] = rot.inv()
        e = rot.as_euler('xyz', degrees=True)
        print(f"[Calib] {key} full-rot zeroed "
              f"(first euler xyz = {e[0]:+.1f},{e[1]:+.1f},{e[2]:+.1f}°)")
    return (rot * _rot_calib[key]).as_quat().astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────
#  掌心偏移标定：手柄竖直安装于手背，掌心在手柄本地系下有固定偏移向量。
#  按 A 时：若双手合拢（距离 < PALM_CALIB_MAX_DIST），取两手柄中点为共同掌心，
#  反解出每个手柄→掌心在本地系下的向量，后续帧 p_palm = p + R·offset。
# ─────────────────────────────────────────────────────────────────────────────
PALM_CALIB_MAX_DIST = 0.40  # 合拢阈值（米），超过视为"没合拢"，跳过标定

_palm_offset_local = {'left': None, 'right': None}  # 本地系下 手柄→掌心
_palm_pending = False                                # A 后尚未完成标定
_palm_snap    = {'left': None, 'right': None}       # A 后首帧 (pos, quat)


def reset_palm_calib():
    """按 A 时调用；不清旧的 offset，标定失败时保留上一次结果。"""
    global _palm_pending
    _palm_pending = True
    _palm_snap['left']  = None
    _palm_snap['right'] = None


def _finalize_palm_calib():
    global _palm_pending
    L, R = _palm_snap['left'], _palm_snap['right']
    if L is None or R is None:
        return
    pL, qL = L
    pR, qR = R
    dist = float(np.linalg.norm(pL - pR))
    if dist > PALM_CALIB_MAX_DIST:
        print(f"[PalmCalib] skipped: hands {dist*100:.1f}cm apart "
              f"(> {PALM_CALIB_MAX_DIST*100:.0f}cm)")
        _palm_pending = False
        return
    mid = (pL + pR) / 2.0
    _palm_offset_local['left']  = Rotation.from_quat(qL).inv().apply(mid - pL)
    _palm_offset_local['right'] = Rotation.from_quat(qR).inv().apply(mid - pR)
    oL, oR = _palm_offset_local['left'], _palm_offset_local['right']
    print(f"[PalmCalib] dist={dist*100:.1f}cm  "
          f"L_offset=({oL[0]:+.3f},{oL[1]:+.3f},{oL[2]:+.3f})m  "
          f"R_offset=({oR[0]:+.3f},{oR[1]:+.3f},{oR[2]:+.3f})m")
    _palm_pending = False


def capture_palm_sample(key: str, pos: np.ndarray, quat: np.ndarray):
    if not _palm_pending or _palm_snap[key] is not None:
        return
    _palm_snap[key] = (pos.copy(), quat.copy())
    _finalize_palm_calib()


def apply_palm_offset(key: str, pos: np.ndarray, quat: np.ndarray) -> np.ndarray:
    off = _palm_offset_local[key]
    if off is None:
        return pos
    return pos + Rotation.from_quat(quat).apply(off).astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────
#  网络
# ─────────────────────────────────────────────────────────────────────────────
def recv_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Connection closed")
        buf += chunk
    return buf


# ─────────────────────────────────────────────────────────────────────────────
#  静态场景
# ─────────────────────────────────────────────────────────────────────────────
def log_grid():
    xs = np.arange(-GRID_HALF, GRID_HALF + GRID_STEP, GRID_STEP)
    ys = np.arange(-GRID_HALF, GRID_HALF + GRID_STEP, GRID_STEP)
    strips = []
    for x in xs:
        strips.append([[float(x), float(ys[0]),  0.0],
                       [float(x), float(ys[-1]), 0.0]])
    for y in ys:
        strips.append([[float(xs[0]),  float(y), 0.0],
                       [float(xs[-1]), float(y), 0.0]])
    rr.log(
        "world/grid",
        rr.LineStrips3D(strips, colors=[60, 60, 60], radii=0.0015),
        static=True,
    )


_AXIS_COLORS = [[255, 80, 80], [80, 220, 80], [80, 140, 255]]  # X 红 / Y 绿 / Z 蓝


def _log_triad(entity: str, length: float, radius: float = 0.004):
    """在 entity 下作为子实体画 XYZ 三轴箭头（受父 Transform3D 影响）。"""
    origins = np.zeros((3, 3), dtype=np.float32)
    vectors = np.eye(3, dtype=np.float32) * length
    rr.log(
        f"{entity}/axes",
        rr.Arrows3D(
            origins=origins,
            vectors=vectors,
            colors=_AXIS_COLORS,
            radii=radius,
        ),
    )


def log_world_origin(yaw_deg: float = 0.0):
    q = Rotation.from_euler('z', yaw_deg, degrees=True).as_quat()  # xyzw
    rr.log(
        "world/origin",
        rr.Transform3D(
            translation=[0.0, 0.0, 0.0],
            quaternion=rr.Quaternion(xyzw=q),
        ),
    )
    _log_triad("world/origin", length=0.18, radius=0.006)


def log_static_scene():
    # X 前 / Y 左 / Z 上（右手坐标系）
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    log_grid()
    log_world_origin(0.0)
    rr.log(
        "world/origin/marker",
        rr.Points3D(
            [[0.0, 0.0, 0.0]],
            colors=[[255, 255, 255]],
            radii=0.012,
            labels=["World Origin"],
        ),
        static=True,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  数据记录
# ─────────────────────────────────────────────────────────────────────────────
def clear_trails():
    for k in DEVICE_CFG:
        trails[k].clear()
        rr.log(f"world/trails/{k}", rr.Clear(recursive=False))


def log_pose(key: str, pos: np.ndarray, quat: np.ndarray):
    label, color = DEVICE_CFG[key]

    # 位姿
    rr.log(
        f"world/poses/{key}",
        rr.Transform3D(
            translation=pos,
            quaternion=rr.Quaternion(xyzw=quat),
        ),
    )
    _log_triad(f"world/poses/{key}", length=AXIS_LEN)
    # 本地原点上的圆点 + 标签（作为 Transform3D 子实体，跟随位姿）
    rr.log(
        f"world/poses/{key}/marker",
        rr.Points3D(
            [[0.0, 0.0, 0.0]],
            colors=[color],
            radii=0.015,
            labels=[label],
        ),
    )

    # 轨迹（独立于 Transform3D，处于世界坐标系）
    trails[key].append(pos.astype(np.float32).copy())
    if len(trails[key]) >= 2:
        pts = np.asarray(trails[key], dtype=np.float32)
        rr.log(
            f"world/trails/{key}",
            rr.LineStrips3D([pts], colors=[color], radii=0.003),
        )

    # 位置标量曲线
    rr.log(f"pose/{key}/x", rr.Scalars(float(pos[0])))
    rr.log(f"pose/{key}/y", rr.Scalars(float(pos[1])))
    rr.log(f"pose/{key}/z", rr.Scalars(float(pos[2])))


def recv_loop():
    pose_key_map = {
        TYPE_POSE_LEFT:  'left',
        TYPE_POSE_RIGHT: 'right',
        TYPE_POSE_HEAD:  'head',
    }

    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((HOST, PORT))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[Recv] Connected to {HOST}:{PORT}")
            rr.log("status",
                   rr.TextLog(f"Connected {HOST}:{PORT}",
                              level=rr.TextLogLevel.INFO))

            while True:
                hdr = recv_exact(sock, HEADER_SIZE)
                magic, ftype = struct.unpack('BB', hdr[:2])
                if magic != 0xAB:
                    continue
                plen = struct.unpack('<I', hdr[10:14])[0]
                if plen > 8 * 1024 * 1024:
                    continue
                payload = recv_exact(sock, plen)

                rr.set_time("time", duration=time.monotonic())

                if ftype == TYPE_WORLD_RESET:
                    yaw = (struct.unpack('<f', payload[:4])[0]
                           if len(payload) >= 4 else 0.0)
                    log_world_origin(yaw)
                    clear_trails()
                    reset_rot_calib()
                    reset_palm_calib()
                    rr.log("status",
                           rr.TextLog(f"World reset (yaw={yaw:.1f}°)",
                                      level=rr.TextLogLevel.INFO))

                elif ftype in (TYPE_CAM_LEFT, TYPE_CAM_RIGHT):
                    arr = np.frombuffer(payload, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        side = 'left' if ftype == TYPE_CAM_LEFT else 'right'
                        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        rr.log(f"camera/{side}", rr.Image(rgb))

                else:
                    key = pose_key_map.get(ftype)
                    if key and len(payload) >= 28:
                        f = struct.unpack('<7f', payload[:28])
                        pos  = np.array(f[:3], dtype=np.float32)
                        quat = np.array(f[3:], dtype=np.float32)  # xyzw
                        if key in _rot_calib:
                            quat = apply_rot_calib(key, quat)
                            capture_palm_sample(key, pos, quat)
                            pos  = apply_palm_offset(key, pos, quat)
                        log_pose(key, pos, quat)

        except Exception as e:
            print(f"[Recv] {e}, reconnecting in 2s...")
            try:
                rr.log("status",
                       rr.TextLog(f"Disconnected: {e}",
                                  level=rr.TextLogLevel.WARN))
            except Exception:
                pass
            time.sleep(2)


# ─────────────────────────────────────────────────────────────────────────────
#  Blueprint：3D 视图 + 双目相机 + 位置曲线
# ─────────────────────────────────────────────────────────────────────────────
def build_blueprint():
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/world", name="3D Pose"),
            rrb.Vertical(
                rrb.Spatial2DView(origin="/camera/left",  name="Left Camera"),
                rrb.Spatial2DView(origin="/camera/right", name="Right Camera"),
                rrb.TimeSeriesView(origin="/pose",        name="Position (m)"),
                row_shares=[2, 2, 1],
            ),
            column_shares=[3, 2],
        ),
        collapse_panels=True,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  主函数
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rr.init("PICO 6DOF Viewer", spawn=True,
            default_blueprint=build_blueprint())

    log_static_scene()

    print('[Main] Rerun visualizer started.')
    print('[Main] Press A on PICO to set world origin.')

    # Rerun Viewer 是独立进程，本进程直接跑接收循环即可
    recv_loop()


if __name__ == '__main__':
    main()
