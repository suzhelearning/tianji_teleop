#!/usr/bin/env python3
"""
PICO 实时可视化 —— 两个独立窗口
  窗口1 (OpenCV)：双目相机画面
  窗口2 (matplotlib)：3D 位姿 + 轨迹（RViz 风格）

用法：
  1. adb forward tcp:9999 tcp:9999
  2. pip install matplotlib scipy numpy opencv-python
  3. python visualizer.py

快捷键：
  相机窗口  Q / ESC  关闭相机窗口
  3D 窗口   C  清除轨迹   R  重置视图   Q  退出全部
  3D 窗口   左键拖动  旋转视角（显示旋转轴圆环）
"""

import socket
import struct
import threading
import time
import collections
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from scipy.spatial.transform import Rotation

# ─────────────────────────────────────────────────────────────────────────────
#  配置
# ─────────────────────────────────────────────────────────────────────────────
HOST       = 'localhost'
PORT       = 9999
TRAIL_LEN  = 200
AXIS_LEN   = 0.07
GRID_STEP  = 0.25
FPS        = 30

BG_COLOR   = '#111111'
GRID_COLOR = '#2a2a2a'

DEVICE_CFG = {
    'left':  ('Left Hand',  '#3399FF', '#1155AA'),
    'right': ('Right Hand', '#FF4444', '#AA1111'),
    'head':  ('Head',       '#44DD88', '#117744'),
}

TYPE_CAM_LEFT    = 0x01
TYPE_CAM_RIGHT   = 0x02
TYPE_POSE_LEFT   = 0x03
TYPE_POSE_RIGHT  = 0x04
TYPE_POSE_HEAD   = 0x05
TYPE_WORLD_RESET = 0x06
HEADER_SIZE      = 14

# ─────────────────────────────────────────────────────────────────────────────
#  共享状态
# ─────────────────────────────────────────────────────────────────────────────
class DeviceState:
    def __init__(self):
        self.pos   = np.zeros(3)
        self.quat  = np.array([0., 0., 0., 1.])
        self.trail = collections.deque(maxlen=TRAIL_LEN)
        self.active = False

shared     = {k: DeviceState() for k in DEVICE_CFG}
state_lock = threading.Lock()
clear_flag = threading.Event()
connected  = threading.Event()
reset_events = collections.deque()
reset_lock   = threading.Lock()

cam_lock  = threading.Lock()
cam_left  = [None]   # BGR ndarray or None
cam_right = [None]
# 相机帧率统计
cam_fps_lock    = threading.Lock()
_fps_counts     = {'left': 0, 'right': 0}
_fps_values     = {'left': 0.0, 'right': 0.0}
_fps_last_time  = [time.monotonic()]

# 位姿帧率统计（独立于相机）
pose_fps_lock     = threading.Lock()
_pose_fps_counts  = {'left': 0, 'right': 0, 'head': 0}
_pose_fps_values  = {'left': 0.0, 'right': 0.0, 'head': 0.0}
_pose_fps_last_time = [time.monotonic()]

# ─────────────────────────────────────────────────────────────────────────────
#  网络接收线程
# ─────────────────────────────────────────────────────────────────────────────
def recv_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Connection closed")
        buf += chunk
    return buf


def recv_loop():
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((HOST, PORT))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            connected.set()
            print(f"[Recv] Connected to {HOST}:{PORT}")
            while True:
                hdr = recv_exact(sock, HEADER_SIZE)
                magic, ftype = struct.unpack('BB', hdr[:2])
                if magic != 0xAB:
                    continue
                plen = struct.unpack('<I', hdr[10:14])[0]
                if plen > 8 * 1024 * 1024:
                    continue
                payload = recv_exact(sock, plen)

                if ftype == TYPE_WORLD_RESET:
                    yaw = struct.unpack('<f', payload[:4])[0] if len(payload) >= 4 else 0.
                    with reset_lock:
                        reset_events.append(yaw)

                elif ftype in (TYPE_CAM_LEFT, TYPE_CAM_RIGHT):
                    arr = np.frombuffer(payload, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        key_cam = 'left' if ftype == TYPE_CAM_LEFT else 'right'
                        with cam_lock:
                            if ftype == TYPE_CAM_LEFT:
                                cam_left[0] = img
                            else:
                                cam_right[0] = img
                        # fps 计数
                        with cam_fps_lock:
                            _fps_counts[key_cam] += 1
                            now = time.monotonic()
                            dt = now - _fps_last_time[0]
                            if dt >= 1.0:
                                _fps_values['left']  = _fps_counts['left']  / dt
                                _fps_values['right'] = _fps_counts['right'] / dt
                                _fps_counts['left']  = 0
                                _fps_counts['right'] = 0
                                _fps_last_time[0] = now

                else:
                    key_map = {TYPE_POSE_LEFT: 'left',
                               TYPE_POSE_RIGHT: 'right',
                               TYPE_POSE_HEAD:  'head'}
                    key = key_map.get(ftype)
                    if key and len(payload) >= 28:
                        f = struct.unpack('<7f', payload[:28])
                        pos  = np.array(f[:3])
                        quat = np.array(f[3:])
                        with state_lock:
                            if clear_flag.is_set():
                                for d in shared.values():
                                    d.trail.clear()
                                clear_flag.clear()
                            ds = shared[key]
                            ds.pos  = pos
                            ds.quat = quat
                            ds.trail.append(pos.copy())
                            ds.active = True
                        # 位姿 fps 计数
                        with pose_fps_lock:
                            _pose_fps_counts[key] += 1
                            now = time.monotonic()
                            dt  = now - _pose_fps_last_time[0]
                            if dt >= 1.0:
                                for k in _pose_fps_counts:
                                    _pose_fps_values[k] = _pose_fps_counts[k] / dt
                                    _pose_fps_counts[k] = 0
                                _pose_fps_last_time[0] = now

        except Exception as e:
            connected.clear()
            print(f"[Recv] {e}, reconnecting in 2s...")
            time.sleep(2)


# ─────────────────────────────────────────────────────────────────────────────
#  OpenCV 相机窗口线程
# ─────────────────────────────────────────────────────────────────────────────
_cam_win_running = threading.Event()
_cam_win_running.set()

def camera_thread():
    WIN = 'PICO Stereo Camera  [Q/ESC to close]'
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 480)

    placeholder = np.full((480, 1280, 3), 30, dtype=np.uint8)
    cv2.putText(placeholder, 'Waiting for camera stream...',
                (380, 240), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, (80, 80, 80), 2)

    while _cam_win_running.is_set():
        with cam_lock:
            l = cam_left[0]
            r = cam_right[0]
        with cam_fps_lock:
            fps_l = _fps_values['left']
            fps_r = _fps_values['right']

        if l is not None and r is not None:
            if l.shape != r.shape:
                r = cv2.resize(r, (l.shape[1], l.shape[0]))
            frame = np.hstack([l, r])
            h, w = l.shape[:2]
            # 左右标注（左上角）
            cv2.putText(frame, 'L', (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 1)
            cv2.putText(frame, 'R', (w + 8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 1)
            # FPS（左下角）
            cv2.putText(frame, f'L {fps_l:.1f} fps', (8, h - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 1)
            cv2.putText(frame, f'R {fps_r:.1f} fps', (w + 8, h - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 1)
            cv2.imshow(WIN, frame)
        elif l is not None:
            cv2.putText(l, f'{fps_l:.1f} fps', (8, l.shape[0] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 1)
            cv2.imshow(WIN, l)
        elif r is not None:
            cv2.putText(r, f'{fps_r:.1f} fps', (8, r.shape[0] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 1)
            cv2.imshow(WIN, r)
        else:
            cv2.imshow(WIN, placeholder)

        key = cv2.waitKey(16) & 0xFF   # ~60 fps 显示刷新
        if key in (ord('q'), ord('Q'), 27):
            _cam_win_running.clear()
            break

    cv2.destroyAllWindows()


# ─────────────────────────────────────────────────────────────────────────────
#  3D 位姿辅助类
# ─────────────────────────────────────────────────────────────────────────────
class AxisTriad:
    COLORS = ('#FF4444', '#44DD44', '#4488FF')

    def __init__(self, ax, length=AXIS_LEN, lw=2.2, alpha=0.92):
        self.length = length
        self.lines  = []
        for c in self.COLORS:
            ln, = ax.plot([], [], [], '-', color=c, linewidth=lw,
                          alpha=alpha, solid_capstyle='round')
            self.lines.append(ln)

    def update(self, origin, quat):
        o = np.asarray(origin, dtype=float)
        r = Rotation.from_quat(quat)
        for i, ln in enumerate(self.lines):
            tip = o + r.apply(np.eye(3)[i] * self.length)
            ln.set_data([o[0], tip[0]], [o[1], tip[1]])
            ln.set_3d_properties([o[2], tip[2]])


def make_grid(ax, cx, cy, half=1.0, step=GRID_STEP):
    xs = np.arange(cx - half, cx + half + step, step)
    ys = np.arange(cy - half, cy + half + step, step)
    segs = []
    for x in xs:
        segs.append([(x, ys[0], 0.), (x, ys[-1], 0.)])
    for y in ys:
        segs.append([(xs[0], y, 0.), (xs[-1], y, 0.)])
    lc = Line3DCollection(segs, colors=GRID_COLOR, linewidths=0.6, alpha=0.9)
    ax.add_collection3d(lc)
    return lc


_RING_N = 64

def _ring_pts(axis, cx, cy, cz, r):
    t = np.linspace(0, 2 * np.pi, _RING_N)
    c, s = np.cos(t), np.sin(t)
    if axis == 0:
        return np.column_stack([np.full(_RING_N, cx), cy + c*r, cz + s*r])
    elif axis == 1:
        return np.column_stack([cx + c*r, np.full(_RING_N, cy), cz + s*r])
    else:
        return np.column_stack([cx + c*r, cy + s*r, np.full(_RING_N, cz)])


class RotationRings:
    COLORS = ['#FF6666', '#66DD66', '#6699FF']

    def __init__(self, ax):
        self.ax    = ax
        self.lines = []
        for c in self.COLORS:
            ln, = ax.plot([], [], [], '-', color=c, linewidth=1.8,
                          alpha=0., zorder=20)
            self.lines.append(ln)
        self._dragging = False

    def _cr(self):
        xl, xh = self.ax.get_xlim()
        yl, yh = self.ax.get_ylim()
        zl, zh = self.ax.get_zlim()
        r = min(xh-xl, yh-yl, zh-zl) * 0.42
        return (xl+xh)/2, (yl+yh)/2, (zl+zh)/2, r

    def show(self, dx, dy):
        cx, cy, cz, r = self._cr()
        active = set()
        if abs(dx) > 2: active.add(2)
        if abs(dy) > 2: active.add(1)
        for i, ln in enumerate(self.lines):
            pts = _ring_pts(i, cx, cy, cz, r)
            ln.set_data(pts[:, 0], pts[:, 1])
            ln.set_3d_properties(pts[:, 2])
            ln.set_alpha(0.85 if i in active else 0.20)
            ln.set_linewidth(2.4 if i in active else 1.0)
        self._dragging = True

    def hide(self):
        for ln in self.lines:
            ln.set_alpha(0.)
        self._dragging = False

    def refresh(self):
        if not self._dragging:
            return
        cx, cy, cz, r = self._cr()
        for i, ln in enumerate(self.lines):
            if ln.get_alpha() > 0.05:
                pts = _ring_pts(i, cx, cy, cz, r)
                ln.set_data(pts[:, 0], pts[:, 1])
                ln.set_3d_properties(pts[:, 2])


# ─────────────────────────────────────────────────────────────────────────────
#  主函数
# ─────────────────────────────────────────────────────────────────────────────
def main():
    # 启动接收线程和相机窗口线程
    threading.Thread(target=recv_loop,     daemon=True).start()
    threading.Thread(target=camera_thread, daemon=True).start()

    # ── matplotlib 3D 窗口 ────────────────────────────────────────────────
    fig = plt.figure(figsize=(13, 9), facecolor=BG_COLOR)
    try:
        fig.canvas.manager.set_window_title('PICO 6DOF Pose Viewer')
    except Exception:
        pass

    gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1], figure=fig,
                           left=0.02, right=0.98, top=0.96, bottom=0.04,
                           wspace=0.03)
    ax      = fig.add_subplot(gs[0], projection='3d')
    ax_info = fig.add_subplot(gs[1])
    ax_info.set_facecolor(BG_COLOR)
    ax_info.axis('off')

    # 3D 轴样式
    ax.set_facecolor(BG_COLOR)
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.set_facecolor('#1a1a1a')
        pane.set_edgecolor('#333333')
    ax.grid(False)
    ax.set_xlabel('X (前)', color='#aaaaaa', fontsize=9, labelpad=6)
    ax.set_ylabel('Y (左)', color='#aaaaaa', fontsize=9, labelpad=6)
    ax.set_zlabel('Z (上)', color='#aaaaaa', fontsize=9, labelpad=6)
    ax.tick_params(colors='#555555', labelsize=7)

    VIEW_R = 0.8
    ax.set_xlim(-VIEW_R, VIEW_R)
    ax.set_ylim(-VIEW_R, VIEW_R)
    ax.set_zlim(-0.1, 1.6)
    ax.set_box_aspect([1, 1, 1.2])

    title_txt = ax.set_title('PICO 6DOF — 等待连接...',
                              color='#cccccc', fontsize=11, pad=10,
                              fontfamily='monospace')

    # 世界原点
    world_triad = AxisTriad(ax, length=0.18, lw=3.0, alpha=0.85)
    world_triad.update([0., 0., 0.], [0., 0., 0., 1.])
    ax.plot([0.], [0.], [0.], 'o', color='white', markersize=7, zorder=10,
            markeredgecolor='#aaaaaa', markeredgewidth=0.8)
    ax.text(0., 0., 0.22, 'World\nOrigin', color='#eeeeee',
            fontsize=7.5, ha='center', fontfamily='monospace')

    def refresh_world_origin(yaw_deg):
        r = Rotation.from_euler('z', yaw_deg, degrees=True)
        world_triad.update([0., 0., 0.], r.as_quat())

    # 地面网格
    grid_artists = [make_grid(ax, 0., 0., half=VIEW_R)]

    # 每个设备
    device_plots = {}
    for key, (label, color, trail_color) in DEVICE_CFG.items():
        trail, = ax.plot([], [], [], '-', color=trail_color,
                         alpha=0.55, linewidth=1.2, solid_capstyle='round')
        dot,   = ax.plot([], [], [], 'o', color=color, markersize=9,
                         zorder=5, markeredgecolor='white', markeredgewidth=0.6)
        triad  = AxisTriad(ax)
        device_plots[key] = {'trail': trail, 'dot': dot, 'triad': triad,
                              'color': color, 'label': label}

    # 图例
    legend_handles = [
        Line2D([0], [0], marker='o', color='w',
               markerfacecolor=cfg[1], markersize=8,
               label=cfg[0], linewidth=0)
        for cfg in DEVICE_CFG.values()
    ] + [
        Line2D([0], [0], color='#FF4444', lw=2, label='X (前)'),
        Line2D([0], [0], color='#44DD44', lw=2, label='Y (左)'),
        Line2D([0], [0], color='#4488FF', lw=2, label='Z (上)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='white',
               markersize=6, label='World Origin', linewidth=0),
    ]
    ax.legend(handles=legend_handles, loc='upper left',
              facecolor='#1c1c1c', edgecolor='#444444',
              labelcolor='#cccccc', fontsize=8, framealpha=0.85)

    # 信息面板
    info_text = ax_info.text(
        0.04, 0.97, '', transform=ax_info.transAxes,
        color='#cccccc', fontsize=8.5, verticalalignment='top',
        fontfamily='monospace',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#1c1c1c',
                  edgecolor='#333333', linewidth=0.8))

    # 旋转圆环
    rings  = RotationRings(ax)
    _mouse = {'down': False, 'x0': 0., 'y0': 0.}

    def on_key(event):
        if event.key in ('q', 'Q', 'escape'):
            _cam_win_running.clear()   # 同时关闭相机窗口
            plt.close('all')
        elif event.key in ('c', 'C'):
            clear_flag.set()
        elif event.key in ('r', 'R'):
            with state_lock:
                positions = [ds.pos.copy() for ds in shared.values() if ds.active]
            ctr = np.mean(positions, axis=0) if positions else np.zeros(3)
            ax.set_xlim(ctr[0] - VIEW_R, ctr[0] + VIEW_R)
            ax.set_ylim(ctr[1] - VIEW_R, ctr[1] + VIEW_R)
            ax.set_zlim(ctr[2] - 0.1,    ctr[2] + VIEW_R * 2)
            rings.refresh()
            fig.canvas.draw_idle()

    def on_press(event):
        if event.inaxes is ax and event.button == 1:
            _mouse.update(down=True, x0=event.x, y0=event.y)

    def on_motion(event):
        if _mouse['down'] and event.inaxes is ax:
            rings.show(event.x - _mouse['x0'], event.y - _mouse['y0'])

    def on_release(event):
        if event.button == 1:
            _mouse['down'] = False
            rings.hide()

    fig.canvas.mpl_connect('key_press_event',    on_key)
    fig.canvas.mpl_connect('button_press_event',  on_press)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    fig.canvas.mpl_connect('button_release_event', on_release)

    # 动画更新
    frame_count = [0]

    def update(_f):
        frame_count[0] += 1
        fc = frame_count[0]

        with reset_lock:
            while reset_events:
                refresh_world_origin(reset_events.popleft())

        with state_lock:
            snap = {k: (ds.pos.copy(), ds.quat.copy(),
                        list(ds.trail), ds.active)
                    for k, ds in shared.items()}
        with pose_fps_lock:
            pose_fps = dict(_pose_fps_values)
        is_conn = connected.is_set()

        title_txt.set_text('PICO 6DOF  ●  Connected' if is_conn
                           else 'PICO 6DOF  ○  Waiting...')
        title_txt.set_color('#44DD88' if is_conn else '#FF8844')

        info_lines = ['  Device        Pos (m)\n',
                      '  ────────────────────────────────\n']
        active_pos = []

        for key, (pos, quat, trail_pts, active) in snap.items():
            dp = device_plots[key]
            if not active:
                info_lines.append(f'  {dp["label"]:<12} —\n')
                continue

            active_pos.append(pos)

            if len(trail_pts) > 1:
                pts = np.array(trail_pts)
                dp['trail'].set_data(pts[:, 0], pts[:, 1])
                dp['trail'].set_3d_properties(pts[:, 2])

            dp['dot'].set_data([pos[0]], [pos[1]])
            dp['dot'].set_3d_properties([pos[2]])
            dp['triad'].update(pos, quat)

            ex, ey, ez = Rotation.from_quat(quat).as_euler('xyz', degrees=True)
            info_lines.append(
                f'  {dp["label"]}  [{pose_fps.get(key, 0):.1f} fps]\n'
                f'    pos  ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})\n'
                f'    euler({ex:+6.1f},{ey:+6.1f},{ez:+6.1f}) deg\n\n'
            )

        info_text.set_text(''.join(info_lines))

        if fc % 60 == 0 and active_pos:
            ctr = np.mean(active_pos, axis=0)
            for lc in grid_artists:
                try: lc.remove()
                except Exception: pass
            grid_artists[0] = make_grid(ax, ctr[0], ctr[1],
                                        half=max(VIEW_R, 0.8))

    print('[Main] Visualizer started.')
    print('[Main] Camera window: Q/ESC to close')
    print('[Main] Pose window:   C=clear  R=recenter  Q=quit')
    print('[Main] Press A on PICO to set world origin.')

    ani = FuncAnimation(fig, update, interval=int(1000 / FPS),  # noqa: F841
                        blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
