#!/usr/bin/env python3
"""One-terminal new-person TCP calibration, followed by PICO skeleton display."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from tianji_runtime.resources import workspace

ROOT = workspace()
from calibrate_pico_simple import calibrate
from teleop_profile import _name
from pico_simple_artifact import height_lengths
from cleanup_tianji_pico_processes import find_target_processes


def preflight(root):
    if not os.environ.get("DISPLAY"):
        raise RuntimeError("需要图形桌面 DISPLAY，标定后将显示 MuJoCo 骨架。")
    if not (root / "install" / os.environ.get("TIANJI_ENVIRONMENT", "default") / "local_setup.bash").is_file():
        raise RuntimeError("请先运行 pixi run build")
    session = subprocess.run(["tmux", "has-session", "-t", "pico_tianji_teleop"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
    if session.returncode == 0 or find_target_processes():
        raise RuntimeError("已有 PICO 输入会话或进程；请确认归属并停止后再标定，不会自动抢占。")
    device = subprocess.run(["adb", "get-state"], capture_output=True, text=True, timeout=5)
    if device.returncode or device.stdout.strip() != "device":
        raise RuntimeError("PICO USB 未授权或设备不唯一；请检查 adb devices -l。")


def wait_raw_input(driver, timeout):
    import rclpy
    from geometry_msgs.msg import PoseArray, PoseStamped
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = None
    try:
        node = rclpy.create_node(f"pico_setup_readiness_{os.getpid()}")
        progress = {}
        def observe(topic, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            old = progress.get(topic)
            progress[topic] = (stamp, bool(old and stamp > old[0] and stamp > 0), time.monotonic())
        subscriptions = []
        for topic, kind in (("/pico/pose/left_hand", PoseStamped),
                            ("/pico/pose/right_hand", PoseStamped),
                            ("/pico/smpl_raw", PoseArray)):
            subscriptions.append(node.create_subscription(kind, topic,
                lambda msg, topic=topic: observe(topic, msg), qos_profile_sensor_data))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if driver.poll() is not None:
                raise RuntimeError("原始 PICO driver 提前退出，请查看本次 driver.log。")
            rclpy.spin_once(node, timeout_sec=.1)
            if len(progress) == 3 and all(advancing and time.monotonic() - received < .5
                                         for _, advancing, received in progress.values()):
                return
        raise RuntimeError("等待左右手柄/原始骨架新数据超时；未开始标定。")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def stop_owned_driver(driver):
    """Only signal the isolated process group created by this invocation."""
    for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(driver.pid, sig)
        except ProcessLookupError:
            driver.wait(timeout=2)
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            driver.poll()
            try:
                os.killpg(driver.pid, 0)
            except ProcessLookupError:
                driver.wait(timeout=2)
                return
            time.sleep(.05)
    raise RuntimeError("本次 driver 进程组未完全退出；拒绝启动下一套输入。")


def confirm_skeleton(read=input):
    """Translate the wizard choice to the existing explicit publication contract."""
    while True:
        answer = read("回车确认使用并显示骨架，输入 q 取消：").strip().lower()
        if not answer:
            return "publish"
        if answer == "q":
            return "cancel"
        print("请输入回车确认，或输入 q 取消；尚未发布。", flush=True)


def setup(user, height, timeout=30, root=ROOT, *, check=preflight,
          collect=calibrate, ready=wait_raw_input, stop=stop_owned_driver,
          popen=subprocess.Popen, run=subprocess.run, confirm=input):
    user = _name(user, "user")
    height_lengths(height)
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    check(root)
    log_root = root / "logs/pico-setup"
    log_root.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="setup-", dir=log_root))
    print(f"原始输入日志：{directory / 'driver.log'}", flush=True)
    published = []
    with (directory / "driver.log").open("x") as log:
        driver = popen(["bash", str(root / "bash/start_pico_driver.sh")],
                       cwd=root, stdin=subprocess.DEVNULL, stdout=log,
                       stderr=subprocess.STDOUT, start_new_session=True)
        try:
            print("等待左右手柄和原始骨架数据…", flush=True)
            ready(driver, timeout)
            result = collect(user, height, root=root,
                             confirm=lambda _prompt: confirm_skeleton(confirm),
                             on_publish=published.append)
        finally:
            stop(driver)
    if result:
        return result
    if not published:
        print("已取消发布；不会加载旧人员配置或打开骨架。", flush=True)
        return 0
    # Use the exact revision published by this operation, not a mutable active pointer.
    print("标定已发布，原始 driver 已退出；启动校正骨架窗口（不启动机械臂执行器）。", flush=True)
    result = run(["bash", str(root / "bash/start_tianji_pico_teleop.sh"),
                  "--detach", "--calibration-dir", str(published[0]),
                  "--pico-world-x-offset", "0.20"], cwd=root)
    if result.returncode:
        print("标定仍已保存；骨架输入启动未通过，请查看保留的 tmux 窗口。", file=sys.stderr)
        return result.returncode
    print("请检查骨架、掌心位置和转腕方向。确认后另开终端运行 pixi run sim。\n"
          "输入会话继续运行；停止：pixi run stop-pico", flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--user")
    parser.add_argument("--height-m", type=float)
    parser.add_argument("--accept-symmetric-model", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=30)
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error("需要交互终端进行 TCP 采集及发布确认")
    def interrupted(_sig, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        user = _name(args.user or input("新人员名：").strip(), "user")
        height = args.height_m if args.height_m is not None else float(input("身高（米，例如 1.70）："))
        height_lengths(height)
        print("只实测左 TCP；右 TCP 按镜像握持推导，双侧骨长/腕掌距离按身高估计。")
        if not args.accept_symmetric_model and input("接受该假设？输入 yes 继续：").strip() != "yes":
            print("已取消，未启动设备。")
            return 0
        return setup(user, height, args.timeout_s)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        parser.exit(2, str(error) + "\n")
    except (KeyboardInterrupt, EOFError):
        print("\n向导已中断；不会自动接入遥操。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
