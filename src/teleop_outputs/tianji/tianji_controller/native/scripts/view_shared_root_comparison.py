#!/usr/bin/env python3
"""Side-by-side SPARK/Ceres online replay, one clock and loopback fan-out (X11)."""
import argparse
import ctypes as C
import ctypes.util
import json
from pathlib import Path
import re
import socket
import subprocess
import time
import yaml
from run_pico_trace_algorithm_benchmark import _read_trace
from tianji_runtime import controller_profile, native_executable
from tianji_runtime.resources import controller_resource


def arrange(pid, title, x):
    """Touch only a window whose X11 owner is our freshly launched child."""
    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    windows = re.findall(r'(0x[0-9a-f]+) "Tianji dual-arm QP-IK V1"', tree)
    for window in windows:
        owner = subprocess.run(['xprop', '-id', window, '_NET_WM_PID'],
                               capture_output=True, text=True, check=True).stdout
        if not re.search(r'=\s*' + str(pid) + r'\s*$', owner):
            continue
        lib = C.CDLL(ctypes.util.find_library('X11'))
        lib.XOpenDisplay.argtypes = [C.c_char_p]
        lib.XOpenDisplay.restype = C.c_void_p
        lib.XInternAtom.argtypes = [C.c_void_p, C.c_char_p, C.c_int]
        lib.XInternAtom.restype = C.c_ulong
        lib.XMoveResizeWindow.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_int, C.c_uint, C.c_uint]
        lib.XStoreName.argtypes = [C.c_void_p, C.c_ulong, C.c_char_p]
        lib.XChangeProperty.argtypes = [C.c_void_p, C.c_ulong, C.c_ulong, C.c_ulong,
                                       C.c_int, C.c_int, C.c_char_p, C.c_int]
        lib.XRaiseWindow.argtypes = [C.c_void_p, C.c_ulong]
        lib.XFlush.argtypes = [C.c_void_p]
        lib.XCloseDisplay.argtypes = [C.c_void_p]
        display = lib.XOpenDisplay(None)
        if not display:
            raise RuntimeError('X11 display unavailable')
        try:
            wid = int(window, 16)
            label = title.encode()
            lib.XStoreName(display, wid, label)
            name = lib.XInternAtom(display, b'_NET_WM_NAME', 0)
            utf8 = lib.XInternAtom(display, b'UTF8_STRING', 0)
            lib.XChangeProperty(display, wid, name, utf8, 8, 0, label, len(label))
            lib.XMoveResizeWindow(display, wid, x, 100, 1240, 1000)
            lib.XRaiseWindow(display, wid)
            lib.XFlush(display)
        finally:
            lib.XCloseDisplay(display)
        return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.input = args.input.resolve()
    args.output = args.output.resolve()
    viewer = native_executable('tianji_qp_ik_viewer')
    _, records = _read_trace(args.input)
    if not records or any(b[0] < a[0] for a, b in zip(records, records[1:])):
        raise ValueError('empty or non-monotonic recording')
    if records[0][0] < 0:
        raise ValueError('negative recording time')
    args.output.mkdir(parents=True, exist_ok=False)
    processes, logs, endpoints = [], [], []
    manifest = {'trace': str(args.input.resolve()), 'frames': len(records),
                'mode': 'online model-reference only; common UDP send clock; not lockstep',
                'complete': False, 'commands': []}
    try:
        for name, profile in [('SPARK default', 'qp_ik_pico_shared_root_reachable.yaml'),
                              ('Ceres LM + Ruckig', 'qp_ik_pico_shared_root_ceres.yaml')]:
            source = controller_profile(profile)
            config = yaml.safe_load(source.read_text())
            config['spark_shared_root']['enabled'] = True
            for key in ('input_contract_artifact', 'robot_geometry_artifact'):
                config['spark_shared_root'][key] = str(controller_resource(source, config['spark_shared_root'][key]))
            for key in ('home_config', 'pico_ee_dls_kinematics_urdf_path'):
                if config['controller'].get(key):
                    config['controller'][key] = str(controller_resource(source, config['controller'][key]))
            stem = 'spark' if not processes else 'ceres'
            path = args.output/(stem+'.yaml')
            with path.open('x') as stream:
                yaml.safe_dump(config, stream, sort_keys=False)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
                reserve.bind(('127.0.0.1', 0))
                port = reserve.getsockname()[1]
            if ('127.0.0.1', port) in endpoints:
                raise RuntimeError('duplicate input port')
            endpoints.append(('127.0.0.1', port))
            command = [str(viewer), '--config', str(path),
                       '--pico-teleop', '--pico-bind', '127.0.0.1', '--pico-port', str(port),
                       '--model-state-only', '--telemetry', str(args.output/(stem+'.csv'))]
            manifest['commands'].append(command)
            log = (args.output/(stem+'.log')).open('x')
            logs.append(log)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            processes.append(process)
            for _ in range(100):
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited before replay; inspect {args.output}')
                if arrange(process.pid, name+' | SAME INPUT - ONLINE IK | '+args.input.parent.name,
                           10 if stem == 'spark' else 1270):
                    break
                time.sleep(.1)
            else:
                raise RuntimeError('Viewer window did not appear')
        manifest['pids'] = [p.pid for p in processes]
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        print('READY: left SPARK, right Ceres; replay starts in 10 seconds', flush=True)
        start = time.monotonic_ns()+10_000_000_000
        max_send_span = 0
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for relative, packet, _ in records:
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('A comparison window closed; stopping both')
                deadline = start+relative
                while (remaining := deadline-time.monotonic_ns()) > 0:
                    time.sleep(remaining*1e-9)
                before = time.monotonic_ns()
                for endpoint in endpoints:
                    sender.sendto(packet, endpoint)
                max_send_span = max(max_send_span, time.monotonic_ns()-before)
        manifest['complete'] = True
        manifest['max_fanout_send_span_us'] = max_send_span/1000
        print('REPLAY COMPLETE', len(records), 'frames per viewer', flush=True)
        time.sleep(5)
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in logs:
            log.close()
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


if __name__ == '__main__':
    main()
