#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

usage() {
  printf 'Usage: bash view.sh [--date YYYYMMDD]\n'
  printf 'View $HOME/Documents/TianjiData/YYYYMMDD_compressed in the browser.\n'
  printf 'Default: today. Open http://127.0.0.1:8777/ after startup.\n'
  printf 'Automatically stops the previous viewer for this project on port 8777; unrelated programs are left running.\n'
}

case "${1:-}" in
  '') day="$(date +%Y%m%d)" ;;
  --date)
    if [[ $# != 2 || ! "$2" =~ ^[0-9]{8}$ ]]; then
      usage >&2; exit 2
    fi
    day="$2"
    ;;
  --help|-h) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac
if ! normalized="$(date -d "${day:0:4}-${day:4:2}-${day:6:2}" +%Y%m%d 2>/dev/null)" || [[ "$normalized" != "$day" ]]; then
  printf 'Invalid date: %s (expected YYYYMMDD).\n' "$day" >&2
  exit 2
fi

source scripts/environment.sh
dataset_dir="$HOME/Documents/TianjiData/${day}_compressed"
printf 'Dataset: %s\n' "$dataset_dir"
if [[ ! -d "$dataset_dir" ]]; then
  printf 'Dataset directory not found: %s\n' "$dataset_dir" >&2
  exit 2
fi

# Only replace this checkout's read-only viewer, never an unrelated port owner.
# pidfds keep signals attached to the inspected process even if its PID is reused.
"$TIANJI_PYTHON" - <<'PY'
import ctypes
import os
from pathlib import Path
import select
import signal
import subprocess

# The Conda Python build omits os.pidfd_open; use the host libc API directly.
libc = ctypes.CDLL(None, use_errno=True)
def checked(result, function, arguments):
    if result < 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    return result
libc.pidfd_open.argtypes = [ctypes.c_int, ctypes.c_uint]
libc.pidfd_send_signal.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_uint]
libc.pidfd_open.errcheck = checked
libc.pidfd_send_signal.errcheck = checked

root = Path.cwd()
owners = subprocess.run(
    ["fuser", "-n", "tcp", "8777"], capture_output=True, text=True,
)
if owners.returncode not in (0, 1):
    raise SystemExit(owners.stderr.strip() or "Cannot inspect port 8777")
handles = []
try:
    for value in owners.stdout.split():
        pid = int(value)
        process = Path(f"/proc/{pid}")
        try:
            fd = libc.pidfd_open(pid, 0)
        except ProcessLookupError:
            continue
        handles.append((pid, fd))
        try:
            args = process.joinpath("cmdline").read_bytes().split(b"\0")
            local_viewer = (
                args[1:3] == [b"-m", b"data_collection.visualize"]
                or args[1:2] == [os.fsencode(root / "data_collection/visualize.py")]
            )
            owned = process.stat().st_uid == os.getuid()
            same_checkout = process.joinpath("cwd").resolve() == root
        except FileNotFoundError:
            handles.pop()
            os.close(fd)
            continue
        if not (local_viewer and owned and same_checkout):
            raise SystemExit(f"Port 8777 is occupied by another program (PID {pid}); refusing to stop it.")
    for pid, fd in handles:
        print(f"Stopping previous viewer on port 8777 (PID {pid})...", flush=True)
        try:
            libc.pidfd_send_signal(fd, signal.SIGTERM, None, 0)
        except ProcessLookupError:
            pass
    for pid, fd in handles:
        if not select.select([fd], [], [], 5)[0]:
            try:
                libc.pidfd_send_signal(fd, signal.SIGKILL, None, 0)
            except ProcessLookupError:
                pass
            if not select.select([fd], [], [], 5)[0]:
                raise SystemExit(f"Previous viewer did not exit (PID {pid})")
finally:
    for _, fd in handles:
        os.close(fd)
PY
exec "$TIANJI_PYTHON" -m tianji visualize "$dataset_dir" --port 8777
