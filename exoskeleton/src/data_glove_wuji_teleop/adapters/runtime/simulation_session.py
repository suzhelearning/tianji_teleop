"""维护并校验跨终端的同代、同侧 MuJoCo 运行租约。"""

from __future__ import annotations

import json
import os
import time
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path


SUPPORTED_GENERATIONS = ("v1", "v2")
SUPPORTED_HANDS = ("left", "right")


def default_runtime_dir() -> Path:
    """返回当前用户私有的跨终端运行状态目录。"""

    configured = os.environ.get("DATAGLOVE_RUNTIME_DIR")
    if configured:
        return Path(configured)
    xdg_runtime = os.environ.get("XDG_RUNTIME_DIR")
    if xdg_runtime:
        return Path(xdg_runtime) / "data-glove-wuji-teleop"
    return Path("/tmp") / f"data-glove-wuji-teleop-{os.getuid()}"


def _validate_identity(generation: str, hand: str) -> None:
    if generation not in SUPPORTED_GENERATIONS:
        raise ValueError(f"generation 必须是 v1 或 v2，实际为 {generation}")
    if hand not in SUPPORTED_HANDS:
        raise ValueError(f"hand 必须是 left 或 right，实际为 {hand}")


def _process_start_ticks(pid: int) -> int:
    stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    return int(stat.rsplit(")", 1)[1].split()[19])


@dataclass(frozen=True)
class SimulationSession:
    """一个仍存活的 MuJoCo 查看器租约。"""

    generation: str
    hand: str
    pid: int
    process_start_ticks: int
    started_ns: int
    token: str


def _session_path(
    generation: str,
    hand: str,
    runtime_dir: Path,
) -> Path:
    _validate_identity(generation, hand)
    return runtime_dir / f"simulation-{generation}-{hand}.json"


def _read_session(path: Path) -> SimulationSession:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
        return SimulationSession(
            generation=str(raw["generation"]),
            hand=str(raw["hand"]),
            pid=int(raw["pid"]),
            process_start_ticks=int(raw["process_start_ticks"]),
            started_ns=int(raw["started_ns"]),
            token=str(raw["token"]),
        )
    except (FileNotFoundError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"没有有效的仿真运行记录：{path}") from exc


def _session_process_is_alive(session: SimulationSession) -> bool:
    try:
        os.kill(session.pid, 0)
        return _process_start_ticks(session.pid) == session.process_start_ticks
    except (OSError, ValueError):
        return False


def require_active_simulation(
    *,
    generation: str,
    hand: str,
    runtime_dir: Path | None = None,
) -> SimulationSession:
    """要求同代、同侧查看器存活，否则拒绝真机控制。"""

    directory = runtime_dir or default_runtime_dir()
    path = _session_path(generation, hand, directory)
    session = _read_session(path)
    if session.generation != generation or session.hand != hand:
        raise RuntimeError(f"仿真运行记录与请求不一致：{path}")
    if not _session_process_is_alive(session):
        raise RuntimeError(f"MuJoCo 查看器已经退出：{path}")
    return session


class SimulationLease:
    """在 MuJoCo 查看器生命周期内维护私有运行记录。"""

    def __init__(
        self,
        *,
        generation: str,
        hand: str,
        runtime_dir: Path | None = None,
    ):
        directory = runtime_dir or default_runtime_dir()
        self._path = _session_path(generation, hand, directory)
        self._session = SimulationSession(
            generation=generation,
            hand=hand,
            pid=os.getpid(),
            process_start_ticks=_process_start_ticks(os.getpid()),
            started_ns=time.time_ns(),
            token=uuid.uuid4().hex,
        )

    def __enter__(self) -> SimulationSession:
        self._path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        try:
            existing = _read_session(self._path)
        except RuntimeError:
            existing = None
        if existing is not None and _session_process_is_alive(existing):
            raise RuntimeError(
                f"同代同侧仿真已经运行：pid={existing.pid} {self._path}"
            )

        temporary = self._path.with_name(
            f".{self._path.name}.{self._session.token}.tmp"
        )
        temporary.write_text(
            json.dumps(asdict(self._session), ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        temporary.chmod(0o600)
        temporary.replace(self._path)
        return self._session

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        try:
            current = _read_session(self._path)
        except RuntimeError:
            return
        if current.token == self._session.token:
            self._path.unlink(missing_ok=True)
