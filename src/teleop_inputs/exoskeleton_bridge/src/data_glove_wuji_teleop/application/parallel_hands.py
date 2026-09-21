"""左右手用例共用的线程、信号与停止状态编排。"""

from __future__ import annotations

import signal
import threading
from collections.abc import Callable


HandWorker = Callable[[str, threading.Event], int]


def run_parallel_hands(
    worker: HandWorker,
    *,
    thread_prefix: str,
) -> int:
    """并行运行左右手；任一侧结束后请求另一侧停止。"""

    stop = threading.Event()
    results: dict[str, int] = {}
    errors: list[BaseException] = []
    result_lock = threading.Lock()

    def run_one(hand: str) -> None:
        try:
            result = worker(hand, stop)
        except BaseException as exc:
            with result_lock:
                errors.append(exc)
        else:
            with result_lock:
                results[hand] = result
        finally:
            stop.set()

    previous_handlers = {}

    def request_stop(_signum=None, _frame=None) -> None:
        stop.set()

    for name in ("SIGINT", "SIGTERM", "SIGHUP"):
        signum = getattr(signal, name, None)
        if signum is not None:
            previous_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, request_stop)

    threads = [
        threading.Thread(
            target=run_one,
            args=(hand,),
            name=f"{thread_prefix}-{hand}",
        )
        for hand in ("left", "right")
    ]
    try:
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
    finally:
        stop.set()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)

    if errors:
        raise errors[0]
    return max(results.values(), default=0)
