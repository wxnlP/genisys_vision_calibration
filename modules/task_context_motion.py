from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional


@dataclass(frozen=True)
class MoveOutcome:
    ok: bool
    task_id: int = 0
    duration_ms: float = 0.0
    reason: str = ""

    def __bool__(self) -> bool:
        return self.ok


# genisys_msgs/msg/TaskContext.state
RUNNING = 0
FINISHED = 3
TOLER_ERROR = 2
SAMPLE_ERROR = 1
TERMINAL_STATES = (FINISHED, SAMPLE_ERROR, TOLER_ERROR)
STATE_NAMES = {
    -1: "UNKNOWN",
    0: "RUNNING",
    1: "SAMPLE_ERROR",
    2: "TOLER_ERROR",
    3: "FINISHED",
}


class TaskContextMotionClient:
    def __init__(
        self,
        rpc,
        *,
        log_info: Optional[Callable[[str], None]] = None,
        log_warn: Optional[Callable[[str], None]] = None,
        should_stop: Optional[Callable[[], bool]] = None,
        timeout_margin_s: float = 3.0,
        min_timeout_s: float = 5.0,
        move_retry: int = 2,
        retry_delay_s: float = 0.5,
        wait_tick_s: float = 0.05,
    ) -> None:
        self.rpc = rpc
        self._log_info = log_info or (lambda _msg: None)
        self._log_warn = log_warn or (lambda _msg: None)
        self._should_stop = should_stop or (lambda: False)
        self.timeout_margin_s = float(timeout_margin_s)
        self.min_timeout_s = float(min_timeout_s)
        self.retry = max(0, int(move_retry))
        self.retry_delay_s = float(retry_delay_s)
        self.wait_tick_s = float(wait_tick_s)

        self._cond = threading.Condition()
        self._contexts = {}

    def on_context(self, msg) -> None:
        task_id = int(msg.id)
        if task_id <= 0:
            return
        ctx = (int(msg.state), int(msg.code), float(msg.duration_ms))
        with self._cond:
            old = self._contexts.get(task_id)
            if old is not None and old[0] in TERMINAL_STATES and ctx[0] not in TERMINAL_STATES:
                return
            self._contexts[task_id] = ctx
            for old_id in sorted(self._contexts)[:-16]:
                del self._contexts[old_id]
            self._cond.notify_all()

    def move_to_pose(self, x: float, y: float, z: float,
                     roll: float, pitch: float, yaw: float,
                     label: str = "move",
                     v: float = 0.8, a: float = 0.4) -> MoveOutcome:
        # task id 可能因 genisys 重启而复用；丢弃已结束任务的缓存，保留仍在运行的
        with self._cond:
            self._contexts = {
                tid: ctx for tid, ctx in self._contexts.items()
                if ctx[0] == RUNNING
            }

        result = None
        for attempt in range(self.retry + 1):
            if self._should_stop():
                return MoveOutcome(False, reason="stopped")
            result = self.rpc.movej_pose(x, y, z, roll, pitch, yaw, v=v, a=a)
            if result.accepted:
                break
            self._log_warn(
                f"[{label}] MoveJPose rejected "
                f"(attempt {attempt + 1}/{self.retry + 1}): "
                f"{result.msg or self.rpc.last_error}")
            if attempt < self.retry:
                self._wait_idle(label)
                if not self._sleep(self.retry_delay_s):
                    return MoveOutcome(False, reason="stopped")
        else:
            return MoveOutcome(False, reason=result.msg or self.rpc.last_error or "rejected")

        if result.task_id <= 0:
            return MoveOutcome(False, reason="missing task_id")

        self._log_info(
            f"[{label}] MoveJPose accepted task={result.task_id} "
            f"duration={result.duration_ms:.0f}ms")

        ok, reason = self._wait_task(result.task_id, result.duration_ms, label)
        return MoveOutcome(ok, result.task_id, result.duration_ms, reason)

    def _sleep(self, seconds: float) -> bool:
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            if self._should_stop():
                return False
            time.sleep(min(self.wait_tick_s, max(0.0, deadline - time.monotonic())))
        return not self._should_stop()

    def _wait_idle(self, label: str, timeout_s: float = 5.0) -> None:
        """被拒绝时等上一个 task 进入终态，避免 motion state is not idle。"""
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while any(ctx[0] == RUNNING for ctx in self._contexts.values()):
                if self._should_stop():
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._log_warn(
                        f"[{label}] task still RUNNING after {timeout_s:.1f}s")
                    return
                self._cond.wait(min(self.wait_tick_s, remaining))

    def _wait_task(self, task_id: int, duration_ms: float, label: str):
        timeout_s = max(
            self.min_timeout_s,
            float(duration_ms) / 1000.0 + self.timeout_margin_s,
        )
        deadline = time.monotonic() + timeout_s

        with self._cond:
            while True:
                if self._should_stop():
                    return False, "stopped"

                ctx = self._contexts.get(task_id)
                if ctx is not None:
                    state, code, _ = ctx
                    if state == FINISHED:
                        self._log_info(
                            f"[{label}] task_context FINISHED task={task_id}")
                        return True, ""
                    if state in (SAMPLE_ERROR, TOLER_ERROR):
                        reason = (f"task_context {STATE_NAMES[state]} "
                                  f"code={code} task={task_id}")
                        self._log_warn(f"[{label}] {reason}")
                        return False, reason

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    if ctx is None:
                        reason = f"task_context timeout, no message for task={task_id}"
                    else:
                        state, code, _ = ctx
                        reason = (f"task_context timeout, state="
                                  f"{STATE_NAMES.get(state, state)} code={code} "
                                  f"task={task_id}")
                    self._log_warn(f"[{label}] {reason} after {timeout_s:.1f}s")
                    return False, reason

                self._cond.wait(min(self.wait_tick_s, remaining))
