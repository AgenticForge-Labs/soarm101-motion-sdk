"""Trusted-host heartbeat supervision, independent of the action dispatch lock."""
from __future__ import annotations

import math
import threading
import time
from typing import Callable


class BrokerWatchdog:
    """One-shot monotonic deadline. Heartbeats never grant or extend authority."""

    def __init__(
        self, *, timeout_s: float, validate: Callable[[], None],
        trip: Callable[[str], None], clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not math.isfinite(timeout_s) or not 0.5 <= timeout_s <= 10:
            raise ValueError("watchdog timeout must be within 0.5..10 seconds")
        self.timeout_s = timeout_s
        self._validate = validate
        self._trip_callback = trip
        self._clock = clock
        self._lock = threading.RLock()
        self._deadline: float | None = None
        self._reason: str | None = None
        self._stopped = False
        self._stop_result: dict[str, object] | None = None
        self._stop_error: str | None = None
        self._closed = threading.Event()
        self._worker: threading.Thread | None = None

    @property
    def status(self) -> dict[str, object]:
        with self._lock:
            return {
                "state": (
                    "tripped" if self._reason else
                    "stopped" if self._stopped else
                    "watching" if self._deadline is not None else "waiting"
                ),
                "timeout_s": self.timeout_s, "reason": self._reason,
                "stop_result": self._stop_result, "stop_error": self._stop_error,
            }

    def heartbeat(self) -> dict[str, object]:
        # Check the old deadline BEFORE renewal; a delayed heartbeat cannot
        # recover a lost supervisor even if the polling thread was delayed.
        self.check(require_started=False)
        self._validate()
        with self._lock:
            if self._reason or self._stopped or self._closed.is_set():
                raise PermissionError("watchdog is terminal; human restart required")
            if self._deadline is not None and self._clock() >= self._deadline:
                raise PermissionError("watchdog heartbeat arrived after deadline")
            self._deadline = self._clock() + self.timeout_s
        return self.status

    def check(self, *, require_started: bool = True) -> None:
        reason = None
        with self._lock:
            if self._reason or self._stopped or self._closed.is_set():
                raise PermissionError("watchdog is terminal; human restart required")
            if self._deadline is None:
                if require_started:
                    raise PermissionError("trusted supervisor heartbeat required before motion")
                return
            if self._clock() >= self._deadline:
                reason = "supervisor heartbeat expired"
            else:
                try:
                    self._validate()
                except Exception as exc:
                    reason = f"authority/session validation failed: {exc}"
            if reason:
                self._reason = reason  # Fence admission before STOP can block.
        if reason:
            self._trip_callback(reason)
            raise PermissionError(f"watchdog tripped: {reason}")

    def begin_stop(self) -> None:
        with self._lock:
            if self._stopped:
                raise PermissionError("STOP already attempted; inspect recorded HOLD outcome")
            self._stopped = True

    def finish_stop(self, result: dict[str, object] | None = None, error: str | None = None) -> None:
        with self._lock:
            self._stop_result = result
            self._stop_error = error

    def start(self) -> None:
        if self._worker is not None:
            raise RuntimeError("watchdog already started")

        def monitor() -> None:
            while not self._closed.wait(min(0.1, self.timeout_s / 4)):
                try:
                    self.check(require_started=False)
                except Exception:
                    # Terminal state remains visible even if STOP/auditing fails.
                    return

        self._worker = threading.Thread(target=monitor, name="broker-watchdog", daemon=True)
        self._worker.start()

    def close(self) -> None:
        self._closed.set()
        if self._worker is not None:
            self._worker.join(timeout=2)
