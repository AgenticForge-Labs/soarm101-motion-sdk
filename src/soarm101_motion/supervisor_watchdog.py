"""Deterministic broker-owned supervisor liveness gate.

A trusted host must pulse this lease; model or sandbox traffic cannot extend
it. Once expired, stopped or faulted, a fresh broker session is required.
The timeout callback is invoked exactly once and outside the state lock so an
independent SDK STOP can run while the ordinary motion command is in flight.

This is a software watchdog, not a safety-rated emergency-stop circuit.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable


class SupervisorHeartbeatWatchdog:
    def __init__(
        self,
        *,
        timeout_s: float,
        on_expire: Callable[[], None],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not math.isfinite(timeout_s) or not 1.0 <= timeout_s <= 120.0:
            raise ValueError("supervisor timeout must be within 1..120 seconds")
        self.timeout_s = timeout_s
        self._on_expire = on_expire
        self._clock = clock
        self._lock = threading.Lock()
        self._state = "unarmed"
        self._deadline: float | None = None
        self._last_error: str | None = None
        self._shutdown = threading.Event()
        self._thread: threading.Thread | None = None

    def pulse(self) -> dict[str, object]:
        with self._lock:
            if self._state not in {"unarmed", "armed"}:
                raise PermissionError(
                    f"supervisor watchdog is {self._state}; restart broker to rearm"
                )
            self._state = "armed"
            self._deadline = self._clock() + self.timeout_s
            return self.status

    def require_fresh(self) -> None:
        with self._lock:
            if (
                self._state != "armed"
                or self._deadline is None
                or self._clock() >= self._deadline
            ):
                raise PermissionError(
                    "motion rejected: trusted supervisor heartbeat missing or stale"
                )

    def stop(self) -> None:
        with self._lock:
            if self._state in {"unarmed", "armed"}:
                self._state = "stopped"

    @property
    def status(self) -> dict[str, object]:
        # Accessed under the lock from pulse(); other callers can read a
        # self-contained snapshot via snapshot().
        return {
            "state": self._state,
            "timeout_s": self.timeout_s,
            "last_error": self._last_error,
        }

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return dict(self.status)

    def check(self) -> bool:
        """Issue one independent STOP callback after heartbeat expires."""
        with self._lock:
            if (
                self._state != "armed" or self._deadline is None
                or self._clock() < self._deadline
            ):
                return False
            self._state = "expired"
        try:
            self._on_expire()
        except Exception as exc:
            with self._lock:
                self._last_error = repr(exc)
        return True

    def start(self) -> None:
        if self._thread is not None:
            return

        def monitor() -> None:
            while not self._shutdown.wait(min(0.2, self.timeout_s / 4)):
                self.check()

        self._thread = threading.Thread(
            target=monitor, name="soarm101-supervisor-watchdog", daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        self._shutdown.set()
        thread = self._thread
        if thread is not None and threading.current_thread() is not thread:
            thread.join(timeout=5)
