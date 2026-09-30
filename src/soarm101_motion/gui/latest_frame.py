"""Bounded latest-frame handoff for GUI camera previews."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class PreviewFrame:
    """A camera observation waiting to be presented by the GUI."""

    image: object
    acquired_ns: int
    superseded_frames: int
    capture_fps: float


class LatestFrameMailbox:
    """Keep at most one pending preview frame and discard replaced frames."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pending: tuple[object, int, float] | None = None
        self._superseded = 0

    def publish(
        self,
        image: object,
        acquired_ns: int | None = None,
        capture_fps: float = 0.0,
    ) -> None:
        """Publish the newest observation without creating a GUI event per frame."""
        timestamp = time.monotonic_ns() if acquired_ns is None else acquired_ns
        with self._lock:
            if self._pending is not None:
                self._superseded += 1
            self._pending = (image, timestamp, capture_fps)

    def take_latest(self) -> PreviewFrame | None:
        """Return the freshest pending frame and clear the one-slot mailbox."""
        with self._lock:
            if self._pending is None:
                return None
            image, acquired_ns, capture_fps = self._pending
            frame = PreviewFrame(image, acquired_ns, self._superseded, capture_fps)
            self._pending = None
            self._superseded = 0
            return frame
