"""Background camera capture worker for the PySide6 GUI."""

from __future__ import annotations

import threading
import time
from pathlib import Path

from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage

from soarm101_motion.camera import (
    CameraCapture,
    CameraDeviceUnavailableError,
    CameraFrameReadError,
    CameraSettings,
    camera_device_available,
)


MAX_CONSECUTIVE_FRAME_FAILURES = 8
MAX_FRAME_RECOVERY_CYCLES = 3
FRAME_RETRY_DELAY_MS = 8
MAX_OPEN_FAILURES = 3
OPEN_RETRY_DELAY_MS = 250
DEVICE_RETURN_TIMEOUT_MS = 30_000
DEVICE_RETURN_POLL_MS = 500


class CameraWorker(QThread):
    """Own one camera session and fan its frames out to multiple GUI views."""

    frame_ready = Signal(object)
    status_changed = Signal(object)
    error_message = Signal(str)
    snapshot_saved = Signal(str)

    def __init__(self, settings: CameraSettings) -> None:
        super().__init__()
        self._lock = threading.Lock()
        self._settings = settings.validated()
        self._streaming = False
        self._shutdown = False
        self._generation = 0
        self._snapshot_path: str | None = None

    def settings(self) -> CameraSettings:
        with self._lock:
            return self._settings

    def configure(self, settings: CameraSettings) -> None:
        with self._lock:
            self._settings = settings.validated()
            self._generation += 1

    def start_stream(self) -> None:
        with self._lock:
            self._streaming = True
        if not self.isRunning():
            self.start()

    def stop_stream(self) -> None:
        with self._lock:
            self._streaming = False

    def request_snapshot(self, output: str | Path | None = None) -> None:
        with self._lock:
            self._snapshot_path = str(Path(output).expanduser()) if output is not None else ""

    def shutdown(self) -> None:
        with self._lock:
            self._shutdown = True
            self._streaming = False
        if self.isRunning():
            self.wait(3000)

    def run(self) -> None:
        capture: CameraCapture | None = None
        opened_generation = -1
        last_connected = False
        consecutive_frame_failures = 0
        open_failures = 0
        recovering = False
        recovery_cycles = 0
        device_wait_started: float | None = None
        device_wait_last_second = -1
        observed_generation = -1
        had_good_frame = False
        while True:
            with self._lock:
                shutdown = self._shutdown
                streaming = self._streaming
                settings = self._settings
                generation = self._generation
                snapshot_path = self._snapshot_path
            if shutdown:
                break
            if not streaming:
                if capture is not None:
                    capture.close()
                    capture = None
                if last_connected:
                    self.status_changed.emit({"connected": False, "device": settings.device})
                    last_connected = False
                consecutive_frame_failures = 0
                open_failures = 0
                recovering = False
                recovery_cycles = 0
                device_wait_started = None
                device_wait_last_second = -1
                observed_generation = generation
                had_good_frame = False
                self.msleep(50)
                continue

            if generation != observed_generation:
                if capture is not None:
                    capture.close()
                    capture = None
                observed_generation = generation
                opened_generation = -1
                last_connected = False
                consecutive_frame_failures = 0
                open_failures = 0
                recovering = False
                recovery_cycles = 0
                device_wait_started = None
                device_wait_last_second = -1
                had_good_frame = False

            if capture is None or generation != opened_generation:
                if capture is not None:
                    capture.close()
                    capture = None

                if not camera_device_available(settings.device):
                    now = time.monotonic()
                    if device_wait_started is None:
                        device_wait_started = now
                        device_wait_last_second = -1
                    waited_s = max(0.0, now - device_wait_started)
                    timeout_s = DEVICE_RETURN_TIMEOUT_MS / 1000.0
                    if waited_s >= timeout_s:
                        message = (
                            f"camera device did not return within {timeout_s:.0f} s: "
                            f"{settings.device}"
                        )
                        with self._lock:
                            self._streaming = False
                        recovering = False
                        self.error_message.emit(message)
                        self.status_changed.emit(
                            {
                                "connected": False,
                                "device": settings.device,
                                "recovering": False,
                                "device_missing": True,
                                "waited_s": waited_s,
                                "wait_timeout_s": timeout_s,
                                "error": message,
                            }
                        )
                        continue
                    waited_second = int(waited_s)
                    if waited_second != device_wait_last_second:
                        device_wait_last_second = waited_second
                        recovering = True
                        self.status_changed.emit(
                            {
                                "connected": False,
                                "device": settings.device,
                                "recovering": True,
                                "device_missing": True,
                                "waited_s": waited_s,
                                "wait_timeout_s": timeout_s,
                                "warning": (
                                    "camera disconnected; waiting for the saved device "
                                    "identity to return"
                                ),
                            }
                        )
                    self.msleep(DEVICE_RETURN_POLL_MS)
                    continue

                try:
                    capture = CameraCapture(settings).open()
                    opened_generation = generation
                    last_connected = True
                    consecutive_frame_failures = 0
                    open_failures = 0
                    recovering = False
                    device_wait_started = None
                    device_wait_last_second = -1
                    self.status_changed.emit(
                        {
                            "connected": True,
                            "recovering": False,
                            "device_missing": False,
                            "dropped_frames": 0,
                            **capture.actual_format(),
                        }
                    )
                except CameraDeviceUnavailableError as exc:
                    capture = None
                    last_connected = False
                    if device_wait_started is None:
                        device_wait_started = time.monotonic()
                        device_wait_last_second = -1
                    recovering = True
                    self.status_changed.emit(
                        {
                            "connected": False,
                            "device": settings.device,
                            "recovering": True,
                            "device_missing": True,
                            "waited_s": 0.0,
                            "wait_timeout_s": DEVICE_RETURN_TIMEOUT_MS / 1000.0,
                            "warning": str(exc),
                        }
                    )
                    self.msleep(DEVICE_RETURN_POLL_MS)
                    continue
                except Exception as exc:
                    capture = None
                    last_connected = False
                    if had_good_frame or device_wait_started is not None:
                        now = time.monotonic()
                        if device_wait_started is None:
                            device_wait_started = now
                            device_wait_last_second = -1
                        waited_s = max(0.0, now - device_wait_started)
                        timeout_s = DEVICE_RETURN_TIMEOUT_MS / 1000.0
                        if waited_s < timeout_s:
                            recovering = True
                            self.status_changed.emit(
                                {
                                    "connected": False,
                                    "device": settings.device,
                                    "recovering": True,
                                    "device_missing": not camera_device_available(
                                        settings.device
                                    ),
                                    "waited_s": waited_s,
                                    "wait_timeout_s": timeout_s,
                                    "warning": (
                                        "camera was live but is not ready to reopen; "
                                        "waiting for USB/V4L2 recovery"
                                    ),
                                }
                            )
                            self.msleep(DEVICE_RETURN_POLL_MS)
                            continue
                        message = (
                            f"camera did not recover within {timeout_s:.0f} s: "
                            f"{settings.device}"
                        )
                        with self._lock:
                            self._streaming = False
                        recovering = False
                        self.error_message.emit(message)
                        self.status_changed.emit(
                            {
                                "connected": False,
                                "device": settings.device,
                                "recovering": False,
                                "device_missing": not camera_device_available(
                                    settings.device
                                ),
                                "waited_s": waited_s,
                                "wait_timeout_s": timeout_s,
                                "error": message,
                            }
                        )
                        continue

                    open_failures += 1
                    if open_failures < MAX_OPEN_FAILURES:
                        self.status_changed.emit(
                            {
                                "connected": False,
                                "device": settings.device,
                                "recovering": True,
                                "device_missing": False,
                                "open_attempt": open_failures,
                                "open_attempt_limit": MAX_OPEN_FAILURES,
                                "warning": str(exc),
                            }
                        )
                        self.msleep(OPEN_RETRY_DELAY_MS)
                        continue
                    with self._lock:
                        self._streaming = False
                    self.error_message.emit(str(exc))
                    self.status_changed.emit(
                        {
                            "connected": False,
                            "device": settings.device,
                            "recovering": False,
                            "device_missing": False,
                            "error": str(exc),
                        }
                    )
                    continue

            try:
                frame = capture.read_bgr()
                if consecutive_frame_failures or recovering:
                    self.status_changed.emit(
                        {
                            "connected": True,
                            "recovering": False,
                            "dropped_frames": consecutive_frame_failures,
                            **capture.actual_format(),
                        }
                    )
                consecutive_frame_failures = 0
                recovering = False
                recovery_cycles = 0
                device_wait_started = None
                device_wait_last_second = -1
                had_good_frame = True

                height, width = frame.shape[:2]
                image = QImage(
                    frame.data,
                    width,
                    height,
                    int(frame.strides[0]),
                    QImage.Format.Format_BGR888,
                ).copy()
                self.frame_ready.emit(image)

                if snapshot_path is not None:
                    output = capture.save_frame(frame, snapshot_path or None)
                    self.snapshot_saved.emit(str(output))
                    with self._lock:
                        if self._snapshot_path == snapshot_path:
                            self._snapshot_path = None

            except CameraDeviceUnavailableError as exc:
                capture.close()
                capture = None
                last_connected = False
                consecutive_frame_failures = 0
                recovery_cycles = 0
                recovering = True
                if device_wait_started is None:
                    device_wait_started = time.monotonic()
                    device_wait_last_second = -1
                self.status_changed.emit(
                    {
                        "connected": False,
                        "device": settings.device,
                        "recovering": True,
                        "device_missing": True,
                        "waited_s": 0.0,
                        "wait_timeout_s": DEVICE_RETURN_TIMEOUT_MS / 1000.0,
                        "warning": str(exc),
                    }
                )
                self.msleep(DEVICE_RETURN_POLL_MS)
                continue

            except CameraFrameReadError as exc:
                consecutive_frame_failures += 1
                recovering = True
                if consecutive_frame_failures < MAX_CONSECUTIVE_FRAME_FAILURES:
                    if consecutive_frame_failures == 1:
                        self.status_changed.emit(
                            {
                                "connected": True,
                                "device": settings.device,
                                "recovering": True,
                                "dropped_frames": consecutive_frame_failures,
                                "drop_limit": MAX_CONSECUTIVE_FRAME_FAILURES,
                                "warning": str(exc),
                            }
                        )
                    self.msleep(FRAME_RETRY_DELAY_MS)
                    continue

                capture.close()
                capture = None
                last_connected = False
                consecutive_frame_failures = 0
                recovery_cycles += 1
                if recovery_cycles >= MAX_FRAME_RECOVERY_CYCLES:
                    message = (
                        "camera repeatedly opened but did not return usable frames"
                    )
                    with self._lock:
                        self._streaming = False
                    self.error_message.emit(message)
                    self.status_changed.emit(
                        {
                            "connected": False,
                            "device": settings.device,
                            "recovering": False,
                            "error": message,
                        }
                    )
                    continue
                self.status_changed.emit(
                    {
                        "connected": False,
                        "device": settings.device,
                        "recovering": True,
                        "recovery_cycle": recovery_cycles,
                        "recovery_cycle_limit": MAX_FRAME_RECOVERY_CYCLES,
                        "warning": (
                            "camera missed several consecutive frames; reopening the device"
                        ),
                    }
                )
                self.msleep(OPEN_RETRY_DELAY_MS)
                continue

            except Exception as exc:
                self.error_message.emit(str(exc))
                capture.close()
                capture = None
                last_connected = False
                with self._lock:
                    self._streaming = False
                self.status_changed.emit(
                    {
                        "connected": False,
                        "device": settings.device,
                        "recovering": False,
                        "error": str(exc),
                    }
                )
                continue

            # VideoCapture normally blocks to the device frame cadence. Yield briefly
            # so stop/configuration requests remain responsive without halving FPS.
            self.msleep(1)

        if capture is not None:
            capture.close()
        if last_connected:
            self.status_changed.emit({"connected": False, "device": self._settings.device})
