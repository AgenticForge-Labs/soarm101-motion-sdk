"""Background camera capture worker for the PySide6 GUI."""

from __future__ import annotations

import threading
from pathlib import Path

from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage

from soarm101_motion.camera import CameraCapture, CameraSettings


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
        while True:
            with self._lock:
                shutdown = self._shutdown
                streaming = self._streaming
                settings = self._settings
                generation = self._generation
                snapshot_path = self._snapshot_path
                self._snapshot_path = None
            if shutdown:
                break
            if not streaming:
                if capture is not None:
                    capture.close()
                    capture = None
                if last_connected:
                    self.status_changed.emit({"connected": False, "device": settings.device})
                    last_connected = False
                self.msleep(50)
                continue
            if capture is None or generation != opened_generation:
                if capture is not None:
                    capture.close()
                try:
                    capture = CameraCapture(settings).open()
                    opened_generation = generation
                    last_connected = True
                    self.status_changed.emit(
                        {"connected": True, **capture.actual_format()}
                    )
                except Exception as exc:
                    capture = None
                    last_connected = False
                    with self._lock:
                        self._streaming = False
                    self.error_message.emit(str(exc))
                    self.status_changed.emit(
                        {"connected": False, "device": settings.device, "error": str(exc)}
                    )
                    continue
            try:
                frame = capture.read_bgr()
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
            except Exception as exc:
                self.error_message.emit(str(exc))
                capture.close()
                capture = None
                last_connected = False
                with self._lock:
                    self._streaming = False
                self.status_changed.emit(
                    {"connected": False, "device": settings.device, "error": str(exc)}
                )
                continue
            # VideoCapture normally blocks to the device frame cadence. Yield briefly
            # so stop/configuration requests remain responsive without halving FPS.
            self.msleep(1)
        if capture is not None:
            capture.close()
        if last_connected:
            self.status_changed.emit({"connected": False, "device": self._settings.device})
