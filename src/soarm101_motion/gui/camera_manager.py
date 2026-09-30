"""Named multi-camera session manager for the PySide6 GUI."""

from __future__ import annotations

from collections.abc import Mapping

from PySide6.QtCore import QObject, QTimer, Signal

from soarm101_motion.camera import CameraSettings
from soarm101_motion.gui.camera_worker import CameraWorker
from soarm101_motion.gui.latest_frame import LatestFrameMailbox


class CameraSessionManager(QObject):
    """Own at most one worker per named camera/device and fan events out by name."""

    frame_ready = Signal(str, object)
    status_changed = Signal(str, object)
    error_message = Signal(str, str)
    snapshot_saved = Signal(str, str)

    def __init__(self) -> None:
        super().__init__()
        self._workers: dict[str, CameraWorker] = {}
        self._settings: dict[str, CameraSettings] = {}
        self._preview_mailboxes: dict[str, LatestFrameMailbox] = {}
        self._preview_timer = QTimer(self)
        self._preview_timer.setInterval(33)
        self._preview_timer.timeout.connect(self._deliver_latest_frames)
        self._preview_timer.start()

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(self._workers)

    def settings(self, name: str) -> CameraSettings:
        return self._settings[name]

    def sync(self, cameras: Mapping[str, CameraSettings]) -> None:
        requested = {str(name): settings.validated() for name, settings in cameras.items()}
        for name in tuple(self._workers):
            if name not in requested:
                worker = self._workers.pop(name)
                self._settings.pop(name, None)
                self._preview_mailboxes.pop(name, None)
                worker.shutdown()
                worker.deleteLater()
        for name, settings in requested.items():
            worker = self._workers.get(name)
            if worker is None:
                mailbox = LatestFrameMailbox()
                worker = CameraWorker(settings, preview_mailbox=mailbox)
                worker.status_changed.connect(
                    lambda status, n=name: self.status_changed.emit(n, status)
                )
                worker.error_message.connect(
                    lambda message, n=name: self.error_message.emit(n, message)
                )
                worker.snapshot_saved.connect(
                    lambda path, n=name: self.snapshot_saved.emit(n, path)
                )
                self._workers[name] = worker
                self._preview_mailboxes[name] = mailbox
            else:
                worker.configure(settings)
            self._settings[name] = settings

    def start(self, name: str) -> None:
        self._workers[name].start_stream()

    def stop(self, name: str) -> None:
        self._workers[name].stop_stream()

    def start_auto(self) -> None:
        for name, settings in self._settings.items():
            if settings.auto_start:
                self._workers[name].start_stream()

    def start_all(self) -> None:
        for worker in self._workers.values():
            worker.start_stream()

    def stop_all(self) -> None:
        for worker in self._workers.values():
            worker.stop_stream()

    def request_snapshot(self, name: str, output: str | None = None) -> None:
        self._workers[name].request_snapshot(output)

    def _deliver_latest_frames(self) -> None:
        """Present at most one fresh pending frame per named camera per UI tick."""
        for name, mailbox in tuple(self._preview_mailboxes.items()):
            frame = mailbox.take_latest()
            if frame is not None:
                self.frame_ready.emit(name, frame)

    def shutdown(self) -> None:
        for worker in tuple(self._workers.values()):
            worker.shutdown()
            worker.deleteLater()
        self._workers.clear()
        self._settings.clear()
        self._preview_mailboxes.clear()
        self._preview_timer.stop()
