from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pytest

from soarm101_motion.camera import (
    CameraCapture,
    CameraFrameReadError,
    CameraSettings,
)


def test_camera_capture_marks_empty_frame_as_transient_read_error() -> None:
    class FakeCapture:
        def isOpened(self) -> bool:
            return True

        def read(self):
            return False, None

    camera = CameraCapture(CameraSettings())
    camera._capture = FakeCapture()

    with pytest.raises(CameraFrameReadError, match="did not return a frame"):
        camera.read_bgr()


def _wait_until(app, predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return True
        time.sleep(0.01)
    app.processEvents()
    return bool(predicate())


def test_camera_worker_recovers_after_one_dropped_frame(monkeypatch) -> None:
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui import camera_worker

    app = QApplication.instance() or QApplication([])
    frame = np.zeros((12, 16, 3), dtype=np.uint8)

    class FakeCapture:
        def __init__(self, settings: CameraSettings) -> None:
            self.settings = settings
            self.read_count = 0
            self.closed = False

        def open(self):
            return self

        def close(self) -> None:
            self.closed = True

        def actual_format(self) -> dict[str, object]:
            return {
                "device": self.settings.device,
                "width": 16,
                "height": 12,
                "fps": 30.0,
                "fourcc": "MJPG",
                "mirror": False,
            }

        def read_bgr(self):
            self.read_count += 1
            if self.read_count == 1:
                raise CameraFrameReadError("camera did not return a frame")
            return frame

        def save_frame(self, _frame, output=None) -> Path:
            return Path(output or "/tmp/camera-test.jpg")

    fake = FakeCapture(CameraSettings(device="/dev/video-test"))
    monkeypatch.setattr(camera_worker, "CameraCapture", lambda settings: fake)

    worker = camera_worker.CameraWorker(fake.settings)
    frames: list[object] = []
    statuses: list[dict[str, object]] = []
    errors: list[str] = []
    worker.frame_ready.connect(frames.append)
    worker.status_changed.connect(lambda status: statuses.append(dict(status)))
    worker.error_message.connect(errors.append)

    worker.start_stream()
    assert _wait_until(app, lambda: bool(frames))

    worker.stop_stream()
    worker.shutdown()
    app.processEvents()

    assert errors == []
    assert fake.read_count >= 2
    assert any(status.get("recovering") is True for status in statuses)
    assert any(
        status.get("connected") is True and status.get("recovering") is False
        for status in statuses
    )


def test_camera_worker_keeps_snapshot_request_across_transient_drop(
    monkeypatch, tmp_path
) -> None:
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui import camera_worker

    app = QApplication.instance() or QApplication([])
    frame = np.zeros((12, 16, 3), dtype=np.uint8)
    output = tmp_path / "snapshot.jpg"

    class FakeCapture:
        def __init__(self, settings: CameraSettings) -> None:
            self.settings = settings
            self.read_count = 0
            self.saved: list[str] = []

        def open(self):
            return self

        def close(self) -> None:
            pass

        def actual_format(self) -> dict[str, object]:
            return {
                "device": self.settings.device,
                "width": 16,
                "height": 12,
                "fps": 30.0,
                "fourcc": "MJPG",
                "mirror": False,
            }

        def read_bgr(self):
            self.read_count += 1
            if self.read_count == 1:
                raise CameraFrameReadError("camera did not return a frame")
            return frame

        def save_frame(self, _frame, requested=None) -> Path:
            path = Path(requested or output)
            self.saved.append(str(path))
            return path

    fake = FakeCapture(CameraSettings(device="/dev/video-test"))
    monkeypatch.setattr(camera_worker, "CameraCapture", lambda settings: fake)

    worker = camera_worker.CameraWorker(fake.settings)
    snapshots: list[str] = []
    errors: list[str] = []
    worker.snapshot_saved.connect(snapshots.append)
    worker.error_message.connect(errors.append)

    worker.request_snapshot(output)
    worker.start_stream()
    assert _wait_until(app, lambda: bool(snapshots))

    worker.stop_stream()
    worker.shutdown()
    app.processEvents()

    assert errors == []
    assert snapshots == [str(output)]
    assert fake.saved == [str(output)]
    assert fake.read_count >= 2



def test_camera_worker_stops_after_bounded_empty_frame_recovery(monkeypatch) -> None:
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui import camera_worker

    app = QApplication.instance() or QApplication([])

    class EmptyCapture:
        def __init__(self, settings: CameraSettings) -> None:
            self.settings = settings
            self.open_count = 0

        def open(self):
            self.open_count += 1
            return self

        def close(self) -> None:
            pass

        def actual_format(self) -> dict[str, object]:
            return {
                "device": self.settings.device,
                "width": 640,
                "height": 480,
                "fps": 30.0,
                "fourcc": "MJPG",
                "mirror": False,
            }

        def read_bgr(self):
            raise CameraFrameReadError("camera did not return a frame")

    fake = EmptyCapture(CameraSettings(device="/dev/video-empty"))
    monkeypatch.setattr(camera_worker, "CameraCapture", lambda settings: fake)

    worker = camera_worker.CameraWorker(fake.settings)
    errors: list[str] = []
    statuses: list[dict[str, object]] = []
    worker.error_message.connect(errors.append)
    worker.status_changed.connect(lambda status: statuses.append(dict(status)))

    worker.start_stream()
    assert _wait_until(app, lambda: bool(errors), timeout=3.0)

    worker.shutdown()
    app.processEvents()

    assert errors[-1] == "camera repeatedly opened but did not return usable frames"
    assert fake.open_count == camera_worker.MAX_FRAME_RECOVERY_CYCLES
    assert statuses[-1]["connected"] is False
    assert statuses[-1]["recovering"] is False
    assert statuses[-1]["error"] == errors[-1]
