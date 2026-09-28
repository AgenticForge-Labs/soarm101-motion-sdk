"""Shared USB-camera capture, settings, and still-image helpers.

The camera layer is intentionally perception-free. It owns device selection, capture
configuration, fresh-frame acquisition, and snapshot persistence so GUI, CLI, and
higher-level agents can consume the same deterministic camera surface.
"""

from __future__ import annotations

import json
import os
import sys
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any


DEFAULT_CAMERA_CONFIG_PATH = Path.home() / ".config" / "soarm101" / "camera.json"
DEFAULT_CAPTURE_DIR = Path.home() / ".local" / "share" / "soarm101" / "captures"


def _opencv() -> Any:
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "Camera support requires OpenCV. Install with: pip install -e '.[camera]'"
        ) from exc
    return cv2


@dataclass(frozen=True)
class CameraSettings:
    """Persisted settings shared by CLI and GUI camera consumers."""

    device: str = "/dev/video0" if sys.platform.startswith("linux") else "0"
    width: int = 1280
    height: int = 720
    fps: float = 30.0
    fourcc: str = "MJPG"
    mirror: bool = False
    auto_start: bool = False
    snapshot_dir: str = str(DEFAULT_CAPTURE_DIR)

    def validated(self) -> "CameraSettings":
        if not str(self.device).strip():
            raise ValueError("camera device cannot be empty")
        if int(self.width) <= 0 or int(self.height) <= 0:
            raise ValueError("camera width and height must be positive")
        if float(self.fps) <= 0:
            raise ValueError("camera fps must be positive")
        fourcc = str(self.fourcc).strip().upper()
        if len(fourcc) != 4:
            raise ValueError("camera fourcc must contain exactly four characters")
        return replace(
            self,
            device=str(self.device).strip(),
            width=int(self.width),
            height=int(self.height),
            fps=float(self.fps),
            fourcc=fourcc,
            snapshot_dir=str(Path(self.snapshot_dir).expanduser()),
        )

    def with_overrides(self, **values: object) -> "CameraSettings":
        return replace(self, **values).validated()


class CameraSettingsStore:
    def __init__(self, path: str | Path | None = None) -> None:
        self.path = Path(path).expanduser() if path is not None else DEFAULT_CAMERA_CONFIG_PATH

    def load(self) -> CameraSettings:
        if not self.path.exists():
            return CameraSettings().validated()
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        known = {field.name for field in __import__("dataclasses").fields(CameraSettings)}
        values = {key: value for key, value in payload.items() if key in known}
        return CameraSettings(**values).validated()

    def save(self, settings: CameraSettings) -> CameraSettings:
        settings = settings.validated()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(json.dumps(asdict(settings), indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, self.path)
        return settings


def discover_camera_devices() -> list[str]:
    """Return likely local camera device identifiers without opening robot hardware."""

    if sys.platform.startswith("linux"):
        devices = sorted(
            (path for path in Path("/dev").glob("video*") if path.is_char_device()),
            key=lambda path: path.name,
        )
        return [str(path) for path in devices]

    cv2 = _opencv()
    devices: list[str] = []
    for index in range(10):
        capture = cv2.VideoCapture(index)
        try:
            if capture.isOpened():
                devices.append(str(index))
        finally:
            capture.release()
    return devices


def _device_value(device: str) -> str | int:
    text = str(device).strip()
    if text.isdigit():
        return int(text)
    return text


class CameraCapture:
    """One-owner OpenCV camera session used by GUI workers and CLI commands."""

    def __init__(self, settings: CameraSettings) -> None:
        self.settings = settings.validated()
        self._capture: Any | None = None

    @property
    def is_open(self) -> bool:
        return bool(self._capture is not None and self._capture.isOpened())

    def open(self) -> "CameraCapture":
        cv2 = _opencv()
        device = _device_value(self.settings.device)
        if sys.platform.startswith("linux") and hasattr(cv2, "CAP_V4L2"):
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        else:
            capture = cv2.VideoCapture(device)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"could not open camera {self.settings.device!r}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.settings.width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.settings.height))
        capture.set(cv2.CAP_PROP_FPS, float(self.settings.fps))
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.set(
            cv2.CAP_PROP_FOURCC,
            float(cv2.VideoWriter_fourcc(*self.settings.fourcc)),
        )
        self._capture = capture
        return self

    def close(self) -> None:
        capture, self._capture = self._capture, None
        if capture is not None:
            capture.release()

    def __enter__(self) -> "CameraCapture":
        return self.open()

    def __exit__(self, *_: object) -> None:
        self.close()

    def read_bgr(self) -> Any:
        if not self.is_open:
            raise RuntimeError("camera is not open")
        ok, frame = self._capture.read()
        if not ok or frame is None:
            raise RuntimeError("camera did not return a frame")
        if self.settings.mirror:
            frame = _opencv().flip(frame, 1)
        return frame

    def actual_format(self) -> dict[str, object]:
        if not self.is_open:
            return {}
        cv2 = _opencv()
        return {
            "device": self.settings.device,
            "width": int(round(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))),
            "height": int(round(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))),
            "fps": float(self._capture.get(cv2.CAP_PROP_FPS)),
            "fourcc": self.settings.fourcc,
            "mirror": self.settings.mirror,
        }

    def save_frame(self, frame: Any, output: str | Path | None = None) -> Path:
        if output is None:
            directory = Path(self.settings.snapshot_dir).expanduser()
            stamp = time.strftime("%Y%m%d-%H%M%S")
            millis = int((time.time() % 1) * 1000)
            output_path = directory / f"camera-{stamp}-{millis:03d}.jpg"
        else:
            output_path = Path(output).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not _opencv().imwrite(str(output_path), frame):
            raise RuntimeError(f"could not write camera frame to {output_path}")
        return output_path

    def snapshot(self, output: str | Path | None = None) -> tuple[Path, dict[str, object]]:
        frame = self.read_bgr()
        path = self.save_frame(frame, output)
        height, width = frame.shape[:2]
        return path, {
            "path": str(path),
            "timestamp": time.time(),
            "width": int(width),
            "height": int(height),
            "device": self.settings.device,
        }
