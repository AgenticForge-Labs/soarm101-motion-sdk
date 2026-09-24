"""Persistent, opt-out JSONL audit log for GUI sessions."""

from __future__ import annotations

import json
import logging
import os
import atexit
from logging.handlers import QueueHandler, QueueListener
from queue import SimpleQueue
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_logger = logging.getLogger("soarm101_motion.gui.session")
_path: Path | None = None
_listener: QueueListener | None = None


def current_path() -> Path | None:
    return _path


def close() -> None:
    global _listener
    if _listener is not None:
        _listener.stop()
        for handler in _listener.handlers:
            handler.close()
        _listener = None
    for handler in tuple(_logger.handlers):
        _logger.removeHandler(handler)
        handler.close()


atexit.register(close)


def configure(*, enabled: bool = True, directory: Path | None = None) -> Path | None:
    global _path, _listener
    close()
    _path = None
    if not enabled:
        return None
    folder = directory or Path.home() / ".local" / "state" / "soarm101" / "gui"
    folder.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    _path = folder / f"session-{stamp}-{os.getpid()}.jsonl"
    handler = logging.FileHandler(_path, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(message)s"))
    queue: SimpleQueue[logging.LogRecord] = SimpleQueue()
    _listener = QueueListener(queue, handler)
    _listener.start()
    _logger.addHandler(QueueHandler(queue))
    _logger.setLevel(logging.INFO)
    _logger.propagate = False
    record("session_start", pid=os.getpid())
    return _path


def record(event: str, **fields: Any) -> None:
    if _path is None:
        return
    payload = {
        "time_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "event": event,
        **fields,
    }
    _logger.info(json.dumps(payload, default=str, ensure_ascii=False))
