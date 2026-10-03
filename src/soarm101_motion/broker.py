"""Narrow HTTP broker for bounded SO-ARM101 agent capabilities.

The broker is intentionally a transport boundary, not a second robot-control stack.
Every physical action is serialized and delegated to the existing bounded agent CLI,
so calibration, authority leases, motion validation, hold semantics, and camera policy
remain owned by the Motion SDK.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import hmac
import json
import os
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Mapping, Sequence


DEFAULT_BROKER_HOST = "127.0.0.1"
DEFAULT_BROKER_PORT = 8765
DEFAULT_EVENT_PATH = Path.home() / ".local" / "state" / "soarm101" / "broker" / "events.jsonl"


class BrokerCommandError(RuntimeError):
    """A bounded agent command was rejected or failed."""


@dataclass(frozen=True)
class BrokerResponse:
    status: int
    body: dict[str, object]


class AgentCommandExecutor:
    """Invoke only the bounded agent command namespace."""

    def __init__(self, *, robot_id: str = "so101") -> None:
        self.robot_id = str(robot_id)

    def run(self, arguments: Sequence[str]) -> dict[str, object]:
        command = [
            sys.executable,
            "-m",
            "soarm101_motion.cli.main",
            "agent",
            *arguments,
        ]
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=120.0,
            )
        except subprocess.TimeoutExpired as exc:
            raise BrokerCommandError(
                "bounded agent command exceeded the 120 second broker timeout"
            ) from exc
        stdout = completed.stdout.strip()
        stderr = completed.stderr.strip()
        if completed.returncode != 0:
            detail = stderr or stdout or f"agent command exited {completed.returncode}"
            raise BrokerCommandError(detail)
        if not stdout:
            return {}
        try:
            payload = json.loads(stdout)
        except json.JSONDecodeError as exc:
            raise BrokerCommandError(
                f"bounded agent command returned non-JSON output: {stdout[:200]!r}"
            ) from exc
        if not isinstance(payload, dict):
            raise BrokerCommandError("bounded agent command JSON root must be an object")
        return payload


class RobotBrokerService:
    """Serialize and audit a narrow set of agent-safe robot/camera requests."""

    def __init__(
        self,
        *,
        executor: AgentCommandExecutor | None = None,
        token: str,
        event_path: str | Path = DEFAULT_EVENT_PATH,
    ) -> None:
        if not str(token):
            raise ValueError("broker token cannot be empty")
        self.executor = executor or AgentCommandExecutor()
        self.token = str(token)
        self.event_path = Path(event_path).expanduser()
        self._operation_lock = threading.Lock()
        self._event_lock = threading.Lock()

    def authorized(self, authorization: str | None) -> bool:
        if authorization is None:
            return False
        return hmac.compare_digest(authorization, f"Bearer {self.token}")

    @staticmethod
    def _number(payload: Mapping[str, object], key: str, default: float = 0.0) -> float:
        raw = payload.get(key, default)
        if isinstance(raw, bool):
            raise ValueError(f"{key} must be numeric")
        value = float(raw)
        if not (float("-inf") < value < float("inf")):
            raise ValueError(f"{key} must be finite")
        return value

    @staticmethod
    def _text(payload: Mapping[str, object], key: str) -> str:
        value = str(payload.get(key, "")).strip()
        if not value:
            raise ValueError(f"{key} is required")
        return value

    def _record(
        self,
        *,
        request_id: str,
        action: str,
        request: Mapping[str, object],
        ok: bool,
        duration_s: float,
        result: Mapping[str, object] | None = None,
        error: str | None = None,
    ) -> None:
        event = {
            "schema_version": 1,
            "request_id": request_id,
            "timestamp": time.time(),
            "action": action,
            "request": dict(request),
            "ok": ok,
            "duration_s": duration_s,
            "result": dict(result or {}),
            "error": error,
        }
        self.event_path.parent.mkdir(parents=True, exist_ok=True)
        line = json.dumps(event, sort_keys=True) + "\n"
        with self._event_lock:
            with self.event_path.open("a", encoding="utf-8") as handle:
                handle.write(line)

    def _execute(
        self,
        *,
        request_id: str,
        action: str,
        request: Mapping[str, object],
        arguments: Sequence[str],
    ) -> BrokerResponse:
        started = time.monotonic()
        try:
            with self._operation_lock:
                result = self.executor.run(arguments)
        except (BrokerCommandError, ValueError, KeyError, PermissionError) as exc:
            duration = time.monotonic() - started
            self._record(
                request_id=request_id,
                action=action,
                request=request,
                ok=False,
                duration_s=duration,
                error=str(exc),
            )
            return BrokerResponse(
                status=HTTPStatus.CONFLICT,
                body={"ok": False, "request_id": request_id, "error": str(exc)},
            )
        duration = time.monotonic() - started
        self._record(
            request_id=request_id,
            action=action,
            request=request,
            ok=True,
            duration_s=duration,
            result=result,
        )
        return BrokerResponse(
            status=HTTPStatus.OK,
            body={"ok": True, "request_id": request_id, "result": result},
        )

    def dispatch(
        self,
        method: str,
        path: str,
        payload: Mapping[str, object] | None = None,
    ) -> BrokerResponse:
        method = method.upper()
        request = dict(payload or {})
        request_id = uuid.uuid4().hex

        if method == "GET" and path == "/v1/health":
            return BrokerResponse(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "request_id": request_id,
                    "result": {"service": "soarm101-agent-broker", "status": "ok"},
                },
            )
        if method == "GET" and path == "/v1/capabilities":
            return self._execute(
                request_id=request_id,
                action="capabilities",
                request=request,
                arguments=["capabilities", "--robot-id", self.executor.robot_id],
            )
        if method == "GET" and path == "/v1/state":
            return self._execute(
                request_id=request_id,
                action="state",
                request=request,
                arguments=["state", "--robot-id", self.executor.robot_id],
            )
        if method != "POST":
            return BrokerResponse(
                HTTPStatus.NOT_FOUND,
                {"ok": False, "request_id": request_id, "error": "unknown endpoint"},
            )

        try:
            if path == "/v1/capture":
                name = self._text(request, "camera")
                started = time.monotonic()
                try:
                    # Keep capture, host-file read, and evidence hashing in one serialized
                    # operation. Otherwise a second capture could race the first request's
                    # evidence read after the device command returns.
                    with self._operation_lock:
                        result = self.executor.run(["capture", name])
                        capture_path = Path(str(result["path"])).expanduser()
                        image_bytes = capture_path.read_bytes()
                        image_sha256 = hashlib.sha256(image_bytes).hexdigest()
                except (BrokerCommandError, ValueError, KeyError, OSError) as exc:
                    duration = time.monotonic() - started
                    self._record(
                        request_id=request_id,
                        action="capture",
                        request=request,
                        ok=False,
                        duration_s=duration,
                        error=str(exc),
                    )
                    return BrokerResponse(
                        status=HTTPStatus.CONFLICT,
                        body={"ok": False, "request_id": request_id, "error": str(exc)},
                    )

                duration = time.monotonic() - started
                capture_metadata = {
                    key: value
                    for key, value in result.items()
                    if key != "path"
                }
                capture_metadata["sha256"] = image_sha256
                self._record(
                    request_id=request_id,
                    action="capture",
                    request=request,
                    ok=True,
                    duration_s=duration,
                    result=capture_metadata,
                )
                self._record(
                    request_id=request_id,
                    action="capture_evidence",
                    request={"camera": name},
                    ok=True,
                    duration_s=0.0,
                    result={
                        **capture_metadata,
                        "host_path": str(capture_path),
                    },
                )
                return BrokerResponse(
                    status=HTTPStatus.OK,
                    body={
                        "ok": True,
                        "request_id": request_id,
                        "result": {
                            **capture_metadata,
                            "image_base64": base64.b64encode(image_bytes).decode("ascii"),
                        },
                    },
                )

            if path == "/v1/go-pose":
                name = self._text(request, "name")
                return self._execute(
                    request_id=request_id,
                    action="go_pose",
                    request=request,
                    arguments=["go-pose", name, "--robot-id", self.executor.robot_id],
                )

            if path == "/v1/joint":
                joint = self._text(request, "joint")
                delta = self._number(request, "delta_deg")
                return self._execute(
                    request_id=request_id,
                    action="joint",
                    request=request,
                    arguments=[
                        "joint",
                        joint,
                        "--delta-deg",
                        f"{delta:g}",
                        "--robot-id",
                        self.executor.robot_id,
                    ],
                )

            if path == "/v1/jog":
                frame = str(request.get("frame", "world")).strip()
                if frame not in {"world", "tool"}:
                    raise ValueError("frame must be 'world' or 'tool'")
                x = self._number(request, "x_mm")
                y = self._number(request, "y_mm")
                z = self._number(request, "z_mm")
                return self._execute(
                    request_id=request_id,
                    action="jog",
                    request=request,
                    arguments=[
                        "jog",
                        "--frame",
                        frame,
                        "--x-mm",
                        f"{x:g}",
                        "--y-mm",
                        f"{y:g}",
                        "--z-mm",
                        f"{z:g}",
                        "--robot-id",
                        self.executor.robot_id,
                    ],
                )

            if path == "/v1/gripper":
                target = self._text(request, "target")
                if target not in {"open", "close"}:
                    raise ValueError("gripper target must be 'open' or 'close'")
                return self._execute(
                    request_id=request_id,
                    action="gripper",
                    request=request,
                    arguments=["gripper", target, "--robot-id", self.executor.robot_id],
                )

            if path == "/v1/sleep":
                return self._execute(
                    request_id=request_id,
                    action="sleep",
                    request=request,
                    arguments=["sleep", "--robot-id", self.executor.robot_id],
                )

            if path == "/v1/stop":
                return self._execute(
                    request_id=request_id,
                    action="stop",
                    request=request,
                    arguments=["stop", "--robot-id", self.executor.robot_id],
                )
        except (BrokerCommandError, ValueError, KeyError, OSError) as exc:
            self._record(
                request_id=request_id,
                action=path.removeprefix("/v1/") or "unknown",
                request=request,
                ok=False,
                duration_s=0.0,
                error=str(exc),
            )
            return BrokerResponse(
                HTTPStatus.BAD_REQUEST,
                {"ok": False, "request_id": request_id, "error": str(exc)},
            )

        return BrokerResponse(
            HTTPStatus.NOT_FOUND,
            {"ok": False, "request_id": request_id, "error": "unknown endpoint"},
        )


class _BrokerHandler(BaseHTTPRequestHandler):
    server_version = "SOARM101AgentBroker/1"

    @property
    def service(self) -> RobotBrokerService:
        return self.server.service  # type: ignore[attr-defined,no-any-return]

    def _send(self, response: BrokerResponse) -> None:
        data = json.dumps(response.body).encode("utf-8")
        self.send_response(int(response.status))
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _authorized(self) -> bool:
        if self.service.authorized(self.headers.get("Authorization")):
            return True
        self._send(BrokerResponse(HTTPStatus.UNAUTHORIZED, {"ok": False, "error": "unauthorized"}))
        return False

    def _payload(self) -> dict[str, object]:
        length = int(self.headers.get("Content-Length", "0"))
        if length == 0:
            return {}
        if length > 65536:
            raise ValueError("request body exceeds 64 KiB")
        raw = self.rfile.read(length)
        parsed = json.loads(raw.decode("utf-8"))
        if not isinstance(parsed, dict):
            raise ValueError("request JSON root must be an object")
        return parsed

    def do_GET(self) -> None:  # noqa: N802
        if self._authorized():
            self._send(self.service.dispatch("GET", self.path))

    def do_POST(self) -> None:  # noqa: N802
        if not self._authorized():
            return
        try:
            payload = self._payload()
        except (ValueError, json.JSONDecodeError) as exc:
            self._send(BrokerResponse(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)}))
            return
        self._send(self.service.dispatch("POST", self.path, payload))

    def log_message(self, format: str, *args: object) -> None:
        del format, args


class RobotBrokerHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], service: RobotBrokerService) -> None:
        self.service = service
        super().__init__(address, _BrokerHandler)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_BROKER_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_BROKER_PORT)
    parser.add_argument("--robot-id", default="so101")
    parser.add_argument("--events", type=Path, default=DEFAULT_EVENT_PATH)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    token = os.environ.get("SOARM101_BROKER_TOKEN", "").strip()
    if not token:
        raise SystemExit(
            "SOARM101_BROKER_TOKEN is required; generate a per-run token before starting the broker"
        )
    service = RobotBrokerService(
        executor=AgentCommandExecutor(robot_id=args.robot_id),
        token=token,
        event_path=args.events,
    )
    server = RobotBrokerHTTPServer((args.host, args.port), service)
    print(
        f"SO-ARM101 agent broker listening on http://{args.host}:{args.port}; "
        "human arming remains external via 'soarm101 agent arm'"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
