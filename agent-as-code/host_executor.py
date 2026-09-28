#!/usr/bin/env python3
"""Authenticated host bridge from OpenShell agents to the guarded SO-ARM101 CLI."""

from __future__ import annotations

import argparse
import base64
import json
import math
import secrets
import subprocess
import sys
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Mapping

import capture_observation


DEFAULT_PORT = 8765
MAX_REQUEST_BYTES = 64 * 1024


class RequestError(ValueError):
    """Client request is invalid and should be reported as HTTP 400."""


@dataclass(frozen=True)
class Limits:
    max_jog_mm: float = 10.0
    max_jog_deg: float = 5.0
    max_speed_mm_s: float = 10.0
    max_acceleration_mm_s2: float = 40.0
    max_observation_bytes: int = 25_000_000
    command_timeout_seconds: float = 180.0


def _positive_number(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise RequestError(f"{name} must be a number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise RequestError(f"{name} must be a number") from exc
    if not math.isfinite(number) or number <= 0:
        raise RequestError(f"{name} must be positive and finite")
    return number


def _finite_number(value: object, name: str, default: float = 0.0) -> float:
    if value is None:
        return default
    if isinstance(value, bool):
        raise RequestError(f"{name} must be a number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise RequestError(f"{name} must be a number") from exc
    if not math.isfinite(number):
        raise RequestError(f"{name} must be finite")
    return number


def load_setup(path: Path) -> dict[str, Any]:
    setup = capture_observation._load_setup(path)
    robot = setup.get("robot")
    if not isinstance(robot, dict):
        raise ValueError("setup.robot must be an object")
    if not str(robot.get("port", "")).strip():
        raise ValueError("setup.robot.port is required")
    if not str(robot.get("robot_id", "")).strip():
        raise ValueError("setup.robot.robot_id is required")
    return setup


def limits_from_setup(setup: Mapping[str, Any]) -> Limits:
    raw = setup.get("executor", {})
    if not isinstance(raw, Mapping):
        raise ValueError("setup.executor must be an object")
    return Limits(
        max_jog_mm=_positive_number(raw.get("max_jog_mm", 10.0), "max_jog_mm"),
        max_jog_deg=_positive_number(raw.get("max_jog_deg", 5.0), "max_jog_deg"),
        max_speed_mm_s=_positive_number(raw.get("max_speed_mm_s", 10.0), "max_speed_mm_s"),
        max_acceleration_mm_s2=_positive_number(
            raw.get("max_acceleration_mm_s2", 40.0), "max_acceleration_mm_s2"
        ),
        max_observation_bytes=int(
            _positive_number(raw.get("max_observation_bytes", 25_000_000), "max_observation_bytes")
        ),
        command_timeout_seconds=_positive_number(
            raw.get("command_timeout_seconds", 180.0), "command_timeout_seconds"
        ),
    )


def _robot_args(setup: Mapping[str, Any]) -> list[str]:
    robot = setup["robot"]
    args = ["--port", str(robot["port"]), "--robot-id", str(robot["robot_id"])]
    calibration = robot.get("calibration")
    if calibration:
        args.extend(["--calibration", str(calibration)])
    return args


def validate_jog(payload: Mapping[str, Any], limits: Limits) -> dict[str, Any]:
    allowed = {
        "frame",
        "x_mm",
        "y_mm",
        "z_mm",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "orientation_mode",
        "speed_mm_s",
        "acceleration_mm_s2",
    }
    unknown = sorted(set(payload) - allowed)
    if unknown:
        raise RequestError(f"unknown jog fields: {', '.join(unknown)}")

    values = {
        "x_mm": _finite_number(payload.get("x_mm"), "x_mm"),
        "y_mm": _finite_number(payload.get("y_mm"), "y_mm"),
        "z_mm": _finite_number(payload.get("z_mm"), "z_mm"),
        "roll_deg": _finite_number(payload.get("roll_deg"), "roll_deg"),
        "pitch_deg": _finite_number(payload.get("pitch_deg"), "pitch_deg"),
        "yaw_deg": _finite_number(payload.get("yaw_deg"), "yaw_deg"),
    }
    translation = math.sqrt(sum(values[key] ** 2 for key in ("x_mm", "y_mm", "z_mm")))
    rotation = math.sqrt(sum(values[key] ** 2 for key in ("roll_deg", "pitch_deg", "yaw_deg")))
    if translation <= 0 and rotation <= 0:
        raise RequestError("jog must request a non-zero translation or rotation")
    if translation > limits.max_jog_mm + 1e-9:
        raise RequestError(
            f"jog translation {translation:.3f} mm exceeds executor limit "
            f"{limits.max_jog_mm:.3f} mm"
        )
    if rotation > limits.max_jog_deg + 1e-9:
        raise RequestError(
            f"jog rotation {rotation:.3f} deg exceeds executor limit "
            f"{limits.max_jog_deg:.3f} deg"
        )

    frame = str(payload.get("frame", "world"))
    if frame not in {"world", "tool"}:
        raise RequestError("frame must be 'world' or 'tool'")
    orientation = str(payload.get("orientation_mode", "compatible"))
    if orientation not in {"compatible", "position_only", "exact"}:
        raise RequestError("orientation_mode must be compatible, position_only, or exact")

    speed = _positive_number(payload.get("speed_mm_s", limits.max_speed_mm_s), "speed_mm_s")
    acceleration = _positive_number(
        payload.get("acceleration_mm_s2", limits.max_acceleration_mm_s2),
        "acceleration_mm_s2",
    )
    if speed > limits.max_speed_mm_s + 1e-9:
        raise RequestError(f"speed_mm_s exceeds executor limit {limits.max_speed_mm_s:g}")
    if acceleration > limits.max_acceleration_mm_s2 + 1e-9:
        raise RequestError(
            f"acceleration_mm_s2 exceeds executor limit {limits.max_acceleration_mm_s2:g}"
        )
    return {
        **values,
        "frame": frame,
        "orientation_mode": orientation,
        "speed_mm_s": speed,
        "acceleration_mm_s2": acceleration,
    }


def validate_gripper(payload: Mapping[str, Any]) -> str:
    unknown = sorted(set(payload) - {"target"})
    if unknown:
        raise RequestError(f"unknown gripper fields: {', '.join(unknown)}")
    if "target" not in payload:
        raise RequestError("gripper target is required")
    target = payload["target"]
    if isinstance(target, str):
        text = target.strip().lower()
        if text in {"open", "close"}:
            return text
        try:
            number = float(text)
        except ValueError as exc:
            raise RequestError("gripper target must be open, close, or a number from 0 to 1") from exc
    elif isinstance(target, (int, float)) and not isinstance(target, bool):
        number = float(target)
    else:
        raise RequestError("gripper target must be open, close, or a number from 0 to 1")
    if not math.isfinite(number) or not 0.0 <= number <= 1.0:
        raise RequestError("numeric gripper target must be between 0 and 1")
    return f"{number:g}"


class RobotExecutor:
    def __init__(self, setup_path: Path, token: str) -> None:
        self.setup_path = setup_path.resolve()
        self.setup = load_setup(self.setup_path)
        self.limits = limits_from_setup(self.setup)
        self.token = token

    def _run(self, args: list[str]) -> dict[str, Any]:
        command = ["soarm101", *args]
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=self.limits.command_timeout_seconds,
        )
        result: dict[str, Any] = {
            "ok": completed.returncode == 0,
            "returncode": completed.returncode,
        }
        if completed.stdout.strip():
            try:
                result["output"] = json.loads(completed.stdout)
            except json.JSONDecodeError:
                result["stdout"] = completed.stdout.strip()
        if completed.stderr.strip():
            result["stderr"] = completed.stderr.strip()
        if completed.returncode != 0:
            raise RuntimeError(json.dumps(result))
        return result

    def health(self) -> dict[str, Any]:
        enabled = [
            camera
            for camera in self.setup["cameras"]
            if isinstance(camera, dict) and camera.get("enabled", True)
        ]
        enabled.sort(key=lambda camera: (not bool(camera.get("primary")), str(camera["name"])))
        return {
            "ok": True,
            "service": "agenticforge-robot-executor",
            "capabilities": ["observe", "state", "diagnose", "jog", "gripper"],
            "robot_id": self.setup["robot"]["robot_id"],
            "primary_camera": next(camera["name"] for camera in enabled if camera.get("primary")),
            "cameras": [camera["name"] for camera in enabled],
            "limits": {
                "max_jog_mm": self.limits.max_jog_mm,
                "max_jog_deg": self.limits.max_jog_deg,
                "max_speed_mm_s": self.limits.max_speed_mm_s,
                "max_acceleration_mm_s2": self.limits.max_acceleration_mm_s2,
            },
        }

    def state(self) -> dict[str, Any]:
        return self._run(["read", *_robot_args(self.setup)])

    def diagnose(self) -> dict[str, Any]:
        return self._run(["diagnose", *_robot_args(self.setup), "--json"])

    def jog(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        values = validate_jog(payload, self.limits)
        args = [
            "jog",
            *_robot_args(self.setup),
            "--frame",
            values["frame"],
            "--x-mm",
            str(values["x_mm"]),
            "--y-mm",
            str(values["y_mm"]),
            "--z-mm",
            str(values["z_mm"]),
            "--roll-deg",
            str(values["roll_deg"]),
            "--pitch-deg",
            str(values["pitch_deg"]),
            "--yaw-deg",
            str(values["yaw_deg"]),
            "--orientation-mode",
            values["orientation_mode"],
            "--speed-mm-s",
            str(values["speed_mm_s"]),
            "--acceleration-mm-s2",
            str(values["acceleration_mm_s2"]),
            "--yes",
        ]
        return self._run(args)

    def gripper(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        target = validate_gripper(payload)
        return self._run(["gripper", *_robot_args(self.setup), target, "--yes"])

    def observe(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        unknown = sorted(set(payload) - {"label"})
        if unknown:
            raise RequestError(f"unknown observe fields: {', '.join(unknown)}")
        label = str(payload.get("label", "observation"))
        manifest_path = capture_observation._capture(self.setup, self.setup_path, label)
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        total = 0
        cameras: list[dict[str, Any]] = []
        for camera in manifest["cameras"]:
            image_path = Path(camera["path"])
            data = image_path.read_bytes()
            total += len(data)
            if total > self.limits.max_observation_bytes:
                raise RuntimeError(
                    f"observation images exceed {self.limits.max_observation_bytes} bytes"
                )
            cameras.append(
                {
                    "name": camera["name"],
                    "primary": camera["primary"],
                    "metadata": camera.get("metadata", {}),
                    "filename": image_path.name,
                    "media_type": "image/jpeg",
                    "data_base64": base64.b64encode(data).decode("ascii"),
                }
            )
        return {
            "ok": True,
            "observation_id": manifest_path.parent.name,
            "label": manifest.get("label", label),
            "timestamp": manifest.get("timestamp"),
            "cameras": cameras,
        }


class ExecutorHTTPServer(ThreadingHTTPServer):
    executor: RobotExecutor


class Handler(BaseHTTPRequestHandler):
    server: ExecutorHTTPServer

    def log_message(self, fmt: str, *args: object) -> None:
        sys.stderr.write("robot-executor: " + (fmt % args) + "\n")

    def _authorized(self) -> bool:
        header = self.headers.get("Authorization", "")
        prefix = "Bearer "
        if not header.startswith(prefix):
            return False
        return secrets.compare_digest(header[len(prefix) :], self.server.executor.token)

    def _send(self, status: int, payload: Mapping[str, Any]) -> None:
        body = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self) -> dict[str, Any]:
        raw_length = self.headers.get("Content-Length", "0")
        try:
            length = int(raw_length)
        except ValueError as exc:
            raise RequestError("invalid Content-Length") from exc
        if length < 0 or length > MAX_REQUEST_BYTES:
            raise RequestError("request body is too large")
        if length == 0:
            return {}
        try:
            payload = json.loads(self.rfile.read(length))
        except json.JSONDecodeError as exc:
            raise RequestError("request body must be JSON") from exc
        if not isinstance(payload, dict):
            raise RequestError("request body must be a JSON object")
        return payload

    def _dispatch(self, method: str) -> None:
        if not self._authorized():
            self._send(HTTPStatus.UNAUTHORIZED, {"ok": False, "error": "unauthorized"})
            return
        try:
            if method == "GET" and self.path == "/v1/health":
                result = self.server.executor.health()
            elif method == "GET" and self.path == "/v1/state":
                result = self.server.executor.state()
            elif method == "GET" and self.path == "/v1/diagnose":
                result = self.server.executor.diagnose()
            elif method == "POST" and self.path == "/v1/observe":
                result = self.server.executor.observe(self._body())
            elif method == "POST" and self.path == "/v1/jog":
                result = self.server.executor.jog(self._body())
            elif method == "POST" and self.path == "/v1/gripper":
                result = self.server.executor.gripper(self._body())
            else:
                self._send(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
                return
            self._send(HTTPStatus.OK, result)
        except RequestError as exc:
            self._send(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
        except subprocess.TimeoutExpired:
            self._send(HTTPStatus.GATEWAY_TIMEOUT, {"ok": False, "error": "robot command timed out"})
        except Exception as exc:
            self._send(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {"ok": False, "error": f"{type(exc).__name__}: {exc}"},
            )

    def do_GET(self) -> None:  # noqa: N802
        self._dispatch("GET")

    def do_POST(self) -> None:  # noqa: N802
        self._dispatch("POST")


def _load_token(path: Path) -> str:
    token = path.read_text(encoding="utf-8").strip()
    if len(token) < 32:
        raise ValueError("executor token is missing or too short")
    return token


def main() -> int:
    parser = argparse.ArgumentParser(description="Host-side constrained robot executor")
    parser.add_argument(
        "--setup",
        type=Path,
        default=Path(__file__).with_name("setup.local.json"),
    )
    parser.add_argument(
        "--token-file",
        type=Path,
        default=Path(__file__).with_name(".secrets") / "robot-executor.token",
    )
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()

    executor = RobotExecutor(args.setup, _load_token(args.token_file))
    server = ExecutorHTTPServer((args.bind_host, args.port), Handler)
    server.executor = executor
    print(
        f"agenticforge robot executor listening on {args.bind_host}:{args.port}",
        flush=True,
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
