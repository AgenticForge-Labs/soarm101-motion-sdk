"""Minimal sandbox-side client for the SO-ARM101 agent broker.

This module intentionally depends only on the Python standard library so a benchmark
sandbox can copy this single file without installing the Motion SDK.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import hmac
import json
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Mapping


DEFAULT_BROKER_URL = "http://127.0.0.1:8765"


def _request(
    *,
    method: str,
    path: str,
    payload: Mapping[str, object] | None = None,
) -> dict[str, object]:
    base = os.environ.get("SOARM101_BROKER_URL", DEFAULT_BROKER_URL).rstrip("/")
    token = os.environ.get("SOARM101_BROKER_TOKEN", "").strip()
    if not token:
        raise RuntimeError("SOARM101_BROKER_TOKEN is required")
    data = None if payload is None else json.dumps(dict(payload)).encode("utf-8")
    request = urllib.request.Request(
        f"{base}{path}",
        data=data,
        method=method,
        headers={
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=120.0) as response:
            body = response.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"broker HTTP {exc.code}: {detail}") from exc
    parsed = json.loads(body)
    if not isinstance(parsed, dict):
        raise RuntimeError("broker response root must be an object")
    if not parsed.get("ok", False):
        raise RuntimeError(str(parsed.get("error", "broker request failed")))
    return parsed


def _print_result(response: dict[str, object]) -> None:
    print(json.dumps(response, indent=2))


def _capture(args: argparse.Namespace) -> int:
    response = _request(
        method="POST",
        path="/v1/capture",
        payload={"camera": args.camera},
    )
    result = response.get("result")
    if not isinstance(result, dict):
        raise RuntimeError("capture response is missing result")
    encoded = result.pop("image_base64", None)
    if not isinstance(encoded, str):
        raise RuntimeError("capture response is missing image bytes")
    image = base64.b64decode(encoded.encode("ascii"), validate=True)
    expected_sha = str(result.get("sha256") or "").strip()
    actual_sha = hashlib.sha256(image).hexdigest()
    if not expected_sha:
        raise RuntimeError("capture response is missing SHA-256")
    if not hmac.compare_digest(actual_sha, expected_sha):
        raise RuntimeError(
            "capture image SHA-256 does not match trusted broker metadata"
        )
    if args.output:
        output = Path(args.output)
    else:
        directory = Path(args.observation_dir)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        request_suffix = str(response.get("request_id") or "")[:12] or f"{time.time_ns():x}"
        output = directory / f"{args.camera}-{stamp}-{request_suffix}.jpg"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(image)
    result["sandbox_path"] = str(output)
    _print_result(response)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("health")
    sub.add_parser("capabilities")
    sub.add_parser("state")

    capture = sub.add_parser("capture")
    capture.add_argument("camera", choices=("overhead", "wrist"))
    capture.add_argument("--output")
    capture.add_argument("--observation-dir", default="observations")

    pose = sub.add_parser("go-pose")
    pose.add_argument("name")

    joint = sub.add_parser("joint")
    joint.add_argument(
        "joint",
        choices=("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"),
    )
    joint.add_argument("--delta-deg", type=float, required=True)

    jog = sub.add_parser("jog")
    jog.add_argument("--frame", choices=("world", "tool"), default="world")
    jog.add_argument("--x-mm", type=float, default=0.0)
    jog.add_argument("--y-mm", type=float, default=0.0)
    jog.add_argument("--z-mm", type=float, default=0.0)

    gripper = sub.add_parser("gripper")
    gripper.add_argument("target", choices=("open", "close"))

    sub.add_parser("sleep")
    sub.add_parser("stop")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "health":
            _print_result(_request(method="GET", path="/v1/health"))
        elif args.command == "capabilities":
            _print_result(_request(method="GET", path="/v1/capabilities"))
        elif args.command == "state":
            _print_result(_request(method="GET", path="/v1/state"))
        elif args.command == "capture":
            return _capture(args)
        elif args.command == "go-pose":
            _print_result(
                _request(
                    method="POST",
                    path="/v1/go-pose",
                    payload={"name": args.name},
                )
            )
        elif args.command == "joint":
            _print_result(
                _request(
                    method="POST",
                    path="/v1/joint",
                    payload={"joint": args.joint, "delta_deg": args.delta_deg},
                )
            )
        elif args.command == "jog":
            _print_result(
                _request(
                    method="POST",
                    path="/v1/jog",
                    payload={
                        "frame": args.frame,
                        "x_mm": args.x_mm,
                        "y_mm": args.y_mm,
                        "z_mm": args.z_mm,
                    },
                )
            )
        elif args.command == "gripper":
            _print_result(
                _request(
                    method="POST",
                    path="/v1/gripper",
                    payload={"target": args.target},
                )
            )
        elif args.command == "sleep":
            _print_result(_request(method="POST", path="/v1/sleep", payload={}))
        elif args.command == "stop":
            _print_result(_request(method="POST", path="/v1/stop", payload={}))
        else:
            raise RuntimeError(f"unsupported command {args.command!r}")
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
