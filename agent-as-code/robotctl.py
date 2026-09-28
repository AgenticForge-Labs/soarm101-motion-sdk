#!/usr/bin/env python3
"""Sandbox-side client for the constrained AgenticForge robot executor."""

from __future__ import annotations

import argparse
import base64
import json
import os
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


DEFAULT_URL = "http://host.openshell.internal:8765"


def _request(method: str, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
    token = os.environ.get("ROBOT_EXECUTOR_TOKEN", "")
    if not token:
        raise RuntimeError("ROBOT_EXECUTOR_TOKEN is not available; attach the robot executor provider")
    base = os.environ.get("ROBOT_EXECUTOR_URL", DEFAULT_URL).rstrip("/")
    data = None
    headers = {"Authorization": f"Bearer {token}", "Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(base + path, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=190) as response:
            result = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"robot executor HTTP {exc.code}: {body.strip()}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"could not reach robot executor at {base}: {exc}") from exc
    if not isinstance(result, dict):
        raise RuntimeError("robot executor returned a non-object response")
    return result


def _observe(args: argparse.Namespace) -> int:
    result = _request("POST", "/v1/observe", {"label": args.label})
    root = Path(args.output_dir).expanduser()
    run_dir = root / str(result["observation_id"])
    run_dir.mkdir(parents=True, exist_ok=False)
    summary: list[dict[str, Any]] = []
    for camera in result["cameras"]:
        path = run_dir / str(camera["filename"])
        path.write_bytes(base64.b64decode(camera["data_base64"], validate=True))
        summary.append(
            {
                "name": camera["name"],
                "primary": bool(camera["primary"]),
                "path": str(path.resolve()),
                "metadata": camera.get("metadata", {}),
            }
        )
    manifest = {
        "observation_id": result["observation_id"],
        "label": result.get("label"),
        "timestamp": result.get("timestamp", time.time()),
        "cameras": summary,
    }
    manifest_path = run_dir / "observation.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"manifest": str(manifest_path.resolve()), "cameras": summary}, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="robotctl",
        description="Constrained SO-ARM101 observation and motion interface",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    health = sub.add_parser("health", help="show available robot capabilities and camera names")
    health.set_defaults(method="GET", path="/v1/health")

    state = sub.add_parser("state", help="read calibrated joints and TCP state")
    state.set_defaults(method="GET", path="/v1/state")

    diagnose = sub.add_parser("diagnose", help="read motor diagnostics")
    diagnose.set_defaults(method="GET", path="/v1/diagnose")

    observe = sub.add_parser("observe", help="capture all configured cameras into local files")
    observe.add_argument("--label", default="observation")
    observe.add_argument("--output-dir", default="/workspace/runs")
    observe.set_defaults(func=_observe)

    jog = sub.add_parser("jog", help="request one bounded relative Cartesian jog")
    jog.add_argument("--frame", choices=("world", "tool"), default="world")
    for axis in ("x", "y", "z"):
        jog.add_argument(f"--{axis}-mm", type=float, default=0.0)
    for axis in ("roll", "pitch", "yaw"):
        jog.add_argument(f"--{axis}-deg", type=float, default=0.0)
    jog.add_argument(
        "--orientation-mode",
        choices=("compatible", "position_only", "exact"),
        default="compatible",
    )
    jog.add_argument("--speed-mm-s", type=float)
    jog.add_argument("--acceleration-mm-s2", type=float)
    jog.set_defaults(method="POST", path="/v1/jog")

    gripper = sub.add_parser("gripper", help="open, close, or position the stock gripper")
    gripper.add_argument("target", help="open, close, or normalized position 0..1")
    gripper.set_defaults(method="POST", path="/v1/gripper")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if hasattr(args, "func"):
        return int(args.func(args))

    if args.command == "jog":
        payload: dict[str, Any] = {
            "frame": args.frame,
            "x_mm": args.x_mm,
            "y_mm": args.y_mm,
            "z_mm": args.z_mm,
            "roll_deg": args.roll_deg,
            "pitch_deg": args.pitch_deg,
            "yaw_deg": args.yaw_deg,
            "orientation_mode": args.orientation_mode,
        }
        if args.speed_mm_s is not None:
            payload["speed_mm_s"] = args.speed_mm_s
        if args.acceleration_mm_s2 is not None:
            payload["acceleration_mm_s2"] = args.acceleration_mm_s2
    elif args.command == "gripper":
        payload = {"target": args.target}
    else:
        payload = None

    result = _request(args.method, args.path, payload)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
