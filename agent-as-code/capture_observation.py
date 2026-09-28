#!/usr/bin/env python3
"""Capture a fresh multi-camera observation through the public soarm101 CLI."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from pathlib import Path
from typing import Any


def _load_setup(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("setup root must be a JSON object")
    cameras = payload.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise ValueError("setup.cameras must contain at least one camera")

    enabled = [camera for camera in cameras if isinstance(camera, dict) and camera.get("enabled", True)]
    if not enabled:
        raise ValueError("at least one camera must be enabled")
    primary = [camera for camera in enabled if camera.get("primary") is True]
    if len(primary) != 1:
        raise ValueError("exactly one enabled camera must have primary=true")

    names: set[str] = set()
    devices: set[str] = set()
    for camera in enabled:
        name = str(camera.get("name", "")).strip()
        device = str(camera.get("device", "")).strip()
        if not name:
            raise ValueError("every enabled camera requires a non-empty name")
        if not device:
            raise ValueError(f"camera {name!r} requires a device")
        if name in names:
            raise ValueError(f"duplicate camera name: {name}")
        if device in devices:
            raise ValueError(f"duplicate enabled camera device: {device}")
        names.add(name)
        devices.add(device)
        for key in ("width", "height"):
            if int(camera.get(key, 0)) <= 0:
                raise ValueError(f"camera {name!r} requires positive {key}")
        if float(camera.get("fps", 0)) <= 0:
            raise ValueError(f"camera {name!r} requires positive fps")
        fourcc = str(camera.get("fourcc", "")).strip()
        if len(fourcc) != 4:
            raise ValueError(f"camera {name!r} fourcc must contain exactly four characters")
    return payload


def _slug(text: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "-", text.strip()).strip("-")
    return value or "observation"


def _camera_command(camera: dict[str, Any], output: Path) -> list[str]:
    command = [
        "soarm101",
        "camera",
        "capture",
        "--device",
        str(camera["device"]),
        "--width",
        str(int(camera["width"])),
        "--height",
        str(int(camera["height"])),
        "--fps",
        str(float(camera["fps"])),
        "--fourcc",
        str(camera["fourcc"]),
        "--output",
        str(output),
        "--json",
    ]
    command.append("--mirror" if bool(camera.get("mirror", False)) else "--no-mirror")
    return command


def _capture(setup: dict[str, Any], setup_path: Path, label: str) -> Path:
    cameras = [
        camera
        for camera in setup["cameras"]
        if isinstance(camera, dict) and camera.get("enabled", True)
    ]
    cameras.sort(key=lambda camera: (not bool(camera.get("primary")), str(camera["name"])))

    run_cfg = setup.get("run", {})
    output_root = Path(str(run_cfg.get("output_dir", "agent-as-code/runs"))).expanduser()
    if not output_root.is_absolute():
        repo_root = setup_path.resolve().parent.parent
        output_root = repo_root / output_root

    stamp = time.strftime("%Y%m%d-%H%M%S")
    observation_dir = output_root / f"{stamp}-{_slug(label)}"
    observation_dir.mkdir(parents=True, exist_ok=False)

    manifest: dict[str, Any] = {
        "timestamp": time.time(),
        "label": label,
        "setup": str(setup_path.resolve()),
        "agent": setup.get("agent", {}),
        "vlm": setup.get("vlm", {}),
        "robot": setup.get("robot", {}),
        "cameras": [],
    }

    for camera in cameras:
        output = observation_dir / f"{_slug(str(camera['name']))}.jpg"
        command = _camera_command(camera, output)
        completed = subprocess.run(command, check=True, capture_output=True, text=True)
        try:
            metadata = json.loads(completed.stdout)
        except json.JSONDecodeError:
            metadata = {"stdout": completed.stdout.strip()}
        manifest["cameras"].append(
            {
                "name": camera["name"],
                "primary": bool(camera.get("primary")),
                "device": camera["device"],
                "path": str(output),
                "metadata": metadata,
            }
        )

    manifest_path = observation_dir / "observation.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest_path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture all configured agent-as-code USB cameras through soarm101 CLI"
    )
    parser.add_argument(
        "--setup",
        type=Path,
        default=Path(__file__).with_name("setup.local.json"),
        help="machine-local JSON setup file",
    )
    parser.add_argument("--label", default="observation")
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate setup and print the enabled camera order without opening hardware",
    )
    args = parser.parse_args()

    setup = _load_setup(args.setup)
    if args.check:
        enabled = [
            camera
            for camera in setup["cameras"]
            if isinstance(camera, dict) and camera.get("enabled", True)
        ]
        enabled.sort(key=lambda camera: (not bool(camera.get("primary")), str(camera["name"])))
        print(
            json.dumps(
                {
                    "valid": True,
                    "primary": next(camera["name"] for camera in enabled if camera.get("primary")),
                    "cameras": [
                        {"name": camera["name"], "device": camera["device"]}
                        for camera in enabled
                    ],
                },
                indent=2,
            )
        )
        return 0

    manifest = _capture(setup, args.setup, args.label)
    print(manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
