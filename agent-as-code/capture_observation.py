#!/usr/bin/env python3
"""Capture named workstation cameras through the public soarm101 CLI."""

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
    names = payload.get("camera_names", [])
    if not isinstance(names, list) or not all(
        isinstance(name, str) and name.strip() for name in names
    ):
        raise ValueError("setup.camera_names must be a list of non-empty camera names")
    if len(set(names)) != len(names):
        raise ValueError("setup.camera_names must not contain duplicates")
    return payload


def _slug(text: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "-", text.strip()).strip("-")
    return value or "observation"


def _run_json(command: list[str]) -> dict[str, Any]:
    completed = subprocess.run(command, check=True, capture_output=True, text=True)
    payload = json.loads(completed.stdout)
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object from {' '.join(command)}")
    return payload


def _workstation() -> dict[str, Any]:
    return _run_json(["soarm101", "workstation", "show", "--json"])


def _resolved_camera_names(setup: dict[str, Any], workstation: dict[str, Any]) -> list[str]:
    configured = workstation.get("cameras", {})
    if not isinstance(configured, dict):
        raise ValueError("workstation cameras must be a JSON object")
    requested = [str(name).strip() for name in setup.get("camera_names", [])]
    names = requested or list(configured)
    missing = [name for name in names if name not in configured]
    if missing:
        raise ValueError(
            "unknown workstation camera profile(s): " + ", ".join(missing)
        )
    if not names:
        raise ValueError("no workstation camera profiles are configured")
    return names


def _capture(setup: dict[str, Any], setup_path: Path, label: str) -> Path:
    workstation = _workstation()
    names = _resolved_camera_names(setup, workstation)

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
        "follower": workstation.get("follower", {}),
        "leader": workstation.get("leader", {}),
        "cameras": [],
    }

    for name in names:
        output = observation_dir / f"{_slug(name)}.jpg"
        metadata = _run_json(
            [
                "soarm101",
                "camera",
                "capture",
                "--name",
                name,
                "--output",
                str(output),
                "--json",
            ]
        )
        manifest["cameras"].append(metadata)

    manifest_path = observation_dir / "observation.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest_path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture named cameras from the shared SO-ARM101 workstation profile"
    )
    parser.add_argument(
        "--setup",
        type=Path,
        default=Path(__file__).with_name("setup.local.json"),
        help="experiment-local JSON; hardware addressing comes from workstation.json",
    )
    parser.add_argument("--label", default="observation")
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate requested camera names against the workstation profile",
    )
    args = parser.parse_args()

    setup = _load_setup(args.setup)
    workstation = _workstation()
    names = _resolved_camera_names(setup, workstation)
    if args.check:
        print(
            json.dumps(
                {
                    "valid": True,
                    "cameras": names,
                    "selected_camera": workstation.get("selected_camera"),
                    "follower": workstation.get("follower", {}),
                    "leader": workstation.get("leader", {}),
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
