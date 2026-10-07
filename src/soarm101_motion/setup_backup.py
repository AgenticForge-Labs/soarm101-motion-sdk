"""Portable, validated setup backups. Never restore paths supplied by an archive."""

from __future__ import annotations

import json
import os
import re
import tempfile
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath

from soarm101_motion.calibration import SO101Calibration
from soarm101_motion.workstation import WorkstationProfileStore, default_workstation_profile_path
from soarm101_motion.workspace import WorkspaceCalibration


def setup_root() -> Path:
    return Path.home() / ".config" / "soarm101"


def _validate(name: str, payload: object) -> None:
    parts = PurePosixPath(name).parts
    if not parts or any(not re.fullmatch(r"[A-Za-z0-9_.-]+", p) or p in (".", "..") for p in parts):
        raise ValueError(f"Invalid backup path: {name}")
    if not isinstance(payload, dict):
        raise ValueError(f"Expected JSON object: {name}")
    if (
        parts[0] == "calibration"
        and (len(parts) == 2 or (len(parts) == 4 and parts[1] == "history"))
        and name.endswith(".json")
    ):
        calibration = SO101Calibration.from_mapping(payload)
        calibration.validate()
        if (
            payload.get("calibration_id") is not None
            and payload["calibration_id"] != calibration.calibration_id
        ):
            raise ValueError(f"Calibration fingerprint mismatch: {name}")
        if len(parts) == 4 and Path(parts[-1]).stem != calibration.fingerprint:
            raise ValueError(f"History fingerprint mismatch: {name}")
    elif parts[0] == "workspace" and len(parts) == 2 and name.endswith(".json"):
        workspace = WorkspaceCalibration(**payload).validated()
        if workspace.robot_id != Path(parts[-1]).stem:
            raise ValueError(f"Workspace identity mismatch: {name}")
    elif name == "workstation.json":
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / name
            path.write_text(json.dumps(payload), encoding="utf-8")
            profile = WorkstationProfileStore(path=path).load()
            for arm in (profile.follower, profile.leader):
                if not re.fullmatch(r"[A-Za-z0-9_-]+", arm.robot_id):
                    raise ValueError("Invalid robot identity in workstation backup")
    else:
        raise ValueError(f"Unsupported backup entry: {name}")


def _atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary:
            temporary.unlink(missing_ok=True)


def export_setup(destination: Path, *, root: Path | None = None) -> int:
    workstation_path = root / "workstation.json" if root else default_workstation_profile_path()
    root = root or setup_root()
    files = {}
    for pattern in (
        "calibration/*.json",
        "calibration/history/*/*.json",
        "workspace/*.json",
        "workstation.json",
    ):
        for path in sorted(root.glob(pattern)):
            if path.is_symlink():
                raise ValueError(f"Refusing symbolic link: {path}")
            name = path.relative_to(root).as_posix()
            payload = json.loads(path.read_text(encoding="utf-8"))
            _validate(name, payload)
            files[name] = payload
    if workstation_path.is_file():
        payload = json.loads(workstation_path.read_text(encoding="utf-8"))
        _validate("workstation.json", payload)
        files["workstation.json"] = payload
    if not files:
        raise ValueError("No saved setup files found")
    # Explicit external references must not silently produce an incomplete backup.
    if "workstation.json" in files:
        for role in ("follower", "leader"):
            arm = files["workstation.json"].get(role, {})
            reference = arm.get("calibration")
            if reference and Path(reference).expanduser().exists():
                path = Path(reference).expanduser()
                name = f"calibration/{arm['robot_id']}.json"
                payload = json.loads(path.read_text(encoding="utf-8"))
                _validate(name, payload)
                files[name] = payload
    if any((parent / ".git").exists() for parent in destination.resolve().parents):
        raise ValueError("Choose a backup destination outside a Git repository")
    if destination.resolve().is_relative_to(root.resolve()):
        raise ValueError("Choose a backup destination outside the live setup folder")
    _atomic_write(
        destination,
        json.dumps({"format": "soarm101-setup", "version": 1, "files": files}, indent=2).encode(),
    )
    return len(files)


def inspect_backup(source: Path) -> dict:
    if source.stat().st_size > 20_000_000:
        raise ValueError("Setup backup exceeds 20 MB")
    bundle = json.loads(source.read_text(encoding="utf-8"))
    if (
        not isinstance(bundle, dict)
        or bundle.get("format") != "soarm101-setup"
        or bundle.get("version") != 1
    ):
        raise ValueError("Unsupported setup backup")
    files = bundle.get("files")
    if not isinstance(files, dict) or not files or len(files) > 10000:
        raise ValueError("Invalid backup contents")
    for name, payload in files.items():
        _validate(name, payload)
    return files


def restore_setup(source: Path, *, root: Path | None = None) -> Path:
    """Validate everything, retain originals, then install with rollback on failure."""
    workstation_path = root / "workstation.json" if root else default_workstation_profile_path()
    root = root or setup_root()
    files = inspect_backup(source)
    if "workstation.json" in files:
        # Calibration paths belong to the destination machine, never the source machine.
        for role in ("follower", "leader"):
            arm = files["workstation.json"].get(role)
            if arm:
                arm["calibration"] = str(root / "calibration" / f"{arm['robot_id']}.json")
    targets = {
        (workstation_path if name == "workstation.json" else root / name): json.dumps(
            payload, indent=2
        ).encode()
        for name, payload in files.items()
    }
    for path in targets:
        if path != workstation_path and not path.resolve().is_relative_to(root.resolve()):
            raise ValueError("Restore destination escapes setup folder")
        if path.is_symlink():
            raise ValueError("Refusing to replace a symbolic link")
    for name, payload in files.items():
        if name.startswith("calibration/history/") and (root / name).exists():
            existing = json.loads((root / name).read_text(encoding="utf-8"))
            _validate(name, existing)
    for name in files:
        if name.startswith("calibration/history/") and (root / name).exists():
            targets.pop(root / name, None)
    originals = {path: path.read_bytes() if path.exists() else None for path in targets}
    recovery = root / "restore-history" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    for path, data in originals.items():
        if data is not None:
            _atomic_write(
                recovery
                / ("workstation.json" if path == workstation_path else path.relative_to(root)),
                data,
            )
    written = []
    try:
        for path, data in targets.items():
            _atomic_write(path, data)
            written.append(path)
    except Exception:
        for path in reversed(written):
            previous = originals[path]
            if previous is None:
                path.unlink(missing_ok=True)
            else:
                _atomic_write(path, previous)
        raise
    return recovery
