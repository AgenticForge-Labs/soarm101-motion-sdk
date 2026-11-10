"""Operator-managed lifecycle for the existing persistent SDK simulation broker."""

from __future__ import annotations

import argparse
import json
import os
import secrets
import signal
import subprocess
import sys
import threading
import fcntl
import shlex
import time
import urllib.error
import urllib.request
from pathlib import Path

from soarm101_motion.broker import (
    AgentMotionRates,
    RobotBrokerHTTPServer,
    RobotBrokerService,
    load_trusted_choreography_catalog,
)
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.config import SOARM101Config
from soarm101_motion.broker import BrokerResponse
from http import HTTPStatus


DEFAULT_STATE_DIR = Path.home() / ".local" / "state" / "soarm101" / "sim-daemon"
DEFAULT_PORT = 8765
WATCHDOG_TIMEOUT_S = 2.0


def _paths(state_dir: Path) -> tuple[Path, Path]:
    return state_dir / "daemon.json", state_dir / "broker.token"


def _trusted_token_path(state_dir: Path) -> Path:
    return state_dir / "trusted-host.token"


def _lock(state_dir: Path):
    """Serialize lifecycle changes; the lock file lives in the private state dir."""
    path = state_dir / "daemon.lock"
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
    fcntl.flock(fd, fcntl.LOCK_EX)
    return os.fdopen(fd, "r+")


def _proc_identity(pid: int) -> tuple[int, str]:
    # Linux proc start ticks prevent a recycled PID from matching old state.
    stat = Path(f"/proc/{pid}/stat").read_text()
    rest = stat[stat.rfind(")") + 2:].split()
    start_ticks = int(rest[19])
    cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode()
    return start_ticks, cmdline


def _owned(record: dict[str, object], state_dir: Path) -> bool:
    try:
        pid = int(record["pid"])
        ticks, cmdline = _proc_identity(pid)
        expected = ["-m", "soarm101_motion.local_sim_daemon", "_serve", "--state-dir",
                    str(state_dir.resolve()), "--port", str(int(record["port"]))]
        return ticks == int(record["start_ticks"]) and all(x in shlex.split(cmdline) for x in expected)
    except (OSError, ValueError, KeyError, IndexError, TypeError):
        return False


def _private_state(state_dir: Path) -> None:
    state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    os.chmod(state_dir, 0o700)
    if state_dir.stat().st_uid != os.getuid():
        raise RuntimeError("simulation daemon state directory is not owned by this user")


def _request(port: int, token: str, path: str, *, method: str = "GET",
             payload: dict[str, object] | None = None, timeout: float = 1.0) -> dict[str, object]:
    data = json.dumps(payload or {}).encode() if method != "GET" else None
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}", data=data, method=method,
        headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=timeout) as response:
        return json.loads(response.read())


def _read_record(state_dir: Path) -> tuple[dict[str, object], str]:
    record_path, token_path = _paths(state_dir)
    record = json.loads(record_path.read_text())
    token = token_path.read_text().strip()
    if not token:
        raise RuntimeError("simulation daemon token file is empty")
    return record, token


def start(state_dir: Path = DEFAULT_STATE_DIR, port: int = DEFAULT_PORT,
          trusted_choreography: Path | None = None) -> int:
    if not 1 <= port <= 65535:
        raise ValueError("port must be in 1..65535")
    state_dir = state_dir.expanduser().resolve()
    _private_state(state_dir)
    with _lock(state_dir):
      return _start_locked(state_dir, port, trusted_choreography)


def _start_locked(state_dir: Path, port: int, trusted_choreography: Path | None = None) -> int:
    record_path, token_path = _paths(state_dir)
    if record_path.exists():
        record, token = _read_record(state_dir)
        if not _owned(record, state_dir):
            raise RuntimeError("simulation daemon state is ambiguous; refusing to replace or signal it")
        _request(int(record["port"]), token, "/v1/health")
        print(f"SO-ARM101 simulation daemon already running (pid {record['pid']}, port {record['port']})")
        return 0
    if token_path.exists() or _trusted_token_path(state_dir).exists():
        raise RuntimeError("simulation daemon has an orphan token; refusing to recreate unknown state")

    token = secrets.token_urlsafe(32)
    fd = os.open(token_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as stream:
        stream.write(token)
    command = [sys.executable, "-m", "soarm101_motion.local_sim_daemon", "_serve",
         "--state-dir", str(state_dir), "--port", str(port)]
    if trusted_choreography is not None:
        catalog = trusted_choreography.expanduser().resolve()
        policy, _programs = load_trusted_choreography_catalog(catalog)
        if policy.allow_physical_trial:
            raise ValueError("local simulation catalog cannot authorize physical trials")
        host_token = secrets.token_urlsafe(32)
        fd = os.open(_trusted_token_path(state_dir), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as stream:
            stream.write(host_token)
        command.extend(["--trusted-choreography", str(catalog)])
    child = subprocess.Popen(
        command,
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True, env=dict(os.environ),
    )
    for _ in range(100):
        try:
            record, saved = _read_record(state_dir)
            if int(record["pid"]) != child.pid or saved != token or not _owned(record, state_dir):
                raise RuntimeError("simulation daemon identity could not be verified")
            _request(port, token, "/v1/health")
            print(f"SO-ARM101 simulation daemon ready (pid {child.pid}, http://127.0.0.1:{port})")
            return 0
        except (OSError, urllib.error.URLError, TimeoutError):
            if child.poll() is not None:
                break
            time.sleep(0.05)
    raise RuntimeError("simulation daemon readiness is unknown; state retained for operator reconciliation")


def status(state_dir: Path = DEFAULT_STATE_DIR) -> int:
    try:
        record, token = _read_record(state_dir.expanduser())
        health = _request(int(record["port"]), token, "/v1/health")
        watchdog = _request(int(record["port"]), token, "/v1/supervisor/watchdog")
        print(json.dumps({"running": True, **record, "health": health.get("result"),
                          "watchdog": watchdog.get("result")}, indent=2))
        return 0
    except Exception:
        print(json.dumps({"running": False}))
        return 1


def stop(state_dir: Path = DEFAULT_STATE_DIR) -> int:
    state_dir = state_dir.expanduser().resolve()
    _private_state(state_dir)
    with _lock(state_dir):
      return _stop_locked(state_dir)


def _stop_locked(state_dir: Path) -> int:
    try:
        record, token = _read_record(state_dir)
    except Exception:
        if _paths(state_dir)[0].exists() or _paths(state_dir)[1].exists():
            raise RuntimeError("simulation daemon state is incomplete; refusing to delete it")
        print("SO-ARM101 simulation daemon is not running")
        return 0
    pid = int(record["pid"])
    if not _owned(record, state_dir):
        raise RuntimeError("simulation daemon PID identity is stale or tampered; refusing to signal or delete state")
    try:
        _request(int(record["port"]), token, "/v1/health")
    except Exception:
        raise RuntimeError("simulation daemon health is unavailable; refusing to signal or delete state")
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        try:
            _request(int(record["port"]), token, "/v1/health", timeout=0.2)
        except Exception:
            break
        time.sleep(0.05)
    else:
        raise RuntimeError("simulation daemon did not stop within 5 seconds")
    _paths(state_dir)[0].unlink(missing_ok=True)
    _paths(state_dir)[1].unlink(missing_ok=True)
    _trusted_token_path(state_dir).unlink(missing_ok=True)
    print("SO-ARM101 simulation daemon stopped; SDK session and watchdog closed")
    return 0


def _serve(state_dir: Path, port: int, trusted_choreography: Path | None = None) -> int:
    record_path, token_path = _paths(state_dir)
    token = token_path.read_text().strip()
    config = SOARM101Config(robot_id="so101", auto_enable_torque=False)
    rates = AgentMotionRates()
    if trusted_choreography is not None:
        # The operator-supplied simulation catalog selects the same bounded
        # rates as a physical daemon; the simulator has no motor authority.
        policy = json.loads(trusted_choreography.read_text())["policy"]
        rates = AgentMotionRates(
            joint_speed_deg_s=policy.get("max_speed_deg_s", rates.joint_speed_deg_s),
            joint_acceleration_deg_s2=policy.get(
                "max_acceleration_deg_s2", rates.joint_acceleration_deg_s2),
        )
    executor = SDKAgentExecutor(
        config=config, rates=rates, simulation=True,
        simulation_actions=True,
    )
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "local-operator-simulation",
        "tools": ["robot_health", "robot_capabilities", "robot_state", "jog_joint", "stop"],
        "cameras": [], "limits": {"max_joint_delta_deg": 2.0},
    })
    class LocalSimulationService(RobotBrokerService):
        """Keep watchdog heartbeat host-owned even though health is queryable."""

        def dispatch(self, method: str, path: str,
                     payload: dict[str, object] | None = None) -> BrokerResponse:
            if method.upper() == "POST" and path == "/v1/supervisor/heartbeat":
                return BrokerResponse(HTTPStatus.FORBIDDEN, {
                    "ok": False, "error": "watchdog heartbeat is host-managed",
                })
            return super().dispatch(method, path, payload)

    choreography_policy = choreography_catalog = trusted_host_token = None
    if trusted_choreography is not None:
        choreography_policy, choreography_catalog = load_trusted_choreography_catalog(trusted_choreography)
        if choreography_policy.allow_physical_trial:
            raise ValueError("local simulation catalog cannot authorize physical trials")
        trusted_host_token = _trusted_token_path(state_dir).read_text(encoding="utf-8").strip()
        if not trusted_host_token or trusted_host_token == token:
            raise RuntimeError("trusted-host token must be distinct from broker token")
    service = LocalSimulationService(
        executor=executor, token=token, profile=profile,
        allow_simulated_sdk_posts=True, watchdog_timeout_s=WATCHDOG_TIMEOUT_S,
        event_path=state_dir / "events.jsonl",
        trusted_host_token=trusted_host_token,
        choreography_policy=choreography_policy,
        choreography_catalog=choreography_catalog,
    )
    server = RobotBrokerHTTPServer(("127.0.0.1", port), service)
    service.watchdog.start()
    # Bind the watchdog to the persistent simulated SDK session before heartbeats.
    if service.dispatch("GET", "/v1/state").status != 200:
        executor.close()
        server.server_close()
        return 1
    service.watchdog.heartbeat()
    heartbeat_done = threading.Event()

    def heartbeat_loop() -> None:
        while not heartbeat_done.wait(WATCHDOG_TIMEOUT_S / 4):
            try:
                service.watchdog.heartbeat()
            except Exception:
                return

    heartbeat = threading.Thread(target=heartbeat_loop, name="local-sim-host-heartbeat", daemon=True)
    heartbeat.start()
    stopping = threading.Event()

    def request_shutdown(_signum: int, _frame: object) -> None:
        if not stopping.is_set():
            stopping.set()
            threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    ticks, _ = _proc_identity(os.getpid())
    record_path.write_text(json.dumps({"pid": os.getpid(), "port": port, "start_ticks": ticks}) + "\n")
    try:
        server.serve_forever(poll_interval=0.1)
    finally:
        stopping.set()
        heartbeat_done.set()
        heartbeat.join(timeout=2)
        if service.watchdog.status["state"] == "watching":
            service._watchdog_trip("local simulation daemon shutting down")
        service.watchdog.close()
        executor.close()
        server.server_close()
        record_path.unlink(missing_ok=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("start", "status", "stop"):
        command = sub.add_parser(name)
        command.add_argument("--state-dir", type=Path, default=DEFAULT_STATE_DIR)
        if name == "start":
            command.add_argument("--port", type=int, default=DEFAULT_PORT)
            command.add_argument("--trusted-choreography", type=Path)
    serve = sub.add_parser("_serve")
    serve.add_argument("--state-dir", type=Path, required=True)
    serve.add_argument("--port", type=int, required=True)
    serve.add_argument("--trusted-choreography", type=Path)
    args = parser.parse_args(argv)
    if args.command == "start":
        return start(args.state_dir, args.port, args.trusted_choreography)
    if args.command == "status":
        return status(args.state_dir)
    if args.command == "stop":
        return stop(args.state_dir)
    return _serve(args.state_dir, args.port, args.trusted_choreography)


if __name__ == "__main__":
    raise SystemExit(main())
