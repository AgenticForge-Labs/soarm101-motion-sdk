"""End-to-end lifecycle checks for the operator-managed SDK simulation daemon."""

from __future__ import annotations

import json
import socket
import urllib.error
import urllib.request
import threading
import os
import pytest

from soarm101_motion.local_sim_daemon import start, status, stop


def _port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _call(port: int, token: str, path: str, *, method: str = "GET",
          payload: dict[str, object] | None = None) -> dict[str, object]:
    data = json.dumps(payload or {}).encode() if method != "GET" else None
    request = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}", data=data, method=method,
        headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=3) as response:
        return json.loads(response.read())


def test_simulation_daemon_start_ready_client_and_shutdown(tmp_path) -> None:
    port = _port()
    try:
        assert start(tmp_path, port) == 0
        token = (tmp_path / "broker.token").read_text().strip()
        assert status(tmp_path) == 0
        watchdog = _call(port, token, "/v1/supervisor/watchdog")["result"]
        assert watchdog["state"] == "watching"
        try:
            _call(port, token, "/v1/supervisor/heartbeat", method="POST")
        except urllib.error.HTTPError as exc:
            assert exc.code == 403
        else:
            raise AssertionError("client must not control the host watchdog heartbeat")

        before = _call(port, token, "/v1/state")["result"]
        assert before["calibration_id"] == "simulation"
        moved = _call(port, token, "/v1/joint", method="POST",
                      payload={"joint": "shoulder_pan", "delta_deg": 1.0})["result"]
        after = _call(port, token, "/v1/state")["result"]
        assert before["broker_session"]["id"] == after["broker_session"]["id"]
        assert moved["broker_session"]["id"] == after["broker_session"]["id"]
        assert after["authority"]["armed"] is False
        assert after["broker_session"]["mode"] == "simulation_actions"

        assert stop(tmp_path) == 0
        assert status(tmp_path) == 1
        assert not (tmp_path / "daemon.json").exists()
        assert not (tmp_path / "broker.token").exists()
    finally:
        stop(tmp_path)


def test_concurrent_starts_share_one_daemon(tmp_path) -> None:
    port = _port()
    results: list[object] = []
    errors: list[BaseException] = []

    def run() -> None:
        try:
            results.append(start(tmp_path, port))
        except BaseException as exc:
            errors.append(exc)

    workers = [threading.Thread(target=run) for _ in range(2)]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join()
    try:
        assert errors == []
        assert results == [0, 0]
        record = json.loads((tmp_path / "daemon.json").read_text())
        assert record["pid"] > 0
    finally:
        stop(tmp_path)


def test_stop_refuses_tampered_pid_state_without_deleting(tmp_path) -> None:
    port = _port()
    assert start(tmp_path, port) == 0
    try:
        record_path = tmp_path / "daemon.json"
        original = json.loads(record_path.read_text())
        record_path.write_text(json.dumps({**original, "pid": os.getpid()}))
        with pytest.raises(RuntimeError, match="identity"):
            stop(tmp_path)
        assert record_path.exists()
        assert (tmp_path / "broker.token").exists()
    finally:
        record_path.write_text(json.dumps(original))
        stop(tmp_path)


def test_start_preserves_state_on_transient_health_failure(tmp_path, monkeypatch) -> None:
    import soarm101_motion.local_sim_daemon as daemon

    port = _port()
    assert start(tmp_path, port) == 0
    record_before = (tmp_path / "daemon.json").read_text()
    token_before = (tmp_path / "broker.token").read_text()
    original = daemon._request

    def flaky(requested_port, token, path, **kwargs):
        if path == "/v1/health":
            raise TimeoutError("temporary health timeout")
        return original(requested_port, token, path, **kwargs)

    monkeypatch.setattr(daemon, "_request", flaky)
    try:
        with pytest.raises(TimeoutError):
            daemon.start(tmp_path, port)
        assert (tmp_path / "daemon.json").read_text() == record_before
        assert (tmp_path / "broker.token").read_text() == token_before
    finally:
        monkeypatch.setattr(daemon, "_request", original)
        stop(tmp_path)
