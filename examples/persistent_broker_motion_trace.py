"""Supervised physical five-leg validation of the *persistent SDK* HTTP broker.

Unlike motion_quality_trace.py, this program does not call arm.move_*.
It starts a trusted loopback HTTP broker, then executes every leg through
authenticated POST /v1/sleep or /v1/go-pose requests. The broker owns one
SOARM101 connection, motor safety, lease checks, STOP and passive tracing.

This is a physical bench experiment, NOT the released broker or OpenShell agent.
"""

from __future__ import annotations

import argparse
import json
import math
import secrets
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.broker import (
    AgentMotionRates, RobotBrokerHTTPServer, RobotBrokerService,
)
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.poses import PoseLibrary
from soarm101_motion.provenance import require_calibration_compatibility


ROUTE = (
    ("SLEEP", "/v1/sleep", {}),
    ("OVERHEAD", "/v1/go-pose", {"name": "agent_start_overhead"}),
    ("LEFT", "/v1/go-pose", {"name": "agent_start_overhead_left"}),
    ("RIGHT", "/v1/go-pose", {"name": "agent_start_overhead_right"}),
    ("SLEEP", "/v1/sleep", {}),
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="real follower tty, e.g. /dev/ttyACM1")
    parser.add_argument("--robot-id", default="so101")
    parser.add_argument("--speed-deg-s", type=float, default=15.0)
    parser.add_argument("--acceleration-deg-s2", type=float, default=150.0)
    parser.add_argument("--command-frequency-hz", type=float, default=50.0)
    parser.add_argument("--pause-s", type=float, default=0.5)
    parser.add_argument(
        "--include-gripper", action="store_true",
        help="also command saved gripper targets and Sleep close; default is joint-only",
    )
    parser.add_argument("--yes", action="store_true", help="skip RUN confirmation")
    parser.add_argument(
        "--output", type=Path, required=True,
        help="local trusted-host motor trace JSONL (not an agent-controlled path)",
    )
    return parser


def _request(base: str, token: str, method: str, path: str,
             payload: dict[str, Any] | None = None) -> dict[str, Any]:
    request = urllib.request.Request(
        base + path, method=method,
        headers={
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
        },
        data=(json.dumps(payload or {}).encode("utf-8") if method == "POST" else None),
    )
    try:
        with urllib.request.urlopen(request, timeout=200) as response:
            body = json.load(response)
    except urllib.error.HTTPError as exc:
        try:
            message = json.loads(exc.read().decode("utf-8")).get("error", str(exc))
        except (ValueError, UnicodeError):
            message = str(exc)
        raise RuntimeError(f"broker {path} rejected: {message}") from exc
    if not body.get("ok") or not isinstance(body.get("result"), dict):
        raise RuntimeError(f"broker {path} returned invalid response: {body!r}")
    return body


def _preflight_saved_poses(robot_id: str, calibration_id: str) -> None:
    library = PoseLibrary(robot_id)
    for name in ("agent_start_overhead", "agent_start_overhead_left",
                 "agent_start_overhead_right"):
        pose = library.require(name)
        require_calibration_compatibility(
            {
                "source_robot_id": pose.source_robot_id,
                "source_calibration_id": pose.source_calibration_id,
                "target_robot_id": pose.target_robot_id,
                "target_calibration_id": pose.target_calibration_id,
            },
            current_robot_id=robot_id,
            current_calibration_id=calibration_id,
            artifact_label=name,
        )
        print(f"Calibration compatible: {name}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    for name, value in (
        ("speed-deg-s", args.speed_deg_s),
        ("acceleration-deg-s2", args.acceleration_deg_s2),
        ("command-frequency-hz", args.command_frequency_hz),
    ):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"--{name} must be finite and positive")
    if not math.isfinite(args.pause_s) or args.pause_s < 0:
        raise ValueError("--pause-s must be finite and nonnegative")
    if not args.port.startswith("/dev/"):
        raise ValueError("physical trial requires an explicit /dev/ tty port")

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    events_path = output.with_suffix(".broker-events.jsonl")
    summary_path = output.with_suffix(".summary.json")
    if any(path.exists() for path in (output, events_path, summary_path)):
        raise FileExistsError("trial output or its events/summary already exists")

    config = SOARM101Config.from_motion_limits(
        robot_id=args.robot_id,
        port=args.port,
        configure_motors_on_connect=False,
        disable_torque_on_disconnect=False,
        command_frequency_hz=args.command_frequency_hz,
        max_joint_speed_deg_s=args.speed_deg_s,
        max_joint_acceleration_deg_s2=args.acceleration_deg_s2,
    )
    rates = AgentMotionRates(
        joint_speed_deg_s=args.speed_deg_s,
        joint_acceleration_deg_s2=args.acceleration_deg_s2,
    ).validated(config)
    token = secrets.token_urlsafe(32)
    executor = SDKAgentExecutor(
        config=config, rates=rates, simulation=False,
        physical_trial=True, trace_path=output,
        arm_factory=lambda: SOARM101(config),
    )
    profile = CapabilityProfile.from_document({
        "schema_version": 1,
        "name": "physical-persistent-broker-route-trial",
        "tools": ["robot_health", "robot_capabilities", "robot_state",
                  "go_pose", "sleep", "stop"],
        "cameras": [],
        "limits": {},
    })
    service = RobotBrokerService(
        executor=executor, token=token, event_path=events_path,
        profile=profile, allow_physical_sdk_trial=True,
    )
    server = RobotBrokerHTTPServer(("127.0.0.1", 0), service)
    host, port = server.server_address
    base = f"http://{host}:{port}"
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    summary: dict[str, Any] = {
        "kind": "persistent_sdk_broker_physical_trial",
        "robot_id": args.robot_id,
        "port": args.port,
        "joint_speed_deg_s": args.speed_deg_s,
        "joint_acceleration_deg_s2": args.acceleration_deg_s2,
        "joint_only": not args.include_gripper,
        "trace": str(output),
        "broker_events": str(events_path),
        "route": [label for label, _, _ in ROUTE],
        "legs": [],
        "passed": False,
    }
    error: str | None = None
    try:
        health = _request(base, token, "GET", "/v1/health")
        assert health["result"]["status"] == "ok"
        state = _request(base, token, "GET", "/v1/state")["result"]
        session = state["broker_session"]
        if session["mode"] != "physical_trial" or session["sdk_connect_count"] != 1:
            raise RuntimeError(f"unexpected broker session: {session}")
        summary["broker_session"] = session
        if not state["authority"]["armed"]:
            raise PermissionError(
                "human motion lease inactive: run 'soarm101 agent arm --minutes 30' "
                "BEFORE this trial, then close its serial session"
            )
        if not isinstance(state["calibration_id"], str) or not state["calibration_id"]:
            raise RuntimeError("physical follower has no active calibration ID")
        _preflight_saved_poses(args.robot_id, state["calibration_id"])
        print(f"Persistent SDK session: {session['id']} (connections=1)")
        print(f"Trace: {output}")
        print("Route: Sleep -> Overhead -> Left -> Right -> Sleep")
        print(
            f"Motion: {args.speed_deg_s:g} deg/s, "
            f"{args.acceleration_deg_s2:g} deg/s^2; "
            f"joint-only={not args.include_gripper}"
        )
        if not args.yes and input("Type RUN to start supervised physical route: ").strip() != "RUN":
            print("Not executed; operator declined.")
            return 1

        for index, (label, path, defaults) in enumerate(ROUTE, start=1):
            print(f"[{index}/5] {label}", flush=True)
            started = time.monotonic()
            reply = _request(
                base, token, "POST", path,
                {**defaults, "joint_only": not args.include_gripper},
            )
            result = reply["result"]
            if result.get("broker_session") != session:
                raise RuntimeError("broker SDK session was replaced mid-route")
            if not result.get("accepted") or not result.get("completed"):
                raise RuntimeError(f"{label} did not complete: {result}")
            summary["legs"].append({
                "index": index, "label": label, "request_id": reply["request_id"],
                "duration_s": time.monotonic() - started,
                "completed": True, "broker_session": result["broker_session"],
            })
            if args.pause_s and index < len(ROUTE):
                time.sleep(args.pause_s)
        final_state = _request(base, token, "GET", "/v1/state")["result"]
        if final_state["broker_session"] != session:
            raise RuntimeError("post-route broker connection count changed")
        summary["final_calibration_id"] = final_state["calibration_id"]
        summary["passed"] = True
        print("Five legs completed through one broker-owned SDK session.")
        print("Follower remains torque-held.")
    except BaseException as exc:
        error = repr(exc)
        print(f"Trial interrupted or failed: {error}", flush=True)
        # Independent HTTP STOP worker may interrupt an active SDK command.
        # A failed STOP remains a visible failure, never reported as success.
        try:
            stopped = _request(base, token, "POST", "/v1/stop", {})
            summary["stop"] = stopped["result"]
            print("STOP/HOLD response:", stopped["result"])
        except Exception as stop_exc:
            summary["stop_error"] = repr(stop_exc)
            print(f"STOP/HOLD could not be verified: {stop_exc}")
    finally:
        server.shutdown()
        server.server_close()
        try:
            trace = executor._trace
            if trace is not None:
                summary["trace_counts"] = trace.summary
            executor.close()
        except Exception as cleanup_exc:
            summary["cleanup_error"] = repr(cleanup_exc)
        if error is not None:
            summary["error"] = error
        summary["sdk_connect_count"] = executor.connection_count
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"Summary: {summary_path}")
        print(f"Motor trace: {output}")
        print(f"Broker events: {events_path}")
    return 0 if summary["passed"] and "cleanup_error" not in summary else 1


if __name__ == "__main__":
    raise SystemExit(main())
