"""Operator-only physical STOP-in-flight qualification using the real loopback broker.

DO NOT run unattended or inside OpenShell. This is a supervised mechanical
experiment, not a safety-rated emergency stop. Keep the hardware power cutoff
in reach throughout. No agent is permitted to invoke or parameterize it.

Start with an unloaded, secured, calibrated SO-ARM101 already physically at
Sleep; require a human-issued `soarm101 agent arm --minutes 30` lease.
The script verifies the measured Sleep joints without moving, completes
Overhead -> Sleep through the restricted HTTP broker, then obtains a SECOND
typed confirmation and interrupts a separate Overhead movement with STOP.
Because STOP revokes motion authority, the arm may finish between Sleep and
Overhead; returning it to Sleep after the test requires a separate human-
authorized recovery. It never rearms or resumes motion automatically.
"""

from __future__ import annotations

import argparse
import json
import math
import secrets
import sys
import threading
import time
from pathlib import Path
from collections.abc import Mapping
from typing import Any

from persistent_broker_motion_trace import _preflight_saved_poses, _request

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.broker import AgentMotionRates, RobotBrokerHTTPServer, RobotBrokerService
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.poses import PoseLibrary


# The trial must begin from a measured posture reasonably close to the
# calibration-relative Sleep target, not move there silently. A genuine
# Overhead/Sleep transition must also have meaningful joint travel.
START_POSE_TOLERANCE_DEG = 5.0
MIN_OVERHEAD_SEPARATION_DEG = 10.0


def _verify_measured_pose(
    measured: Mapping[str, float],
    expected: Mapping[str, float],
    *,
    label: str,
    tolerance_deg: float = START_POSE_TOLERANCE_DEG,
) -> dict[str, float]:
    """Fail before issuing the next move if measured joints mismatch."""
    errors: dict[str, float] = {}
    for joint, target in expected.items():
        if joint not in measured:
            raise ValueError(f"{label}: measured joint {joint} is missing")
        actual = float(measured[joint])
        if not math.isfinite(actual) or not math.isfinite(float(target)):
            raise ValueError(f"{label}: nonfinite joint position for {joint}")
        errors[joint] = abs(math.degrees(actual - float(target)))
    if not errors:
        raise ValueError(f"{label}: no pose joints to verify")
    if max(errors.values()) > tolerance_deg:
        raise RuntimeError(
            f"{label}: not within {tolerance_deg:g} deg of expected pose "
            f"(largest error {max(errors.values()):.2f} deg); "
            "reposition manually and restart; do not auto-correct"
        )
    return errors


def _verify_meaningful_overhead_travel(
    sleep: Mapping[str, float], overhead: Mapping[str, float],
) -> float:
    """Reject an accidental identical saved pose before motor movement."""
    if set(sleep) != set(overhead):
        raise ValueError("Sleep and Overhead targets have different arm joints")
    if not sleep:
        raise ValueError("Sleep/Overhead targets contain no arm joints")
    displacement = max(abs(math.degrees(float(overhead[j] - sleep[j]))) for j in sleep)
    if not math.isfinite(displacement) or displacement < MIN_OVERHEAD_SEPARATION_DEG:
        raise RuntimeError(
            "saved Overhead pose is too close to Sleep for this STOP trial"
        )
    return displacement


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--robot-id", default="so101")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--speed-deg-s", type=float, default=8.0)
    parser.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    parser.add_argument("--command-frequency-hz", type=float, default=20.0)
    return parser


def _require_operator(args: argparse.Namespace) -> None:
    if not sys.stdin.isatty():
        raise PermissionError("physical STOP trial requires an interactive operator terminal")
    if not args.port.startswith("/dev/"):
        raise ValueError("physical STOP trial needs an explicit Linux /dev/ tty")
    for name in ("speed_deg_s", "acceleration_deg_s2", "command_frequency_hz"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _require_operator(args)  # Before touching files, devices or a lease.
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    events_path = output.with_suffix(".broker-events.jsonl")
    summary_path = output.with_suffix(".summary.json")
    if any(p.exists() for p in (output, events_path, summary_path)):
        raise FileExistsError("STOP trial outputs already exist; choose a new path")

    config = SOARM101Config.from_motion_limits(
        robot_id=args.robot_id, port=args.port,
        configure_motors_on_connect=False,
        disable_torque_on_disconnect=False,
        command_frequency_hz=args.command_frequency_hz,
        default_joint_speed=math.radians(args.speed_deg_s),
        default_joint_acceleration=math.radians(args.acceleration_deg_s2),
        max_joint_speed_deg_s=args.speed_deg_s,
        max_joint_acceleration_deg_s2=args.acceleration_deg_s2,
    )
    rates = AgentMotionRates(
        joint_speed_deg_s=args.speed_deg_s,
        joint_acceleration_deg_s2=args.acceleration_deg_s2,
    ).validated(config)
    token = secrets.token_urlsafe(32)
    executor = SDKAgentExecutor(
        config=config, rates=rates, simulation=False, physical_trial=True,
        trace_path=output, arm_factory=lambda: SOARM101(config),
    )
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "physical-supervised-stop-qualification",
        "tools": ["robot_health", "robot_capabilities", "robot_state",
                  "go_pose", "sleep", "stop"],
        "cameras": [], "limits": {},
    })
    service = RobotBrokerService(
        executor=executor, token=token, event_path=events_path,
        profile=profile, allow_physical_sdk_trial=True,
    )
    server = RobotBrokerHTTPServer(("127.0.0.1", 0), service)
    url = f"http://127.0.0.1:{server.server_address[1]}"
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    summary: dict[str, Any] = {
        "kind": "persistent_broker_stop_qualification", "passed": False,
        "initial_pose_assumed": "Sleep, verified from measured joint angles",
        "automatic_recovery_after_stop": False,
        "robot_id": args.robot_id, "port": args.port,
        "speed_deg_s": args.speed_deg_s,
        "acceleration_deg_s2": args.acceleration_deg_s2,
        "command_frequency_hz": args.command_frequency_hz,
        "trace": str(output), "broker_events": str(events_path),
        "summary": str(summary_path),
    }
    failure: str | None = None
    interrupted_reply: dict[str, Any] = {}
    interrupted_finished = threading.Event()
    worker: threading.Thread | None = None
    try:
        state = _request(url, token, "GET", "/v1/state")["result"]
        if not state["authority"]["armed"]:
            raise PermissionError("operator must issue human motion authority first")
        calibration_id = state.get("calibration_id")
        if not calibration_id or calibration_id == "simulation":
            raise PermissionError("requires measured physical motor calibration")
        _preflight_saved_poses(args.robot_id, calibration_id)
        sdk_arm = executor._session
        if sdk_arm is None:
            raise RuntimeError("physical SDK connection disappeared")
        sleep_target = sdk_arm.get_sleep_joint_positions()
        overhead_target = PoseLibrary(args.robot_id).require("agent_start_overhead").joints
        summary["start_sleep_errors_deg"] = _verify_measured_pose(
            state["joint_positions_rad"], sleep_target, label="initial Sleep",
        )
        summary["overhead_separation_deg"] = _verify_meaningful_overhead_travel(
            sleep_target, overhead_target,
        )
        summary["calibration_id"] = calibration_id
        summary["broker_session"] = state["broker_session"]
        if state["broker_session"]["sdk_connect_count"] != 1:
            raise RuntimeError("unexpected SDK connect count")
        summary["route"] = [
            "verify measured Sleep (no movement)", "Overhead (complete)",
            "Sleep (complete)", "Overhead (STOP while moving)",
        ]
        summary["baseline_legs"] = []
        print("Clear arm workspace; remove payload; keep power cutoff reachable.", flush=True)
        print("Starting pose verified as Sleep; no initial Sleep command will be sent.", flush=True)
        print("Baseline route: Sleep -> Overhead -> Sleep, then interrupt Overhead.", flush=True)
        print("This is NOT an emergency-stop certification.", flush=True)
        if input("Type ROUNDTRIP to move Overhead and back to Sleep: ").strip() != "ROUNDTRIP":
            raise PermissionError("operator declined Overhead/Sleep baseline")

        for label, path, payload, target in (
            ("OVERHEAD", "/v1/go-pose",
             {"name": "agent_start_overhead", "joint_only": True}, overhead_target),
            ("SLEEP", "/v1/sleep", {"joint_only": True}, sleep_target),
        ):
            reply = _request(url, token, "POST", path, payload)
            result = reply["result"]
            if not result.get("accepted") or not result.get("completed"):
                raise RuntimeError(f"{label} baseline motion did not complete")
            if result.get("broker_session", {}).get("id") != summary["broker_session"]["id"]:
                raise RuntimeError(f"{label}: persistent broker session unexpectedly changed")
            measured = _request(url, token, "GET", "/v1/state")["result"]
            errors = _verify_measured_pose(
                measured["joint_positions_rad"], target, label=f"{label} baseline",
            )
            summary["baseline_legs"].append({
                "label": label, "request_id": reply["request_id"],
                "joint_errors_deg": errors, "completed": True,
            })
            print(f"{label}: complete; measured pose verified.", flush=True)

        print(
            "The baseline Overhead -> Sleep round trip is complete. "
            "Next STOP will interrupt a NEW Overhead movement. "
            "The arm may remain between poses afterward and will NOT auto-return.",
            flush=True,
        )
        if input("Type INTERRUPT to test STOP during overhead movement: ").strip() != "INTERRUPT":
            raise PermissionError("operator declined STOP test")

        def move():
            try:
                interrupted_reply["body"] = _request(
                    url, token, "POST", "/v1/go-pose",
                    {"name": "agent_start_overhead", "joint_only": True},
                )
            except Exception as exc:
                interrupted_reply["error"] = str(exc)
            finally:
                interrupted_finished.set()

        worker = threading.Thread(target=move, daemon=True)
        worker.start()
        # Observe only the host motion-controller state, not the HTTP client
        # thread. Avoid SOARM101.is_moving here: that queries each physical
        # servo and could contend with motion/STOP packet I/O.
        deadline = time.monotonic() + 10.0
        while not sdk_arm.motion.is_moving and not interrupted_finished.is_set():
            if time.monotonic() > deadline:
                raise RuntimeError("active motion was not observed within 10 seconds")
            time.sleep(0.01)
        observed_active = bool(sdk_arm.motion.is_moving)
        summary["active_motion_observed"] = observed_active
        if not observed_active:
            raise RuntimeError("movement completed before STOP qualification began")
        # Let motion visibly start, but reject an already completed leg as a
        # valid STOP trial. Rate ceilings and pose provenance stay enforced.
        time.sleep(0.15)
        if not sdk_arm.motion.is_moving:
            raise RuntimeError("motion completed before independent STOP request")

        stop_start = time.monotonic()
        summary["stop_requested_monotonic_s"] = stop_start
        try:
            stop = _request(url, token, "POST", "/v1/stop", {}, timeout_s=5.0)
        finally:
            summary["stop_response_duration_s"] = time.monotonic() - stop_start
        summary["stop_result"] = stop["result"]
        # STOP must not wait behind the in-flight normal broker lock. An
        # unconfirmed STOP, timeout or bus fault remains a failed gate.
        if not stop["result"].get("completed"):
            raise RuntimeError("broker STOP did not report completed HOLD")
        if stop["result"].get("holding") is not True:
            raise RuntimeError("broker STOP did not verify torque-held state")
        if not interrupted_finished.wait(timeout=5):
            raise RuntimeError("interrupted motion did not terminate after STOP")
        if "body" in interrupted_reply and interrupted_reply["body"].get("ok"):
            raise RuntimeError("interrupted movement was falsely acknowledged as success")
        if not interrupted_reply.get("error"):
            raise RuntimeError("expected interrupted motion to return an explicit failure")
        if sdk_arm.motion.is_moving:
            raise RuntimeError("SDK still reports motion active after STOP")
        post = _request(url, token, "GET", "/v1/state")["result"]
        summary["post_stop_state"] = post
        if post["authority"]["armed"]:
            raise RuntimeError("STOP did not revoke human authority")
        time.sleep(1.0)
        summary["post_stop_state_one_second"] = _request(
            url, token, "GET", "/v1/state"
        )["result"]
        print("Review position, stability, trace and camera/eyes-on observation.", flush=True)
        if input("Type STATIONARY only if physically stationary and holding: ").strip() != "STATIONARY":
            raise RuntimeError("operator did not verify physical STOP")
        summary["passed"] = True
    except BaseException as exc:
        failure = repr(exc)
        print(f"STOP qualification FAILED/INTERRUPTED: {failure}", flush=True)
        # After failure, try STOP once more; report any incomplete HOLD.
        # This is not a substitute for human-accessible power cutoff.
        try:
            summary["failure_stop"] = _request(
                url, token, "POST", "/v1/stop", {}, timeout_s=5.0,
            )["result"]
        except Exception as stop_exc:
            summary["failure_stop_error"] = repr(stop_exc)
    finally:
        if worker is not None:
            worker.join(timeout=2)
            if worker.is_alive():
                summary["unfinished_action"] = True
                summary["passed"] = False
        summary["interrupted_move"] = interrupted_reply
        summary["sdk_connections"] = executor.connection_count
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=3)
        try:
            trace = executor._trace
            if trace is not None:
                summary["trace_counts"] = trace.summary
            executor.close()
        except Exception as exc:
            summary["cleanup_error"] = repr(exc)
            summary["passed"] = False
        if failure is not None:
            summary["error"] = failure
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"STOP qualification summary: {summary_path}", flush=True)
        print(f"Motor trace: {output}", flush=True)
        print(f"Broker events: {events_path}", flush=True)
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
