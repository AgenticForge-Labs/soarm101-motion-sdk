"""Operator-only physical STOP-in-flight qualification using the real loopback broker.

DO NOT run unattended or inside OpenShell. This is a supervised mechanical
experiment, not a safety-rated emergency stop. Keep the hardware power cutoff
in reach throughout. No agent is permitted to invoke or parameterize it.

Start with an unloaded, secured, calibrated SO-ARM101 already physically at
Sleep; require a human-issued `soarm101 agent arm --minutes 30` lease.
The default short trial verifies measured Sleep without moving, then
interrupts one fixed +8° shoulder-pan movement through the restricted
broker. There is no baseline roundtrip or post-STOP rearm in short mode.
The historical, larger Overhead trial is available only via explicit
--trial overhead after separate safety review. STOP always revokes authority.
No mode automatically resumes a cancelled trajectory.
"""

from __future__ import annotations

import argparse
import json
import math
import secrets
import select
import sys
import threading
import time
from pathlib import Path
from collections.abc import Callable, Mapping
from typing import Any

from persistent_broker_motion_trace import _request

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.broker import AgentMotionRates, RobotBrokerHTTPServer, RobotBrokerService
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.poses import PoseLibrary
from soarm101_motion.provenance import require_calibration_compatibility


# The trial must begin from a measured posture reasonably close to the
# calibration-relative Sleep target, not move there silently. A genuine
# Overhead/Sleep transition must also have meaningful joint travel.
START_POSE_TOLERANCE_DEG = 5.0
MIN_OVERHEAD_SEPARATION_DEG = 10.0
SHORT_JOINT = "shoulder_pan"
SHORT_DELTA_DEG = 8.0
SHORT_MAX_SPEED_DEG_S = 3.0
SHORT_MAX_ACCEL_DEG_S2 = 15.0
SHORT_LIMIT_INSET_DEG = 5.0


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


def _preflight_short_joint(
    measured: Mapping[str, float],
    limits: Mapping[str, tuple[float, float]],
) -> dict[str, float | str]:
    """Accept only a fixed, inwardly bounded pan move from measured Sleep."""
    if SHORT_JOINT not in measured or SHORT_JOINT not in limits:
        raise ValueError("short STOP: calibrated shoulder_pan is unavailable")
    actual = float(measured[SHORT_JOINT])
    lower, upper = (float(value) for value in limits[SHORT_JOINT])
    target = actual + math.radians(SHORT_DELTA_DEG)
    inset = math.radians(SHORT_LIMIT_INSET_DEG)
    if not all(math.isfinite(value) for value in (actual, lower, upper, target)):
        raise ValueError("short STOP: nonfinite measured position or limit")
    if not lower + inset <= actual <= upper - inset:
        raise RuntimeError("short STOP: shoulder_pan starts too close to its limit")
    if not lower + inset <= target <= upper - inset:
        raise RuntimeError("short STOP: target exceeds inset calibrated pan limit")
    return {
        "joint": SHORT_JOINT,
        "delta_deg": SHORT_DELTA_DEG,
        "start_deg": math.degrees(actual),
        "target_deg": math.degrees(target),
        "lower_deg": math.degrees(lower),
        "upper_deg": math.degrees(upper),
    }


def _verify_post_stop_stability(
    stop_result: Mapping[str, Any],
    first_state: Mapping[str, Any],
    later_state: Mapping[str, Any],
    *,
    tolerance_deg: float = 1.0,
) -> dict[str, float]:
    """A successful HTTP reply alone cannot qualify a stationary HOLD."""
    stability = stop_result.get("post_stop_stability")
    if not isinstance(stability, Mapping) or stability.get("verified") is not True:
        raise RuntimeError("STOP did not provide verified physical encoder settling")
    a = first_state["joint_positions_rad"]
    b = later_state["joint_positions_rad"]
    latch = stop_result["joint_positions_rad"]
    if set(a) != set(b) or set(a) != set(latch) or not a:
        raise RuntimeError("STOP stability readback changed joint identities")
    drifts = {}
    for joint in a:
        initial, subsequent, latched = (
            float(a[joint]), float(b[joint]), float(latch[joint])
        )
        if not all(math.isfinite(value) for value in (initial, subsequent, latched)):
            raise RuntimeError("STOP stability feedback contained a nonfinite joint")
        drifts[joint] = max(
            abs(math.degrees(subsequent - initial)),
            abs(math.degrees(initial - latched)),
            abs(math.degrees(subsequent - latched)),
        )
    if max(drifts.values()) > tolerance_deg:
        raise RuntimeError(
            f"STOP stationary HOLD drift exceeded {tolerance_deg:g} deg: {drifts}"
        )
    if later_state["authority"]["armed"]:
        raise RuntimeError("STOP authority was unexpectedly restored")
    return drifts


def _execute_verified_roundtrip(
    request: Callable[[str, str, dict[str, Any] | None], dict[str, Any]],
    *,
    expected_session_id: str,
    overhead_target: Mapping[str, float],
    sleep_target: Mapping[str, float],
    legs: list[dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    """Complete and measure Overhead -> Sleep without an initial Sleep write."""
    if legs is None:
        legs = []
    for label, path, payload, target in (
        ("OVERHEAD", "/v1/go-pose",
         {"name": "agent_start_overhead", "joint_only": True}, overhead_target),
        ("SLEEP", "/v1/sleep", {"joint_only": True}, sleep_target),
    ):
        reply = request("POST", path, payload)
        result = reply["result"]
        if not result.get("accepted") or not result.get("completed"):
            raise RuntimeError(f"{label} baseline motion did not complete")
        if result.get("broker_session", {}).get("id") != expected_session_id:
            raise RuntimeError(f"{label}: persistent broker session unexpectedly changed")
        measured = request("GET", "/v1/state", None)["result"]
        errors = _verify_measured_pose(
            measured["joint_positions_rad"], target, label=f"{label} baseline",
        )
        legs.append({
            "label": label, "request_id": reply["request_id"],
            "joint_errors_deg": errors, "completed": True,
        })
        print(f"{label}: complete; measured pose verified.", flush=True)
    return legs


def _await_operator_stop(
    *,
    is_moving: Callable[[], bool],
    motion_finished: threading.Event,
    stream: Any,
    timeout_s: float = 90.0,
) -> float:
    """Observe interactive ENTER while the *host* motion is in flight.

    This polls only the stdin fd and controller flags, not physical servo
    registers. Never wait on a blocking input() call that outlives movement.
    """
    if not stream.isatty():
        raise PermissionError("manual STOP requires an operator terminal")
    print(
        "MOTION IS ACTIVE. Press ENTER now to send STOP "
        "before it reaches the target.",
        flush=True,
    )
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if motion_finished.is_set() or not is_moving():
            raise RuntimeError(
                "Movement ended before the operator triggered STOP; "
                "STOP interruption was not qualified"
            )
        ready, _, _ = select.select([stream], [], [], 0.02)
        if ready:
            # The terminal supplies one complete newline. Do not treat
            # any queued keystrokes as authority to start another action.
            if not stream.readline():
                raise RuntimeError("operator STOP terminal closed")
            if motion_finished.is_set() or not is_moving():
                raise RuntimeError(
                    "motion finished before the operator STOP request"
                )
            return time.monotonic()
    raise TimeoutError("operator did not trigger STOP during active movement")


def _operator_rearm_for_completion(
    *, arm: SOARM101, authority_store: AgentAuthorityStore,
    robot_id: str, calibration_id: str,
) -> dict[str, object]:
    """Explicit trusted-host operator recovery, never a sandbox action.

    Called only after a verified STOP and a separate typed rearm decision.
    Reuse the human arming primitive: verify current safe HOLD, then issue
    a fresh calibration-bound, short-lived lease. Never replay a cancelled
    handle or bypass the canonical broker motion checks.
    """
    if not sys.stdin.isatty():
        raise PermissionError("rearming requires a physical operator terminal")
    if arm.motion.is_moving:
        raise RuntimeError("cannot rearm while the previous motion is active")
    state = arm.get_state()
    if not state.connected or state.faulted or not state.torque_enabled:
        raise RuntimeError("cannot rearm without connected, fault-free torque HOLD")
    if arm.config.robot_id != robot_id or arm.calibration_id != calibration_id:
        raise RuntimeError("robot identity/calibration changed after STOP")
    # Re-latch HOLD using the same operation the normal host agent-arm CLI
    # uses. Do not issue new authority until that operation succeeds.
    arm.hold()
    confirmed = arm.get_state()
    if not confirmed.connected or confirmed.faulted or not confirmed.torque_enabled:
        raise RuntimeError("fresh HOLD could not be confirmed before rearming")
    authority = authority_store.issue(
        robot_id=robot_id, calibration_id=calibration_id, minutes=5,
    )
    return {
        "issued_at": authority.issued_at,
        "expires_at": authority.expires_at,
        "calibration_id": calibration_id,
        "method": "human-confirmed trusted-host recovery after STOP",
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--robot-id", default="so101")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--trial", choices=("short", "overhead"), default="short",
        help="short is the default single-joint STOP gate; overhead is the older large route",
    )
    parser.add_argument("--speed-deg-s", type=float)
    parser.add_argument("--acceleration-deg-s2", type=float)
    parser.add_argument("--command-frequency-hz", type=float, default=20.0)
    return parser


def _require_operator(args: argparse.Namespace) -> None:
    if not sys.stdin.isatty():
        raise PermissionError("physical STOP trial requires an interactive operator terminal")
    if not args.port.startswith("/dev/"):
        raise ValueError("physical STOP trial needs an explicit Linux /dev/ tty")
    for name in ("speed_deg_s", "acceleration_deg_s2", "command_frequency_hz"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value <= 0):
            raise ValueError(f"{name} must be finite and positive")
    if args.trial == "short":
        speed = 2.0 if args.speed_deg_s is None else args.speed_deg_s
        accel = 10.0 if args.acceleration_deg_s2 is None else args.acceleration_deg_s2
        if speed > SHORT_MAX_SPEED_DEG_S or accel > SHORT_MAX_ACCEL_DEG_S2:
            raise ValueError("short STOP rate caps: at most 3 deg/s and 15 deg/s²")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _require_operator(args)  # Before touching files, devices or a lease.
    if args.speed_deg_s is None:
        args.speed_deg_s = 2.0 if args.trial == "short" else 8.0
    if args.acceleration_deg_s2 is None:
        args.acceleration_deg_s2 = 10.0 if args.trial == "short" else 25.0
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
        "tools": (
            ["robot_health", "robot_capabilities", "robot_state", "jog_joint", "stop"]
            if args.trial == "short"
            else ["robot_health", "robot_capabilities", "robot_state",
                  "go_pose", "sleep", "stop"]
        ),
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
        "trial": args.trial,
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
    stop_attempted = False
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
        sdk_arm = executor._session
        if sdk_arm is None:
            raise RuntimeError("physical SDK connection disappeared")
        sleep_target = sdk_arm.get_sleep_joint_positions()
        summary["start_sleep_errors_deg"] = _verify_measured_pose(
            state["joint_positions_rad"], sleep_target, label="initial Sleep",
        )
        summary["calibration_id"] = calibration_id
        summary["broker_session"] = state["broker_session"]
        if state["broker_session"]["sdk_connect_count"] != 1:
            raise RuntimeError("unexpected SDK connect count")

        summary["baseline_legs"] = []
        print("Clear arm workspace; remove payload; keep power cutoff reachable.", flush=True)
        print("Starting pose verified as Sleep; no initial Sleep command will be sent.", flush=True)
        print("This is NOT an emergency-stop certification.", flush=True)
        if args.trial == "short":
            # The trial uses a fixed single-joint broker route, not an
            # unchecked raw servo command or operator-selected direction.
            # The canonical SDK is still responsible for motion validation.
            limits = sdk_arm.motion._effective_limits()
            summary["short_preflight"] = _preflight_short_joint(
                state["joint_positions_rad"], limits,
            )
            summary["route"] = [
                "verify measured Sleep (no movement)",
                "shoulder_pan +8 deg (manual STOP while moving)",
            ]
            movement_path = "/v1/joint"
            movement_payload = {
                "joint": SHORT_JOINT,
                "delta_deg": SHORT_DELTA_DEG,
            }
            print(
                "SHORT STOP trial: one +8 deg shoulder-pan move at up to "
                f"{args.speed_deg_s:g} deg/s. No baseline legs, no "
                "automatic return or post-STOP REARM.",
                flush=True,
            )
        else:
            # The original higher-energy characterization is preserved but
            # requires an explicit opt-in --trial overhead.
            overhead_pose = PoseLibrary(args.robot_id).require("agent_start_overhead")
            require_calibration_compatibility(
                {
                    "source_robot_id": overhead_pose.source_robot_id,
                    "source_calibration_id": overhead_pose.source_calibration_id,
                    "target_robot_id": overhead_pose.target_robot_id,
                    "target_calibration_id": overhead_pose.target_calibration_id,
                },
                current_robot_id=args.robot_id,
                current_calibration_id=calibration_id,
                artifact_label="agent_start_overhead",
            )
            overhead_target = overhead_pose.joints
            summary["overhead_separation_deg"] = _verify_meaningful_overhead_travel(
                sleep_target, overhead_target,
            )
            summary["route"] = [
                "verify measured Sleep (no movement)", "Overhead (complete)",
                "Sleep (complete)", "Overhead (STOP while moving)",
            ]
            print("Baseline route: Sleep -> Overhead -> Sleep, then interrupt Overhead.", flush=True)
            if input("Type ROUNDTRIP to move Overhead and back to Sleep: ").strip() != "ROUNDTRIP":
                raise PermissionError("operator declined Overhead/Sleep baseline")
            _execute_verified_roundtrip(
                lambda method, path, payload: _request(url, token, method, path, payload),
                expected_session_id=summary["broker_session"]["id"],
                overhead_target=overhead_target,
                sleep_target=sleep_target,
                legs=summary["baseline_legs"],
            )
            print(
                "Baseline complete. Next STOP will interrupt a NEW Overhead movement.",
                flush=True,
            )
            movement_path = "/v1/go-pose"
            movement_payload = {
                "name": "agent_start_overhead", "joint_only": True,
            }

        if input("Type START to begin the movement you will interrupt: ").strip() != "START":
            raise PermissionError("operator declined manual STOP trial")

        def move():
            try:
                interrupted_reply["body"] = _request(
                    url, token, "POST", movement_path, movement_payload,
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
        # The operator, not a fixed 150 ms timer, chooses when STOP fires.
        # This waits only on terminal input and the host motion state; it
        # never launches competing servo read requests during motion.
        stop_start = _await_operator_stop(
            is_moving=lambda: sdk_arm.motion.is_moving,
            motion_finished=interrupted_finished,
            stream=sys.stdin,
        )
        summary["stop_trigger"] = "human_pressed_enter_during_active_motion"
        summary["stop_requested_monotonic_s"] = stop_start
        try:
            stop_attempted = True
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
        summary["post_stop_max_drift_by_joint_deg"] = _verify_post_stop_stability(
            stop["result"], post, summary["post_stop_state_one_second"],
        )
        print("Review position, stability, trace and camera/eyes-on observation.", flush=True)
        if input("Type STATIONARY only if physically stationary and holding: ").strip() != "STATIONARY":
            raise RuntimeError("operator did not verify physical STOP")
        summary["stop_qualification_passed"] = True
        summary["passed"] = True

        if args.trial == "short":
            # No rearm or second motor command, even after a verified hold.
            summary["post_stop_recovery_requested"] = False
            print(
                "SHORT STOP gate complete. No REARM or new position goal. "
                "The arm remains held; review saved trace before any further test.",
                flush=True,
            )
        else:
            # The original lease was revoked by STOP. Finishing the goal is a
            # separate new motion requiring explicit human reauthorization,
            # fresh HOLD/identity checks, and normal validated broker dispatch.
            print(
                "STOP qualified; robot remains held between Sleep and Overhead. "
                "To finish Overhead, a NEW 5-minute operator lease is required.",
                flush=True,
            )
            if input(
                "Type REARM to authorize a NEW joint-only move to Overhead; "
                "press ENTER to leave the arm holding: "
            ).strip() == "REARM":
                summary["post_stop_recovery_requested"] = True
                summary["post_stop_human_rearm"] = _operator_rearm_for_completion(
                    arm=sdk_arm, authority_store=executor._authority,
                    robot_id=args.robot_id, calibration_id=calibration_id,
                )
                recovery = _request(
                    url, token, "POST", "/v1/go-pose",
                    {"name": "agent_start_overhead", "joint_only": True},
                )
                if not recovery["result"].get("accepted") or not recovery["result"].get("completed"):
                    raise RuntimeError("separately authorized Overhead completion failed")
                final = _request(url, token, "GET", "/v1/state")["result"]
                summary["post_stop_overhead_errors_deg"] = _verify_measured_pose(
                    final["joint_positions_rad"], overhead_target,
                    label="human-reauthorized Overhead finish",
                )
                summary["post_stop_recovery_completed"] = True
                summary["post_stop_recovery_request_id"] = recovery["request_id"]
                # Limit subsequent motion after operator recovery. Hold remains
                # enabled; this clears only agent motion authority.
                executor._authority.clear()
                print("Overhead reached and measured. Motion authority removed.", flush=True)
            else:
                summary["post_stop_recovery_requested"] = False
                print("No further motion requested. Arm remains torque-held.", flush=True)
    except BaseException as exc:
        failure = repr(exc)
        # Preserve a separately validated STOP but never report the whole
        # requested trial as passed when a selected recovery subsequently fails.
        summary["passed"] = False
        print(f"STOP qualification FAILED/INTERRUPTED: {failure}", flush=True)
        # Do not silently send a SECOND latch after an already attempted
        # STOP that returned unverified HOLD. The 2026-10-11 trace showed
        # major physical overshoot *after* successful 1/1 servo latch writes.
        # A blind repeated latch could aggravate the motion and conceal
        # the earliest STOP failure. Keep hardware power cutoff accessible.
        if stop_attempted:
            summary["failure_stop_retry"] = "prohibited_after_first_attempt"
            print(
                "Original STOP may not have physically held the arm. "
                "No automatic retry or REARM; use the accessible physical "
                "power cutoff if unsafe and inspect hardware.",
                flush=True,
            )
        elif worker is not None and worker.is_alive():
            # If motion was launched but STOP was never requested (e.g.
            # operator missed the window), try the ONE allowed STOP here.
            try:
                stop_attempted = True
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
