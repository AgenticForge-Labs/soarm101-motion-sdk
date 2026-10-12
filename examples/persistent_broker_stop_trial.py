"""Operator-only physical STOP-in-flight qualification using the real loopback broker.

DO NOT run unattended or inside OpenShell. This is a supervised mechanical
experiment, not a safety-rated emergency stop. Keep the hardware power cutoff
in reach throughout. No agent is permitted to invoke or parameterize it.

Start with an unloaded, secured, calibrated SO-ARM101 in a stationary
posture with a visually clear and operator-approved path to Sleep; require
a human-issued `soarm101 agent arm --minutes 30` lease.
The default short trial takes a measured starting pose, requires explicit
operator clearance approval, moves to calibrated Sleep through the same
authenticated broker with the joint-only SDK Sleep primitive, verifies
measured Sleep, then interrupts one fixed +8° shoulder-pan movement.
No automatic REARM or post-STOP return is permitted. The software
qualification requires fresh physical encoder-motion evidence at STOP.
A separate --trial overhead-single performs ONE faster, supervised,
joint-only move toward the validated saved Overhead from current encoder
pose. It neither moves to Sleep first nor authorizes a subsequent move.
The historical, larger Overhead route remains an explicit legacy opt-in.
STOP always revokes authority.
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
from soarm101_motion.broker import (
    AgentMotionRates,
    PHYSICAL_SHORT_STOP_DELTA_DEG,
    PHYSICAL_SHORT_STOP_JOINT,
    RobotBrokerHTTPServer,
    RobotBrokerService,
)
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.poses import PoseLibrary
from soarm101_motion.provenance import require_calibration_compatibility


# The historical Overhead trial requires starting at calculated Sleep.
# The short trial checks current encoder state and requires local operator
# approval before its one canonical broker-owned Sleep preparation.
START_POSE_TOLERANCE_DEG = 5.0
MIN_OVERHEAD_SEPARATION_DEG = 10.0
# Single source of truth for the hardware trial's broker-side action policy.
SHORT_JOINT = PHYSICAL_SHORT_STOP_JOINT
SHORT_DELTA_DEG = PHYSICAL_SHORT_STOP_DELTA_DEG
SHORT_MAX_SPEED_DEG_S = 3.0
SHORT_MAX_ACCEL_DEG_S2 = 15.0
SHORT_LIMIT_INSET_DEG = 5.0
# A different physical qualification with explicit rate/travel ceilings.
# 6 deg/s is 3x the previous 2 deg/s short-pan trial; all motors remain
# under the canonical SDK's per-joint rate/acceleration/fault guards.
OVERHEAD_SINGLE_DEFAULT_SPEED_DEG_S = 6.0
OVERHEAD_SINGLE_DEFAULT_ACCEL_DEG_S2 = 30.0
OVERHEAD_SINGLE_MAX_SPEED_DEG_S = 8.0
OVERHEAD_SINGLE_MAX_ACCEL_DEG_S2 = 40.0
OVERHEAD_SINGLE_MAX_JOINT_TRAVEL_DEG = 100.0
OVERHEAD_SINGLE_MIN_TRAVEL_DEG = 12.0
OVERHEAD_SINGLE_LIMIT_INSET_DEG = 5.0
OVERHEAD_POSE = "agent_start_overhead"


def _pose_joint_errors(
    measured: Mapping[str, float],
    expected: Mapping[str, float],
    *,
    label: str,
) -> dict[str, float]:
    """Read-only per-joint comparison; retain evidence on a failed start."""
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
    return errors


def _verify_measured_pose(
    measured: Mapping[str, float],
    expected: Mapping[str, float],
    *,
    label: str,
    tolerance_deg: float = START_POSE_TOLERANCE_DEG,
) -> dict[str, float]:
    """Fail before issuing the next move if measured joints mismatch."""
    errors = _pose_joint_errors(measured, expected, label=label)
    if max(errors.values()) > tolerance_deg:
        worst_joint = max(errors, key=errors.__getitem__)
        raise RuntimeError(
            f"{label}: not within {tolerance_deg:g} deg of expected pose "
            f"({worst_joint} differs by {errors[worst_joint]:.2f} deg); "
            "reposition manually only if safe, or use the short trial's "
            "explicit measured-start option; do not auto-correct"
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


def _preflight_joints_within_limits(
    measured: Mapping[str, float],
    limits: Mapping[str, tuple[float, float]],
) -> None:
    """Refuse invalid or uncalibrated measured poses before any Sleep move."""
    if not limits:
        raise ValueError("Sleep preflight has no executable joint limits")
    for name, (lower, upper) in limits.items():
        if name not in measured:
            raise ValueError(f"Sleep preflight: measured joint {name} is missing")
        value = float(measured[name])
        if not all(math.isfinite(number) for number in (value, lower, upper)):
            raise ValueError(f"Sleep preflight: nonfinite measured {name} or limit")
        if not lower <= value <= upper:
            raise RuntimeError(
                f"Sleep preflight: {name} is outside executable calibrated range"
            )


def _preflight_short_joint(
    measured: Mapping[str, float],
    limits: Mapping[str, tuple[float, float]],
) -> dict[str, float | str]:
    """Accept only the fixed pan move from a validated measured posture.

    All arm joints must be present and within the active executable
    calibration; the moving pan joint also requires a 5° endpoint inset.
    """
    _preflight_joints_within_limits(measured, limits)
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


def _load_compatible_overhead(
    robot_id: str, calibration_id: str,
) -> Mapping[str, float]:
    """Use only the existing calibration-bound saved pose."""
    pose = PoseLibrary(robot_id).require(OVERHEAD_POSE)
    require_calibration_compatibility(
        {
            "source_robot_id": pose.source_robot_id,
            "source_calibration_id": pose.source_calibration_id,
            "target_robot_id": pose.target_robot_id,
            "target_calibration_id": pose.target_calibration_id,
        },
        current_robot_id=robot_id,
        current_calibration_id=calibration_id,
        artifact_label=OVERHEAD_POSE,
    )
    return pose.joints


def _preflight_overhead_single(
    measured: Mapping[str, float],
    target: Mapping[str, float],
    limits: Mapping[str, tuple[float, float]],
) -> dict[str, float]:
    """Fail closed on large or near-stop saved Overhead trajectories."""
    if not measured or set(measured) != set(target) or set(target) != set(limits):
        raise ValueError("Overhead trial requires matching measured, pose and limit joints")
    _preflight_joints_within_limits(measured, limits)
    delta: dict[str, float] = {}
    inset = math.radians(OVERHEAD_SINGLE_LIMIT_INSET_DEG)
    for name in measured:
        start = float(measured[name])
        end = float(target[name])
        lower, upper = limits[name]
        if not all(math.isfinite(x) for x in (start, end, lower, upper)):
            raise ValueError(f"nonfinite Overhead pose/limit for {name}")
        if not (lower + inset <= start <= upper - inset
                and lower + inset <= end <= upper - inset):
            raise RuntimeError(
                f"Overhead trial: {name} starts or ends within "
                f"{OVERHEAD_SINGLE_LIMIT_INSET_DEG:g} deg of calibrated stop"
            )
        degrees = abs(math.degrees(end - start))
        if degrees > OVERHEAD_SINGLE_MAX_JOINT_TRAVEL_DEG:
            raise RuntimeError(
                f"Overhead trial: {name} has {degrees:.2f} deg travel, "
                f"exceeding {OVERHEAD_SINGLE_MAX_JOINT_TRAVEL_DEG:g} deg limit"
            )
        delta[name] = math.degrees(end - start)
    if max(abs(value) for value in delta.values()) < OVERHEAD_SINGLE_MIN_TRAVEL_DEG:
        raise RuntimeError("saved Overhead is too close to current pose for STOP trial")
    if sum(abs(value) >= OVERHEAD_SINGLE_MIN_TRAVEL_DEG for value in delta.values()) < 2:
        raise RuntimeError(
            "Overhead trial requires at least two significant moving joints"
        )
    return delta


def _assess_overhead_inflight_feedback(
    windows: Mapping[str, Mapping[str, Any]],
    *,
    start_joints: Mapping[str, float],
    target_joints: Mapping[str, float],
) -> dict[str, Any]:
    """Assess a *multi-joint* STOP using only passively recorded encoders.

    Each moving joint is normalized into its direction of travel so the
    existing 0.5-deg progress / 2-deg remaining / 0.15-deg recent motion
    evidence gates are identical for positive and negative joint rotation.
    """
    findings: dict[str, dict[str, Any]] = {}
    observed: list[str] = []
    reasons: list[str] = []
    if set(start_joints) != set(target_joints):
        raise ValueError("Overhead evidence joints changed")
    for name, start in start_joints.items():
        delta_deg = math.degrees(float(target_joints[name]) - float(start))
        if abs(delta_deg) < OVERHEAD_SINGLE_MIN_TRAVEL_DEG:
            continue
        window = windows.get(name)
        if window is None:
            reasons.append(f"{name}: missing passive feedback")
            continue
        direction = 1.0 if delta_deg > 0 else -1.0
        oriented = {
            **window,
            "samples": [
                {**sample, "rad": direction * float(sample["rad"])}
                for sample in window.get("samples", [])
            ],
        }
        evidence = _assess_physical_inflight_feedback(
            oriented,
            start_pan_rad=direction * float(start),
            target_delta_deg=abs(delta_deg),
        )
        findings[name] = evidence
        if evidence["verified"]:
            observed.append(name)
    if len(observed) < 2:
        reasons.append(
            "fewer than two joints had fresh moving-encoder evidence with "
            "at least 2 degrees of travel remaining"
        )
    return {
        "verified": not reasons,
        "reasons": reasons,
        "observed_moving_joints": observed,
        "by_joint": findings,
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


def _authorize_sleep_preparation(
    *,
    state: Any,
    motion_active: bool,
    confirm: Callable[[str], str],
) -> None:
    """Require healthy stationary HOLD and human clearance for the Sleep route.

    This does not claim validated physical clearance from a paper workspace.
    The SDK owns the canonical path and limits, not the trial script.
    """
    if (
        not state.connected or state.faulted
        or not state.torque_enabled or state.moving or motion_active
    ):
        raise RuntimeError(
            "Sleep preparation requires a stationary, healthy torque HOLD; "
            f"connected={state.connected}, torque_enabled={state.torque_enabled}, "
            f"moving={state.moving}, faulted={state.faulted}, "
            f"host_motion_active={motion_active}, "
            f"fault_message={getattr(state, 'fault_message', None)}"
        )
    print(
        "Preparing calibrated Sleep from the CURRENT encoder position. "
        "Inspect the full multi-joint route to Sleep, floor, table, "
        "payload, cabling and physical power cutoff. The clearance "
        "cannot be certified from the current workspace map.",
        flush=True,
    )
    if confirm(
        "Type PREPARE_SLEEP to authorize ONE joint-only calibrated Sleep move: "
    ).strip() != "PREPARE_SLEEP":
        raise PermissionError("operator declined canonical Sleep preparation")


def _assess_physical_inflight_feedback(
    window: Mapping[str, Any],
    *,
    start_pan_rad: float,
    target_delta_deg: float = SHORT_DELTA_DEG,
) -> dict[str, Any]:
    """Evidence gate; does not command hardware or approve a second attempt.

    The previous apparent STOP success occurred after a pan encoder had
    stopped changing and hardware reported Moving=false. Require fresh,
    changing actual encoder feedback plus substantial remaining displacement
    AT THE OPERATOR'S STOP KEYPRESS. Never infer motion from a host controller
    flag or the Feetech Moving bit alone: the latter has reported false while
    real encoder positions increased under slow commanded motion.
    """
    captured = float(window["captured_s"])
    samples = list(window.get("samples", []))
    moving = list(window.get("hardware_moving_samples", []))
    reasons: list[str] = []
    warnings: list[str] = []
    if len(samples) < 2:
        reasons.append("insufficient passive encoder samples")
    if not math.isfinite(start_pan_rad) or not math.isfinite(captured):
        reasons.append("nonfinite starting pose or observation time")
    age = None
    progress = None
    remaining = None
    recent_encoder_deg = None
    last_change_age = None
    last_hardware_moving = None
    if samples and math.isfinite(captured) and math.isfinite(start_pan_rad):
        final = samples[-1]
        last_rad = float(final["rad"])
        age = captured - float(final["elapsed_s"])
        progress = math.degrees(last_rad - start_pan_rad)
        remaining = target_delta_deg - progress
        if not all(math.isfinite(x) for x in (last_rad, age, progress, remaining)):
            reasons.append("nonfinite encoder feedback")
        else:
            if age < 0 or age > 0.12:
                reasons.append("encoder sample too stale at STOP")
            if progress < 0.5:
                reasons.append("insufficient measured pan progress before STOP")
            if remaining < 2.0:
                reasons.append("STOP requested too near pan target")
            recent = [
                sample for sample in samples
                if captured - float(sample["elapsed_s"]) <= 0.25
            ]
            if len(recent) < 2:
                reasons.append("insufficient recent pan feedback")
            else:
                recent_encoder_deg = abs(
                    math.degrees(last_rad - float(recent[0]["rad"]))
                )
                if recent_encoder_deg < 0.15:
                    reasons.append("pan encoder did not show recent movement")
            # Date the CHANGE at the *new* encoder sample, not the previous
            # sample with a different value. Otherwise 10-Hz feedback adds an
            # artificial ~100-ms age and rejects actual moving-servo evidence.
            change_times = [
                float(current["elapsed_s"])
                for previous, current in zip(samples, samples[1:])
                if int(previous["raw"]) != int(current["raw"])
            ]
            if change_times:
                last_change_age = captured - change_times[-1]
            if last_change_age is None or not 0 <= last_change_age <= 0.12:
                reasons.append("pan encoder not changing immediately before STOP")
    latest_status = moving[-1] if moving else None
    if latest_status is not None:
        moving_age = captured - float(latest_status["elapsed_s"])
        last_hardware_moving = bool(latest_status["moving"])
        if moving_age < 0 or moving_age > 0.16:
            reasons.append("servo Moving status too stale")
        if not last_hardware_moving:
            # The 2026-10-12 08:23 operator trace had four fresh Moving=false
            # reads DURING 0.62 degrees of measured pan encoder travel over
            # 0.30 seconds. Keep a fresh status observation as diagnostics,
            # but don't override independently observed encoder movement.
            warnings.append("servo Moving bit was false during measured pan motion")
    else:
        reasons.append("no passive servo Moving status available")
    return {
        "verified": not reasons,
        "reasons": reasons,
        "warnings": warnings,
        "capture_elapsed_s": captured,
        "last_feedback_age_s": age,
        "progress_deg": progress,
        "remaining_deg": remaining,
        "recent_encoder_motion_deg": recent_encoder_deg,
        "last_encoder_change_age_s": last_change_age,
        "last_hardware_moving": last_hardware_moving,
        "feedback_count": len(samples),
    }


def _execute_prepared_sleep(
    request: Callable[[str, str, dict[str, Any] | None], dict[str, Any]],
    *,
    sleep_target: Mapping[str, float],
    expected_session_id: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """One canonical broker Sleep then measured verification, no pan yet."""
    reply = request("POST", "/v1/sleep", {"joint_only": True})
    result = reply["result"]
    if not (result.get("accepted") and result.get("completed")):
        raise RuntimeError("Sleep preparation did not complete")
    measured = request("GET", "/v1/state", None)["result"]
    if measured["broker_session"]["id"] != expected_session_id:
        raise RuntimeError("Sleep preparation changed broker SDK session")
    if not measured["authority"]["armed"]:
        raise RuntimeError("Sleep preparation lost calibration-bound authority")
    errors = _verify_measured_pose(
        measured["joint_positions_rad"], sleep_target,
        label="prepared Sleep",
    )
    return measured, {
        "request_id": reply["request_id"], "action": "sleep",
        "joint_only": True, "completed": True,
        "joint_errors_deg": errors,
    }


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


def _physical_trial_profile(trial: str) -> CapabilityProfile:
    """Build the exact capability envelope the trusted broker will accept."""
    if trial not in ("short", "overhead", "overhead-single"):
        raise ValueError(f"unknown supervised STOP trial {trial!r}")
    return CapabilityProfile.from_document({
        "schema_version": 1, "name": "physical-supervised-stop-qualification",
        "tools": (
            ["robot_health", "robot_capabilities", "robot_state",
             "sleep", "jog_joint", "stop"]
            if trial == "short"
            else (["robot_health", "robot_capabilities", "robot_state",
                   "go_pose", "stop"]
                  if trial == "overhead-single"
                  else ["robot_health", "robot_capabilities", "robot_state",
                        "go_pose", "sleep", "stop"])
        ),
        "cameras": [],
        "limits": (
            {"max_joint_delta_deg": SHORT_DELTA_DEG}
            if trial == "short" else {}
        ),
    })


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--robot-id", default="so101")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--trial", choices=("short", "overhead-single", "overhead"), default="short",
        help="short is single pan; overhead-single is one faster saved-pose movement; "
             "overhead is the older multi-leg route",
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
        if args.command_frequency_hz > 20:
            raise ValueError("short STOP command frequency must be at most 20 Hz")
    if args.trial == "overhead-single":
        speed = (OVERHEAD_SINGLE_DEFAULT_SPEED_DEG_S
                 if args.speed_deg_s is None else args.speed_deg_s)
        accel = (OVERHEAD_SINGLE_DEFAULT_ACCEL_DEG_S2
                 if args.acceleration_deg_s2 is None else args.acceleration_deg_s2)
        if (speed > OVERHEAD_SINGLE_MAX_SPEED_DEG_S
                or accel > OVERHEAD_SINGLE_MAX_ACCEL_DEG_S2):
            raise ValueError(
                "overhead-single rate caps: at most "
                "8 deg/s and 40 deg/s²"
            )
        if args.command_frequency_hz > 20:
            raise ValueError("overhead-single command frequency must be at most 20 Hz")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _require_operator(args)  # Before touching files, devices or a lease.
    if args.speed_deg_s is None:
        args.speed_deg_s = (
            2.0 if args.trial == "short"
            else (OVERHEAD_SINGLE_DEFAULT_SPEED_DEG_S
                  if args.trial == "overhead-single" else 8.0)
        )
    if args.acceleration_deg_s2 is None:
        args.acceleration_deg_s2 = (
            10.0 if args.trial == "short"
            else (OVERHEAD_SINGLE_DEFAULT_ACCEL_DEG_S2
                  if args.trial == "overhead-single" else 25.0)
        )
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
    profile = _physical_trial_profile(args.trial)
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
        "initial_pose_policy": (
            "broker_prepares_and_verifies_sleep"
            if args.trial == "short"
            else ("measured_start_direct_overhead" if args.trial == "overhead-single"
                  else "overhead_sleep")
        ),
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
        measured_joints = state["joint_positions_rad"]
        errors = _pose_joint_errors(
            measured_joints, sleep_target, label="initial Sleep comparison",
        )
        summary["start_sleep_errors_deg"] = errors
        summary["initial_joint_positions_deg"] = {
            joint: math.degrees(float(position))
            for joint, position in measured_joints.items()
        }
        summary["calibration_id"] = calibration_id
        summary["broker_session"] = state["broker_session"]
        if state["broker_session"]["sdk_connect_count"] != 1:
            raise RuntimeError("unexpected SDK connect count")

        summary["baseline_legs"] = []
        print("Clear arm workspace; remove payload; keep power cutoff reachable.", flush=True)
        print("This is NOT an emergency-stop certification.", flush=True)
        print("Measured starting joints versus calculated Sleep (degrees):", flush=True)
        for joint, error in errors.items():
            print(
                f"  {joint:16s} actual={math.degrees(measured_joints[joint]):+8.2f}"
                f" Sleep={math.degrees(sleep_target[joint]):+8.2f}"
                f" difference={error:6.2f}",
                flush=True,
            )
        if args.trial == "short":
            # Never issue an unrestricted move to Sleep. Require a measured,
            # calibrated, physically stationary start and an explicit human
            # approval of the FULL joint-space path first. The canonical
            # SDK Sleep primitive, not bespoke motor targets, owns execution.
            limits = sdk_arm.motion._effective_limits()
            _preflight_joints_within_limits(measured_joints, limits)
            hold_state = sdk_arm.get_state()
            summary["preparation_hardware_state"] = {
                "connected": hold_state.connected,
                "torque_enabled": hold_state.torque_enabled,
                "moving": hold_state.moving,
                "faulted": hold_state.faulted,
                "fault_message": hold_state.fault_message,
                "host_motion_active": bool(sdk_arm.motion.is_moving),
            }
            if (
                not hold_state.connected or hold_state.faulted
                or not hold_state.torque_enabled or hold_state.moving
                or sdk_arm.motion.is_moving
            ):
                # Diagnostics only on failure: identify WHICH servo reports
                # Moving/Status, without sending new goals or changing torque.
                try:
                    summary["preparation_motor_diagnostics"] = [
                        {
                            "name": item.name,
                            "moving": item.moving,
                            "status": item.status,
                            "position_raw": item.position_raw,
                            "error": item.error,
                        }
                        for item in sdk_arm.backend.diagnostics()
                    ]
                except Exception as exc:
                    summary["preparation_diagnostic_error"] = repr(exc)
            _authorize_sleep_preparation(
                state=hold_state,
                motion_active=bool(sdk_arm.motion.is_moving),
                confirm=input,
            )
            summary["prepare_sleep_operator_confirmed"] = True
            print(
                "Executing ONE joint-only canonical Sleep through the trusted "
                "broker; no gripper motion or separate CLI session.",
                flush=True,
            )
            prepared, sleep_record = _execute_prepared_sleep(
                lambda method, path, payload: _request(
                    url, token, method, path, payload, timeout_s=200.0,
                ),
                sleep_target=sleep_target,
                expected_session_id=summary["broker_session"]["id"],
            )
            summary["prepared_sleep_errors_deg"] = sleep_record["joint_errors_deg"]
            summary["sleep_preparation"] = sleep_record
            summary["short_preflight"] = _preflight_short_joint(
                prepared["joint_positions_rad"], limits,
            )
            summary["short_start_pan_rad"] = float(
                prepared["joint_positions_rad"][SHORT_JOINT]
            )
            summary["route"] = [
                "human-confirmed joint-only Sleep from measured posture",
                "verify encoder-measured Sleep",
                "shoulder_pan +8 deg (manual STOP while moving)",
            ]
            movement_path = "/v1/joint"
            movement_payload = {
                "joint": SHORT_JOINT,
                "delta_deg": SHORT_DELTA_DEG,
            }
            print(
                "Prepared Sleep verified. SHORT STOP trial: one +8 deg "
                f"shoulder-pan move at {args.speed_deg_s:g} deg/s. "
                "Press ENTER EARLY while pan is visibly moving, not at "
                "the end. No automatic return or REARM.",
                flush=True,
            )
        elif args.trial == "overhead-single":
            # No preparatory Sleep/Overhead roundtrip; validate the measured
            # current pose against the already-saved calibration-bound target.
            limits = sdk_arm.motion._effective_limits()
            overhead_target = _load_compatible_overhead(
                args.robot_id, calibration_id,
            )
            deltas = _preflight_overhead_single(
                measured_joints, overhead_target, limits,
            )
            summary["overhead_delta_by_joint_deg"] = deltas
            hold = sdk_arm.get_state()
            summary["preparation_hardware_state"] = {
                "connected": hold.connected,
                "torque_enabled": hold.torque_enabled,
                "moving": hold.moving,
                "faulted": hold.faulted,
                "fault_message": hold.fault_message,
                "host_motion_active": bool(sdk_arm.motion.is_moving),
            }
            if (not hold.connected or hold.faulted or not hold.torque_enabled
                    or hold.moving or sdk_arm.motion.is_moving):
                try:
                    summary["preparation_motor_diagnostics"] = [
                        {
                            "name": item.name, "moving": item.moving,
                            "status": item.status, "position_raw": item.position_raw,
                            "error": item.error,
                        }
                        for item in sdk_arm.backend.diagnostics()
                    ]
                except Exception as exc:
                    summary["preparation_diagnostic_error"] = repr(exc)
                raise RuntimeError(
                    "one-shot Overhead requires healthy stationary torque HOLD"
                )
            print(
                "One-shot Overhead: CURRENT measured posture -> "
                f"{OVERHEAD_POSE} (joint-only, no gripper) at "
                f"{args.speed_deg_s:g} deg/s, {args.acceleration_deg_s2:g} deg/s².",
                flush=True,
            )
            for joint, difference in deltas.items():
                print(f"  {joint:16s} travel {difference:+.2f} deg", flush=True)
            print(
                "Inspect the entire swept Overhead route, cables, table and "
                "reachable power cutoff before authorizing it. "
                "No automatic return, recovery or second motor goal.",
                flush=True,
            )
            summary["route"] = [
                "measured calibrated current posture; no preparatory motion",
                "human-approved single saved Overhead (STOP while moving)",
            ]
            movement_path = "/v1/go-pose"
            movement_payload = {"name": OVERHEAD_POSE, "joint_only": True}
        else:
            # Preserve the historical Sleep-only gate for the larger route.
            _verify_measured_pose(
                measured_joints, sleep_target, label="initial Sleep",
            )
            print("Verified measured Sleep; no initial movement.", flush=True)
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

        start_word = "OVERHEAD_START" if args.trial == "overhead-single" else "START"
        if input(
            f"Type {start_word} to begin the ONE movement you will interrupt: "
        ).strip() != start_word:
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
        # Passive tracer snapshots the actual encoder / hardware state
        # before independent STOP, without ANY additional motor bus reads.
        # Always issue the requested STOP even if this evidence is weak.
        pre_stop_window = None
        overhead_windows = None
        if args.trial in {"short", "overhead-single"}:
            if executor._trace is None:
                raise RuntimeError("STOP evidence recorder unavailable")
            if args.trial == "short":
                pre_stop_window = executor._trace.feedback_window(
                    SHORT_JOINT, window_s=0.35,
                )
                summary["pre_stop_passive_window"] = pre_stop_window
            else:
                overhead_windows = {
                    joint: executor._trace.feedback_window(joint, window_s=0.35)
                    for joint in overhead_target
                }
                summary["pre_stop_passive_windows"] = overhead_windows
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
        if args.trial == "short":
            if pre_stop_window is None:
                raise RuntimeError("missing passive physical movement evidence")
            evidence = _assess_physical_inflight_feedback(
                pre_stop_window,
                start_pan_rad=summary["short_start_pan_rad"],
            )
            summary["physical_inflight_stop_evidence"] = evidence
            if not evidence["verified"]:
                raise RuntimeError(
                    "STOP was stable but moving-servo braking NOT qualified: "
                    + "; ".join(evidence["reasons"])
                )
        if args.trial == "overhead-single":
            if overhead_windows is None:
                raise RuntimeError("missing multi-joint passive STOP evidence")
            evidence = _assess_overhead_inflight_feedback(
                overhead_windows,
                start_joints=measured_joints,
                target_joints=overhead_target,
            )
            summary["physical_inflight_stop_evidence"] = evidence
            if not evidence["verified"]:
                raise RuntimeError(
                    "STOP was stable but multi-joint Overhead braking "
                    "NOT qualified: " + "; ".join(evidence["reasons"])
                )
        summary["stop_qualification_passed"] = True
        summary["passed"] = True

        if args.trial in {"short", "overhead-single"}:
            # No rearm or second motor command, even after a verified hold.
            summary["post_stop_recovery_requested"] = False
            print(
                "ONE-SHOT STOP gate complete. No REARM or new position goal. "
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
