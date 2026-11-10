"""Trusted-host bounded live joint stream (breathing, listening, talking).

A pinned choreography is a finished trajectory: it starts and ends at rest, so
presence motion built from them pauses between programs and cannot follow
live audio.  This module lets the supervised trusted host send *setpoints*
inside an operator-pinned envelope instead:

* The operator pins a :class:`JointEnvelope`: a centre pose (neutral) and a
  small per-joint offset range (at most 15 degrees) by SHA-256, exactly like a
  choreography or joint target.
* The host sends offsets.  The SDK clamps them to the envelope, then slews its
  own command toward them under the pinned speed and acceleration ceilings, so
  a bug or burst on the host side cannot make a jump.
* Every sample goes through the existing guarded stream
  (``MotionController.stream_joint_target``): joint limits, step/speed/
  acceleration checks, feedback monitoring.  The host watchdog is checked on
  every sample, and a STOP ends the stream.
* If setpoints stop arriving the stream returns to the centre pose by itself.

There is no gripper motion, no raw servo access and no agent route.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import re
import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.hardware import SimulationBackend
from soarm101_motion.safety import validate_workspace_path_from_measured_start

if TYPE_CHECKING:
    from soarm101_motion.broker_sdk import SDKAgentExecutor
    from soarm101_motion.broker_watchdog import BrokerWatchdog
    from soarm101_motion.choreography import ChoreographyPolicy

LIVE_STREAM_HZ = 20.0
# No setpoint for this long: head back to the centre pose on our own.
SETPOINT_TIMEOUT_S = 0.6
MAX_SESSION_S = 3600.0
MAX_ENVELOPE_RAD = math.radians(15.0)
CLOSE_TIMEOUT_S = 6.0
_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$")
_REV = re.compile(r"^sha256:[0-9a-f]{64}$")
# Stay under the SDK's own 1.001 tolerance on speed/acceleration.
_RATE_MARGIN = 0.95
_BRAKE_MARGIN = 0.8
_APPROACH_GAIN_PER_S = 8.0


@dataclass(frozen=True)
class JointEnvelope:
    """Immutable operator-pinned centre pose and per-joint offset range."""

    name: str
    robot_id: str
    calibration_id: str
    center_rad: tuple[float, ...]
    max_offset_rad: tuple[float, ...]
    revision: str

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not _ID.fullmatch(self.name):
            raise ValueError("invalid joint envelope name")
        if any(not isinstance(x, str) or not x or len(x) > 256
               for x in (self.robot_id, self.calibration_id)):
            raise ValueError("robot and calibration identity required")
        for label, values in (("center", self.center_rad), ("offset", self.max_offset_rad)):
            if (not isinstance(values, tuple) or len(values) != len(ARM_JOINTS)
                    or any(isinstance(x, bool) or not isinstance(x, (int, float))
                           or not math.isfinite(x) for x in values)):
                raise ValueError(f"envelope {label} requires all five finite arm joints")
        if any(not 0.0 <= x <= MAX_ENVELOPE_RAD for x in self.max_offset_rad):
            raise ValueError("envelope offsets must be within 0..15 degrees per joint")
        if not any(x > 0 for x in self.max_offset_rad):
            raise ValueError("envelope needs at least one joint with a nonzero range")
        if not isinstance(self.revision, str) or not _REV.fullmatch(self.revision):
            raise ValueError("joint envelope requires exact SHA-256 revision")
        if self.revision != self.content_revision():
            raise ValueError("joint envelope content differs from pinned SHA-256 revision")

    def content_revision(self) -> str:
        return _revision(self.name, self.robot_id, self.calibration_id,
                         self.center_rad, self.max_offset_rad)

    @classmethod
    def author(cls, name: str, robot_id: str, calibration_id: str,
               center_rad: tuple[float, ...], max_offset_rad: tuple[float, ...]) -> JointEnvelope:
        return cls(name, robot_id, calibration_id, tuple(center_rad), tuple(max_offset_rad),
                   _revision(name, robot_id, calibration_id, tuple(center_rad),
                             tuple(max_offset_rad)))


def _revision(name, robot_id, calibration_id, center_rad, max_offset_rad) -> str:
    payload = {
        "schema": "soarm101.joint_envelope.v1", "name": name, "robot_id": robot_id,
        "calibration_id": calibration_id,
        "center_rad": [float(x) for x in center_rad],
        "max_offset_rad": [float(x) for x in max_offset_rad],
    }
    return "sha256:" + hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def slew_step(position: float, velocity: float, target: float,
              max_speed: float, max_acceleration: float, dt: float) -> tuple[float, float]:
    """One acceleration-limited step toward ``target`` that can always stop in time."""
    error = target - position
    braking = math.sqrt(2.0 * _BRAKE_MARGIN * max_acceleration * abs(error))
    wanted = math.copysign(
        min(max_speed * _RATE_MARGIN, _APPROACH_GAIN_PER_S * abs(error), braking), error)
    limit = max_acceleration * _RATE_MARGIN * dt
    velocity = velocity + max(-limit, min(limit, wanted - velocity))
    return position + velocity * dt, velocity


class LiveJointStream:
    """One supervised stream session; created and owned by the trusted-host broker."""

    def __init__(self, executor: SDKAgentExecutor, watchdog: BrokerWatchdog,
                 policy: ChoreographyPolicy, envelope: JointEnvelope,
                 *, hz: float = LIVE_STREAM_HZ, setpoint_timeout_s: float = SETPOINT_TIMEOUT_S,
                 max_session_s: float = MAX_SESSION_S) -> None:
        self.executor, self.watchdog, self.policy, self.envelope = executor, watchdog, policy, envelope
        self.hz, self.setpoint_timeout_s, self.max_session_s = hz, setpoint_timeout_s, max_session_s
        self._lock = threading.Lock()
        self._offsets = [0.0] * len(ARM_JOINTS)
        self._setpoint_at = 0.0
        self._closing = False
        self._done = threading.Event()
        self._thread: threading.Thread | None = None
        self._arm = None
        self._position: list[float] = []
        self._velocity = [0.0] * len(ARM_JOINTS)
        self._opened_at = 0.0
        self._samples = 0
        self._clamped = 0
        self.error: str | None = None
        self.stop_outcome: dict[str, object] | None = None

    # -- opening -----------------------------------------------------------------
    def open(self) -> dict[str, object]:
        """Validate, start the guarded SDK stream from the measured pose, start sampling."""
        from soarm101_motion.choreography import _require_choreography_authority

        executor, watchdog, policy, envelope = self.executor, self.watchdog, self.policy, self.envelope
        if (getattr(executor.motion_guard, "__self__", None) is not watchdog
                or watchdog._worker is None or not watchdog._worker.is_alive()):
            raise PermissionError("active bound host watchdog required")
        if not (executor.simulation and executor.simulation_actions):
            if not (executor.physical_actions and policy.allow_physical_trial):
                raise PermissionError("physical live stream is not operator-qualified")
        if (policy.robot_id != executor.robot_id or envelope.robot_id != policy.robot_id
                or envelope.calibration_id != policy.calibration_id
                or envelope.revision not in policy.allowed_revisions):
            raise PermissionError("joint envelope is not pinned for this robot and calibration")
        if (policy.max_speed_deg_s > executor.rates.joint_speed_deg_s
                or policy.max_acceleration_deg_s2 > executor.rates.joint_acceleration_deg_s2):
            raise PermissionError("live stream cannot widen trusted host motion rates")
        with executor._dispatch_lock:
            executor._require_active()
            arm = executor._session
            if arm is None:
                raise PermissionError("preexisting connected SDK session required")
            executor._assert_session_identity(arm)
            watchdog.check()
            _require_choreography_authority(executor, arm)
            simulation = isinstance(arm.backend, SimulationBackend)
            calibration = "simulation" if simulation else arm.calibration_id
            if calibration != policy.calibration_id or envelope.robot_id != arm.config.robot_id:
                raise PermissionError("active SDK identity does not match the reviewed envelope")
            if not simulation and not arm.config.enable_workspace_checks:
                raise PermissionError("physical live stream requires enabled workspace validation")
            state = arm.get_state()
            if not state.connected or state.faulted or state.moving:
                raise PermissionError("healthy, connected and stationary SDK state required")
            if not simulation and not state.torque_enabled:
                raise PermissionError("operator must already enable physical torque HOLD")
            if arm.tool.is_moving:
                raise PermissionError("tool is already moving; live stream never commands the gripper")
            actual = arm.get_joint_positions().positions
            start = [float(actual[name]) for name in ARM_JOINTS]
            error = max(abs(a - c) for a, c in zip(start, envelope.center_rad, strict=True))
            if error > policy.max_start_error_rad:
                raise PermissionError("measured start is not at the pinned envelope centre")
            self._check_workspace(arm)
            speed = min(math.radians(policy.max_speed_deg_s), arm.config.stream_joint_speed_limit)
            acceleration = min(math.radians(policy.max_acceleration_deg_s2),
                               arm.config.stream_joint_acceleration_limit)
            self._speed, self._acceleration = speed, acceleration
            with watchdog._lock:
                watchdog.check()
                _require_choreography_authority(executor, arm)
                if not simulation:
                    executor._motion_action_attempted = True
                    executor._verified_physical_stop = False
                elif not state.torque_enabled:
                    arm.enable()
                baseline = arm.start_joint_stream(
                    frequency_hz=self.hz, max_speed=speed, max_acceleration=acceleration)
            self._arm = arm
            self._position = [float(baseline[name]) for name in ARM_JOINTS]
            self._offsets = [
                max(-limit, min(limit, p - c)) for p, c, limit in
                zip(self._position, envelope.center_rad, envelope.max_offset_rad, strict=True)]
            now = time.monotonic()
            self._opened_at = self._setpoint_at = now
        self._thread = threading.Thread(target=self._run, name="live-joint-stream", daemon=True)
        self._thread.start()
        return self.status()

    def _check_workspace(self, arm) -> None:
        center = self.envelope.center_rad
        spans = [(-o, o) if o > 0 else (0.0,) for o in self.envelope.max_offset_rad]
        samples = []
        for corner in itertools.product(*spans):
            for fraction in (0.0, 0.5, 1.0):
                samples.append({name: center[i] + corner[i] * fraction
                                for i, name in enumerate(ARM_JOINTS)})
        validate_workspace_path_from_measured_start(
            arm.model, samples, tcp=arm.active_tcp, **arm._workspace_kwargs())

    # -- host calls ----------------------------------------------------------------
    @property
    def active(self) -> bool:
        return self._thread is not None and not self._done.is_set() and not self._closing

    def set_offsets(self, offsets_rad: list[float]) -> dict[str, object]:
        """Record the latest requested offsets (clamped to the envelope)."""
        if not self.active:
            raise PermissionError("live stream is not active")
        if (not isinstance(offsets_rad, list) or len(offsets_rad) != len(ARM_JOINTS)
                or any(isinstance(x, bool) or not isinstance(x, (int, float))
                       or not math.isfinite(x) for x in offsets_rad)):
            raise ValueError("offsets require five finite numbers in canonical joint order")
        clamped = False
        values = []
        for value, limit in zip(offsets_rad, self.envelope.max_offset_rad, strict=True):
            bounded = max(-limit, min(limit, float(value)))
            clamped = clamped or bounded != float(value)
            values.append(bounded)
        with self._lock:
            self._offsets = values
            self._setpoint_at = time.monotonic()
            self._clamped += int(clamped)
        return {"active": True, "clamped": clamped}

    def status(self) -> dict[str, object]:
        with self._lock:
            return {
                "active": self.active, "name": self.envelope.name,
                "revision": self.envelope.revision, "samples": self._samples,
                "clamped_setpoints": self._clamped, "error": self.error,
                "stop": self.stop_outcome,
                "position_rad": dict(zip(ARM_JOINTS, self._position, strict=False)),
            }

    def close(self) -> dict[str, object]:
        """Return to the centre pose, end the SDK stream and hold."""
        with self._lock:
            self._closing = True
        thread = self._thread
        if thread is not None:
            thread.join(timeout=CLOSE_TIMEOUT_S)
        if thread is not None and thread.is_alive():
            self._fail(TimeoutError("live stream did not return to centre in time"))
            thread.join(timeout=2.0)
        if self.error is not None:
            raise RuntimeError(f"live stream failed: {self.error}")
        return self.status()

    # -- sampling thread ---------------------------------------------------------
    def _target(self, now: float) -> tuple[list[float], bool]:
        with self._lock:
            offsets, stamp, closing = list(self._offsets), self._setpoint_at, self._closing
        recentre = closing or now - stamp > self.setpoint_timeout_s or now - self._opened_at > self.max_session_s
        centre = self.envelope.center_rad
        if recentre:
            return list(centre), closing
        return [c + o for c, o in zip(centre, offsets, strict=True)], False

    def _run(self) -> None:
        period = 1.0 / self.hz
        due = time.monotonic()
        try:
            while True:
                due += period
                target, closing = self._target(time.monotonic())
                self._tick(target)
                if closing and self._at_centre():
                    self._finish()
                    return
                delay = due - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -3 * period:
                    due = time.monotonic()
        except BaseException as exc:  # noqa: BLE001 - every failure must STOP
            self._fail(exc)
        finally:
            self._done.set()

    def _at_centre(self) -> bool:
        return all(abs(p - c) < 1e-3 and abs(v) < 0.02 for p, c, v in
                   zip(self._position, self.envelope.center_rad, self._velocity, strict=True))

    def _tick(self, target: list[float]) -> None:
        from soarm101_motion.choreography import _require_choreography_authority

        dt = 1.0 / self.hz
        steps = [slew_step(p, v, t, self._speed, self._acceleration, dt)
                 for p, v, t in zip(self._position, self._velocity, target, strict=True)]
        positions = [min(c + o, max(c - o, s[0])) for s, c, o in
                     zip(steps, self.envelope.center_rad, self.envelope.max_offset_rad, strict=True)]
        executor, watchdog, arm = self.executor, self.watchdog, self._arm
        if not executor._dispatch_lock.acquire(timeout=0.5):
            return  # another request holds the lock; try again next tick
        try:
            executor._require_active()
            watchdog.check()
            executor._assert_session_identity(arm)
            _require_choreography_authority(executor, arm)
            arm.stream_joint_target(dict(zip(ARM_JOINTS, positions, strict=True)))
        finally:
            executor._dispatch_lock.release()
        with self._lock:
            self._velocity = [(new - old) / dt for new, old in zip(positions, self._position, strict=True)]
            self._position = positions
            self._samples += 1

    def _finish(self) -> None:
        with self.executor._dispatch_lock:
            self._arm.stop_joint_stream(hold=True)

    def _fail(self, exc: BaseException) -> None:
        from soarm101_motion.simulated_replay import _stop_after_failure

        if self.error is None:
            self.error = f"{type(exc).__name__}: {str(exc)[:300]}"
        status = self.watchdog.status
        if status.get("state") in {"tripped", "stopped"}:
            self.stop_outcome = {"outcome": "already_stopped"}
        else:
            self.stop_outcome = _stop_after_failure(self.executor, self.watchdog)
        self._done.set()
