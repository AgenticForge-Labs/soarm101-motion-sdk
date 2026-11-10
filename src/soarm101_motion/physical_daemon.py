"""Operator-managed supervisor for bounded, SDK-owned physical actions.

This entry point is intentionally opt-in. An explicit operator-only startup
HOLD option may latch measured positions and enable torque through the SDK. The SDK
executor, authority store, broker watchdog and trusted catalog remain the only
physical execution path.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
import secrets
import signal
import threading
from pathlib import Path

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.broker import (
    AgentMotionRates, RobotBrokerHTTPServer, RobotBrokerService,
    load_trusted_choreography_catalog,
)
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.config import SOARM101Config

WATCHDOG_TIMEOUT_S = 2.0


def _heartbeat_loop(watchdog, stopped: threading.Event) -> None:
    while not stopped.wait(WATCHDOG_TIMEOUT_S / 4):
        try:
            watchdog.heartbeat()
        except Exception:
            return


def _load_config(path: Path) -> dict[str, object]:
    config = json.loads(path.read_text(encoding="utf-8"))
    required = {"physical", "robot_id", "port", "calibration_id", "lease_path",
                "operator_choreography_only",
                "trusted_catalog_path", "loopback_port", "state_dir"}
    if (not isinstance(config, dict) or not required <= set(config)
            or set(config) - required - {"establish_hold_on_startup", "joint_speed_deg_s", "joint_acceleration_deg_s2"}
            or config["physical"] is not True):
        raise ValueError("physical daemon requires exact config fields and physical: true")
    for key in ("robot_id", "port", "calibration_id"):
        if not isinstance(config[key], str) or not config[key].strip():
            raise ValueError(f"{key} must be a nonempty string")
    for key in (("trusted_catalog_path", "state_dir") if config["operator_choreography_only"]
                else ("lease_path", "trusted_catalog_path", "state_dir")):
        if not isinstance(config[key], str) or not Path(config[key]).is_absolute():
            raise ValueError(f"{key} must be an absolute path")
    if type(config["loopback_port"]) is not int or not 1 <= config["loopback_port"] <= 65535:
        raise ValueError("loopback_port must be in 1..65535")
    if type(config["operator_choreography_only"]) is not bool:
        raise ValueError("operator_choreography_only must be boolean")
    if type(config.get("establish_hold_on_startup", False)) is not bool:
        raise ValueError("establish_hold_on_startup must be boolean")
    if config.get("establish_hold_on_startup") and not config["operator_choreography_only"]:
        raise ValueError("startup HOLD is limited to the explicitly selected operator-only daemon")
    # The selected SDK host envelope remains authoritative. These trusted
    # operator values do not become agent-provided motion parameters.
    AgentMotionRates(joint_speed_deg_s=config.get("joint_speed_deg_s", 8.0),
        joint_acceleration_deg_s2=config.get("joint_acceleration_deg_s2", 25.0)).validated(SOARM101Config())
    return config


def _preflight(executor: SDKAgentExecutor, robot_id: str, calibration_id: str,
               authority: AgentAuthorityStore, *, require_hold: bool = True) -> dict[str, object]:
    """Connect/read only, then require exact identity, lease and sensed HOLD."""
    if not getattr(executor, "operator_choreography_only", False):
        authority.require(robot_id=robot_id, calibration_id=calibration_id)
    executor.execute("state", {})  # Opens the single bound session, read only.
    arm = executor._session
    if arm is None:
        raise PermissionError("physical preflight did not bind an SDK session")
    executor._assert_session_identity(arm)
    if (arm.config.robot_id != robot_id
            or executor._calibration_id(arm) != calibration_id):
        raise PermissionError("physical preflight robot/calibration identity mismatch")
    sensed = arm.get_state()
    if (sensed.connected is not True or sensed.faulted is not False
            or sensed.moving is not False or (require_hold and sensed.torque_enabled is not True)):
        raise PermissionError("physical preflight requires healthy stationary torque HOLD")
    return {"connected": sensed.connected, "faulted": sensed.faulted,
            "moving": sensed.moving, "torque_enabled": sensed.torque_enabled,
            "robot_id": robot_id, "calibration_id": calibration_id}


def _make_executor(config: dict[str, object], *, authority: AgentAuthorityStore,
                   executor_factory=None, start_tolerance_rad: float | None = None):
    default_tolerance = SOARM101Config().joint_position_tolerance_rad
    sdk_config = SOARM101Config(
        robot_id=str(config["robot_id"]), port=str(config["port"]),
        auto_enable_torque=False, configure_motors_on_connect=False,
        disable_torque_on_disconnect=False,
        # Completion must satisfy the reviewed next segment's entry contract.
        # The operator-reviewed policy controls both completion and admission.
        joint_position_tolerance_rad=(start_tolerance_rad
            if start_tolerance_rad is not None else default_tolerance),
    )
    rates = AgentMotionRates(joint_speed_deg_s=config.get("joint_speed_deg_s", 8.0),
        joint_acceleration_deg_s2=config.get("joint_acceleration_deg_s2", 25.0)).validated(sdk_config)
    factory = executor_factory or SDKAgentExecutor
    return factory(config=sdk_config, rates=rates, physical_actions=True,
                   operator_choreography_only=bool(config["operator_choreography_only"]),
                   authority_store=None if config["operator_choreography_only"] else authority)


def _shutdown_supervisor(service, executor) -> None:
    """Fence admission, then resolve owned motion before executor.close waits."""
    if executor is None or not getattr(executor, "_motion_action_attempted", False):
        return
    watchdog = getattr(service, "watchdog", None) if service is not None else None
    if watchdog is None:
        raise RuntimeError("owned physical dispatch has no watchdog for shutdown STOP")

    # This lock-free terminal fence also revokes new motion admission. STOP
    # remains available on the existing session and does not take the normal
    # dispatch lock, so it can interrupt an in-flight action.
    executor._fault(RuntimeError("physical daemon shutting down"))
    from soarm101_motion.simulated_replay import _stop_after_failure

    outcome = _stop_after_failure(executor, watchdog)
    if outcome.get("outcome") != "confirmed":
        raise RuntimeError(
            "physical daemon shutdown STOP/HOLD remains unconfirmed: "
            f"{outcome.get('detail', 'unknown outcome')}"
        )


def serve(config: dict[str, object], *, executor_factory=None) -> int:
    """Serve one selected physical robot. executor_factory is for offline tests."""
    state_dir = Path(str(config["state_dir"])).resolve()
    state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    os.chmod(state_dir, 0o700)
    if state_dir.stat().st_uid != os.getuid():
        raise RuntimeError("physical daemon state directory is not owned by this user")
    lock_fd = None
    owned_token_paths: list[Path] = []
    executor = None
    server = None
    service = None
    stopped = threading.Event()
    heart = None
    record = state_dir / "daemon.json"
    hold_failure = state_dir / "startup-hold-failure.json"
    quarantine = state_dir / "operator-reconciliation-required.json"
    if hold_failure.exists() or quarantine.exists():
        raise RuntimeError("prior physical outcome unresolved; operator reconciliation required")
    owns_record = False
    try:
        lock_fd = os.open(state_dir / "daemon.lock", os.O_CREAT | os.O_RDWR, 0o600)
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        operator_only = bool(config["operator_choreography_only"])
        authority = (None if operator_only else
                     AgentAuthorityStore(Path(str(config["lease_path"]))))
        if not operator_only:
            authority.require(robot_id=str(config["robot_id"]),
                              calibration_id=str(config["calibration_id"]))
        broker_token = secrets.token_urlsafe(32)
        trusted_token = secrets.token_urlsafe(32)
        if broker_token == trusted_token:
            raise RuntimeError("token generation collision")
        broker_token_path = state_dir / "broker.token"
        trusted_token_path = state_dir / "trusted-host.token"
        for path, token in ((broker_token_path, broker_token), (trusted_token_path, trusted_token)):
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            owned_token_paths.append(path)
            with os.fdopen(fd, "w") as stream:
                stream.write(token)
        policy, programs = load_trusted_choreography_catalog(
            Path(str(config["trusted_catalog_path"]))
        )
        if (policy.robot_id != config["robot_id"] or policy.calibration_id != config["calibration_id"]
                or not policy.allow_physical_trial):
            raise ValueError("trusted catalog must pin selected robot/calibration and allow physical trial")
        executor = _make_executor(config, authority=authority,
                                  executor_factory=executor_factory,
                                  start_tolerance_rad=policy.max_start_error_rad)
        profile = CapabilityProfile.from_document({
            "schema_version": 1, "name": "operator-physical-joint-supervisor",
            "tools": (["robot_health", "robot_capabilities", "robot_state", "stop"]
                      if operator_only else
                      ["robot_health", "robot_capabilities", "robot_state", "jog_joint", "stop"]),
            "cameras": [], "limits": {} if operator_only else {"max_joint_delta_deg": 2.0},
        })
        establish_hold = config.get("establish_hold_on_startup", False)
        if type(establish_hold) is not bool or (establish_hold and not operator_only):
            raise ValueError("startup HOLD requires an explicit operator-only configuration")
        # Read identity and health before any potential torque latch. The
        # optional latch runs only once supervision is active, before serving.
        try:
            _preflight(executor, str(config["robot_id"]), str(config["calibration_id"]), authority,
                       require_hold=not establish_hold)
        except BaseException:
            # No physical action has been dispatched; close only this owned session.
            executor.close()
            executor = None
            raise
        service = RobotBrokerService(
            executor=executor, token=broker_token, profile=profile,
            allow_physical_sdk_actions=not operator_only, watchdog_timeout_s=WATCHDOG_TIMEOUT_S,
            trusted_host_token=trusted_token, choreography_policy=policy,
            choreography_catalog=programs, event_path=state_dir / "events.jsonl",
        )
        server = RobotBrokerHTTPServer(("127.0.0.1", int(config["loopback_port"])), service)
        assert service.watchdog is not None
        service.watchdog.start()
        service.watchdog.heartbeat()
        heart = threading.Thread(target=_heartbeat_loop, args=(service.watchdog, stopped),
                             name="physical-sdk-heartbeat", daemon=True)
        heart.start()
        serving = threading.Event()
        def shutdown(_signum, _frame):
            if not serving.is_set():
                # Fence the startup worker immediately; the existing watchdog
                # supervisor performs independent STOP rather than waiting for
                # the latch worker or its dispatch lock.
                stopped.set()
                executor._fault(RuntimeError("operator interrupted startup HOLD"))
                threading.Thread(target=service._watchdog_trip,
                    args=("operator interrupted startup HOLD",), daemon=True).start()
            else:
                threading.Thread(target=server.shutdown, daemon=True).start()
        signal.signal(signal.SIGTERM, shutdown)
        signal.signal(signal.SIGINT, shutdown)
        if establish_hold:
            try:
                evidence = executor.establish_operator_startup_hold(service.watchdog)
                evidence_path = state_dir / "startup-hold.json"
                evidence_path.write_text(json.dumps(evidence) + "\n")
                os.chmod(evidence_path, 0o600)
            except BaseException as exc:
                if getattr(executor, "_startup_hold_attempted", False):
                    hold_failure.write_text(json.dumps({"error": type(exc).__name__,
                        "detail": str(exc), "retry_safe": False}) + "\n")
                    os.chmod(hold_failure, 0o600)
                raise
        executor._require_active()
        record_fd = os.open(record, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        owns_record = True
        with os.fdopen(record_fd, "w") as stream:
            stream.write(json.dumps({"pid": os.getpid(), "port": config["loopback_port"],
                                     "robot_id": config["robot_id"], "physical": True}) + "\n")
        serving.set()
        server.serve_forever(poll_interval=0.1)
    finally:
        stopped.set()
        if heart is not None:
            heart.join(timeout=2)
        shutdown_error = None
        try:
            _shutdown_supervisor(service, executor)
        except BaseException as exc:
            shutdown_error = exc
            quarantine.write_text(json.dumps({"error": type(exc).__name__,
                "detail": str(exc), "retry_safe": False}) + "\n")
            os.chmod(quarantine, 0o600)
        if service is not None and service.watchdog is not None:
            try:
                service.watchdog.close()
            except Exception:
                pass
        if executor is not None:
            try:
                executor.close()
            except Exception:
                pass
        if server is not None:
            try:
                server.server_close()
            except Exception:
                pass
        if owns_record:
            record.unlink(missing_ok=True)
        for path in owned_token_paths:
            path.unlink(missing_ok=True)
        if lock_fd is not None:
            try:
                fcntl.flock(lock_fd, fcntl.LOCK_UN)
            finally:
                os.close(lock_fd)
        if shutdown_error is not None:
            raise shutdown_error
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args(argv)
    return serve(_load_config(args.config))


if __name__ == "__main__":
    raise SystemExit(main())
