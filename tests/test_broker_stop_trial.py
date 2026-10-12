"""Static operator-gate checks for the physically attended STOP trial.

The actual trial is NEVER invoked with a terminal or physical port in CI.
"""
from __future__ import annotations

import importlib.util
import io
from pathlib import Path

import pytest


def _load_trial(monkeypatch):
    examples = Path(__file__).resolve().parents[1] / "examples"
    monkeypatch.syspath_prepend(str(examples))
    script = examples / "persistent_broker_stop_trial.py"
    spec = importlib.util.spec_from_file_location("test_stop_physical_trial", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_stop_trial_never_supports_unattended_yes_and_requires_tty(
    tmp_path, monkeypatch,
) -> None:
    trial = _load_trial(monkeypatch)
    args = ["--port", "/dev/ttyACM1", "--output", str(tmp_path / "trace.jsonl")]
    with pytest.raises(SystemExit):
        trial.build_parser().parse_args([*args, "--yes"])
    monkeypatch.setattr(trial.sys, "stdin", io.StringIO(""))
    with pytest.raises(PermissionError, match="interactive operator terminal"):
        trial.main(args)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("argument", [
    ("--speed-deg-s", "nan"),
    ("--acceleration-deg-s2", "-1"),
    ("--command-frequency-hz", "inf"),
])
def test_stop_trial_rejects_nonfinite_or_nonpositive_rates_without_hardware(
    tmp_path, monkeypatch, argument,
) -> None:
    trial = _load_trial(monkeypatch)
    monkeypatch.setattr(trial.sys, "stdin", type("TTY", (), {"isatty": lambda self: True})())
    args = trial.build_parser().parse_args([
        "--port", "/dev/ttyACM1",
        "--output", str(tmp_path / "trace.jsonl"), *argument,
    ])
    with pytest.raises(ValueError, match="finite and positive"):
        trial._require_operator(args)
    assert not list(tmp_path.iterdir())


def test_stop_trial_refuses_non_dev_port_before_creating_output(
    tmp_path, monkeypatch,
) -> None:
    trial = _load_trial(monkeypatch)
    monkeypatch.setattr(trial.sys, "stdin", type("TTY", (), {"isatty": lambda self: True})())
    with pytest.raises(ValueError, match="Linux /dev/ tty"):
        trial.main([
            "--port", "FAKE", "--output", str(tmp_path / "trace.jsonl"),
        ])
    assert not list(tmp_path.iterdir())


def test_stop_trial_requires_measured_sleep_without_moving(
    monkeypatch,
) -> None:
    import math

    trial = _load_trial(monkeypatch)
    target = {"shoulder_pan": math.radians(20), "elbow_flex": math.radians(-40)}
    measured = {"shoulder_pan": math.radians(21), "elbow_flex": math.radians(-39)}
    errors = trial._verify_measured_pose(measured, target, label="initial Sleep")
    assert max(errors.values()) == pytest.approx(1)
    with pytest.raises(RuntimeError, match="reposition manually"):
        trial._verify_measured_pose(
            {"shoulder_pan": math.radians(26), "elbow_flex": math.radians(-40)},
            target, label="initial Sleep",
        )
    with pytest.raises(ValueError, match="missing"):
        trial._verify_measured_pose(
            {"shoulder_pan": math.radians(20)}, target, label="initial Sleep",
        )


def test_stop_trial_refuses_identical_overhead_and_sleep_target(monkeypatch) -> None:
    import math

    trial = _load_trial(monkeypatch)
    sleep = {"shoulder_pan": 0.0, "elbow_flex": 0.0}
    with pytest.raises(RuntimeError, match="too close"):
        trial._verify_meaningful_overhead_travel(sleep, dict(sleep))
    overhead = {"shoulder_pan": 0.0, "elbow_flex": math.radians(15)}
    assert trial._verify_meaningful_overhead_travel(
        sleep, overhead,
    ) == pytest.approx(15)
    with pytest.raises(ValueError, match="different arm joints"):
        trial._verify_meaningful_overhead_travel(sleep, {"shoulder_pan": 0.1})


def test_stop_trial_baseline_completes_overhead_then_sleep_no_initial_sleep(
    monkeypatch,
) -> None:
    import math

    trial = _load_trial(monkeypatch)
    sleep = {"shoulder_pan": 0.0, "elbow_flex": 0.0}
    overhead = {"shoulder_pan": math.radians(20), "elbow_flex": math.radians(25)}
    log = []
    position = dict(sleep)

    def request(method, path, payload):
        log.append((method, path))
        if method == "POST":
            assert payload["joint_only"] is True
            if path == "/v1/go-pose":
                assert payload["name"] == "agent_start_overhead"
                position.update(overhead)
            elif path == "/v1/sleep":
                position.update(sleep)
            else:
                raise AssertionError(f"unexpected baseline motion route: {path}")
            return {
                "request_id": f"req-{len(log)}",
                "result": {
                    "accepted": True, "completed": True,
                    "broker_session": {"id": "single-sdk-session"},
                },
            }
        assert method == "GET" and path == "/v1/state"
        return {"result": {"joint_positions_rad": dict(position)}}

    legs = trial._execute_verified_roundtrip(
        request, expected_session_id="single-sdk-session",
        overhead_target=overhead, sleep_target=sleep,
    )
    assert [name for _, name in log] == [
        "/v1/go-pose", "/v1/state", "/v1/sleep", "/v1/state",
    ]
    assert [leg["label"] for leg in legs] == ["OVERHEAD", "SLEEP"]
    assert all(leg["completed"] for leg in legs)
    assert position == sleep


def test_stop_trial_never_proceeds_from_unverified_overhead(
    monkeypatch,
) -> None:
    import math

    trial = _load_trial(monkeypatch)
    sleep = {"shoulder_pan": 0.0}
    overhead = {"shoulder_pan": math.radians(20)}
    log = []

    def request(method, path, payload):
        log.append((method, path))
        if method == "POST":
            return {
                "request_id": "one", "result": {
                    "accepted": True, "completed": True,
                    "broker_session": {"id": "session"},
                },
            }
        # Deliberately report no measured Overhead displacement.
        return {"result": {"joint_positions_rad": sleep}}

    with pytest.raises(RuntimeError, match="OVERHEAD baseline"):
        trial._execute_verified_roundtrip(
            request, expected_session_id="session",
            overhead_target=overhead, sleep_target=sleep,
        )
    assert log == [("POST", "/v1/go-pose"), ("GET", "/v1/state")]


def test_manual_stop_requires_enter_while_motion_still_active(
    monkeypatch,
) -> None:
    import threading

    trial = _load_trial(monkeypatch)
    finished = threading.Event()
    class OperatorTTY:
        def isatty(self):
            return True
        def readline(self):
            return "\n"
    stream = OperatorTTY()
    selected = []
    def select_ready(inputs, *_):
        selected.append(True)
        assert inputs == [stream]
        return [stream], [], []
    monkeypatch.setattr(trial.select, "select", select_ready)
    seen = trial._await_operator_stop(
        is_moving=lambda: True,
        motion_finished=finished, stream=stream, timeout_s=1.0,
    )
    assert isinstance(seen, float)
    assert len(selected) == 1


def test_manual_stop_fails_if_motion_finished_before_enter(
    monkeypatch,
) -> None:
    import threading

    trial = _load_trial(monkeypatch)
    ended = threading.Event()
    ended.set()
    class OperatorTTY:
        def isatty(self):
            return True
        def readline(self):
            raise AssertionError("should never consume ENTER after motion completion")
    with pytest.raises(RuntimeError, match="ended before"):
        trial._await_operator_stop(
            is_moving=lambda: False,
            motion_finished=ended, stream=OperatorTTY(), timeout_s=0.2,
        )


def test_manual_stop_refuses_late_enter_even_if_stdin_is_ready(
    monkeypatch,
) -> None:
    import threading

    trial = _load_trial(monkeypatch)
    ended = threading.Event()
    class OperatorTTY:
        def isatty(self):
            return True
        def readline(self):
            ended.set()
            return "\n"
    stream = OperatorTTY()
    monkeypatch.setattr(
        trial.select, "select", lambda *args: ([stream], [], []),
    )
    with pytest.raises(RuntimeError, match="finished before"):
        trial._await_operator_stop(
            is_moving=lambda: True, motion_finished=ended,
            stream=stream, timeout_s=0.2,
        )


def test_post_stop_rearm_requires_stationary_healthy_hold(
    monkeypatch, tmp_path,
) -> None:
    from types import SimpleNamespace

    from soarm101_motion.agent_control import AgentAuthorityStore

    trial = _load_trial(monkeypatch)
    monkeypatch.setattr(
        trial.sys, "stdin",
        type("TTY", (), {"isatty": lambda self: True})(),
    )
    class FakeArm:
        config = SimpleNamespace(robot_id="so101")
        calibration_id = "cal-test"
        motion = SimpleNamespace(is_moving=False)
        hold_count = 0
        def get_state(self):
            return SimpleNamespace(
                connected=True, faulted=False, torque_enabled=True,
            )
        def hold(self):
            self.hold_count += 1
    fake = FakeArm()
    store = AgentAuthorityStore(tmp_path / "authority.json")
    assert not store.status()["armed"]
    fake.motion.is_moving = True
    with pytest.raises(RuntimeError, match="active"):
        trial._operator_rearm_for_completion(
            arm=fake, authority_store=store,
            robot_id="so101", calibration_id="cal-test",
        )
    assert not store.status()["armed"]
    fake.motion.is_moving = False
    fake.calibration_id = "wrong"
    with pytest.raises(RuntimeError, match="identity/calibration"):
        trial._operator_rearm_for_completion(
            arm=fake, authority_store=store,
            robot_id="so101", calibration_id="cal-test",
        )
    assert not store.status()["armed"]
    fake.calibration_id = "cal-test"
    result = trial._operator_rearm_for_completion(
        arm=fake, authority_store=store,
        robot_id="so101", calibration_id="cal-test",
    )
    assert result["calibration_id"] == "cal-test"
    assert fake.hold_count == 1
    lease = store.require(robot_id="so101", calibration_id="cal-test")
    assert lease.expires_at - lease.issued_at == pytest.approx(300)


def test_short_mode_is_default_and_legacy_overhead_requires_opt_in(monkeypatch) -> None:
    trial = _load_trial(monkeypatch)
    minimal = trial.build_parser().parse_args([
        "--port", "/dev/ttyACM1", "--output", "/tmp/trial.jsonl",
    ])
    assert minimal.trial == "short"
    assert minimal.speed_deg_s is None
    old = trial.build_parser().parse_args([
        "--port", "/dev/ttyACM1", "--output", "/tmp/trial.jsonl",
        "--trial", "overhead",
    ])
    assert old.trial == "overhead"


@pytest.mark.parametrize("speed,acceleration,permitted", [
    (2.0, 10.0, True),
    (3.0, 15.0, True),
    (3.01, 10.0, False),
    (2.0, 15.01, False),
])
def test_short_motion_has_immutable_speed_and_acceleration_caps(
    monkeypatch, speed, acceleration, permitted,
) -> None:
    trial = _load_trial(monkeypatch)
    monkeypatch.setattr(trial.sys, "stdin", type("TTY", (), {
        "isatty": lambda self: True,
    })())
    args = trial.build_parser().parse_args([
        "--port", "/dev/ttyACM1", "--output", "/tmp/unused.jsonl",
        "--trial", "short",
        "--speed-deg-s", str(speed),
        "--acceleration-deg-s2", str(acceleration),
    ])
    if permitted:
        trial._require_operator(args)
    else:
        with pytest.raises(ValueError, match="short STOP rate caps"):
            trial._require_operator(args)


def test_short_pan_preflight_is_fixed_and_requires_inset_limits(monkeypatch) -> None:
    import math

    trial = _load_trial(monkeypatch)
    measurement = {"shoulder_pan": math.radians(0)}
    limits = {"shoulder_pan": (math.radians(-90), math.radians(90))}
    result = trial._preflight_short_joint(measurement, limits)
    assert result["joint"] == "shoulder_pan"
    assert result["delta_deg"] == 8.0
    assert result["target_deg"] == pytest.approx(8)
    with pytest.raises(RuntimeError, match="target exceeds"):
        trial._preflight_short_joint(
            {"shoulder_pan": math.radians(78)},
            {"shoulder_pan": (math.radians(-90), math.radians(90))},
        )
    with pytest.raises(RuntimeError, match="starts too close"):
        trial._preflight_short_joint(
            {"shoulder_pan": math.radians(87)}, limits,
        )
    with pytest.raises(ValueError, match="nonfinite"):
        trial._preflight_short_joint(
            {"shoulder_pan": float("nan")}, limits,
        )


def test_short_stop_must_verify_stability_before_success(monkeypatch) -> None:
    import math

    trial = _load_trial(monkeypatch)
    def state(angle, armed=False):
        return {
            "joint_positions_rad": {"shoulder_pan": math.radians(angle)},
            "authority": {"armed": armed},
        }
    hold = {
        "post_stop_stability": {"verified": True},
        "joint_positions_rad": {"shoulder_pan": math.radians(1)},
    }
    drifts = trial._verify_post_stop_stability(
        hold, state(1.1), state(1.2),
    )
    assert drifts["shoulder_pan"] == pytest.approx(0.2)
    with pytest.raises(RuntimeError, match="drift exceeded"):
        trial._verify_post_stop_stability(
            hold, state(1.1), state(3),
        )
    with pytest.raises(RuntimeError, match="verified physical encoder"):
        trial._verify_post_stop_stability(
            {**hold, "post_stop_stability": {"verified": False}},
            state(1), state(1),
        )
    with pytest.raises(RuntimeError, match="authority"):
        trial._verify_post_stop_stability(
            hold, state(1), state(1, armed=True),
        )


def test_short_mode_caps_command_frequency(monkeypatch) -> None:
    trial = _load_trial(monkeypatch)
    monkeypatch.setattr(trial.sys, "stdin", type("TTY", (), {
        "isatty": lambda self: True,
    })())
    args = trial.build_parser().parse_args([
        "--port", "/dev/ttyACM1", "--output", "/tmp/unused.jsonl",
        "--command-frequency-hz", "30",
    ])
    with pytest.raises(ValueError, match="at most 20 Hz"):
        trial._require_operator(args)


def test_short_physical_trial_profile_can_construct_broker_without_hardware(
    tmp_path, monkeypatch,
) -> None:
    """Regression: old constructor rejected every short trial before motion."""
    from soarm101_motion import SOARM101, SOARM101Config
    from soarm101_motion.agent_control import AgentAuthorityStore
    from soarm101_motion.broker import (
        AgentMotionRates, RobotBrokerService,
        PHYSICAL_SHORT_STOP_DELTA_DEG, PHYSICAL_SHORT_STOP_JOINT,
    )
    from soarm101_motion.broker_sdk import SDKAgentExecutor

    trial = _load_trial(monkeypatch)
    config = SOARM101Config(port="/dev/fake", robot_id="so101")
    executor = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(),
        simulation=False, physical_trial=True,
        arm_factory=lambda: SOARM101.simulated(config=config),
        authority_store=AgentAuthorityStore(tmp_path / "lease.json"),
    )
    short = trial._physical_trial_profile("short")
    assert short.allowed_tools == frozenset({
        "robot_health", "robot_capabilities", "robot_state",
        "sleep", "jog_joint", "stop",
    })
    assert short.max_joint_delta_deg == PHYSICAL_SHORT_STOP_DELTA_DEG == 8.0

    service = RobotBrokerService(
        executor=executor, token="local-only-token",
        event_path=tmp_path / "events.jsonl",
        profile=short, allow_physical_sdk_trial=True,
    )
    assert executor.connection_count == 0
    assert executor._session is None

    # Reject a different joint, opposite direction, magnitude change and
    # an injected rate field BEFORE connecting or dispatching the SDK.
    wrong = [
        {"joint": "wrist_roll", "delta_deg": 8.0},
        {"joint": "shoulder_pan", "delta_deg": -8.0},
        {"joint": "shoulder_pan", "delta_deg": 7.0},
        {"joint": "shoulder_pan", "delta_deg": 8.1},
        {"joint": "shoulder_pan", "delta_deg": 8.0, "speed_deg_s": 3.0},
        {"joint": "shoulder_pan", "delta_deg": True},
    ]
    for payload in wrong:
        response = service.dispatch("POST", "/v1/joint", payload)
        assert response.status == 403, payload
        assert response.body["ok"] is False
    for path, payload in [
        ("/v1/go-pose", {"name": "agent_start_overhead"}),
        ("/v1/sleep", {}),
        ("/v1/sleep", {"joint_only": False}),
        ("/v1/sleep", {"joint_only": True, "target": "hijack"}),
        ("/v1/jog", {"x_mm": 2.0}),
        ("/v1/gripper", {"target": "open"}),
    ]:
        assert service.dispatch("POST", path, payload).status == 403
    assert executor._session is None
    assert executor.connection_count == 0

    called = []
    monkeypatch.setattr(
        executor, "execute",
        lambda action, payload: (
            called.append((action, dict(payload)))
            or {"accepted": True, "completed": True, "action": action}
        ),
    )
    exact = {
        "joint": PHYSICAL_SHORT_STOP_JOINT,
        "delta_deg": PHYSICAL_SHORT_STOP_DELTA_DEG,
    }
    # The exact pan command is never admitted BEFORE a canonical Sleep.
    assert service.dispatch("POST", "/v1/joint", exact).status == 409
    assert called == []
    prepared = service.dispatch(
        "POST", "/v1/sleep", {"joint_only": True},
    )
    assert prepared.status == 200
    assert called[0][0] == "sleep"
    assert called[0][1]["joint_only"] is True
    assert service.dispatch(
        "POST", "/v1/sleep", {"joint_only": True},
    ).status == 409  # no repeat
    accepted = service.dispatch("POST", "/v1/joint", exact)
    assert accepted.status == 200
    assert accepted.body["ok"] is True
    assert service.dispatch("POST", "/v1/joint", exact).status == 409
    assert len(called) == 2
    assert called[1][0] == "joint"
    assert all(called[1][1][key] == value for key, value in exact.items())
    assert called[1][1]["speed_deg_s"] == 8.0
    assert called[1][1]["acceleration_deg_s2"] == 25.0
    assert executor.connection_count == 0
    executor.close()


def test_physical_trial_broker_still_rejects_widened_short_profiles(
    tmp_path, monkeypatch,
) -> None:
    from soarm101_motion import SOARM101, SOARM101Config
    from soarm101_motion.agent_control import AgentAuthorityStore
    from soarm101_motion.broker import AgentMotionRates, RobotBrokerService
    from soarm101_motion.broker_sdk import SDKAgentExecutor
    from soarm101_motion.capability_profile import CapabilityProfile

    trial = _load_trial(monkeypatch)
    config = SOARM101Config(port="/dev/fake", robot_id="so101")
    executor = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(),
        simulation=False, physical_trial=True,
        arm_factory=lambda: SOARM101.simulated(config=config),
        authority_store=AgentAuthorityStore(tmp_path / "lease.json"),
    )
    short = trial._physical_trial_profile("short")
    for changes in (
        {"limits": {}},
        {"limits": {"max_joint_delta_deg": 20.0}},
        {"tools": sorted(set(short.allowed_tools) | {"go_pose"})},
        {"tools": sorted(set(short.allowed_tools) | {"move_gripper"})},
    ):
        document = short.as_dict()
        document.update(changes)
        widened = CapabilityProfile.from_document(document)
        with pytest.raises(ValueError, match="physical trial profile"):
            RobotBrokerService(
                executor=executor, token="local-only-token",
                event_path=tmp_path / "events.jsonl",
                profile=widened, allow_physical_sdk_trial=True,
            )
    # The previous Overhead route remains accepted as a separate
    # narrowly authorized trial, not an accidental capability union.
    old = trial._physical_trial_profile("overhead")
    RobotBrokerService(
        executor=executor, token="local-only-token",
        event_path=tmp_path / "events.jsonl",
        profile=old, allow_physical_sdk_trial=True,
    )
    executor.close()


def test_short_sleep_preparation_from_distant_measured_pose(monkeypatch) -> None:
    import math
    from types import SimpleNamespace
    trial = _load_trial(monkeypatch)
    measured = {"shoulder_pan": 0.0, "shoulder_lift": math.radians(10.0)}
    sleep = {"shoulder_pan": 0.0, "shoulder_lift": math.radians(-76.23)}
    limits = {name: (-math.pi / 2, math.pi / 2) for name in measured}
    assert trial._pose_joint_errors(
        measured, sleep, label="initial Sleep",
    )["shoulder_lift"] == pytest.approx(86.23)
    trial._preflight_joints_within_limits(measured, limits)
    hardware = SimpleNamespace(
        connected=True, faulted=False, torque_enabled=True, moving=False,
    )
    trial._authorize_sleep_preparation(
        state=hardware, motion_active=False, confirm=lambda _: "PREPARE_SLEEP",
    )
    assert max(trial._verify_measured_pose(
        sleep, sleep, label="prepared Sleep",
    ).values()) == 0
    assert trial._preflight_short_joint(sleep, limits)["delta_deg"] == 8.0


@pytest.mark.parametrize("condition", [
    "disconnected", "faulted", "torque_disabled", "moving", "host_moving",
])
def test_sleep_preparation_rejects_unsafe_state(monkeypatch, condition) -> None:
    from types import SimpleNamespace
    trial = _load_trial(monkeypatch)
    hardware = SimpleNamespace(
        connected=condition != "disconnected",
        faulted=condition == "faulted",
        torque_enabled=condition != "torque_disabled",
        moving=condition == "moving",
    )
    with pytest.raises(RuntimeError, match="stationary, healthy torque HOLD"):
        trial._authorize_sleep_preparation(
            state=hardware, motion_active=condition == "host_moving",
            confirm=lambda _: pytest.fail("must not request approval"),
        )


def test_sleep_preparation_decline_and_joint_limit_rejection(monkeypatch) -> None:
    from types import SimpleNamespace
    trial = _load_trial(monkeypatch)
    healthy = SimpleNamespace(
        connected=True, faulted=False, torque_enabled=True, moving=False,
    )
    with pytest.raises(PermissionError, match="declined"):
        trial._authorize_sleep_preparation(
            state=healthy, motion_active=False, confirm=lambda _: "NO",
        )
    with pytest.raises(RuntimeError, match="outside executable"):
        trial._preflight_joints_within_limits(
            {"shoulder_pan": 2.0}, {"shoulder_pan": (-1.0, 1.0)},
        )


def test_stop_evidence_requires_real_movement_with_travel_remaining(monkeypatch) -> None:
    import math
    trial = _load_trial(monkeypatch)
    def sample(t, deg):
        return {"elapsed_s": t, "raw": round(deg * 20), "rad": math.radians(deg)}
    def window(values, moving=True):
        return {
            "captured_s": 1.0,
            "samples": [
                sample(0.76, values[0]), sample(0.90, values[1]),
                sample(0.96, values[2]),
            ],
            "hardware_moving_samples": [{"elapsed_s": 0.95, "moving": moving}],
        }
    good = trial._assess_physical_inflight_feedback(
        window([1.0, 1.3, 1.5]), start_pan_rad=0.0,
    )
    assert good["verified"] is True
    assert good["remaining_deg"] == pytest.approx(6.5)
    old = trial._assess_physical_inflight_feedback(
        window([7.3, 7.3, 7.3], moving=False), start_pan_rad=0.0,
    )
    assert old["verified"] is False
    assert any("near pan target" in r for r in old["reasons"])
    assert any("did not report Moving" in r for r in old["reasons"])
    stopped = trial._assess_physical_inflight_feedback(
        window([1.5, 1.5, 1.5], moving=False), start_pan_rad=0.0,
    )
    assert stopped["verified"] is False
    stale = window([1.0, 1.3, 1.5])
    stale["samples"][-1]["elapsed_s"] = 0.8
    assert trial._assess_physical_inflight_feedback(
        stale, start_pan_rad=0.0,
    )["verified"] is False


def test_passive_stop_feedback_snapshot_does_not_read_bus(tmp_path) -> None:
    import time
    from soarm101_motion import SOARM101
    from soarm101_motion.motion.trace import PassiveBackendTrace
    tracer = PassiveBackendTrace(
        SOARM101.simulated(), tmp_path / "trace.jsonl",
    )
    tracer._started = time.perf_counter()
    tracer._record(
        "feedback", joints_raw={"shoulder_pan": 2048},
        joints_rad={"shoulder_pan": 0.0},
    )
    tracer._record("hardware_state", moving=True)
    record = tracer.feedback_window("shoulder_pan")
    assert len(record["samples"]) == 1
    assert record["samples"][0]["raw"] == 2048
    assert record["hardware_moving_samples"][-1]["moving"] is True
