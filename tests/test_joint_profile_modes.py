"""Software-only tests for selectable joint trajectory profile modes and speed quantization fixes."""

from __future__ import annotations

import json
import math
import os
import runpy
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from soarm101_motion import (
    JOINT_PROFILE_MODES,
    Pose,
    SOARM101,
    SOARM101Config,
)
from soarm101_motion.cli.main import main as cli_main
from soarm101_motion.constants import (
    ARM_JOINTS,
    TELEOP_SERVO_ACCELERATION_RAW,
    TELEOP_SERVO_SPEED_RAW,
)
from soarm101_motion.exceptions import ConfigurationError, InvalidCommandError
from soarm101_motion.hardware import SimulationBackend
from soarm101_motion.kinematics import SO101KinematicModel
from soarm101_motion.motion import PassiveBackendTrace
from soarm101_motion.motion.controller import MotionController as BaseMotionController
from soarm101_motion.poses import sleep_joint_positions
from soarm101_motion.sdk_capabilities import SDK_CAPABILITIES

REPO_ROOT = Path(__file__).resolve().parents[1]
ANALYZE_SCRIPT = REPO_ROOT / "scripts" / "analyze_wobble_traces.py"
TRACE_EXAMPLE = REPO_ROOT / "examples" / "motion_quality_trace.py"
COMPARISON_SCRIPT = REPO_ROOT / "scripts" / "run_joint_profile_comparison.sh"

_ANALYZER = runpy.run_path(str(ANALYZE_SCRIPT))
analyze_trace = _ANALYZER["analyze"]


class _CalibratedMotor:
    """Synthetic STS3215 calibration matching 4095 ticks / 2*pi rad."""

    TICKS_PER_RAD = 4095.0 / (2.0 * math.pi)

    def __init__(self, lower_deg: float, upper_deg: float) -> None:
        self.radians_limits = (math.radians(lower_deg), math.radians(upper_deg))

    def radians_to_raw(self, value: float) -> int:
        raw = int(round(2048.0 + float(value) * self.TICKS_PER_RAD))
        return max(0, min(4095, raw))


def _attach_synthetic_calibration(arm: SOARM101) -> None:
    bounds_deg = {
        "shoulder_pan": (-120.0, 120.0),
        "shoulder_lift": (-105.0, 105.0),
        "elbow_flex": (-97.0, 97.0),
        "wrist_flex": (-104.0, 104.0),
        "wrist_roll": (-168.0, 168.0),
    }
    arm.backend.calibration = SimpleNamespace(
        calibration_id="cal-synth-001",
        source="synthetic-test",
        motors={
            name: _CalibratedMotor(*bounds_deg[name])
            for name in ARM_JOINTS
        },
    )


def test_config_validates_joint_profile_mode() -> None:
    assert JOINT_PROFILE_MODES == ("minimum_jerk", "cosine_cruise")
    default_cfg = SOARM101Config()
    assert default_cfg.joint_profile_mode == "minimum_jerk"

    cosine_cfg = SOARM101Config(joint_profile_mode="cosine_cruise")
    assert cosine_cfg.joint_profile_mode == "cosine_cruise"

    with pytest.raises(ConfigurationError, match="joint_profile_mode"):
        SOARM101Config(joint_profile_mode="trapezoid")  # type: ignore[arg-type]


def test_move_primitives_validate_profile_mode_eagerly() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = dict(arm.get_joint_positions().positions)
        target["shoulder_pan"] += 0.1

        with pytest.raises(InvalidCommandError, match="profile_mode"):
            arm.move_joints(target, profile_mode="invalid")  # type: ignore[arg-type]

        with pytest.raises(InvalidCommandError, match="profile_mode"):
            arm.move_sleep(wait=False, profile_mode="invalid")  # type: ignore[arg-type]

        with pytest.raises(InvalidCommandError, match="profile_mode"):
            arm.move_sleep_up(wait=False, profile_mode="invalid")  # type: ignore[arg-type]


def test_cosine_cruise_vs_minimum_jerk_side_by_side_on_calibrated_sleep_fold(
    tmp_path: Path,
) -> None:
    speed = math.radians(8.0)
    acceleration = math.radians(25.0)
    overhead = {
        "shoulder_pan": 0.0,
        "shoulder_lift": -0.35,
        "elbow_flex": 0.35,
        "wrist_flex": 0.65,
        "wrist_roll": 0.0,
    }

    plans: dict[str, tuple[list[dict[str, float]], dict]] = {}
    for mode in JOINT_PROFILE_MODES:
        cfg = SOARM101Config(
            auto_enable_torque=False,
            command_frequency_hz=50.0,
            joint_profile_mode=mode,
        )
        with SOARM101.simulated(
            realtime=False,
            initial_positions=overhead,
            config=cfg,
        ) as arm:
            _attach_synthetic_calibration(arm)
            arm.enable()
            sleep_target = sleep_joint_positions(arm.get_joint_limits())
            plan = arm.motion._plan_joint_motion(
                overhead,
                sleep_target,
                speed=speed,
                acceleration=acceleration,
                profile_mode=mode,
            )
            samples = list(plan.command_samples)
            max_speed, max_accel, max_step = arm.motion._trajectory_metrics(samples)
            assert samples[-1] == pytest.approx(sleep_target)
            assert max_step <= cfg.max_command_step_radians + 1e-6
            assert max_speed <= speed + 1e-6
            assert max_accel <= acceleration + 1e-6

            trace_path = tmp_path / f"{mode}.jsonl"
            with PassiveBackendTrace(
                arm,
                trace_path,
                metadata={
                    "kind": "program_motion_quality",
                    "command_frequency_hz": 50.0,
                    "execution_mode": "streamed",
                    "profile_mode": mode,
                },
            ) as trace:
                trace.mark("leg_start", destination="SLEEP")
                result = arm.move_sleep(
                    speed=speed,
                    acceleration=acceleration,
                    profile_mode=mode,
                    close_gripper=False,
                )
                trace.mark("leg_end", destination="SLEEP", completed=result.completed)
            assert result.completed is True

            # Rewrite synthetic monotonic_s onto an exact 50 Hz nominal clock so
            # fast non-realtime simulation yields deterministic nominal-grid metrics.
            rows = [
                json.loads(line)
                for line in trace_path.read_text(encoding="utf-8").splitlines()
            ]
            # Map each recorded backend write back to its exact planned sample index.
            cmd_idx = 0
            raw_samples = [
                {
                    name: arm.backend.calibration.motors[name].radians_to_raw(s[name])
                    for name in ARM_JOINTS
                }
                for s in samples
            ]
            sample_index_by_seq: dict[int, int] = {}
            search_from = 0
            for row in rows:
                if row.get("event") == "command":
                    cmd_idx += 1
                    row_raw = row["joints_raw"]
                    for k in range(search_from, len(raw_samples)):
                        if raw_samples[k] == row_raw:
                            sample_index_by_seq[cmd_idx] = k
                            search_from = k + 1
                            break
                    row["monotonic_s"] = sample_index_by_seq[cmd_idx] / 50.0
            rows[0]["monotonic_s"] = 0.0
            rows[-1]["monotonic_s"] = len(samples) / 50.0
            for row in rows:
                if row.get("event") == "marker":
                    row["monotonic_s"] = (
                        0.0
                        if row.get("marker") == "leg_start"
                        else (len(samples) - 1) / 50.0
                    )
                elif row.get("event") == "feedback":
                    row["monotonic_s"] = 0.0
            trace_path.write_text(
                "".join(json.dumps(r) + "\n" for r in rows),
                encoding="utf-8",
            )
            analyzed = analyze_trace(trace_path, mode, 50.0, None, False, "SLEEP", 0.5)
            assert len(analyzed) == 1
            assert analyzed[0]["profile_mode"] == mode
            plans[mode] = (samples, analyzed[0])

    mj_samples, mj_report = plans["minimum_jerk"]
    cc_samples, cc_report = plans["cosine_cruise"]

    mj_duration_s = (len(mj_samples) - 1) / 50.0
    cc_duration_s = (len(cc_samples) - 1) / 50.0
    # Cosine cruise reaches its requested 8 deg/s ceiling rather than averaging ~4.3 deg/s.
    assert cc_duration_s < mj_duration_s * 0.65

    mj_joints = {j["joint"]: j for j in mj_report["joints"]}
    cc_joints = {j["joint"]: j for j in cc_report["joints"]}
    for dominant in ("shoulder_lift", "elbow_flex"):
        assert (
            cc_joints[dominant]["subtick_tail_ms_estimate"]
            < mj_joints[dominant]["subtick_tail_ms_estimate"] * 0.15
        )
        assert (
            cc_joints[dominant]["tick_impulses_final25"]
            < mj_joints[dominant]["tick_impulses_final25"]
        )


def test_base_motion_controller_move_linear_uses_teleop_0_254_contract() -> None:
    cfg = SOARM101Config(auto_enable_torque=False, command_frequency_hz=50.0)
    backend = SimulationBackend(
        {
            "shoulder_pan": 0.0,
            "shoulder_lift": -0.35,
            "elbow_flex": 0.35,
            "wrist_flex": 0.65,
            "wrist_roll": 0.0,
        },
        realtime=False,
    )
    backend.connect()
    backend.enable_torque()
    backend.calibration = SimpleNamespace(
        motors={
            name: _CalibratedMotor(-150.0, 150.0)
            for name in ARM_JOINTS
        }
    )
    model = SO101KinematicModel()
    controller = BaseMotionController(backend, cfg, model)

    recorded_speeds: list[object] = []
    recorded_accels: list[object] = []
    original_write = backend.write_joint_positions

    def capture_write(positions, *, speed_raw=None, acceleration_raw=None):
        recorded_speeds.append(speed_raw)
        recorded_accels.append(acceleration_raw)
        return original_write(
            positions,
            speed_raw=speed_raw,
            acceleration_raw=acceleration_raw,
        )

    backend.write_joint_positions = capture_write  # type: ignore[method-assign]
    current_pose = model.forward(backend.read_joint_positions())
    target_pose = Pose(
        current_pose.position + [0.015, 0.0, 0.0],
        current_pose.rotation,
    )
    result = controller.move_linear(
        target_pose,
        orientation_mode="position_only",
        speed=0.02,
        acceleration=0.08,
    )
    assert result.completed is True
    assert recorded_speeds
    assert all(s == TELEOP_SERVO_SPEED_RAW for s in recorded_speeds)
    assert all(a == TELEOP_SERVO_ACCELERATION_RAW for a in recorded_accels)


def test_synchronized_streamed_arrival_never_drops_speed_raw_to_one_on_zero_tick_cycle() -> None:
    with SOARM101.simulated(realtime=False) as arm:
        _attach_synthetic_calibration(arm)
        arm.enable()
        start = dict(arm.get_joint_positions().positions)
        target = dict(start)
        # Pan moves 12 deg while wrist_roll moves only 1.5 deg (~17 ticks total),
        # so wrist_roll has many 0-tick rounding cycles while pan writes on the bus.
        target["shoulder_pan"] += math.radians(12.0)
        target["wrist_roll"] += math.radians(1.5)

        seen_speeds: list[dict[str, int]] = []
        original_write = arm.backend.write_joint_positions

        def capture_write(positions, *, speed_raw=None, acceleration_raw=None):
            if isinstance(speed_raw, dict):
                seen_speeds.append(dict(speed_raw))
            return original_write(
                positions,
                speed_raw=speed_raw,
                acceleration_raw=acceleration_raw,
            )

        arm.backend.write_joint_positions = capture_write  # type: ignore[method-assign]
        result = arm.motion.move_joints(
            target,
            speed=math.radians(8.0),
            acceleration=math.radians(25.0),
            profile_mode="cosine_cruise",
            synchronize_servo_arrival=True,
        )

    assert result.completed is True
    assert seen_speeds
    wrist_speeds = [entry["wrist_roll"] for entry in seen_speeds]
    assert min(wrist_speeds) >= 60


def test_sdk_capabilities_and_cli_accept_profile_mode(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr("builtins.input", lambda *_: "")
    for cap_name in ("move_joints", "jog_joint", "replay_saved_pose", "sleep", "sleep_up"):
        spec = SDK_CAPABILITIES.get(cap_name)
        arg_names = {arg.name: arg for arg in spec.arguments}
        assert "profile_mode" in arg_names
        assert arg_names["profile_mode"].choices == JOINT_PROFILE_MODES

    rc = cli_main(
        [
            "move-joints",
            "--simulation",
            "--yes",
            "--degrees",
            "--speed-deg-s",
            "20",
            "--acceleration-deg-s2",
            "80",
            "--profile-mode",
            "cosine_cruise",
            "5",
            "-20",
            "20",
            "35",
            "0",
        ]
    )
    assert rc == 0

    rc_sleep = cli_main(
        [
            "sleep",
            "--simulation",
            "--yes",
            "--speed-deg-s",
            "20",
            "--acceleration-deg-s2",
            "80",
            "--profile-mode",
            "cosine_cruise",
        ]
    )
    assert rc_sleep == 0


def test_motion_quality_trace_and_comparison_script_in_simulation(
    tmp_path: Path,
) -> None:
    out_trace = tmp_path / "sim-cosine.jsonl"
    completed = subprocess.run(
        [
            sys.executable,
            str(TRACE_EXAMPLE),
            "--simulation",
            "--profile-mode",
            "cosine_cruise",
            "--speed-deg-s",
            "40",
            "--acceleration-deg-s2",
            "200",
            "--pause-s",
            "0",
            "--joint-only",
            "--output",
            str(out_trace),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "profile=cosine_cruise" in completed.stdout
    assert out_trace.is_file()
    summary = json.loads(out_trace.with_suffix(".summary.json").read_text(encoding="utf-8"))
    assert summary["profile_mode"] == "cosine_cruise"
    assert summary["simulation"] is True

    comp_dir = tmp_path / "comparison-out"
    env = dict(os.environ)
    env["SOARM_SIMULATION"] = "1"
    env["SOARM_COMPARISON_OUT_DIR"] = str(comp_dir)
    script_run = subprocess.run(
        ["bash", str(COMPARISON_SCRIPT), "both", "40", "200"],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    assert "COMPARISON COMPLETE" in script_run.stdout
    assert (comp_dir / "minimum-jerk.jsonl").is_file()
    assert (comp_dir / "cosine-cruise.jsonl").is_file()
    assert (comp_dir / "wobble-comparison.json").is_file()
    assert (comp_dir / "wobble-comparison.csv").is_file()
    comp_json = json.loads((comp_dir / "wobble-comparison.json").read_text(encoding="utf-8"))
    seen_profiles = {entry["profile_mode"] for entry in comp_json}
    assert seen_profiles == {"minimum_jerk", "cosine_cruise"}
