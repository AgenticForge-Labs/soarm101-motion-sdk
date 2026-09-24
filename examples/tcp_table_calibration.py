"""Measure the TCP/table-frame error without commanding the arm.

Use with the follower torque disabled. At each prompt, place the same physical
tool point at the same table point, then press Enter. The report separates a
constant base/table offset from pose-dependent TCP or kinematic error.
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from soarm101_motion.calibration import SO101Calibration, default_calibration_path
from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import ALL_MOTORS, ARM_JOINTS
from soarm101_motion.hardware import FeetechBackend
from soarm101_motion.kinematics.model import SO101KinematicModel


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--robot-id", default="so101")
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument(
        "--table-z-mm",
        type=float,
        default=0.0,
        help="known table height in the SDK base frame, usually 0 for this test",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("tcp_table_calibration.json"),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.samples < 2:
        raise SystemExit("--samples must be at least 2")

    calibration = SO101Calibration.load(default_calibration_path(args.robot_id))
    model = SO101KinematicModel()
    config = SOARM101Config(
        port=args.port,
        robot_id=args.robot_id,
        configure_motors_on_connect=False,
        auto_enable_torque=False,
        disable_torque_on_disconnect=True,
    )
    backend = FeetechBackend(config)
    records: list[dict[str, object]] = []

    print("Read-only TCP/table test: no torque or goal writes will be sent.")
    print("Keep the GUI disconnected. Confirm the follower is relaxed before continuing.")
    input("Press Enter when ready… ")
    try:
        backend.connect()
        for index in range(args.samples):
            print(
                f"\nSample {index + 1}/{args.samples}: place the same TCP/tool point "
                "at the same table point."
            )
            input("Press Enter to capture… ")
            raw = {name: backend.read_raw_position(name) for name in ALL_MOTORS}
            joints = {
                name: calibration.motors[name].raw_to_radians(raw[name])
                for name in ARM_JOINTS
            }
            tcp = model.forward(joints).position.tolist()
            record = {
                "index": index + 1,
                "raw": raw,
                "joints_rad": joints,
                "tcp_xyz_m": tcp,
                "model_z_error_m": float(tcp[2] - args.table_z_mm / 1000.0),
            }
            records.append(record)
            print(
                "  TCP model: "
                f"x={tcp[0] * 1000:.1f} mm, y={tcp[1] * 1000:.1f} mm, "
                f"z={tcp[2] * 1000:.1f} mm"
            )
    finally:
        if backend.is_connected:
            backend.disconnect()

    errors = [float(record["model_z_error_m"]) for record in records]
    mean_error = sum(errors) / len(errors)
    residuals = [error - mean_error for error in errors]
    rms_residual = (sum(value * value for value in residuals) / len(residuals)) ** 0.5
    print("\nResult")
    print(f"  Mean model/table z offset: {mean_error * 1000:.1f} mm")
    print(f"  Pose-to-pose residual RMS: {rms_residual * 1000:.1f} mm")
    if rms_residual <= 0.005:
        print("  Interpretation: mostly a constant table/base-frame offset.")
    else:
        print("  Interpretation: pose-dependent TCP or kinematic geometry error.")

    report = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "port": args.port,
        "robot_id": args.robot_id,
        "table_z_mm": args.table_z_mm,
        "mean_model_table_z_offset_m": mean_error,
        "residual_rms_m": rms_residual,
        "samples": records,
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"  Report written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
