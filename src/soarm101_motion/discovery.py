"""Read-only discovery of connected SO-101 arms.

Discovery probes candidate serial adapters without enabling torque or changing motor
configuration.  A successful probe must communicate with the expected six STS3215
servos.  The average reported supply voltage is then used as a convenient role hint
for the common SO-101 leader/follower setup.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from typing import Any

from soarm101_motion.config import SOARM101Config
from soarm101_motion.hardware import FeetechBackend

LEADER_VOLTAGE_RANGE_V = (4.0, 7.0)
FOLLOWER_VOLTAGE_RANGE_V = (10.0, 14.0)


def classify_supply_voltage(voltage_v: float) -> str:
    """Return a role hint from measured servo-bus voltage."""

    voltage = float(voltage_v)
    if LEADER_VOLTAGE_RANGE_V[0] <= voltage <= LEADER_VOLTAGE_RANGE_V[1]:
        return "leader"
    if FOLLOWER_VOLTAGE_RANGE_V[0] <= voltage <= FOLLOWER_VOLTAGE_RANGE_V[1]:
        return "follower"
    return "unknown"


def discover_so101_arms(
    ports: Iterable[str] | None = None,
    *,
    backend_factory: Callable[[SOARM101Config], Any] = FeetechBackend,
) -> list[dict[str, object]]:
    """Probe candidate serial ports and return read-only SO-101 discovery results.

    A normal backend connection verifies the six expected motor IDs/model numbers.
    Discovery explicitly disables stored-calibration matching, motor configuration,
    torque-on-disconnect writes, and torque enable.  Calibration EEPROM is read only
    because the backend uses it to establish whether the arm can be safely identified.
    """

    candidates = list(ports) if ports is not None else FeetechBackend.candidate_ports()
    results: list[dict[str, object]] = []

    for port in candidates:
        backend = backend_factory(
            SOARM101Config(
                port=port,
                robot_id="discovery",
                allow_uncalibrated=True,
                use_stored_calibration=False,
                verify_calibration_on_connect=False,
                configure_motors_on_connect=False,
                auto_enable_torque=False,
                disable_torque_on_disconnect=False,
                verify_model_numbers=True,
            )
        )
        try:
            backend.connect()
            diagnostics = list(backend.diagnostics())
            readable = [
                item
                for item in diagnostics
                if getattr(item, "error", None) is None
                and getattr(item, "voltage_v", None) is not None
            ]
            if not readable:
                raise RuntimeError("SO-101 responded but no servo voltage readings were available")
            voltages = [float(item.voltage_v) for item in readable]
            average_voltage = sum(voltages) / len(voltages)
            results.append(
                {
                    "port": port,
                    "status": "ok",
                    "role": classify_supply_voltage(average_voltage),
                    "voltage_v": average_voltage,
                    "motor_count": len(readable),
                    "motor_total": len(diagnostics),
                    "motor_voltages_v": voltages,
                    "error": None,
                }
            )
        except Exception as exc:
            results.append(
                {
                    "port": port,
                    "status": "unavailable",
                    "role": "unknown",
                    "voltage_v": None,
                    "motor_count": 0,
                    "motor_total": 6,
                    "motor_voltages_v": [],
                    "error": str(exc),
                }
            )
        finally:
            try:
                backend.disconnect()
            except Exception:
                pass

    return results
