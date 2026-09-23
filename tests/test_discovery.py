from __future__ import annotations

from types import SimpleNamespace

import pytest

from soarm101_motion.discovery import classify_supply_voltage, discover_so101_arms


@pytest.mark.parametrize(
    ("voltage", "role"),
    [
        (4.0, "leader"),
        (4.9, "leader"),
        (7.0, "leader"),
        (8.5, "unknown"),
        (10.0, "follower"),
        (12.0, "follower"),
        (14.0, "follower"),
        (15.0, "unknown"),
    ],
)
def test_classify_supply_voltage(voltage: float, role: str) -> None:
    assert classify_supply_voltage(voltage) == role


def test_discovery_verifies_ports_and_assigns_voltage_roles() -> None:
    configs = []
    disconnected: list[str] = []

    class FakeBackend:
        def __init__(self, config) -> None:
            self.config = config
            configs.append(config)

        def connect(self) -> None:
            if self.config.port == "BAD":
                raise RuntimeError("no expected servos")

        def diagnostics(self):
            voltage = 49 if self.config.port == "LEADER" else 120
            return [
                SimpleNamespace(error=None, voltage_v=voltage / 10.0)
                for _ in range(6)
            ]

        def disconnect(self) -> None:
            disconnected.append(str(self.config.port))

    results = discover_so101_arms(
        ["LEADER", "FOLLOWER", "BAD"],
        backend_factory=FakeBackend,
    )

    assert results[0]["role"] == "leader"
    assert results[0]["voltage_v"] == pytest.approx(4.9)
    assert results[0]["motor_count"] == 6
    assert results[1]["role"] == "follower"
    assert results[1]["voltage_v"] == pytest.approx(12.0)
    assert results[2]["status"] == "unavailable"

    assert all(config.allow_uncalibrated for config in configs)
    assert all(not config.use_stored_calibration for config in configs)
    assert all(not config.verify_calibration_on_connect for config in configs)
    assert all(not config.configure_motors_on_connect for config in configs)
    assert all(not config.auto_enable_torque for config in configs)
    assert all(not config.disable_torque_on_disconnect for config in configs)
    assert disconnected == ["LEADER", "FOLLOWER", "BAD"]
