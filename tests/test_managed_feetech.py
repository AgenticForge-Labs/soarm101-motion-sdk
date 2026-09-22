from __future__ import annotations

import threading

import pytest

from soarm101_motion.hardware import FeetechBackend


def _backend() -> FeetechBackend:
    backend = object.__new__(FeetechBackend)
    backend._io_lock = threading.RLock()
    backend._connected = True
    backend._torque_enabled = False
    return backend


def _install_enable_limits(
    backend: FeetechBackend,
    limits: dict[str, tuple[int, int]],
) -> None:
    def read_register(motor: str, register: str) -> int:
        minimum, maximum = limits[motor]
        if register == "Min_Position_Limit":
            return minimum
        if register == "Max_Position_Limit":
            return maximum
        raise AssertionError(f"unexpected register read: {register}")

    backend.read_register = read_register


def test_disable_torque_does_not_unlock_eeprom() -> None:
    backend = _backend()
    backend._torque_enabled = True
    writes: list[tuple[str, str, int]] = []
    backend.write_register = lambda motor, register, value: writes.append(
        (motor, register, value)
    )

    backend.disable_torque(["shoulder_pan"])

    assert writes == [("shoulder_pan", "Torque_Enable", 0)]
    assert all(register != "Lock" for _, register, _ in writes)


def test_enable_locks_eeprom_before_enabling_torque() -> None:
    backend = _backend()
    writes: list[tuple[str, str, int]] = []
    latched: list[dict[str, int]] = []
    backend.read_raw_position = lambda _: 2048
    _install_enable_limits(backend, {"shoulder_pan": (100, 3995)})
    backend._write_raw_positions = lambda positions, **_: latched.append(dict(positions))
    backend.write_register = lambda motor, register, value: writes.append(
        (motor, register, value)
    )

    backend.enable_torque(["shoulder_pan"])

    assert latched == [{"shoulder_pan": 2048}]
    assert writes == [
        ("shoulder_pan", "Lock", 1),
        ("shoulder_pan", "Torque_Enable", 1),
    ]


def test_failed_enable_rolls_back_torque_without_unlocking_eeprom() -> None:
    backend = _backend()
    writes: list[tuple[str, str, int]] = []
    backend.read_raw_position = lambda _: 2048
    _install_enable_limits(
        backend,
        {
            "shoulder_pan": (100, 3995),
            "shoulder_lift": (100, 3995),
        },
    )
    backend._write_raw_positions = lambda positions, **_: None

    def write(motor: str, register: str, value: int) -> None:
        writes.append((motor, register, value))
        if motor == "shoulder_lift" and register == "Torque_Enable" and value == 1:
            raise RuntimeError("injected failure")

    backend.write_register = write

    with pytest.raises(RuntimeError, match="injected failure"):
        backend.enable_torque(["shoulder_pan", "shoulder_lift"])

    assert ("shoulder_pan", "Torque_Enable", 0) in writes
    assert all(not (register == "Lock" and value == 0) for _, register, value in writes)


def test_enable_refuses_out_of_range_position_before_any_goal_or_torque_write() -> None:
    backend = _backend()
    raw_positions = {
        "shoulder_pan": 2048,
        "shoulder_lift": 90,
    }
    backend.read_raw_position = lambda motor: raw_positions[motor]
    _install_enable_limits(
        backend,
        {
            "shoulder_pan": (100, 3995),
            "shoulder_lift": (100, 3995),
        },
    )

    latched: list[dict[str, int]] = []
    writes: list[tuple[str, str, int]] = []
    backend._write_raw_positions = lambda positions, **_: latched.append(dict(positions))
    backend.write_register = lambda motor, register, value: writes.append(
        (motor, register, value)
    )

    with pytest.raises(
        Exception,
        match=r"shoulder_lift: present 90 outside EEPROM limits 100\.\.3995",
    ):
        backend.enable_torque(["shoulder_pan", "shoulder_lift"])

    assert latched == []
    assert writes == []
    assert not backend._torque_enabled


def test_enable_refuses_invalid_eeprom_range_before_any_write() -> None:
    backend = _backend()
    backend.read_raw_position = lambda _: 2048
    _install_enable_limits(backend, {"shoulder_pan": (3000, 1000)})

    latched: list[dict[str, int]] = []
    writes: list[tuple[str, str, int]] = []
    backend._write_raw_positions = lambda positions, **_: latched.append(dict(positions))
    backend.write_register = lambda motor, register, value: writes.append(
        (motor, register, value)
    )

    with pytest.raises(Exception, match=r"invalid EEPROM limits 3000\.\.1000"):
        backend.enable_torque(["shoulder_pan"])

    assert latched == []
    assert writes == []
    assert not backend._torque_enabled


def test_enable_accepts_positions_exactly_on_eeprom_boundaries() -> None:
    backend = _backend()
    positions = {"shoulder_pan": 100, "shoulder_lift": 3995}
    backend.read_raw_position = lambda motor: positions[motor]
    _install_enable_limits(
        backend,
        {
            "shoulder_pan": (100, 3995),
            "shoulder_lift": (100, 3995),
        },
    )
    latched: list[dict[str, int]] = []
    writes: list[tuple[str, str, int]] = []
    backend._write_raw_positions = lambda values, **_: latched.append(dict(values))
    backend.write_register = lambda motor, register, value: writes.append(
        (motor, register, value)
    )

    backend.enable_torque(["shoulder_pan", "shoulder_lift"])

    assert latched == [positions]
    assert writes == [
        ("shoulder_pan", "Lock", 1),
        ("shoulder_pan", "Torque_Enable", 1),
        ("shoulder_lift", "Lock", 1),
        ("shoulder_lift", "Torque_Enable", 1),
    ]
