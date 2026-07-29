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
