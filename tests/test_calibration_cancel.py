"""Cancellation restores motor settings in both hardware backend variants."""

from __future__ import annotations

import threading
import time
from types import MethodType, SimpleNamespace

import pytest

from soarm101_motion.exceptions import CalibrationCancelledError, CalibrationError
from soarm101_motion.constants import ALL_MOTORS
from soarm101_motion.hardware.feetech import FeetechBackend as ProtocolBackend
from soarm101_motion.hardware.managed_feetech import FeetechBackend as ManagedBackend


@pytest.mark.parametrize("backend_type", [ProtocolBackend, ManagedBackend])
def test_cancelled_sweep_rolls_back_and_does_not_apply_new_calibration(backend_type):
    backend = object.__new__(backend_type)
    backend._io_lock = threading.RLock()
    backend.config = SimpleNamespace(robot_id="test-cancel")
    prior = object()
    backend.calibration = prior
    applied = []
    cancel = threading.Event()
    backend._require_connected = MethodType(lambda self: None, backend)
    backend.read_calibration_from_motors = MethodType(lambda self: prior, backend)
    backend.disable_torque = MethodType(lambda self: None, backend)
    backend.reset_calibration = MethodType(lambda self: None, backend)
    backend.read_all_raw_positions = MethodType(
        lambda self: {"shoulder_pan": 2000}, backend
    )
    backend.apply_calibration = MethodType(
        lambda self, calibration: applied.append(calibration), backend
    )
    backend._verify_calibration_matches_motors = MethodType(
        lambda self, expected, actual: None, backend
    )

    def request_cancel(_progress):
        cancel.set()

    with pytest.raises(CalibrationCancelledError):
        backend.interactive_calibration(
            record_seconds=5.0,
            cancel_event=cancel,
            progress_callback=request_cancel,
        )

    assert applied == [prior]
    assert backend.calibration is prior


@pytest.mark.parametrize("backend_type", [ProtocolBackend, ManagedBackend])
def test_incomplete_sweep_restores_previous_motor_settings(backend_type):
    backend = object.__new__(backend_type)
    backend._io_lock = threading.RLock()
    backend.config = SimpleNamespace(robot_id="test-incomplete")
    prior = object()
    backend.calibration = prior
    applied = []
    backend._require_connected = MethodType(lambda self: None, backend)
    backend.read_calibration_from_motors = MethodType(lambda self: prior, backend)
    backend.disable_torque = MethodType(lambda self: None, backend)
    backend.reset_calibration = MethodType(lambda self: None, backend)
    backend.read_all_raw_positions = MethodType(
        lambda self: {name: 2000 for name in ALL_MOTORS}, backend
    )
    backend.apply_calibration = MethodType(
        lambda self, calibration: applied.append(calibration), backend
    )
    backend._verify_calibration_matches_motors = MethodType(
        lambda self, expected, actual: None, backend
    )

    with pytest.raises(CalibrationError, match="moved only 0 encoder ticks"):
        backend.interactive_calibration(record_seconds=0.01, poll_interval=0.001)

    assert applied == [prior]
    assert backend.calibration is prior


@pytest.mark.parametrize("backend_type", [ProtocolBackend, ManagedBackend])
def test_sweep_finishes_as_soon_as_all_motors_complete_twice(backend_type):
    backend = object.__new__(backend_type)
    backend._io_lock = threading.RLock()
    backend.config = SimpleNamespace(robot_id="unit-test-early-finish")
    prior = object()
    backend.calibration = prior
    applied = []
    samples = iter([1000, 2000, 3200, 2000, 1000])
    progress = []
    backend._require_connected = MethodType(lambda self: None, backend)
    backend.read_calibration_from_motors = MethodType(
        lambda self: applied[-1] if applied else prior, backend
    )
    backend.disable_torque = MethodType(lambda self: None, backend)
    backend.reset_calibration = MethodType(lambda self: None, backend)

    def read_positions(_self):
        raw = next(samples, None)
        if raw is None:
            raise AssertionError("read past completed sweep")
        return {name: raw for name in ALL_MOTORS}

    backend.read_all_raw_positions = MethodType(read_positions, backend)
    backend.apply_calibration = MethodType(
        lambda self, calibration: applied.append(calibration), backend
    )
    backend._verify_calibration_matches_motors = MethodType(
        lambda self, expected, actual: None, backend
    )

    started = time.monotonic()
    calibration = backend.interactive_calibration(
        record_seconds=3.0,
        poll_interval=0.001,
        progress_callback=progress.append,
    )

    assert time.monotonic() - started < 1.0
    assert applied == [calibration]
    assert all(item["passed"] for item in progress[-1].values())
