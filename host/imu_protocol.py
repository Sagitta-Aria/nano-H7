"""Shared IMU1 telemetry parser for the local H723 console (Python 3.8+).

Use parse_frame() on complete lines. USB timestamps are MCU uptime, not wall
clock. This module does not command motors or flash/reconfigure the MCU.
"""
import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class ImuSample:
    """One body-frame measurement; use valid before feeding a controller.

    Angles are degrees, angular velocities degrees/second, accelerations m/s².
    Quaternion order is w,x,y,z. Yaw has no magnetic reference and will drift.
    """
    timestamp_ms: int
    sample_count: int
    ready: bool
    calibrated: bool
    accel_trusted: bool
    error: int
    calibration_count: int
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    gyro_dps: Tuple[float, float, float]
    accel_ms2: Tuple[float, float, float]
    linear_accel_ms2: Tuple[float, float, float]
    temperature_c: float
    quaternion_wxyz: Tuple[float, float, float, float]

    @property
    def valid(self) -> bool:
        """Require a fresh, calibrated, fault-free sample for normal use."""
        return self.ready and self.calibrated and self.error == 0


def parse_frame(line: bytes) -> ImuSample:
    """Decode one complete IMU1 line; reject damaged/version-mismatched frames.

    This pure parser also works on recorded data, without importing pyserial.
    It raises ValueError for incomplete lines, nonfinite values or invalid flags.
    """
    if not line.endswith(b"\n") or len(line) > 768:
        raise ValueError("incomplete or oversized IMU frame")
    try:
        fields = line.decode("ascii").strip().split(",")
        if len(fields) != 25 or fields[0] != "IMU1":
            raise ValueError("expected 25-field IMU1 frame")
        integers = [int(v) for v in fields[1:8]]
        floats = [float(v) for v in fields[8:]]
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError("invalid IMU frame: " + str(exc)) from exc
    if any(v < 0 for v in integers) or any(v not in (0, 1) for v in integers[2:5]):
        raise ValueError("invalid status fields")
    if not all(math.isfinite(v) for v in floats):
        raise ValueError("nonfinite measurement")
    return ImuSample(
        integers[0], integers[1], bool(integers[2]), bool(integers[3]),
        bool(integers[4]), integers[5], integers[6],
        floats[0], floats[1], floats[2], tuple(floats[3:6]),
        tuple(floats[6:9]), tuple(floats[9:12]), floats[12], tuple(floats[13:17]))
