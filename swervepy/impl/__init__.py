"""
Contains default implementations of components (motors, sensors, modules). The user should instantiate these when
creating their drive base.
"""

__all__ = [
    "Falcon500CoaxialAzimuthComponent",
    "Falcon500CoaxialDriveComponent",
    "NEOCoaxialAzimuthComponent",
    "NEOCoaxialDriveComponent",
    "NEOCoaxialAzimuthComponent",
    "AbsoluteCANCoder",
    "AbsoluteDutyCycleEncoder",
    "PigeonGyro",
    "Pigeon2Gyro",
    "CoaxialSwerveModule",
    "SparkMaxEncoderType",
    "SparkMaxAbsoluteEncoder",
    "DummyCoaxialComponent",
    "DummyGyro",
]

from .motor import (
    Falcon500CoaxialAzimuthComponent,
    Falcon500CoaxialDriveComponent,
    NEOCoaxialDriveComponent,
    NEOCoaxialAzimuthComponent,
    NEOCoaxialAzimuthComponent,
    DummyCoaxialComponent,
)
from .sensor import (
    AbsoluteCANCoder,
    AbsoluteDutyCycleEncoder,
    PigeonGyro,
    Pigeon2Gyro,
    SparkMaxEncoderType,
    SparkMaxAbsoluteEncoder,
    DummyGyro,
)
from .system import CoaxialSwerveModule
