"""
Contains interfaces for components used in a swerve drive base. These are sensors, motors, and the swerve module itself.
Implementations can be found in the impl module, or the user may define their own.
"""

__all__ = [
    "SendableABCMeta",
    "CoaxialDriveComponent",
    "CoaxialAzimuthComponent",
    "Gyro",
    "AbsoluteEncoder",
    "SwerveModule",
]

from abc import ABCMeta
from wpiutil import Sendable


class SendableABCMeta(ABCMeta, type(Sendable)):
    pass


from .motor import CoaxialDriveComponent, CoaxialAzimuthComponent
from .sensor import Gyro, AbsoluteEncoder
from .system import SwerveModule
