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
