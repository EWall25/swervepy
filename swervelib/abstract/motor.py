from abc import abstractmethod
from typing import Protocol

from wpimath.geometry import Rotation2d


class CoaxialDriveComponent(Protocol):
    """Drive motor and encoder"""

    @abstractmethod
    def follow_velocity_open(self, velocity: float):
        """
        Follow a velocity using open loop control

        :param velocity: Desired velocity in m/s
        """
        raise NotImplementedError

    @abstractmethod
    def follow_velocity_closed(self, velocity: float):
        """
        Follow a velocity using closed loop control (i.e., PID)

        :param velocity: Desired velocity in m/s
        """
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        """Reset driven distance to zero or otherwise reinitialize the motor"""
        raise NotImplementedError

    @property
    @abstractmethod
    def velocity(self) -> float:
        """Drive velocity in m/s"""
        raise NotImplementedError

    @property
    @abstractmethod
    def distance(self) -> float:
        """Driven distance in metres"""
        raise NotImplementedError


class CoaxialAzimuthComponent(Protocol):
    """
    Azimuth (turning) motor and encoder.
    Also includes a method for resetting to an absolute position (often provided by an absolute encoder)
    """

    @abstractmethod
    def follow_angle(self, angle: Rotation2d):
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        """Reset module angle to absolute position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def rotational_velocity(self) -> float:
        """Rotational velocity in rad/s"""
        raise NotImplementedError

    @property
    @abstractmethod
    def angle(self) -> Rotation2d:
        raise NotImplementedError
