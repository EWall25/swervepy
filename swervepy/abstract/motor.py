from abc import abstractmethod
from typing import Protocol

from wpimath.geometry import Rotation2d


class CoaxialDriveComponent(Protocol):
    """The component of a swerve module that drives the wheel (forward and backward)"""

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
    def set_voltage(self, volts: float):
        """
        Power the underlying motor with the specified voltage

        :param volts: Voltage in volts (between -12 and +12 for most FRC motors)
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

    @property
    @abstractmethod
    def voltage(self) -> float:
        """Applied motor voltage in volts, between -12 and +12 for most FRC motors"""
        raise NotImplementedError


class CoaxialAzimuthComponent(Protocol):
    """
    The component of a swerve module that turns the wheel. This component also initializes the wheel to its absolute
    position (usually via an absolute encoder).
    """

    @abstractmethod
    def follow_angle(self, angle: Rotation2d):
        """
        Move the wheel to an angle

        :param angle: The desired wheel angle
        """
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        """Reset the wheel's angle to its absolute position"""
        raise NotImplementedError

    @property
    @abstractmethod
    def rotational_velocity(self) -> float:
        """Rotational velocity in rad/s"""
        raise NotImplementedError

    @property
    @abstractmethod
    def angle(self) -> Rotation2d:
        """The wheel angle"""
        raise NotImplementedError
