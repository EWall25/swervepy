from abc import abstractmethod
from typing import Protocol

from wpimath.geometry import Rotation2d


class CoaxialDriveComponent(Protocol):
    @abstractmethod
    def follow_velocity_open(self, velocity: float):
        raise NotImplementedError

    @abstractmethod
    def follow_velocity_closed(self, velocity: float):
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        raise NotImplementedError

    @property
    @abstractmethod
    def velocity(self) -> float:
        raise NotImplementedError

    @property
    @abstractmethod
    def distance(self) -> float:
        raise NotImplementedError


class CoaxialAzimuthComponent(Protocol):
    @abstractmethod
    def follow_angle(self, angle: Rotation2d):
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        raise NotImplementedError

    @property
    @abstractmethod
    def rotational_velocity(self) -> float:
        raise NotImplementedError

    @property
    @abstractmethod
    def angle(self) -> Rotation2d:
        raise NotImplementedError
