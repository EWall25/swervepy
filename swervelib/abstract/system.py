from abc import abstractmethod

from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpiutil import Sendable

from . import SendableABCMeta


class SwerveModule(Sendable, metaclass=SendableABCMeta):
    placement: Translation2d

    def desire_state(self, state: SwerveModuleState, drive_open_loop):
        # TODO: Optimize state
        self.desire_drive_velocity(state.speed, drive_open_loop)
        self.desire_azimuth_angle(state.angle)

    @property
    def module_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drive_distance, self.azimuth_angle)

    @abstractmethod
    def desire_drive_velocity(self, velocity: float, open_loop):
        raise NotImplementedError

    @abstractmethod
    def desire_azimuth_angle(self, angle: Rotation2d):
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        raise NotImplementedError

    @property
    @abstractmethod
    def drive_velocity(self) -> float:
        raise NotImplementedError

    @property
    @abstractmethod
    def drive_distance(self) -> float:
        raise NotImplementedError

    @property
    @abstractmethod
    def azimuth_angle(self) -> Rotation2d:
        raise NotImplementedError

    @property
    @abstractmethod
    def azimuth_velocity(self) -> float:
        raise NotImplementedError
