from abc import abstractmethod, ABCMeta
from typing import Protocol

from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpiutil import Sendable, SendableBuilder


class SendableABCMeta(ABCMeta, type(Sendable)):
    pass


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


class CoaxialDriveMotor(Protocol):
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


class CoaxialAzimuthMotor(Protocol):
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


class Gyro(Sendable, metaclass=SendableABCMeta):
    @abstractmethod
    def zero_heading(self):
        raise NotImplementedError

    @property
    @abstractmethod
    def heading(self) -> Rotation2d:
        raise NotImplementedError

    def initSendable(self, builder: SendableBuilder):
        builder.setSmartDashboardType("Gyro")
        builder.addDoubleProperty("Value", lambda: self.heading.degrees(), lambda: None)
        builder.addDoubleProperty("Heading (rad)", lambda: self.heading.radians(), lambda: None)


class AbsoluteEncoder(Sendable, metaclass=SendableABCMeta):
    @property
    @abstractmethod
    def absolute_position(self) -> Rotation2d:
        raise NotImplementedError

    def initSendable(self, builder: SendableBuilder):
        builder.setSmartDashboardType("Encoder")
        builder.addDoubleProperty("Absolute Rotation (rad)", lambda: self.absolute_position.radians(), lambda: None)
        builder.addDoubleProperty("Absolute Rotation (deg)", lambda: self.absolute_position.degrees(), lambda: None)
