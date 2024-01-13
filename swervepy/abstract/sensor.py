from abc import abstractmethod

from wpimath.geometry import Rotation2d
from wpiutil import Sendable, SendableBuilder

from . import SendableABCMeta


class Gyro(Sendable, metaclass=SendableABCMeta):
    @abstractmethod
    def zero_heading(self):
        """Set the gyro sensor's current heading as zero"""
        raise NotImplementedError

    @property
    @abstractmethod
    def heading(self) -> Rotation2d:
        """CCW+ chassis yaw angle"""
        raise NotImplementedError

    def initSendable(self, builder: SendableBuilder):
        builder.setSmartDashboardType("Gyro")
        builder.addDoubleProperty("Value", lambda: self.heading.degrees(), lambda _: None)
        builder.addDoubleProperty("Heading (rad)", lambda: self.heading.radians(), lambda _: None)


class AbsoluteEncoder(Sendable, metaclass=SendableABCMeta):
    @property
    @abstractmethod
    def absolute_position(self) -> Rotation2d:
        """Absolute rotation"""
        raise NotImplementedError

    def initSendable(self, builder: SendableBuilder):
        builder.setSmartDashboardType("AbsoluteEncoder")
        builder.addDoubleProperty("Absolute Rotation (rad)", lambda: self.absolute_position.radians(), lambda _: None)
        builder.addDoubleProperty("Absolute Rotation (deg)", lambda: self.absolute_position.degrees(), lambda _: None)
