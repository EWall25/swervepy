import ctre.sensors
import wpilib
from wpimath.geometry import Rotation2d

from ..abstract.sensor import AbsoluteEncoder, Gyro


class AbsoluteCANCoder(AbsoluteEncoder):
    def __init__(self, id_: int):
        super().__init__()

        self._encoder = ctre.sensors.CANCoder(id_)
        self._encoder.configAbsoluteSensorRange(ctre.sensors.AbsoluteSensorRange.Unsigned_0_to_360)

        wpilib.SmartDashboard.putData(f"Absolute CANCoder {id_}", self)

    @property
    def absolute_position(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self._encoder.getAbsolutePosition())


class PigeonGyro(Gyro):
    def __init__(self, id_: int, invert: bool = False):
        super().__init__()

        self._gyro = ctre.sensors.PigeonIMU(id_)
        self.invert = invert

        wpilib.SmartDashboard.putData("Pigeon IMU", self)

    def zero_heading(self):
        self._gyro.setYaw(0)

    @property
    def heading(self) -> Rotation2d:
        yaw = self._gyro.getYaw()
        if self.invert:
            yaw = 360 - yaw
        return Rotation2d.fromDegrees(yaw)
