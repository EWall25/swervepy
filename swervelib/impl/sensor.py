import enum

import ctre.sensors
import rev
import wpilib
from wpimath.geometry import Rotation2d

from ..abstract.sensor import AbsoluteEncoder, Gyro


class AbsoluteCANCoder(AbsoluteEncoder):
    def __init__(self, id_: int | tuple[int, str]):
        super().__init__()

        # Construct the CANCoder from either a tuple of motor ID and CAN bus ID or just a motor ID
        try:
            self._encoder = ctre.sensors.CANCoder(*id_)
        except TypeError:
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


class DummyGyro(Gyro):
    """Gyro that does nothing"""

    def __init__(self, *args):
        super().__init__()

    def zero_heading(self):
        pass

    @property
    def heading(self) -> Rotation2d:
        return Rotation2d.fromDegrees(0)


class SparkMaxEncoderType(enum.Enum):
    ANALOG = enum.auto()
    PWM = enum.auto()


class SparkMaxAbsoluteEncoder(AbsoluteEncoder):
    def __init__(self, controller: rev.CANSparkMax, encoder_type: SparkMaxEncoderType):
        """
        Absolute encoder plugged into the SPARK MAX's data port

        :param controller: SPARK MAX instance
        :param encoder_type: Type of encoder plugged in. Based on how the encoder transmits data
        """

        super().__init__()

        # Two types of absolute encoders can be plugged into the SPARK MAX data port: analog and duty cycle/PWM
        if encoder_type is SparkMaxEncoderType.ANALOG:
            self._encoder = controller.getAnalog(rev.SparkMaxAnalogSensor.Mode.kAbsolute)

            # Analog encoders output from 0V - 3.3V
            # Change from voltage to degrees
            self._encoder.setPositionConversionFactor(360 / 3.3)
        elif encoder_type is SparkMaxEncoderType.PWM:
            self._encoder = controller.getAbsoluteEncoder(rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)

            # Duty cycle encoders output from 0 to 1 by default
            # Change into degrees
            self._encoder.setPositionConversionFactor(360)

        wpilib.SmartDashboard.putData(f"Absolute Encoder {controller.getDeviceId()}", self)

    @property
    def absolute_position(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self._encoder.getPosition())
