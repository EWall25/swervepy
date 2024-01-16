from wpimath.geometry import Rotation2d, Translation2d
from wpiutil import SendableBuilder

from ..abstract.motor import CoaxialDriveComponent, CoaxialAzimuthComponent
from ..abstract.system import SwerveModule


class CoaxialSwerveModule(SwerveModule):
    last_commanded_drive_velocity: float = 0
    last_commanded_azimuth_angle = Rotation2d.fromDegrees(0)

    def __init__(self, drive: CoaxialDriveComponent, azimuth: CoaxialAzimuthComponent, placement: Translation2d):
        super().__init__()

        self._drive = drive
        self._azimuth = azimuth
        self.placement = placement

    def desire_drive_velocity(self, velocity: float, open_loop: bool):
        self.last_commanded_drive_velocity = velocity
        if open_loop:
            self._drive.follow_velocity_open(velocity)
        else:
            self._drive.follow_velocity_closed(velocity)

    def set_drive_voltage(self, volts: float):
        self._drive.set_voltage(volts)

    def desire_azimuth_angle(self, angle: Rotation2d):
        self.last_commanded_azimuth_angle = angle
        self._azimuth.follow_angle(angle)

    def reset(self):
        self._drive.reset()
        self._azimuth.reset()

    @property
    def drive_velocity(self) -> float:
        return self._drive.velocity

    @property
    def drive_distance(self) -> float:
        return self._drive.distance

    @property
    def drive_voltage(self) -> float:
        return self._drive.voltage

    @property
    def azimuth_angle(self) -> Rotation2d:
        return self._azimuth.angle

    @property
    def azimuth_velocity(self) -> float:
        return self._azimuth.rotational_velocity

    def initSendable(self, builder: SendableBuilder):
        # fmt: off
        builder.setSmartDashboardType("CoaxialSwerveModule")
        builder.addDoubleProperty("Drive Velocity (mps)", lambda: self._drive.velocity, lambda _: None)
        builder.addDoubleProperty("Drive Distance (m)", lambda: self._drive.distance, lambda _: None)
        builder.addDoubleProperty("Drive Voltage", lambda: self._drive.voltage, lambda _: None)
        builder.addDoubleProperty("Azimuth Velocity (radps)", lambda: self._azimuth.rotational_velocity, lambda _: None)
        builder.addDoubleProperty("Azimuth Position (rad)", lambda: self._azimuth.angle.radians(), lambda _: None)
        builder.addDoubleProperty("Azimuth Position (deg)", lambda: self._azimuth.angle.degrees(), lambda _: None)
        builder.addDoubleProperty("Desired Drive Velocity (mps)", lambda: self.last_commanded_drive_velocity, lambda _: None)
        builder.addDoubleProperty("Desired Azimuth Position (rad)", lambda: self.last_commanded_azimuth_angle.radians(), lambda _: None)
        builder.addDoubleProperty("Desired Azimuth Position (deg)", lambda: self.last_commanded_azimuth_angle.degrees(), lambda _: None)
        # fmt: on
