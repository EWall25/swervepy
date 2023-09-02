import math

import ctre
import wpilib
from wpimath.geometry import Translation2d, Rotation2d

from swervelib import u
from swervelib.systems import (
    SwerveDrive,
    PigeonGyro,
    CoaxialSwerveModule,
    Falcon500CoaxialDriveMotor,
    Falcon500CoaxialAzimuthMotor,
    AbsoluteCANCoder,
)


class RobotContainer:
    def __init__(self):
        field_relative = True
        open_loop = True

        track_width = (21.73 * u.inch).m_as(u.m)
        wheel_base = (21.73 * u.inch).m_as(u.m)
        max_speed = 4.5 * (u.m / u.s)
        max_angular_velocity = 11.5 * (u.rad / u.s)

        drive_params = Falcon500CoaxialDriveMotor.Parameters(
            wheel_circumference=4 * math.pi * u.inch,
            gear_ratio=6.75 / 1,
            max_speed=max_speed,
            open_loop_ramp_rate=0,
            closed_loop_ramp_rate=0,
            continuous_current_limit=40,
            peak_current_limit=60,
            peak_current_duration=0.01,
            neutral_mode=ctre.NeutralMode.Coast,
            kP=0,
            kI=0,
            kD=0,
            kS=0,
            kV=0,
            kA=0,
            invert_motor=False,
        )
        azimuth_params = Falcon500CoaxialAzimuthMotor.Parameters(
            gear_ratio=150 / 7,
            max_angular_velocity=max_angular_velocity,
            ramp_rate=0,
            continuous_current_limit=40,
            peak_current_limit=60,
            peak_current_duration=0.01,
            neutral_mode=ctre.NeutralMode.Brake,
            kP=0,
            kI=0,
            kD=0,
            invert_motor=False,
        )

        gyro = PigeonGyro(0, True)

        # When defining module positions for kinematics, +x values represent moving toward the front of the robot, and
        # +y values represent moving toward the left of the robot
        modules = (
            CoaxialSwerveModule(
                Falcon500CoaxialDriveMotor(0, drive_params),
                Falcon500CoaxialAzimuthMotor(4, Rotation2d.fromDegrees(0), azimuth_params, AbsoluteCANCoder(0)),
                Translation2d(wheel_base / 2, track_width / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveMotor(1, drive_params),
                Falcon500CoaxialAzimuthMotor(5, Rotation2d.fromDegrees(0), azimuth_params, AbsoluteCANCoder(1)),
                Translation2d(wheel_base / 2, track_width / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveMotor(2, drive_params),
                Falcon500CoaxialAzimuthMotor(6, Rotation2d.fromDegrees(0), azimuth_params, AbsoluteCANCoder(2)),
                Translation2d(wheel_base / 2, track_width / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveMotor(3, drive_params),
                Falcon500CoaxialAzimuthMotor(7, Rotation2d.fromDegrees(0), azimuth_params, AbsoluteCANCoder(3)),
                Translation2d(wheel_base / 2, track_width / 2),
            ),
        )

        self.stick = wpilib.Joystick(0)

        self.swerve = SwerveDrive(modules, gyro, max_speed, max_angular_velocity)

        self.swerve.setDefaultCommand(
            self.swerve.teleop_command(
                lambda: deadband(-self.stick.getRawAxis(1), 0.03),
                lambda: deadband(self.stick.getRawAxis(0), 0.03),
                lambda: deadband(-self.stick.getRawAxis(2), 0.03),  # Invert for CCW+
                field_relative,
                open_loop,
            )
        )


def deadband(value, band):
    return value if abs(value) > band else 0
