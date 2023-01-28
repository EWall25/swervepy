import math

import ctre
import rev
import wpilib
from wpimath.geometry import Translation2d, Rotation2d

from swervelib import Swerve, SwerveParameters, SwerveModuleParameters, CANDeviceID, ModuleCorner
from swervelib.units import u


class RobotContainer:
    def __init__(self):
        field_relative = True
        open_loop = True

        track_width = (21.73 * u.inch).m_as(u.m)
        wheel_base = (21.73 * u.inch).m_as(u.m)
        # fmt: off
        swerve_params = SwerveParameters(
            wheel_circumference=4 * math.pi * u.inch,  # SDS Wheel Circumference

            drive_open_loop_ramp=0,
            drive_closed_loop_ramp=0,
            angle_ramp=0,

            # Mk4i L2 Gear Ratios
            drive_gear_ratio=6.75 / 1,
            angle_gear_ratio=(150 / 7) / 1,

            max_speed=4.5 * (u.m / u.s),
            max_angular_velocity=11.5 * (u.rad / u.s),

            angle_continuous_current_limit=40,
            angle_peak_current_limit=60,
            # angle_peak_current_duration=0.01,
            # angle_enable_current_limit=True,

            drive_continuous_current_limit=40,
            drive_peak_current_limit=60,
            # drive_peak_current_duration=0.01,
            # drive_enable_current_limit=True,

            angle_kP=1,
            angle_kI=0,
            angle_kD=0,
            angle_kF=0,

            drive_kP=1,
            drive_kI=0,
            drive_kD=0,
            drive_kF=0,

            drive_kS=1 / 12,
            drive_kV=0 / 12,
            drive_kA=0 / 12,

            angle_neutral_mode=rev.CANSparkMax.IdleMode.kBrake,
            drive_neutral_mode=rev.CANSparkMax.IdleMode.kCoast,

            invert_angle_motor=False,
            invert_drive_motor=False,
            invert_angle_encoder=False,

            invert_gyro=True,
            gyro_id=0,
        )
        # When defining module positions for kinematics, +x values represent moving toward the front of the robot, and
        # +y values represent moving toward the left of the robot
        module_params = (
            SwerveModuleParameters(
                corner=ModuleCorner.FRONT_LEFT,
                relative_position=Translation2d(wheel_base / 2, track_width / 2),
                angle_offset=Rotation2d.fromDegrees(0),
                drive_motor_id=0,
                angle_motor_id=4,
                angle_encoder_id=CANDeviceID(0),
            ),
            SwerveModuleParameters(
                corner=ModuleCorner.FRONT_RIGHT,
                relative_position=Translation2d(wheel_base / 2, -track_width / 2),
                angle_offset=Rotation2d.fromDegrees(0),
                drive_motor_id=1,
                angle_motor_id=5,
                angle_encoder_id=CANDeviceID(1),
            ),
            SwerveModuleParameters(
                corner=ModuleCorner.BACK_LEFT,
                relative_position=Translation2d(-wheel_base / 2, track_width / 2),
                angle_offset=Rotation2d.fromDegrees(0),
                drive_motor_id=2,
                angle_motor_id=6,
                angle_encoder_id=CANDeviceID(2),
            ),
            SwerveModuleParameters(
                corner=ModuleCorner.BACK_RIGHT,
                relative_position=Translation2d(-wheel_base / 2, -track_width / 2),
                angle_offset=Rotation2d.fromDegrees(0),
                drive_motor_id=3,
                angle_motor_id=7,
                angle_encoder_id=CANDeviceID(3),
            ),
        )
        # fmt: on

        self.stick = wpilib.Joystick(0)

        self.swerve = Swerve(module_params, swerve_params)
        self.swerve.setDefaultCommand(
            self.swerve.teleop_command(
                lambda: deadband(-self.stick.getRawAxis(1), 0.01),
                lambda: deadband(self.stick.getRawAxis(0), 0.01),
                lambda: deadband(-self.stick.getRawAxis(2), 0.01),  # Invert for CCW+
                field_relative,
                open_loop,
            )
        )


def deadband(value, band):
    return value if abs(value) > band else 0
