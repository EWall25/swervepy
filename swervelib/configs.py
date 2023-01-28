import copy
import enum
from dataclasses import dataclass
from typing import NamedTuple

import ctre
import rev
import wpimath.trajectory
import wpimath.geometry
from pint import Quantity

from .units import u


class CANDeviceID(NamedTuple):
    number: int
    canbus: str = ""


class ModuleCorner(enum.IntEnum):
    FRONT_LEFT = 0
    FRONT_RIGHT = 1
    BACK_LEFT = 2
    BACK_RIGHT = 3


@dataclass
class SwerveParameters:
    # Drivetrain Constants
    wheel_circumference: Quantity

    drive_open_loop_ramp: float
    drive_closed_loop_ramp: float
    angle_ramp: float

    drive_gear_ratio: float
    angle_gear_ratio: float

    # Swerve Profiling
    max_speed: Quantity
    max_angular_velocity: Quantity

    # Swerve Current Limiting
    angle_continuous_current_limit: int
    angle_peak_current_limit: int
    # angle_peak_current_duration: float
    # angle_enable_current_limit: bool

    drive_continuous_current_limit: int
    drive_peak_current_limit: int
    # drive_peak_current_duration: float
    # drive_enable_current_limit: bool

    # Angle Motor PID
    angle_kP: float
    angle_kI: float
    angle_kD: float
    angle_kF: float

    # Drive Motor PID
    drive_kP: float
    drive_kI: float
    drive_kD: float
    drive_kF: float

    # Drive Motor Characterization
    drive_kS: float
    drive_kV: float
    drive_kA: float

    # Neutral Modes
    angle_neutral_mode: rev.CANSparkMax.IdleMode
    drive_neutral_mode: rev.CANSparkMax.IdleMode

    # Motor Inverts
    invert_angle_motor: bool
    invert_drive_motor: bool

    # Angle Encoder Invert
    invert_angle_encoder: bool

    # Gyro
    invert_gyro: bool  # Gyro should be CCW+
    gyro_id: int
    fake_gyro: bool = False

    def in_standard_units(self):
        data = copy.deepcopy(self)
        data.wheel_circumference = data.wheel_circumference.m_as(u.m)
        data.max_speed = data.max_speed.m_as(u.m / u.s)
        data.max_angular_velocity = data.max_angular_velocity.m_as(u.rad / u.s)
        return data


@dataclass
class SwerveModuleParameters:
    corner: ModuleCorner
    relative_position: wpimath.geometry.Translation2d

    angle_offset: wpimath.geometry.Rotation2d

    drive_motor_id: int
    angle_motor_id: int
    angle_encoder_id: CANDeviceID

    fake: bool = False


@dataclass
class AutoParameters:
    max_speed: Quantity
    max_acceleration: Quantity
    # max_angular_speed: Quantity["angular velocity"]
    # max_angular_acceleration: Quantity["angular acceleration"]

    theta_controller_constraints: wpimath.trajectory.TrapezoidProfile.Constraints


class CTREConfigs:
    __slots__ = "swerve_cancoder_config"

    def __init__(self, params: SwerveParameters):
        swerve_cancoder_config = ctre.CANCoderConfiguration()

        # Swerve CANCoder Configuration
        swerve_cancoder_config.absoluteSensorRange = ctre.AbsoluteSensorRange.Unsigned_0_to_360
        swerve_cancoder_config.sensorDirection = params.invert_angle_encoder
        swerve_cancoder_config.initializationStrategy = ctre.SensorInitializationStrategy.BootToAbsolutePosition
        swerve_cancoder_config.sensorTimeBase = ctre.SensorTimeBase.PerSecond

        self.swerve_cancoder_config = swerve_cancoder_config
