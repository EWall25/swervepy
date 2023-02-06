"""
A collection of configuration classes.

Some classes use 'Quantity' objects from the Pint unit system.
These quantities allow you to input parameters (e.g. wheel circumference) in any valid unit, such as inches or metres.
"""

import copy
import enum
from abc import ABC
from dataclasses import dataclass
from typing import NamedTuple

import ctre
import rev
import wpimath.trajectory
import wpimath.geometry
import robotpy_apriltag as apriltag
from pint import Quantity

from . import u
from .vision import CameraDefinition


class CanFDDeviceID(NamedTuple):
    number: int
    canbus: str = ""


class ModuleCorner(enum.IntEnum):
    FRONT_LEFT = 0
    FRONT_RIGHT = 1
    BACK_LEFT = 2
    BACK_RIGHT = 3


@dataclass(kw_only=True)
class SwerveParameters(ABC):
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

    drive_continuous_current_limit: int
    drive_peak_current_limit: int

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


@dataclass(kw_only=True)
class CTRESwerveParameters(SwerveParameters):
    # Swerve Current Limiting
    angle_peak_current_duration: float
    angle_enable_current_limit: bool

    drive_peak_current_duration: float
    drive_enable_current_limit: bool

    # Neutral Modes
    angle_neutral_mode: ctre.NeutralMode
    drive_neutral_mode: ctre.NeutralMode


@dataclass(kw_only=True)
class REVSwerveParameters(SwerveParameters):
    # Neutral Modes
    angle_neutral_mode: rev.CANSparkMax.IdleMode
    drive_neutral_mode: rev.CANSparkMax.IdleMode


@dataclass(kw_only=True)
class SwerveModuleParameters(ABC):
    corner: ModuleCorner
    relative_position: wpimath.geometry.Translation2d

    angle_offset: wpimath.geometry.Rotation2d

    angle_encoder_id: CanFDDeviceID

    fake: bool = False


@dataclass(kw_only=True)
class CTRESwerveModuleParameters(SwerveModuleParameters):
    drive_motor_id: CanFDDeviceID
    angle_motor_id: CanFDDeviceID


@dataclass(kw_only=True)
class REVSwerveModuleParameters(SwerveModuleParameters):
    drive_motor_id: int
    angle_motor_id: int


@dataclass
class VisionParameters:
    camera_definitions: list[CameraDefinition]
    field_layout: apriltag.AprilTagFieldLayout


@dataclass
class AutoParameters:
    max_speed: Quantity
    max_acceleration: Quantity

    theta_controller_constraints: wpimath.trajectory.TrapezoidProfileRadians.Constraints


def create_TalonFX_angle_config(params: CTRESwerveParameters):
    swerve_angle_config = ctre.TalonFXConfiguration()

    angle_supply_limit = ctre.SupplyCurrentLimitConfiguration(
        params.angle_enable_current_limit,
        params.angle_continuous_current_limit,
        params.angle_peak_current_limit,
        params.angle_peak_current_duration,
    )

    swerve_angle_config.slot0.kP = params.angle_kP
    swerve_angle_config.slot0.kI = params.angle_kI
    swerve_angle_config.slot0.kD = params.angle_kD
    swerve_angle_config.slot0.kF = params.angle_kF
    swerve_angle_config.supplyCurrLimit = angle_supply_limit
    swerve_angle_config.initializationStrategy = ctre.SensorInitializationStrategy.BootToZero
    swerve_angle_config.closedloopRamp = params.angle_ramp

    return swerve_angle_config


def create_TalonFX_drive_config(params: CTRESwerveParameters):
    swerve_drive_config = ctre.TalonFXConfiguration()

    drive_supply_limit = ctre.SupplyCurrentLimitConfiguration(
        params.drive_enable_current_limit,
        params.drive_continuous_current_limit,
        params.drive_peak_current_limit,
        params.drive_peak_current_duration,
    )

    swerve_drive_config.slot0.kP = params.drive_kP
    swerve_drive_config.slot0.kI = params.drive_kI
    swerve_drive_config.slot0.kD = params.drive_kD
    swerve_drive_config.slot0.kF = params.drive_kF
    swerve_drive_config.supplyCurrLimit = drive_supply_limit
    swerve_drive_config.initializationStrategy = ctre.SensorInitializationStrategy.BootToZero
    swerve_drive_config.openloopRamp = params.drive_open_loop_ramp
    swerve_drive_config.closedloopRamp = params.drive_closed_loop_ramp

    return swerve_drive_config


def create_CANCoder_config(params: SwerveParameters):
    swerve_cancoder_config = ctre.CANCoderConfiguration()

    swerve_cancoder_config.absoluteSensorRange = ctre.AbsoluteSensorRange.Unsigned_0_to_360
    swerve_cancoder_config.sensorDirection = params.invert_angle_encoder
    swerve_cancoder_config.initializationStrategy = ctre.SensorInitializationStrategy.BootToAbsolutePosition
    swerve_cancoder_config.sensorTimeBase = ctre.SensorTimeBase.PerSecond

    return swerve_cancoder_config
