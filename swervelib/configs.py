import enum
from dataclasses import dataclass
from typing import NamedTuple

import ctre
import wpimath.trajectory
import wpimath.geometry
from astropy.units import Quantity


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
    wheel_circumference: Quantity["length"]

    open_loop_ramp: float
    closed_loop_ramp: float

    drive_gear_ratio: float
    angle_gear_ratio: float

    # Swerve Profiling
    max_speed: Quantity["velocity"]
    max_angular_velocity: Quantity["angular velocity"]

    # Swerve Current Limiting
    angle_continuous_current_limit: int
    angle_peak_current_limit: int
    angle_peak_current_duration: float
    angle_enable_current_limit: bool

    drive_continuous_current_limit: int
    drive_peak_current_limit: int
    drive_peak_current_duration: float
    drive_enable_current_limit: bool

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
    angle_neutral_mode: ctre.NeutralMode
    drive_neutral_mode: ctre.NeutralMode

    # Motor Inverts
    invert_angle_motor: bool
    invert_drive_motor: bool

    # Angle Encoder Invert
    invert_angle_encoder: bool

    # Gyro
    invert_gyro: bool  # Gyro should be CCW+
    gyro_id: CANDeviceID


@dataclass
class SwerveModuleParameters:
    corner: ModuleCorner
    relative_position: wpimath.geometry.Translation2d

    angle_offset: float

    drive_motor_id: CANDeviceID
    angle_motor_id: CANDeviceID
    angle_encoder_id: CANDeviceID


@dataclass
class AutoParameters:
    max_speed: Quantity["velocity"]
    max_acceleration: Quantity["acceleration"]
    # max_angular_speed: Quantity["angular velocity"]
    # max_angular_acceleration: Quantity["angular acceleration"]

    theta_controller_constraints: wpimath.trajectory.TrapezoidProfile.Constraints


class CTREConfigs:
    __slots__ = "swerve_angle_config", "swerve_drive_config", "swerve_cancoder_config"

    def __init__(self, params: SwerveParameters):
        swerve_angle_config = ctre.TalonFXConfiguration()
        swerve_drive_config = ctre.TalonFXConfiguration()
        swerve_cancoder_config = ctre.CANCoderConfiguration()

        # Swerve Angle Motor Configuration
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

        # Swerve Drive Motor Configuration
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
        swerve_drive_config.openloopRamp = params.open_loop_ramp
        swerve_drive_config.closedloopRamp = params.closed_loop_ramp

        # Swerve CANCoder Configuration
        swerve_cancoder_config.absoluteSensorRange = ctre.AbsoluteSensorRange.Unsigned_0_to_360
        swerve_cancoder_config.sensorDirection = params.invert_angle_encoder
        swerve_cancoder_config.initializationStrategy = ctre.SensorInitializationStrategy.BootToAbsolutePosition
        swerve_cancoder_config.sensorTimeBase = ctre.SensorTimeBase.PerSecond

        self.swerve_angle_config = swerve_angle_config
        self.swerve_drive_config = swerve_drive_config
        self.swerve_cancoder_config = swerve_cancoder_config
