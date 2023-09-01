from dataclasses import dataclass
from typing import TYPE_CHECKING

import commands2
import ctre
import wpimath.kinematics
import wpimath.estimator
from pint import Quantity
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.kinematics import SwerveModulePosition, ChassisSpeeds
from wpimath.controller import SimpleMotorFeedforwardMeters

from . import u, conversions
from .abstracts import SwerveModule, CoaxialDriveMotor, CoaxialAzimuthMotor, Gyro, AbsoluteEncoder

if TYPE_CHECKING:
    from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry


class CoaxialSwerveModule(SwerveModule):
    def __init__(self, drive_motor: CoaxialDriveMotor, azimuth_motor: CoaxialAzimuthMotor):
        self._drive_motor = drive_motor
        self._azimuth_motor = azimuth_motor

    def desire_drive_velocity(self, velocity: float, open_loop: bool):
        if open_loop:
            self._drive_motor.follow_velocity_open(velocity)
        else:
            self._drive_motor.follow_velocity_closed(velocity)

    def desire_azimuth_angle(self, angle: Rotation2d):
        self._azimuth_motor.follow_angle(angle)

    def reset(self):
        self._drive_motor.reset()
        self._azimuth_motor.reset()

    @property
    def drive_velocity(self) -> float:
        return self._drive_motor.velocity

    @property
    def drive_distance(self) -> float:
        return self._drive_motor.distance

    @property
    def azimuth_angle(self) -> Rotation2d:
        return self._azimuth_motor.angle

    @property
    def azimuth_velocity(self) -> Rotation2d:
        return self._azimuth_motor.rotational_velocity


class SwerveDrive(commands2.SubsystemBase):
    def __init__(
        self, modules: tuple[SwerveModule, ...], gyro: Gyro, max_velocity: Quantity, max_angular_velocity: Rotation2d
    ):
        super().__init__()

        self._modules = modules
        self._gyro = gyro
        self.max_velocity: float = max_velocity.m_as(u.m / u.s)
        self.max_angular_velocity = max_angular_velocity

        self.reset_modules()

        # There are different classes for each number of swerve modules in a drive base,
        # so construct the class name from number of modules.
        self._kinematics: "SwerveDrive4Kinematics" = getattr(
            wpimath.kinematics, f"SwerveDrive{len(modules)}Kinematics"
        )(*[module.placement for module in self._modules])
        self._odometry: "SwerveDrive4Odometry" = getattr(wpimath.estimator, f"SwerveDrive{len(modules)}PoseEstimator")(
            self.kinematics, self._gyro.heading, self.module_positions, Pose2d()
        )

    def drive(self, translation: Translation2d, rotation: Rotation2d, field_relative: bool, open_loop: bool):
        speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, self.heading)
            if field_relative
            else ChassisSpeeds(translation.x, translation.y, rotation)
        )
        swerve_module_states = self._kinematics.toSwerveModuleStates(speeds)
        swerve_module_states = self._kinematics.desaturateWheelSpeeds(swerve_module_states, self.max_velocity)

        for i in range(4):
            module: SwerveModule = self._modules[i]
            module.desire_state(swerve_module_states[i], open_loop)

    @property
    def module_positions(self) -> tuple[SwerveModulePosition, ...]:
        return tuple(module.module_position for module in self._modules)

    # TODO: Add utility methods
    # TODO: Impl dashboard
    # TODO: Add commands

    def reset_modules(self):
        for module in self._modules:
            module.reset()


class Falcon500CoaxialDriveMotor(CoaxialDriveMotor):
    @dataclass
    class Parameters:
        wheel_circumference: float  # m
        gear_ratio: float

        max_speed: float  # m/s

        open_loop_ramp_rate: float
        closed_loop_ramp_rate: float

        continuous_current_limit: int
        peak_current_limit: int
        peak_current_duration: float

        neutral_mode: ctre.NeutralMode

        kP: float
        kI: float
        kD: float

        kS: float
        kV: float
        kA: float

        invert_motor: bool

        def create_TalonFX_config(self) -> ctre.TalonFXConfiguration:
            motor_config = ctre.TalonFXConfiguration()

            supply_limit = ctre.SupplyCurrentLimitConfiguration(
                True,
                self.continuous_current_limit,
                self.peak_current_limit,
                self.peak_current_duration,
            )

            motor_config.slot0.kP = self.kP
            motor_config.slot0.kI = self.kI
            motor_config.slot0.kD = self.kD
            motor_config.supplyCurrLimit = supply_limit
            motor_config.initializationStrategy = ctre.SensorInitializationStrategy.BootToZero
            motor_config.openloopRamp = self.open_loop_ramp_rate
            motor_config.closedloopRamp = self.closed_loop_ramp_rate

            return motor_config

    def __init__(self, id_: int, parameters: Parameters):
        self._params = parameters

        # TODO: Accept CAN IDs on other busses
        self._motor = ctre.TalonFX(id_)
        self._config()
        self.reset()

        self._feedforward = SimpleMotorFeedforwardMeters(parameters.kS, parameters.kV, parameters.kA)

    def _config(self):
        settings = self._params.create_TalonFX_config()
        self._motor.configFactoryDefault()
        self._motor.configAllSettings(settings)
        self._motor.setInverted(self._params.invert_motor)
        self._motor.setNeutralMode(self._params.neutral_mode)

    def follow_velocity_open(self, velocity: float):
        percent_out = velocity / self._params.max_speed
        self._motor.set(ctre.ControlMode.PercentOutput, percent_out)

    def follow_velocity_closed(self, velocity: float):
        converted_velocity = conversions.mps_to_falcon(
            velocity, self._params.wheel_circumference, self._params.gear_ratio
        )
        self._motor.set(
            ctre.ControlMode.Velocity,
            converted_velocity,
            ctre.DemandType.ArbitraryFeedForward,
            self._feedforward.calculate(velocity),
        )

    def reset(self):
        self._motor.setSelectedSensorPosition(0)

    @property
    def velocity(self) -> float:
        return conversions.falcon_to_mps(
            self._motor.getSelectedSensorVelocity(),
            self._params.wheel_circumference,
            self._params.gear_ratio,
        )

    @property
    def distance(self) -> float:
        return conversions.falcon_to_metres(
            self._motor.getSelectedSensorPosition(),
            self._params.wheel_circumference,
            self._params.gear_ratio,
        )


class Falcon500CoaxialAzimuthMotor(CoaxialAzimuthMotor):
    @dataclass
    class Parameters:
        gear_ratio: float

        max_angular_velocity: Rotation2d

        ramp_rate: float

        continuous_current_limit: int
        peak_current_limit: int
        peak_current_duration: float

        neutral_mode: ctre.NeutralMode

        kP: float
        kI: float
        kD: float

        invert_motor: bool

        def create_TalonFX_config(self) -> ctre.TalonFXConfiguration:
            motor_config = ctre.TalonFXConfiguration()

            supply_limit = ctre.SupplyCurrentLimitConfiguration(
                True,
                self.continuous_current_limit,
                self.peak_current_limit,
                self.peak_current_duration,
            )

            motor_config.slot0.kP = self.kP
            motor_config.slot0.kI = self.kI
            motor_config.slot0.kD = self.kD
            motor_config.supplyCurrLimit = supply_limit
            motor_config.initializationStrategy = ctre.SensorInitializationStrategy.BootToZero
            motor_config.closedloopRamp = self.ramp_rate

            return motor_config

    def __init__(self, id_: int, azimuth_offset: Rotation2d, parameters: Parameters, absolute_encoder: AbsoluteEncoder):
        self._params = parameters

        # TODO: Accept CAN IDs on other busses
        self._motor = ctre.TalonFX(id_)
        self._absolute_encoder = absolute_encoder
        self._offset = azimuth_offset

        self._config()
        self.reset()

    def _config(self):
        settings = self._params.create_TalonFX_config()
        self._motor.configFactoryDefault()
        self._motor.configAllSettings(settings)
        self._motor.setInverted(self._params.invert_motor)
        self._motor.setNeutralMode(self._params.neutral_mode)

    def follow_angle(self, angle: Rotation2d):
        converted_angle = conversions.degrees_to_falcon(angle, self._params.gear_ratio)
        self._motor.set(ctre.ControlMode.Position, converted_angle)

    def reset(self):
        absolute_position = self._absolute_encoder.absolute_position - self._offset
        converted_position = conversions.degrees_to_falcon(absolute_position, self._params.gear_ratio)
        self._motor.setSelectedSensorPosition(converted_position)

    @property
    def rotational_velocity(self) -> Rotation2d:
        dps = conversions.falcon_to_dps(self._motor.getSelectedSensorVelocity(), self._params.gear_ratio)
        return Rotation2d.fromDegrees(dps)

    @property
    def angle(self) -> Rotation2d:
        return conversions.falcon_to_degrees(self._motor.getSelectedSensorPosition(), self._params.gear_ratio)


class AbsoluteCANCoder(AbsoluteEncoder):
    def __init__(self, id_: int):
        self._encoder = ctre.CANCoder(id_)
        self._encoder.configAbsoluteSensorRange(ctre.AbsoluteSensorRange.Unsigned_0_to_360)

    @property
    def absolute_position(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self._encoder.getAbsolutePosition())


class PigeonGyro(Gyro):
    def __init__(self, id_: int):
        self._gyro = ctre.PigeonIMU(id_)

    def zero_heading(self):
        self._gyro.setYaw(0)

    @property
    def heading(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self._gyro.getYaw())
