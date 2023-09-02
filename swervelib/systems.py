import copy
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable

import commands2
import ctre
import wpilib
import wpimath.kinematics
import wpimath.estimator
from pint import Quantity
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.kinematics import SwerveModulePosition, ChassisSpeeds, SwerveModuleState
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpiutil import SendableBuilder

from . import u, conversions
from .abstracts import SwerveModule, CoaxialDriveMotor, CoaxialAzimuthMotor, Gyro, AbsoluteEncoder

if TYPE_CHECKING:
    from wpimath.kinematics import SwerveDrive4Kinematics
    from wpimath.estimator import SwerveDrive4PoseEstimator


class CoaxialSwerveModule(SwerveModule):
    last_commanded_drive_velocity = 0
    last_commanded_azimuth_angle = Rotation2d.fromDegrees(0)

    def __init__(self, drive_motor: CoaxialDriveMotor, azimuth_motor: CoaxialAzimuthMotor, placement: Translation2d):
        super().__init__()

        self._drive_motor = drive_motor
        self._azimuth_motor = azimuth_motor
        self.placement = placement

    def desire_drive_velocity(self, velocity: float, open_loop: bool):
        self.last_commanded_drive_velocity = velocity
        if open_loop:
            self._drive_motor.follow_velocity_open(velocity)
        else:
            self._drive_motor.follow_velocity_closed(velocity)

    def desire_azimuth_angle(self, angle: Rotation2d):
        self.last_commanded_azimuth_angle = angle
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
    def azimuth_velocity(self) -> float:
        return self._azimuth_motor.rotational_velocity

    def initSendable(self, builder: SendableBuilder):
        # fmt: off
        builder.setSmartDashboardType("CoaxialSwerveModule")
        builder.addDoubleProperty("Drive Velocity (mps)", lambda: self._drive_motor.velocity, lambda: None)
        builder.addDoubleProperty("Drive Distance (m)", lambda: self._drive_motor.distance, lambda: None)
        builder.addDoubleProperty("Azimuth Velocity (radps)", lambda: self._azimuth_motor.rotational_velocity, lambda: None)
        builder.addDoubleProperty("Azimuth Position (rad)", lambda: self._azimuth_motor.angle.radians(), lambda: None)
        builder.addDoubleProperty("Azimuth Position (deg)", lambda: self._azimuth_motor.angle.degrees(), lambda: None)
        builder.addDoubleProperty("Desired Drive Velocity (mps)", lambda: self.last_commanded_drive_velocity, lambda: None)
        builder.addDoubleProperty("Desired Azimuth Position (rad)", lambda: self.last_commanded_azimuth_angle.radians(), lambda: None)
        builder.addDoubleProperty("Desired Azimuth Position (deg)", lambda: self.last_commanded_azimuth_angle.degrees(), lambda: None)
        # fmt: on


class SwerveDrive(commands2.SubsystemBase):
    def __init__(
        self, modules: tuple[SwerveModule, ...], gyro: Gyro, max_velocity: Quantity, max_angular_velocity: Quantity
    ):
        super().__init__()

        self._modules = modules
        self._gyro = gyro
        self.max_velocity: float = max_velocity.m_as(u.m / u.s)
        self.max_angular_velocity: float = max_angular_velocity.m_as(u.rad / u.s)

        self.reset_modules()

        # There are different classes for each number of swerve modules in a drive base,
        # so construct the class name from number of modules.
        self._kinematics: "SwerveDrive4Kinematics" = getattr(
            wpimath.kinematics, f"SwerveDrive{len(modules)}Kinematics"
        )(*[module.placement for module in self._modules])
        self._odometry: "SwerveDrive4PoseEstimator" = getattr(
            wpimath.estimator, f"SwerveDrive{len(modules)}PoseEstimator"
        )(self._kinematics, self._gyro.heading, self.module_positions, Pose2d())

        for i, module in enumerate(modules):
            wpilib.SmartDashboard.putData(f"Module {i}", module)

    def periodic(self):
        self._odometry.update(self._gyro.heading, self.module_positions)

    def drive(self, translation: Translation2d, rotation: float, field_relative: bool, open_loop: bool):
        speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, self._gyro.heading)
            if field_relative
            else ChassisSpeeds(translation.x, translation.y, rotation)
        )
        swerve_module_states = self._kinematics.toSwerveModuleStates(speeds)

        self.desire_module_states(swerve_module_states, open_loop)

    def desire_module_states(self, states: tuple[SwerveModuleState, ...], open_loop: bool = False):
        swerve_module_states = self._kinematics.desaturateWheelSpeeds(states, self.max_velocity)

        for i in range(4):
            module: SwerveModule = self._modules[i]
            module.desire_state(swerve_module_states[i], open_loop)

    @property
    def module_positions(self) -> tuple[SwerveModulePosition, ...]:
        return tuple(module.module_position for module in self._modules)

    # TODO: Add utility methods
    # TODO: Add commands

    def reset_modules(self):
        for module in self._modules:
            module.reset()

    def teleop_command(
        self,
        translation: Callable[[], float],
        strafe: Callable[[], float],
        rotation: Callable[[], float],
        field_relative: bool,
        open_loop: bool,
    ):
        return commands2.RunCommand(
            lambda: self.drive(
                Translation2d(translation(), strafe()) * self.max_velocity,
                rotation() * self.max_angular_velocity,
                field_relative,
                open_loop,
            ),
            self,
        )


class Falcon500CoaxialDriveMotor(CoaxialDriveMotor):
    @dataclass
    class Parameters:
        wheel_circumference: Quantity
        gear_ratio: float

        max_speed: Quantity

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

        def in_standard_units(self):
            data = copy.deepcopy(self)
            data.wheel_circumference = data.wheel_circumference.m_as(u.m)
            data.max_speed = data.max_speed.m_as(u.m / u.s)
            return data

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
        self._params = parameters.in_standard_units()

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

        max_angular_velocity: Quantity

        ramp_rate: float

        continuous_current_limit: int
        peak_current_limit: int
        peak_current_duration: float

        neutral_mode: ctre.NeutralMode

        kP: float
        kI: float
        kD: float

        invert_motor: bool

        def in_standard_units(self):
            data = copy.deepcopy(self)
            data.max_angular_velocity = data.max_angular_velocity.m_as(u.rad / u.s)
            return data

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
        self._params = parameters.in_standard_units()

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
    def rotational_velocity(self) -> float:
        return conversions.falcon_to_radps(self._motor.getSelectedSensorVelocity(), self._params.gear_ratio)

    @property
    def angle(self) -> Rotation2d:
        return conversions.falcon_to_degrees(self._motor.getSelectedSensorPosition(), self._params.gear_ratio)


class AbsoluteCANCoder(AbsoluteEncoder):
    def __init__(self, id_: int):
        super().__init__()

        self._encoder = ctre.CANCoder(id_)
        self._encoder.configAbsoluteSensorRange(ctre.AbsoluteSensorRange.Unsigned_0_to_360)

        wpilib.SmartDashboard.putData(f"Absolute CANCoder {id_}", self)

    @property
    def absolute_position(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self._encoder.getAbsolutePosition())


class PigeonGyro(Gyro):
    def __init__(self, id_: int, invert: bool = False):
        super().__init__()

        self._gyro = ctre.PigeonIMU(id_)
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
