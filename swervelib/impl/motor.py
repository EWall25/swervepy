import copy
from dataclasses import dataclass

import ctre
import rev
from pint import Quantity
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d

from ..abstract.motor import CoaxialDriveComponent, CoaxialAzimuthComponent
from .. import conversions, u
from ..abstract.sensor import AbsoluteEncoder


class Falcon500CoaxialDriveComponent(CoaxialDriveComponent):
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


class Falcon500CoaxialAzimuthComponent(CoaxialAzimuthComponent):
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


class NEOCoaxialDriveComponent(CoaxialDriveComponent):
    @dataclass
    class Parameters:
        wheel_circumference: Quantity
        gear_ratio: float

        max_speed: Quantity

        open_loop_ramp_rate: float
        closed_loop_ramp_rate: float

        continuous_current_limit: int
        peak_current_limit: int

        neutral_mode: rev.CANSparkMax.IdleMode

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

    def __init__(self, id_: int, parameters: Parameters):
        self._params = parameters.in_standard_units()

        self._motor = rev.CANSparkMax(id_, rev.CANSparkMax.MotorType.kBrushless)
        self._controller = self._motor.getPIDController()
        self._encoder = self._motor.getEncoder()
        self._config()
        self.reset()

        self._feedforward = SimpleMotorFeedforwardMeters(parameters.kS, parameters.kV, parameters.kA)

    def _config(self):
        self._motor.restoreFactoryDefaults()

        self._controller.setP(self._params.kP)
        self._controller.setI(self._params.kI)
        self._controller.setD(self._params.kD)

        self._motor.setSmartCurrentLimit(self._params.continuous_current_limit)
        self._motor.setSecondaryCurrentLimit(self._params.peak_current_limit)

        self._motor.setOpenLoopRampRate(self._params.open_loop_ramp_rate)
        self._motor.setClosedLoopRampRate(self._params.closed_loop_ramp_rate)

        self._motor.setInverted(self._params.invert_motor)
        self._motor.setIdleMode(self._params.neutral_mode)

        # TODO: Check velocity conversion factor correctness
        position_conversion_factor = self._params.wheel_circumference / self._params.gear_ratio
        self._encoder.setPositionConversionFactor(position_conversion_factor)
        self._encoder.setVelocityConversionFactor(position_conversion_factor / 60)

    def follow_velocity_open(self, velocity: float):
        percent_out = velocity / self._params.max_speed
        self._motor.set(percent_out)

    def follow_velocity_closed(self, velocity: float):
        self._controller.setReference(
            velocity, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=self._feedforward.calculate(velocity)
        )

    def reset(self):
        self._encoder.setPosition(0)

    @property
    def velocity(self) -> float:
        return self._encoder.getVelocity()

    @property
    def distance(self) -> float:
        return self._encoder.getPosition()


class NEOOnboardSensorCoaxialAzimuthComponent(CoaxialAzimuthComponent):
    @dataclass
    class Parameters:
        gear_ratio: float

        max_angular_velocity: Quantity

        ramp_rate: float

        continuous_current_limit: int
        peak_current_limit: int

        neutral_mode: rev.CANSparkMax.IdleMode

        kP: float
        kI: float
        kD: float

        invert_motor: bool

        def in_standard_units(self):
            data = copy.deepcopy(self)
            data.max_angular_velocity = data.max_angular_velocity.m_as(u.rad / u.s)
            return data

    def __init__(self, id_: int, azimuth_offset: Rotation2d, parameters: Parameters, absolute_encoder: AbsoluteEncoder):
        self._params = parameters.in_standard_units()

        self._motor = rev.CANSparkMax(id_, rev.CANSparkMax.MotorType.kBrushless)
        self._controller = self._motor.getPIDController()
        self._encoder = self._motor.getEncoder()

        self._absolute_encoder = absolute_encoder
        self._offset = azimuth_offset

        self._config()
        self.reset()

    def _config(self):
        self._motor.restoreFactoryDefaults()

        self._controller.setP(self._params.kP)
        self._controller.setI(self._params.kI)
        self._controller.setD(self._params.kD)

        self._motor.setSmartCurrentLimit(self._params.continuous_current_limit)
        self._motor.setSecondaryCurrentLimit(self._params.peak_current_limit)

        self._motor.setInverted(self._params.invert_motor)
        self._motor.setIdleMode(self._params.neutral_mode)

        # TODO: Check position factor for correctness
        position_conversion_factor = 360 / self._params.gear_ratio
        self._encoder.setPositionConversionFactor(position_conversion_factor)
        # TODO: Set velocity conversion factor

    def follow_angle(self, angle: Rotation2d):
        self._controller.setReference(angle.degrees(), rev.CANSparkMax.ControlType.kPosition)

    def reset(self):
        absolute_position = self._absolute_encoder.absolute_position - self._offset
        self._motor.setSelectedSensorPosition(absolute_position.degrees())

    @property
    def rotational_velocity(self) -> float:
        return self._encoder.getVelocity()

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self._encoder.getPosition())


class NEOExternalSensorCoaxialAzimuthComponent(CoaxialAzimuthComponent):
    @dataclass
    class Parameters:
        max_angular_velocity: Quantity

        ramp_rate: float

        continuous_current_limit: int
        peak_current_limit: int

        neutral_mode: rev.CANSparkMax.IdleMode

        kP: float
        kI: float
        kD: float

        invert_motor: bool
        invert_encoder: bool

        def in_standard_units(self):
            data = copy.deepcopy(self)
            data.max_angular_velocity = data.max_angular_velocity.m_as(u.rad / u.s)
            return data

    def __init__(self, id_: int, azimuth_offset: Rotation2d, parameters: Parameters):
        self._params = parameters.in_standard_units()

        self._motor = rev.CANSparkMax(id_, rev.CANSparkMax.MotorType.kBrushless)
        self._controller = self._motor.getPIDController()
        self._encoder = self._motor.getAbsoluteEncoder(rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)

        self._offset = azimuth_offset

        self._config()
        self.reset()

    def _config(self):
        self._controller.setFeedbackDevice(self._encoder)

        self._encoder.setInverted(self._params.invert_encoder)
        self._encoder.setZeroOffset(self._offset)

        self._motor.restoreFactoryDefaults()

        self._controller.setP(self._params.kP)
        self._controller.setI(self._params.kI)
        self._controller.setD(self._params.kD)

        self._motor.setSmartCurrentLimit(self._params.continuous_current_limit)
        self._motor.setSecondaryCurrentLimit(self._params.peak_current_limit)

        self._motor.setInverted(self._params.invert_motor)
        self._motor.setIdleMode(self._params.neutral_mode)

    def follow_angle(self, angle: Rotation2d):
        self._controller.setReference(angle.degrees(), rev.CANSparkMax.ControlType.kPosition)

    def reset(self):
        pass

    @property
    def rotational_velocity(self) -> float:
        return self._encoder.getVelocity()

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self._encoder.getPosition())
