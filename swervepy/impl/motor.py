import copy
from dataclasses import dataclass
from enum import IntEnum

import phoenix6.configs
import phoenix6.controls
import phoenix6.hardware
import phoenix6.signals
import rev
from pint import Quantity
from typing_extensions import deprecated
from wpilib.simulation import SimDeviceSim
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d

from .sensor import SparkMaxEncoderType, SparkMaxAbsoluteEncoder
from .. import conversions, u
from ..abstract.motor import CoaxialDriveComponent, CoaxialAzimuthComponent
from ..abstract.sensor import AbsoluteEncoder


class NeutralMode(IntEnum):
    COAST = 0
    BRAKE = 1


@dataclass
class TypicalDriveComponentParameters:
    wheel_circumference: Quantity
    gear_ratio: float

    max_speed: Quantity

    open_loop_ramp_rate: float
    closed_loop_ramp_rate: float

    continuous_current_limit: int
    peak_current_limit: int
    peak_current_duration: float  # Unused for REV implementation, but makes interoperability easier

    neutral_mode: NeutralMode

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


@dataclass
class TypicalAzimuthComponentParameters:
    gear_ratio: float

    max_angular_velocity: Quantity

    ramp_rate: float

    continuous_current_limit: int
    peak_current_limit: int
    peak_current_duration: float  # Unused for REV implementation, but makes interoperability easier

    neutral_mode: NeutralMode

    kP: float
    kI: float
    kD: float

    invert_motor: bool

    def in_standard_units(self):
        data = copy.deepcopy(self)
        data.max_angular_velocity = data.max_angular_velocity.m_as(u.rad / u.s)
        return data


class Falcon500CoaxialDriveComponent(CoaxialDriveComponent):
    def __init__(self, id_: int | tuple[int, str], parameters: TypicalDriveComponentParameters):
        self._params = parameters.in_standard_units()

        try:
            # Unpack tuple of motor id and CAN bus id into TalonFX constructor
            self._motor = phoenix6.hardware.TalonFX(*id_)
        except TypeError:
            # Only an int was provided for id_
            self._motor = phoenix6.hardware.TalonFX(id_)

        self._config()
        self.reset()

        self._feedforward = SimpleMotorFeedforwardMeters(parameters.kS, parameters.kV, parameters.kA)

    def _config(self):
        configs = phoenix6.configs.TalonFXConfiguration()

        supply_limit = phoenix5.SupplyCurrentLimitConfiguration(
            True,
            self._params.continuous_current_limit,
            self._params.peak_current_limit,
            self._params.peak_current_duration,
        )

        configs.slot0.kP = self._params.kP
        configs.slot0.kI = self._params.kI
        configs.slot0.kD = self._params.kD

        configs.current_limits.supply_current_limit = supply_limit
        configs.initializationStrategy = phoenix5.sensors.SensorInitializationStrategy.BootToZero
        configs.openloopRamp = self._params.open_loop_ramp_rate
        configs.closedloopRamp = self._params.closed_loop_ramp_rate

        self._motor.configFactoryDefault()
        self._motor.configAllSettings(configs)
        self._motor.setInverted(self._params.invert_motor)

        # Convert from generalized neutral mode to CTRE API NeutralMode
        self._motor.setNeutralMode(
            phoenix5.NeutralMode.Brake if self._params.neutral_mode is NeutralMode.BRAKE else phoenix5.NeutralMode.Coast
        )

    def follow_velocity_open(self, velocity: float):
        percent_out = velocity / self._params.max_speed
        self._motor.set(phoenix5.ControlMode.PercentOutput, percent_out)

        converted_velocity = conversions.mps_to_falcon(
            velocity, self._params.wheel_circumference, self._params.gear_ratio
        )
        # CTRE sim requires us to invert sensor readings ourselves
        self._sim_motor.setIntegratedSensorVelocity(int(converted_velocity * -1 if self._params.invert_motor else 1))

    def follow_velocity_closed(self, velocity: float):
        converted_velocity = conversions.mps_to_falcon(
            velocity, self._params.wheel_circumference, self._params.gear_ratio
        )
        self._motor.set(
            phoenix5.ControlMode.Velocity,
            converted_velocity,
            phoenix5.DemandType.ArbitraryFeedForward,
            self._feedforward.calculate(velocity),
        )

        # CTRE sim requires us to invert sensor readings ourselves
        self._sim_motor.setIntegratedSensorVelocity(int(converted_velocity * -1 if self._params.invert_motor else 1))

    def set_voltage(self, volts: float):
        percent_output = volts / self._motor.getBusVoltage()
        self._motor.set(phoenix5.ControlMode.PercentOutput, percent_output)

    def reset(self):
        self._motor.setSelectedSensorPosition(0)

    def simulation_periodic(self, delta_time: float):
        delta_pos = conversions.units_per_100_ms_to_units_per_sec(self._motor.getSelectedSensorVelocity()) * delta_time
        self._sim_motor.addIntegratedSensorPosition(int(delta_pos))

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

    @property
    def voltage(self) -> float:
        return self._motor.getMotorOutputVoltage()


class Falcon500CoaxialAzimuthComponent(CoaxialAzimuthComponent):
    def __init__(
        self,
        id_: int | tuple[int, str],
        azimuth_offset: Rotation2d,
        parameters: TypicalAzimuthComponentParameters,
        absolute_encoder: AbsoluteEncoder,
    ):
        self._params = parameters.in_standard_units()

        try:
            # Unpack tuple of motor id and CAN bus id into TalonFX constructor
            self._motor = phoenix5.WPI_TalonFX(*id_)
        except TypeError:
            # Only an int was provided for id_
            self._motor = phoenix5.WPI_TalonFX(id_)

        self._absolute_encoder = absolute_encoder
        self._offset = azimuth_offset

        self._config()
        self.reset()

        self._sim_motor = self._motor.getSimCollection()

    def _config(self):
        settings = phoenix5.TalonFXConfiguration()

        supply_limit = phoenix5.SupplyCurrentLimitConfiguration(
            True,
            self._params.continuous_current_limit,
            self._params.peak_current_limit,
            self._params.peak_current_duration,
        )

        settings.slot0.kP = self._params.kP
        settings.slot0.kI = self._params.kI
        settings.slot0.kD = self._params.kD
        settings.supplyCurrLimit = supply_limit
        settings.initializationStrategy = phoenix5.sensors.SensorInitializationStrategy.BootToZero
        settings.closedloopRamp = self._params.ramp_rate

        self._motor.configFactoryDefault()
        self._motor.configAllSettings(settings)
        self._motor.setInverted(self._params.invert_motor)
        self._motor.setNeutralMode(
            phoenix5.NeutralMode.Brake if self._params.neutral_mode is NeutralMode.BRAKE else phoenix5.NeutralMode.Coast
        )

    def follow_angle(self, angle: Rotation2d):
        converted_angle = conversions.degrees_to_falcon(angle, self._params.gear_ratio)
        self._motor.set(phoenix5.ControlMode.Position, converted_angle)

        # CTRE sim requires us to invert sensor readings ourselves
        self._sim_motor.setIntegratedSensorRawPosition(int(converted_angle * -1 if self._params.invert_motor else 1))

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
    def __init__(self, id_: int, parameters: TypicalDriveComponentParameters):
        self._params = parameters.in_standard_units()

        self._motor = rev.SparkMax(id_, rev.SparkMax.MotorType.kBrushless)
        self._controller = self._motor.getClosedLoopController()
        self._encoder = self._motor.getEncoder()
        self._config()
        self.reset()

        self._feedforward = SimpleMotorFeedforwardMeters(parameters.kS, parameters.kV, parameters.kA)

        sim_motor = SimDeviceSim(f"SPARK MAX [{id_}]")
        self._sim_velocity = sim_motor.getDouble("Velocity")
        self._sim_position = sim_motor.getDouble("Position")

    def _config(self):
        settings = rev.SparkBaseConfig()

        settings.closedLoop.pid(self._params.kP, self._params.kI, self._params.kD, rev.ClosedLoopSlot.kSlot0)

        settings.smartCurrentLimit(self._params.continuous_current_limit)
        settings.secondaryCurrentLimit(self._params.peak_current_limit)

        settings.openLoopRampRate(self._params.open_loop_ramp_rate)
        settings.closedLoopRampRate(self._params.closed_loop_ramp_rate)

        settings.inverted(self._params.invert_motor)

        # Convert generic neutral mode to REV IdleMode
        settings.setIdleMode(rev.SparkMax.IdleMode(self._params.neutral_mode))

        position_conversion_factor = self._params.wheel_circumference / self._params.gear_ratio
        settings.encoder.positionConversionFactor(position_conversion_factor)
        settings.encoder.velocityConversionFactor(position_conversion_factor / 60)

        self._motor.configure(
            settings,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

    def follow_velocity_open(self, velocity: float):
        percent_out = velocity / self._params.max_speed
        self._motor.set(percent_out)

        self._sim_velocity.set(velocity)

    def follow_velocity_closed(self, velocity: float):
        self._controller.setReference(
            velocity,
            rev.SparkMax.ControlType.kVelocity,
            arbFeedforward=self._feedforward.calculate(velocity),
        )

    def set_voltage(self, volts: float):
        self._motor.setVoltage(volts)

    def reset(self):
        self._encoder.setPosition(0)

    def simulation_periodic(self, delta_time: float):
        new_position = self.distance + self.velocity * delta_time
        self._sim_position.set(new_position)

    @property
    def velocity(self) -> float:
        return self._encoder.getVelocity()

    @property
    def distance(self) -> float:
        return self._encoder.getPosition()

    @property
    def voltage(self) -> float:
        return self._motor.getBusVoltage() * self._motor.getAppliedOutput()


class NEOCoaxialAzimuthComponent(CoaxialAzimuthComponent):
    def __init__(
        self,
        id_: int,
        azimuth_offset: Rotation2d,
        parameters: TypicalAzimuthComponentParameters,
        absolute_encoder: AbsoluteEncoder | SparkMaxEncoderType,
    ):
        self._params = parameters.in_standard_units()

        self._motor = rev.SparkMax(id_, rev.SparkMax.MotorType.kBrushless)
        self._controller = self._motor.getClosedLoopController()
        self._encoder = self._motor.getEncoder()

        # Config must be called before the absolute encoder is set up because config method
        # factory resets the SPARK MAX
        self._config()

        if isinstance(absolute_encoder, SparkMaxEncoderType):
            # Construct an AbsoluteEncoder from sensor plugged into SPARK MAX
            self._absolute_encoder = SparkMaxAbsoluteEncoder(self._motor, absolute_encoder)
        else:
            self._absolute_encoder = absolute_encoder

        self._offset = azimuth_offset

        self.reset()

        sim_motor = SimDeviceSim(f"SPARK MAX [{id_}]")
        self._sim_position = sim_motor.getDouble("Position")

    def _config(self):
        settings = rev.SparkBaseConfig()

        settings.closedLoop.pid(self._params.kP, self._params.kI, self._params.kD, rev.ClosedLoopSlot.kSlot0)

        settings.smartCurrentLimit(self._params.continuous_current_limit)
        settings.secondaryCurrentLimit(self._params.peak_current_limit)

        settings.closedLoopRampRate(self._params.ramp_rate)

        settings.inverted(self._params.invert_motor)

        # Convert generic neutral mode to REV IdleMode
        settings.setIdleMode(rev.SparkMax.IdleMode(self._params.neutral_mode))

        position_conversion_factor = 360 / self._params.gear_ratio
        settings.encoder.positionConversionFactor(position_conversion_factor)
        settings.encoder.velocityConversionFactor(position_conversion_factor / 60)

        self._motor.configure(
            settings,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

    def follow_angle(self, angle: Rotation2d):
        degrees = angle.degrees()
        self._controller.setReference(degrees, rev.SparkMax.ControlType.kPosition)

        self._sim_position.set(degrees)

    def reset(self):
        absolute_position = self._absolute_encoder.absolute_position - self._offset
        self._encoder.setPosition(absolute_position.degrees())

    @property
    def rotational_velocity(self) -> float:
        return self._encoder.getVelocity()

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self._encoder.getPosition())


@deprecated(
    "This component has been replaced by DummyCoaxialDriveComponent and DummyCoaxialAzimuthComponent."
    "Use those if you want simulation support."
)
class DummyCoaxialComponent(CoaxialDriveComponent, CoaxialAzimuthComponent):
    """Coaxial drive or azimuth component that does nothing"""

    def __init__(self, *args):
        pass

    def follow_velocity_open(self, velocity: float):
        pass

    def follow_velocity_closed(self, velocity: float):
        pass

    def set_voltage(self, volts: float):
        pass

    def reset(self):
        pass

    @property
    def velocity(self) -> float:
        return 0

    @property
    def distance(self) -> float:
        return 0

    @property
    def voltage(self) -> float:
        return 0

    def follow_angle(self, angle: Rotation2d):
        pass

    @property
    def rotational_velocity(self) -> float:
        return 0

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(0)


class DummyCoaxialDriveComponent(CoaxialDriveComponent):
    """Coaxial drive component that does nothing on a real robot, but functions normally in simulation"""

    def __init__(self, *args):
        self._velocity = 0
        self._position = 0

    def simulation_periodic(self, delta_time: float):
        self._position += self._velocity * delta_time

    def follow_velocity_open(self, velocity: float):
        self._velocity = velocity

    def follow_velocity_closed(self, velocity: float):
        self._velocity = velocity

    def set_voltage(self, volts: float):
        pass

    def reset(self):
        self._position = 0

    @property
    def velocity(self) -> float:
        return self._velocity

    @property
    def distance(self) -> float:
        return self._position

    @property
    def voltage(self) -> float:
        return 0


class DummyCoaxialAzimuthComponent(CoaxialAzimuthComponent):
    """Coaxial azimuth component that does nothing on a real robot, but functions normally in simulation"""

    def __init__(self, *args):
        self._angle = Rotation2d()

    def follow_angle(self, angle: Rotation2d):
        self._angle = angle

    def reset(self):
        pass

    @property
    def rotational_velocity(self) -> float:
        return 0

    @property
    def angle(self) -> Rotation2d:
        return self._angle
