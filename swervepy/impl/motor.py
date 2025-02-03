import copy
import math
from dataclasses import dataclass
from enum import IntEnum

import phoenix6.configs
import phoenix6.controls
import phoenix6.hardware
import phoenix6.signals
import rev
from pint import Quantity
from typing_extensions import deprecated
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor

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

        self._motor_sim = self._motor.sim_state
        self._motor_sim.orientation = (
            phoenix6.sim.ChassisReference.Clockwise_Positive
            if parameters.invert_motor
            else phoenix6.sim.ChassisReference.CounterClockwise_Positive
        )

        self._duty_cycle_request = phoenix6.controls.DutyCycleOut(0)
        self._velocity_request = phoenix6.controls.VelocityVoltage(0)
        self._voltage_request = phoenix6.controls.VoltageOut(0)

        self._velocity_signal = self._motor.get_velocity()
        self._position_signal = self._motor.get_position()
        self._voltage_output_signal = self._motor.get_motor_voltage()

    def _config(self):
        configs = phoenix6.configs.TalonFXConfiguration()

        configs.current_limits.supply_current_limit_enable = True
        configs.current_limits.supply_current_limit = self._params.peak_current_limit
        configs.current_limits.supply_current_lower_time = self._params.peak_current_duration
        configs.current_limits.supply_current_lower_limit = self._params.continuous_current_limit

        configs.slot0.k_p = self._params.kP
        configs.slot0.k_i = self._params.kI
        configs.slot0.k_d = self._params.kD

        configs.open_loop_ramps.duty_cycle_open_loop_ramp_period = self._params.open_loop_ramp_rate
        configs.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = self._params.closed_loop_ramp_rate

        # Convert from generalized neutral mode and invert value to CTRE API enums
        configs.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue(self._params.neutral_mode)
        configs.motor_output.inverted = phoenix6.signals.InvertedValue(self._params.invert_motor)

        configs.feedback.sensor_to_mechanism_ratio = self._params.gear_ratio

        # Configs are automatically factory-defaulted
        self._motor.configurator.apply(configs)

    def follow_velocity_open(self, velocity: float):
        percent_out = velocity / self._params.max_speed
        self._motor.set_control(self._duty_cycle_request.with_output(percent_out))

        # TODO Improve motor simulation to use DCMotorSim and physics
        converted_velocity = conversions.metres_to_rotations(velocity, self._params.wheel_circumference)
        self._motor_sim.set_rotor_velocity(self._params.gear_ratio * converted_velocity)

    def follow_velocity_closed(self, velocity: float):
        converted_velocity = conversions.metres_to_rotations(velocity, self._params.wheel_circumference)
        ff = self._feedforward.calculate(velocity)
        self._motor.set_control(self._velocity_request.with_velocity(converted_velocity).with_feed_forward(ff))

        self._motor_sim.set_rotor_velocity(self._params.gear_ratio * converted_velocity)

    def set_voltage(self, volts: float):
        self._motor.set_control(self._voltage_request.with_output(volts))

    def reset(self):
        self._motor.set_position(0)

    def simulation_periodic(self, delta_time: float):
        delta_pos = self._velocity_signal.refresh().value * delta_time
        # This is position of the motor's output shaft, so the gear ratio between output shaft and mechanism needs to
        # be factored in
        self._motor_sim.add_rotor_position(self._params.gear_ratio * delta_pos)

    @property
    def velocity(self) -> float:
        return conversions.rotations_to_metres(self._velocity_signal.refresh().value, self._params.wheel_circumference)

    @property
    def distance(self) -> float:
        return conversions.rotations_to_metres(self._position_signal.refresh().value, self._params.wheel_circumference)

    @property
    def voltage(self) -> float:
        return self._voltage_output_signal.refresh().value


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
            self._motor = phoenix6.hardware.TalonFX(*id_)
        except TypeError:
            # Only an int was provided for id_
            self._motor = phoenix6.hardware.TalonFX(id_)

        self._absolute_encoder = absolute_encoder
        self._offset = azimuth_offset

        self._config()
        self.reset()

        self._motor_sim = self._motor.sim_state
        self._motor_sim.orientation = (
            phoenix6.sim.ChassisReference.Clockwise_Positive
            if parameters.invert_motor
            else phoenix6.sim.ChassisReference.CounterClockwise_Positive
        )

        self._position_request = phoenix6.controls.PositionVoltage(0)

        self._velocity_signal = self._motor.get_velocity()
        self._position_signal = self._motor.get_position()

    def _config(self):
        configs = phoenix6.configs.TalonFXConfiguration()

        configs.current_limits.supply_current_limit_enable = True
        configs.current_limits.supply_current_limit = self._params.peak_current_limit
        configs.current_limits.supply_current_lower_time = self._params.peak_current_duration
        configs.current_limits.supply_current_lower_limit = self._params.continuous_current_limit

        configs.slot0.k_p = self._params.kP
        configs.slot0.k_i = self._params.kI
        configs.slot0.k_d = self._params.kD

        configs.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = self._params.ramp_rate

        # Convert from generalized neutral mode and invert value to CTRE API enums
        configs.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue(self._params.neutral_mode)
        configs.motor_output.inverted = phoenix6.signals.InvertedValue(self._params.invert_motor)

        configs.feedback.sensor_to_mechanism_ratio = self._params.gear_ratio

        # Configs are automatically factory-defaulted
        self._motor.configurator.apply(configs)

    def follow_angle(self, angle: Rotation2d):
        rotations = angle.radians() / (2 * math.pi)
        self._motor.set_control(self._position_request.with_position(rotations))

        self._motor_sim.set_raw_rotor_position(self._params.gear_ratio * rotations)

    def reset(self):
        absolute_position = self._absolute_encoder.absolute_position - self._offset
        converted_position = absolute_position.radians() / (2 * math.pi)
        self._motor.set_position(converted_position)

    @property
    def rotational_velocity(self) -> float:
        """Rotational velocity in radians/sec"""
        return self._velocity_signal.refresh().value * (2 * math.pi)

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d(self._position_signal.refresh().value * (2 * math.pi))


class NEOCoaxialDriveComponent(CoaxialDriveComponent):
    def __init__(self, id_: int, parameters: TypicalDriveComponentParameters):
        self._params = parameters.in_standard_units()

        self._motor = rev.SparkMax(id_, rev.SparkMax.MotorType.kBrushless)
        self._controller = self._motor.getClosedLoopController()
        self._encoder = self._motor.getEncoder()
        self._config()
        self.reset()

        self._feedforward = SimpleMotorFeedforwardMeters(parameters.kS, parameters.kV, parameters.kA)

        self._sim_encoder = rev.SparkRelativeEncoderSim(self._motor)

    def _config(self):
        settings = rev.SparkBaseConfig()

        settings.closedLoop.pid(self._params.kP, self._params.kI, self._params.kD, rev.ClosedLoopSlot.kSlot0)

        settings.smartCurrentLimit(self._params.continuous_current_limit)
        settings.secondaryCurrentLimit(self._params.peak_current_limit)

        settings.openLoopRampRate(self._params.open_loop_ramp_rate)
        settings.closedLoopRampRate(self._params.closed_loop_ramp_rate)

        settings.inverted(self._params.invert_motor)

        # Convert generic neutral mode to REV IdleMode
        settings.setIdleMode(rev.SparkBaseConfig.IdleMode(self._params.neutral_mode))

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

        self._sim_encoder.setVelocity(velocity)

    def follow_velocity_closed(self, velocity: float):
        self._controller.setReference(
            velocity,
            rev.SparkMax.ControlType.kVelocity,
            arbFeedforward=self._feedforward.calculate(velocity),
            arbFFUnits=rev.SparkClosedLoopController.ArbFFUnits.kVoltage,
        )

        self._sim_encoder.setVelocity(velocity)

    def set_voltage(self, volts: float):
        self._motor.setVoltage(volts)

    def reset(self):
        self._encoder.setPosition(0)

    def simulation_periodic(self, delta_time: float):
        new_position = self.distance + self.velocity * delta_time
        self._sim_encoder.setPosition(new_position)

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

        self._sim_encoder = rev.SparkRelativeEncoderSim(self._motor)

        self.reset()

    def _config(self):
        settings = rev.SparkBaseConfig()

        settings.closedLoop.pid(self._params.kP, self._params.kI, self._params.kD, rev.ClosedLoopSlot.kSlot0)

        settings.smartCurrentLimit(self._params.continuous_current_limit)
        settings.secondaryCurrentLimit(self._params.peak_current_limit)

        settings.closedLoopRampRate(self._params.ramp_rate)

        settings.inverted(self._params.invert_motor)

        # Convert generic neutral mode to REV IdleMode
        settings.setIdleMode(rev.SparkBaseConfig.IdleMode(self._params.neutral_mode))

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

        self._sim_encoder.setPosition(degrees)

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
