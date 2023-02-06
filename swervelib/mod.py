from abc import ABC, abstractmethod

import ctre
import rev
import wpilib
import wpimath.controller
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from . import configs
from . import conversions
from .configs import (
    SwerveParameters,
    SwerveModuleParameters,
    CTRESwerveModuleParameters,
    CTRESwerveParameters,
    REVSwerveParameters,
    REVSwerveModuleParameters,
)
from .dummy import Dummy


class SwerveModule(ABC):
    """
    One swerve module. A swerve drivetrain is made up of four modules and is represented by the Swerve class.
    """

    __slots__ = "drive_motor", "angle_motor", "angle_encoder", "swerve_params", "angle_offset", "feedforward", "corner"

    def __init__(self, module_params: SwerveModuleParameters, swerve_params: SwerveParameters):
        """
        Constructor for a SwerveModule.

        :param module_params: Module-specific parameters that describe location, device IDs, and offsets
        :param swerve_params: General parameters describing the drivetrain's hardware
        """

        # Convert the parameters to decimal values because doing unit conversions every iteration takes too long
        # noinspection PyTypeChecker
        self.swerve_params: SwerveParameters = swerve_params.in_standard_units()

        self.angle_offset = module_params.angle_offset
        self.corner = module_params.corner

        # Feedforward predicts the voltage required to spin a wheel up to a velocity in closed-loop control.
        # Feedforward is different from feedback (PID) in that it exerts control proactively rather than reactively
        # when error is detected.
        self.feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            swerve_params.drive_kS,
            swerve_params.drive_kV,
            swerve_params.drive_kA,
        )

        # Set up the CAN devices as NoOp objects for testing swerve without all devices on the robot
        if module_params.fake:
            self.angle_encoder = Dummy()
            return

        self.angle_encoder = ctre.WPI_CANCoder(*module_params.angle_encoder_id)
        self._config_angle_encoder()

    def _config_angle_encoder(self):
        settings = configs.create_CANCoder_config(self.swerve_params)
        self.angle_encoder.configFactoryDefault()
        self.angle_encoder.configAllSettings(settings)

    @abstractmethod
    def desire_state(self, desired_state: SwerveModuleState, open_loop: bool):
        """
        Begin exerting control to reach a desired state. This method only needs to be called when there is a change in
        desired state, not periodically, because it offloads feedback control onto the motor controllers.

        :param desired_state: The module's new desired state
        :param open_loop: If False, use velocity control. Else, use percent output
        """

    @abstractmethod
    def zero_distance(self):
        pass

    @abstractmethod
    def reset_to_absolute(self):
        pass

    @property
    @abstractmethod
    def velocity(self) -> float:
        pass

    @property
    @abstractmethod
    def distance(self) -> float:
        pass

    @property
    @abstractmethod
    def angle(self) -> Rotation2d:
        pass

    @property
    def state(self) -> SwerveModuleState:
        return SwerveModuleState(self.velocity, self.angle)

    @property
    def position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.distance, self.angle)

    @property
    def absolute_encoder_rotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.angle_encoder.getAbsolutePosition())


class CTRESwerveModule(SwerveModule):
    def __init__(self, module_params: CTRESwerveModuleParameters, swerve_params: CTRESwerveParameters):
        SwerveModule.__init__(self, module_params, swerve_params)

        self.swerve_params: CTRESwerveParameters = swerve_params.in_standard_units()

        # Set up the CAN devices as NoOp objects for testing swerve without all devices on the robot
        if module_params.fake:
            self.drive_motor, self.angle_motor = [Dummy()] * 2
            return

        self.drive_motor = ctre.WPI_TalonFX(*module_params.drive_motor_id)
        self._config_drive_motor()

        self.angle_motor = ctre.WPI_TalonFX(*module_params.angle_motor_id)
        self._config_angle_motor()

    def desire_state(self, desired_state: SwerveModuleState, open_loop: bool):
        # Optimize the desired state so that the module rotates to it as quick as possible
        desired_state = optimize(desired_state, self.state.angle)

        if open_loop:
            percent_output = desired_state.speed / self.swerve_params.max_speed
            self.drive_motor.set(ctre.ControlMode.PercentOutput, percent_output)
        else:
            velocity = conversions.mps_to_falcon(
                desired_state.speed,
                self.swerve_params.wheel_circumference,
                self.swerve_params.drive_gear_ratio,
            )
            self.drive_motor.set(
                ctre.ControlMode.Velocity,
                velocity,
                ctre.DemandType.ArbitraryFeedForward,
                self.feedforward.calculate(desired_state.speed),
            )

        angle = conversions.degrees_to_falcon(desired_state.angle, self.swerve_params.angle_gear_ratio)
        self.angle_motor.set(ctre.ControlMode.Position, angle)

        wpilib.SmartDashboard.putNumber(f"{self.corner.name} Desired Angle (deg)", desired_state.angle.degrees())
        wpilib.SmartDashboard.putNumber(f"{self.corner.name} Desired Velocity (mps)", desired_state.speed)

    def _config_angle_motor(self):
        settings = configs.create_TalonFX_angle_config(self.swerve_params)
        self.angle_motor.configFactoryDefault()
        self.angle_motor.configAllSettings(settings)
        self.angle_motor.setInverted(self.swerve_params.invert_angle_motor)
        self.angle_motor.setNeutralMode(self.swerve_params.angle_neutral_mode)

    def _config_drive_motor(self):
        settings = configs.create_TalonFX_drive_config(self.swerve_params)
        self.drive_motor.configFactoryDefault()
        self.drive_motor.configAllSettings(settings)
        self.drive_motor.setInverted(self.swerve_params.invert_drive_motor)
        self.drive_motor.setNeutralMode(self.swerve_params.drive_neutral_mode)
        self.drive_motor.setSelectedSensorPosition(0)

    def zero_distance(self):
        self.drive_motor.setSelectedSensorPosition(0)

    def reset_to_absolute(self):
        absolute_position = self.absolute_encoder_rotation
        absolute_position -= self.angle_offset
        falcon_ticks = conversions.degrees_to_falcon(absolute_position, self.swerve_params.angle_gear_ratio)
        self.angle_motor.setSelectedSensorPosition(falcon_ticks)

    @property
    def velocity(self) -> float:
        return conversions.falcon_to_mps(
            self.drive_motor.getSelectedSensorVelocity(),
            self.swerve_params.wheel_circumference,
            self.swerve_params.drive_gear_ratio,
        )

    @property
    def distance(self) -> float:
        return conversions.falcon_to_metres(
            self.drive_motor.getSelectedSensorPosition(),
            self.swerve_params.wheel_circumference,
            self.swerve_params.drive_gear_ratio,
        )

    @property
    def angle(self) -> Rotation2d:
        return conversions.falcon_to_degrees(
            self.angle_motor.getSelectedSensorPosition(),
            self.swerve_params.angle_gear_ratio,
        )


class REVSwerveModule(SwerveModule):
    __slots__ = "drive_controller", "angle_controller", "drive_integrated_encoder", "angle_integrated_encoder"

    def __init__(self, module_params: REVSwerveModuleParameters, swerve_params: REVSwerveParameters):
        SwerveModule.__init__(self, module_params, swerve_params)

        # noinspection PyTypeChecker
        self.swerve_params: REVSwerveParameters = swerve_params.in_standard_units()

        # Set up the CAN devices as NoOp objects for testing swerve without all devices on the robot
        if module_params.fake:
            (
                self.drive_motor,
                self.drive_controller,
                self.drive_integrated_encoder,
                self.angle_motor,
                self.angle_controller,
                self.angle_integrated_encoder,
            ) = [Dummy()] * 6
            return

        self.drive_motor = rev.CANSparkMax(module_params.drive_motor_id, rev.CANSparkMax.MotorType.kBrushless)
        self.drive_controller = self.drive_motor.getPIDController()
        self.drive_integrated_encoder = self.drive_motor.getEncoder()
        self._config_drive_motor()

        self.angle_motor = rev.CANSparkMax(module_params.angle_motor_id, rev.CANSparkMax.MotorType.kBrushless)
        self.angle_controller = self.angle_motor.getPIDController()
        self.angle_integrated_encoder = self.angle_motor.getEncoder()
        self._config_angle_motor()

    def desire_state(self, desired_state: SwerveModuleState, open_loop: bool):
        # Optimize the desired state so that the module rotates to it as quick as possible
        desired_state = optimize(desired_state, self.state.angle)

        velocity = desired_state.speed
        if open_loop:
            percent_output = velocity / self.swerve_params.max_speed
            self.drive_motor.set(percent_output)
        else:
            self.drive_controller.setReference(
                velocity, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=self.feedforward.calculate(velocity)
            )

        angle = desired_state.angle.degrees()
        self.angle_controller.setReference(angle, rev.CANSparkMax.ControlType.kPosition)

        wpilib.SmartDashboard.putNumber(f"{self.corner.name} Desired Angle (deg)", angle)
        wpilib.SmartDashboard.putNumber(f"{self.corner.name} Desired Velocity (mps)", velocity)

    def _config_angle_motor(self):
        self.angle_motor.restoreFactoryDefaults()

        self.angle_controller.setP(self.swerve_params.angle_kP)
        self.angle_controller.setI(self.swerve_params.angle_kI)
        self.angle_controller.setD(self.swerve_params.angle_kD)
        self.angle_controller.setFF(self.swerve_params.angle_kF)

        self.angle_motor.setSmartCurrentLimit(self.swerve_params.angle_continuous_current_limit)
        self.angle_motor.setSecondaryCurrentLimit(self.swerve_params.angle_peak_current_limit)

        self.angle_motor.setInverted(self.swerve_params.invert_angle_motor)
        self.angle_motor.setIdleMode(self.swerve_params.angle_neutral_mode)

        position_conversion_factor = self.swerve_params.wheel_circumference / self.swerve_params.angle_gear_ratio
        self.angle_integrated_encoder.setPositionConversionFactor(position_conversion_factor)

        self.angle_motor.burnFlash()
        self.reset_to_absolute()

    def _config_drive_motor(self):
        self.drive_motor.restoreFactoryDefaults()

        self.drive_controller.setP(self.swerve_params.drive_kP)
        self.drive_controller.setI(self.swerve_params.drive_kI)
        self.drive_controller.setD(self.swerve_params.drive_kD)
        self.drive_controller.setFF(self.swerve_params.drive_kF)

        self.drive_motor.setSmartCurrentLimit(self.swerve_params.drive_continuous_current_limit)
        self.drive_motor.setSecondaryCurrentLimit(self.swerve_params.drive_peak_current_limit)

        self.drive_motor.setOpenLoopRampRate(self.swerve_params.drive_open_loop_ramp)
        self.drive_motor.setClosedLoopRampRate(self.swerve_params.drive_closed_loop_ramp)

        self.drive_motor.setInverted(self.swerve_params.invert_drive_motor)
        self.drive_motor.setIdleMode(self.swerve_params.drive_neutral_mode)

        position_conversion_factor = self.swerve_params.wheel_circumference / self.swerve_params.drive_gear_ratio
        self.drive_integrated_encoder.setPositionConversionFactor(position_conversion_factor)
        self.drive_integrated_encoder.setVelocityConversionFactor(position_conversion_factor / 60)

        self.drive_motor.burnFlash()
        self.drive_integrated_encoder.setPosition(0)

    def zero_distance(self):
        self.drive_integrated_encoder.setPosition(0)

    def reset_to_absolute(self):
        absolute_position = self.absolute_encoder_rotation
        absolute_position -= self.angle_offset
        self.angle_encoder.setPosition(absolute_position.degrees())

    @property
    def velocity(self) -> float:
        return self.drive_integrated_encoder.getVelocity()

    @property
    def distance(self) -> float:
        return self.drive_integrated_encoder.getPosition()

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.angle_integrated_encoder.getPosition())


def _sign(num):
    return 1 if num > 0 else -1 if num < 0 else 0


def place_in_proper_0_to_360_scope(scope_reference: float, new_angle: float) -> float:
    # Place the new_angle in the range that is a multiple of [0, 360] (e.g., [360, 720]) which is closest
    # to the scope_reference
    lower_offset = scope_reference % 360
    lower_bound = scope_reference - lower_offset
    upper_bound = lower_bound + 360

    while new_angle < lower_bound:
        new_angle += 360
    while new_angle > upper_bound:
        new_angle -= 360

    if new_angle - scope_reference > 180:
        new_angle -= 360
    elif new_angle - scope_reference < -180:
        new_angle += 360

    return new_angle


def optimize(desired_state: SwerveModuleState, current_angle: Rotation2d):
    # There are two ways for a swerve module to reach its goal
    # 1) Rotate to its intended rotation and drive at its intended speed
    # 2) Rotate to the mirrored rotation (subtract 180) and drive at the opposite of its intended speed
    # Optimizing finds the option that requires the smallest rotation by the module

    target_angle = place_in_proper_0_to_360_scope(current_angle.degrees(), desired_state.angle.degrees())
    target_speed = desired_state.speed
    delta = target_angle - current_angle.degrees()

    if abs(delta) > 90:
        target_speed *= -1
        target_angle -= 180 * _sign(delta)

    return SwerveModuleState(target_speed, Rotation2d.fromDegrees(target_angle))
