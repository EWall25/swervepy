import ctre
import wpilib
import wpimath.controller
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from . import conversions
from .configs import SwerveParameters, SwerveModuleParameters, CTREConfigs
from .dummy import Dummy
from .units import u, Velocity, Distance


class SwerveModule:
    __slots__ = "drive_motor", "angle_motor", "angle_encoder", "swerve_params", "angle_offset", "feedforward", "corner"

    def __init__(self, module_params: SwerveModuleParameters, swerve_params: SwerveParameters):
        self.swerve_params = swerve_params
        self.angle_offset = module_params.angle_offset
        self.corner = module_params.corner

        self.feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            swerve_params.drive_kS,
            swerve_params.drive_kV,
            swerve_params.drive_kA,
        )

        # Set up the CAN devices as NoOp objects for testing swerve without all devices on the robot
        if module_params.fake:
            self.angle_encoder, self.drive_motor, self.angle_motor = [Dummy()] * 3
            return

        ctre_configs = CTREConfigs(swerve_params)

        self.angle_encoder = ctre.WPI_CANCoder(*module_params.angle_encoder_id)
        self._config_angle_encoder(ctre_configs.swerve_cancoder_config)

        self.drive_motor = ctre.WPI_TalonFX(*module_params.drive_motor_id)
        self._config_drive_motor(ctre_configs.swerve_drive_config)

        self.angle_motor = ctre.WPI_TalonFX(*module_params.angle_motor_id)
        self._config_angle_motor(ctre_configs.swerve_angle_config)

    def desire_state(self, desired_state: SwerveModuleState, open_loop: bool):
        # Optimize the desired state so that the module rotates to it as quick as possible
        desired_state = optimize(desired_state, self.state.angle)

        if open_loop:
            percent_output = desired_state.speed / self.swerve_params.max_speed.m_as(u.m / u.s)
            self.drive_motor.set(ctre.ControlMode.PercentOutput, percent_output)
        else:
            velocity = conversions.mps_to_falcon(
                desired_state.speed * (u.m / u.s),
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

    def _config_angle_encoder(self, config: ctre.CANCoderConfiguration):
        self.angle_encoder.configFactoryDefault()
        self.angle_encoder.configAllSettings(config)

    def _config_angle_motor(self, config: ctre.TalonFXConfiguration):
        self.angle_motor.configFactoryDefault()
        self.angle_motor.configAllSettings(config)
        self.angle_motor.setInverted(self.swerve_params.invert_angle_motor)
        self.angle_motor.setNeutralMode(self.swerve_params.angle_neutral_mode)

    def _config_drive_motor(self, config: ctre.TalonFXConfiguration):
        self.drive_motor.configFactoryDefault()
        self.drive_motor.configAllSettings(config)
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
    def velocity(self) -> Velocity:
        return conversions.falcon_to_mps(
            self.drive_motor.getSelectedSensorVelocity(),
            self.swerve_params.wheel_circumference,
            self.swerve_params.drive_gear_ratio,
        )

    @property
    def distance(self) -> Distance:
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

    @property
    def state(self) -> SwerveModuleState:
        return SwerveModuleState(self.velocity.m, self.angle)

    @property
    def position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.distance.m, self.angle)

    @property
    def absolute_encoder_rotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.angle_encoder.getAbsolutePosition())


def _sign(num):
    return 1 if num > 0 else -1 if num < 0 else 0


def place_in_proper_0_to_360_scope(scope_reference: float, new_angle: float) -> float:
    # Place the new_angle in the range that is a multiple of [0, 360] (e.g., [360, 720]) which is closest
    # to the scope_reference
    # TODO: Write tests
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
