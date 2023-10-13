from abc import abstractmethod

from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpiutil import Sendable

from . import SendableABCMeta


class SwerveModule(Sendable, metaclass=SendableABCMeta):
    placement: Translation2d

    def desire_state(self, state: SwerveModuleState, drive_open_loop: bool, rotate_in_place: bool):
        """
        Command the module to a speed and angle

        :param state: SwerveModuleState representing the module's desired speed and angle
        :param drive_open_loop: Use open loop control (True) or closed loop (False) to drive the wheel
        :param rotate_in_place: Should the modules rotate while not driving
        :return:
        """
        state = optimize(state, self.azimuth_angle)

        # Prevent rotating the module if drive speed is less than 2 cm/s to prevent feedback-loop jitter
        angle = state.angle if rotate_in_place or abs(state.speed) > 0.02 else self.azimuth_angle

        self.desire_drive_velocity(state.speed, drive_open_loop)
        self.desire_azimuth_angle(angle)

    @property
    def module_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drive_distance, self.azimuth_angle)

    @abstractmethod
    def desire_drive_velocity(self, velocity: float, open_loop: bool):
        """
        Drive the wheel

        :param velocity: Desired velocity in m/s
        :param open_loop: Use open loop control (True) or closed loop (False)
        """
        raise NotImplementedError

    @abstractmethod
    def desire_azimuth_angle(self, angle: Rotation2d):
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        raise NotImplementedError

    @property
    @abstractmethod
    def drive_velocity(self) -> float:
        """Drive wheel velocity in m/s"""
        raise NotImplementedError

    @property
    @abstractmethod
    def drive_distance(self) -> float:
        """Driven distance in metres"""
        raise NotImplementedError

    @property
    @abstractmethod
    def azimuth_angle(self) -> Rotation2d:
        """CCW+ module angle"""
        raise NotImplementedError

    @property
    @abstractmethod
    def azimuth_velocity(self) -> float:
        """CCW+ module angular velocity in rad/s"""
        raise NotImplementedError


def sign(num):
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
        target_angle -= 180 * sign(delta)

    return SwerveModuleState(target_speed, Rotation2d.fromDegrees(target_angle))
