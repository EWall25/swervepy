"""The swerve drive subsystem and other classes it relies on"""

import math
import time
from dataclasses import dataclass
from functools import singledispatchmethod
from typing import Callable, Optional, TYPE_CHECKING, Iterable

import commands2
from pathplannerlib.commands import FollowPathCommand, FollowPathWithEvents
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import ReplanningConfig, PIDConstants
from pathplannerlib.path import PathPlannerPath
import wpilib
import wpimath.estimator
import wpimath.kinematics
from pint import Quantity
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveModulePosition
from wpiutil import SendableBuilder

if TYPE_CHECKING:
    from wpimath.estimator import SwerveDrive4PoseEstimator
    from wpimath.kinematics import SwerveDrive4Kinematics

from swervepy import u
from swervepy.abstract import SwerveModule, Gyro


class SwerveDrive(commands2.Subsystem):
    def __init__(
        self,
        modules: tuple[SwerveModule, ...],
        gyro: Gyro,
        max_velocity: Quantity,
        max_angular_velocity: Quantity,
        vision_pose_callback: Callable[[Pose2d], Optional[Pose2d]] = lambda _: None,
    ):
        super().__init__()

        self._modules = modules
        self._gyro = gyro
        self._vision_pose_callback = vision_pose_callback
        self.max_velocity: float = max_velocity.m_as(u.m / u.s)
        self.max_angular_velocity: float = max_angular_velocity.m_as(u.rad / u.s)

        # Pause init for a second before setting module offsets to avoid a bug related to inverting motors.
        # Fixes https://github.com/Team364/BaseFalconSwerve/issues/8.
        time.sleep(1)
        self.reset_modules()

        # Zero heading at startup to set "forward" direction
        self.zero_heading()

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

        # Field to plot auto trajectories and robot pose
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

    def periodic(self):
        robot_pose = self._odometry.update(self._gyro.heading, self.module_positions)

        vision_pose = self._vision_pose_callback(self.pose)
        if vision_pose:
            self._odometry.addVisionMeasurement(vision_pose, wpilib.Timer.getFPGATimestamp())

        # Visualize robot position on field
        self.field.setRobotPose(robot_pose)

    @singledispatchmethod
    def drive(self, translation: Translation2d, rotation: float, field_relative: bool, open_loop: bool):
        """
        Command the robot to provided chassis speeds (translation and rotation)

        :param translation: Translation speed on the XY-plane in m/s where +X is forward and +Y is left
        :param rotation: Rotation speed around the Z-axis in rad/s where CCW+
        :param field_relative: If True, gyroscopic zero is used as the forward direction.
        Else, forward faces the front of the robot.
        :param open_loop: Use open loop control (True) or closed loop (False)
        """

        speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, self._gyro.heading)
            if field_relative
            else ChassisSpeeds(translation.x, translation.y, rotation)
        )
        swerve_module_states = self._kinematics.toSwerveModuleStates(speeds)

        self.desire_module_states(swerve_module_states, open_loop, rotate_in_place=False)

    @drive.register
    def _(self, chassis_speeds: ChassisSpeeds, field_relative: bool, open_loop: bool):
        translation = Translation2d(chassis_speeds.vx, chassis_speeds.vy)
        return self.drive(translation, chassis_speeds.omega, field_relative, open_loop)

    def desire_module_states(
        self, states: tuple[SwerveModuleState, ...], open_loop: bool = False, rotate_in_place: bool = True
    ):
        """
        Command each individual module to a state

        :param states: List of module states in the order of the swerve module list SwerveDrive was created with
        :param open_loop: Use open loop control (True) or closed loop (False)
        :param rotate_in_place: Should the modules rotate while not driving
        """

        swerve_module_states = self._kinematics.desaturateWheelSpeeds(states, self.max_velocity)  # type: ignore

        for i in range(len(self._modules)):
            module: SwerveModule = self._modules[i]
            module.desire_state(swerve_module_states[i], open_loop, rotate_in_place)

    @property
    def module_positions(self) -> tuple[SwerveModulePosition, ...]:
        return tuple(module.module_position for module in self._modules)

    @property
    def pose(self) -> Pose2d:
        return self._odometry.getEstimatedPosition()

    @property
    def heading(self) -> Rotation2d:
        return self._gyro.heading

    @property
    def robot_relative_speeds(self) -> ChassisSpeeds:
        module_states = tuple(module.module_state for module in self._modules)
        return self._kinematics.toChassisSpeeds(module_states)

    def reset_modules(self):
        for module in self._modules:
            module.reset()

    def zero_heading(self):
        self._gyro.zero_heading()

    def reset_odometry(self, pose: Pose2d):
        """
        Reset the drive base's pose to a new one

        :param pose: The new pose
        """

        self._odometry.resetPosition(self._gyro.heading, self.module_positions, pose)  # type: ignore

    def teleop_command(
        self,
        translation: Callable[[], float],
        strafe: Callable[[], float],
        rotation: Callable[[], float],
        field_relative: bool,
        open_loop: bool,
    ):
        return _TeleOpCommand(self, translation, strafe, rotation, field_relative, open_loop)

    def follow_trajectory_command(
        self,
        path: PathPlannerPath,
        parameters: "TrajectoryFollowerParameters",
        first_path: bool = False,
        open_loop: bool = False,
    ):
        """
        Build a command that follows a trajectory

        :param path: The path to follow
        :param parameters: Options that determine how the robot will follow the trajectory
        :param first_path: If True, the robot's pose will be reset to the trajectory's initial pose
        :param open_loop: Use open loop control (True) or closed loop (False) to swerve module speeds. Closed-loop
        positional control will always be used for trajectory following
        :return: Trajectory-follower command
        """

        # TODO: Re-impl trajectory visualisation on Field2d

        # Find the drive base radius (the distance from the center of the robot to the furthest module)
        radius = greatest_distance_from_translations([module.placement for module in self._modules])

        # Position feedback controller for following waypoints
        controller = PPHolonomicDriveController(
            PIDConstants(parameters.xy_kP),
            PIDConstants(parameters.theta_kP),
            parameters.max_drive_velocity.m_as(u.m / u.s),
            radius,
        )

        # Trajectory follower command
        command = FollowPathWithEvents(
            FollowPathCommand(
                path,
                lambda: self.pose,
                lambda: self.robot_relative_speeds,
                lambda speeds: self.drive(speeds, field_relative=False, open_loop=open_loop),
                controller,
                ReplanningConfig(),
                self,
            ),
            path,
            lambda: self.pose,
        )

        # If this is the first path in a sequence, reset the robot's pose so that it aligns with the start of the path
        if first_path:
            initial_pose = path.getPreviewStartingHolonomicPose()
            command = command.beforeStarting(commands2.InstantCommand(lambda: self.reset_odometry(initial_pose)))

        return command


class _TeleOpCommand(commands2.Command):
    def __init__(
        self,
        swerve: SwerveDrive,
        translation: Callable[[], float],
        strafe: Callable[[], float],
        rotation: Callable[[], float],
        field_relative: bool,
        open_loop: bool,
    ):
        super().__init__()
        self.addRequirements(swerve)

        self._swerve = swerve
        self._translation = translation
        self._strafe = strafe
        self._rotation = rotation
        self.field_relative = field_relative
        self.open_loop = open_loop

    def execute(self):
        self._swerve.drive(
            Translation2d(self._translation(), self._strafe()) * self._swerve.max_velocity,
            self._rotation() * self._swerve.max_angular_velocity,
            self.field_relative,
            self.open_loop,
        )

    def initSendable(self, builder: SendableBuilder):
        # fmt: off
        builder.addBooleanProperty("Field Relative", lambda: self.field_relative,
                                   lambda val: setattr(self, "field_relative", val))
        builder.addBooleanProperty("Open Loop", lambda: self.open_loop, lambda val: setattr(self, "open_loop", val))
        # fmt: on

    def toggle_field_relative(self):
        self.field_relative = not self.field_relative

    def toggle_open_loop(self):
        self.open_loop = not self.open_loop


@dataclass
class TrajectoryFollowerParameters:
    max_drive_velocity: Quantity

    # Positional PID constants for X, Y, and theta (rotation) controllers
    theta_kP: float
    xy_kP: float


def greatest_distance_from_translations(translations: Iterable[Translation2d]):
    """
    Calculates the magnitude of the longest translation from a list of translations.
    :param translations: List of translations
    :return: The magnitude
    """
    distances = tuple(math.sqrt(trans.x**2 + trans.y**2) for trans in translations)
    return max(distances)
