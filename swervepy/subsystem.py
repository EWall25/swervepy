"""The swerve drive subsystem and other classes it relies on"""

import math
import time
from dataclasses import dataclass
from functools import singledispatchmethod
from typing import Callable, Optional, TYPE_CHECKING, Iterable

import commands2
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.commands import FollowPathCommand
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import ReplanningConfig, PIDConstants
import wpilib
import wpilib.sysid
import wpimath.estimator
import wpimath.kinematics
from pint import Quantity
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveModulePosition
from wpiutil import SendableBuilder
from commands2.sysid import SysIdRoutine

if TYPE_CHECKING:
    from wpimath.estimator import SwerveDrive4PoseEstimator
    from wpimath.kinematics import SwerveDrive4Kinematics

from swervepy import u
from swervepy.abstract import SwerveModule, Gyro


class SwerveDrive(commands2.Subsystem):
    """
    A Subsystem representing the serve drivetrain.

    Use the drive() method to drive the swerve base at a set of desired velocities. The method handles
    moving the individual swerve modules, so the user only needs to consider the movement of the chassis itself.

    Field relative control is an option available for controlling movement. Field relative uses a gyro sensor
    to lock the robot's translational movement to the field's x- and y-axis. Practically, when a driver pushes "forward"
    on the stick, the robot will always move forward relative to the field rather than relative to the chassis heading.
    """

    def __init__(
        self,
        modules: tuple[SwerveModule, ...],
        gyro: Gyro,
        max_velocity: Quantity,
        max_angular_velocity: Quantity,
        vision_pose_callback: Callable[[], Optional[Pose2d]] = lambda: None,
    ):
        """
        Construct a swerve drivetrain as a Subsystem.

        :param modules: List of swerve modules
        :param gyro: A gyro sensor that provides a CCW+ heading reading of the chassis
        :param max_velocity: The actual maximum velocity of the robot
        :param max_angular_velocity: The actual maximum angular (turning) velocity of the robot
        :param vision_pose_callback: An optional method that returns the robot's pose derived from vision.
               This pose from this method is integrated into the robot's odometry.
        """

        super().__init__()

        self._modules = modules
        self._gyro = gyro
        self._vision_pose_callback = vision_pose_callback
        self.max_velocity: float = max_velocity.m_as(u.m / u.s)
        self.max_angular_velocity: float = max_angular_velocity.m_as(u.rad / u.s)
        self.period_seconds = 0.02

        # Pause init for a second before setting module offsets to avoid a bug related to inverting motors.
        # Fixes https://github.com/Team364/BaseFalconSwerve/issues/8.
        time.sleep(1)
        for module in self._modules:
            module.reset()

        # Zero heading at startup to set "forward" direction
        self._gyro.zero_heading()

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

        # Create a characterization routine to use with the SysId utility
        self._sysid_routine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(self._sysid_drive, self._sysid_log, self, "drive"),
        )

    def periodic(self):
        vision_pose = self._vision_pose_callback()
        # TODO: Add ability to specify custom timestamp
        if vision_pose:
            self._odometry.addVisionMeasurement(vision_pose, wpilib.Timer.getFPGATimestamp())

        robot_pose = self._odometry.update(self._gyro.heading, self.module_positions)

        # Visualize robot position on field
        self.field.setRobotPose(robot_pose)

    def simulationPeriodic(self):
        # Run a periodic simulation method that updates sensor readings based on desired velocities and rotations
        for module in self._modules:
            module.simulation_periodic(self.period_seconds)

        # Calculate the chassis angular velocity produced by the simulated swerve modules
        angular_velocity = self._kinematics.toChassisSpeeds(self.module_states).omega
        # Update the gyro heading reading with the change in heading since the last timestep
        self._gyro.simulation_periodic(angular_velocity * self.period_seconds)

    @singledispatchmethod
    def drive(
        self,
        translation: Translation2d,
        rotation: float,
        field_relative: bool,
        drive_open_loop: bool,
    ):
        """
        Drive the robot at the provided speeds (translation and rotation).

        By default, chassis speeds are discretized on an interval of 20ms.
        If your robot loop has a non-default period, you **must** set this subsystem's ``period_seconds`` field!

        :param translation: Translation speed on the XY-plane in m/s where +X is forward and +Y is left
        :param rotation: Rotation speed around the Z-axis in rad/s where CCW+
        :param field_relative: If True, gyroscopic zero is used as the forward direction.
               Else, forward faces the front of the robot.
        :param drive_open_loop: Use open loop (True) or closed loop (False) velocity control for driving the wheel
        """

        speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, self._gyro.heading)
            if field_relative
            else ChassisSpeeds(translation.x, translation.y, rotation)
        )
        speeds = ChassisSpeeds.discretize(speeds, self.period_seconds)
        swerve_module_states = self._kinematics.toSwerveModuleStates(speeds)

        self.desire_module_states(swerve_module_states, drive_open_loop, rotate_in_place=False)

    @drive.register
    def _(self, chassis_speeds: ChassisSpeeds, drive_open_loop: bool):
        """
        Alternative method to drive the robot at a set of chassis speeds (exclusively robot-relative).

        By default, chassis speeds are discretized on an interval of 20ms.
        If your robot loop has a non-default period, you **must** set this subsystem's ``period_seconds`` field!

        :param chassis_speeds: Robot-relative speeds on the XY-plane in m/s where +X is forward and +Y is left
        :param drive_open_loop: Use open loop (True) or closed loop (False) velocity control for driving the wheel
        """

        translation = Translation2d(chassis_speeds.vx, chassis_speeds.vy)
        return self.drive(translation, chassis_speeds.omega, False, drive_open_loop)

    def desire_module_states(
        self,
        states: tuple[SwerveModuleState, ...],
        drive_open_loop: bool = False,
        rotate_in_place: bool = True,
    ):
        """
        Command each individual module to a state (consisting of velocity and rotation)

        :param states: List of module states in the order of the swerve module list SwerveDrive was created with
        :param drive_open_loop: Use open loop (True) or closed loop (False) velocity control for driving the wheel
        :param rotate_in_place: Should the modules rotate while not driving
        """

        swerve_module_states = self._kinematics.desaturateWheelSpeeds(states, self.max_velocity)  # type: ignore

        for i in range(len(self._modules)):
            module: SwerveModule = self._modules[i]
            module.desire_state(swerve_module_states[i], drive_open_loop, rotate_in_place)

    @property
    def module_states(self) -> tuple[SwerveModuleState, ...]:
        """A tuple of the swerve modules' states (wheel velocity and facing rotation)"""
        return tuple(module.module_state for module in self._modules)

    @property
    def module_positions(self) -> tuple[SwerveModulePosition, ...]:
        """A tuple of the swerve modules' positions (driven distance and facing rotation)"""
        return tuple(module.module_position for module in self._modules)

    @property
    def pose(self) -> Pose2d:
        """The robot's pose on the field (position and heading)"""
        return self._odometry.getEstimatedPosition()

    @property
    def heading(self) -> Rotation2d:
        """The robot's facing direction"""
        return self._gyro.heading

    @property
    def robot_relative_speeds(self) -> ChassisSpeeds:
        """The robot's translational and rotational speeds"""
        module_states = tuple(module.module_state for module in self._modules)
        return self._kinematics.toChassisSpeeds(module_states)

    def reset_modules(self):
        for module in self._modules:
            module.reset()

        # Any time encoder distances are reset, odometry must also be reset
        self.reset_odometry(self.pose)

    def zero_heading(self):
        """Set the chassis' current heading as "zero" or straight forward"""
        self._gyro.zero_heading()

        # Any time gyro angle is reset, odometry must also be reset
        self._odometry.resetPosition(Rotation2d(), self.module_positions, self.pose)

    def reset_odometry(self, pose: Pose2d):
        """
        Reset the drive base's pose to a new one

        :param pose: The new pose
        """

        self._odometry.resetPosition(self._gyro.heading, self.module_positions, pose)  # type: ignore

    def reset_odometry_to_vision(self):
        """Reset the robot's pose to the vision pose estimation"""
        estimated_pose = self._vision_pose_callback()
        if estimated_pose:
            self.reset_odometry(estimated_pose)

    def _sysid_drive(self, volts: float):
        """
        Drive all wheels at the specified voltage and lock them forward

        :param volts: Voltage in volts (between -12 and +12 for most FRC motors)
        """
        for module in self._modules:
            # Lock modules facing forward so the swerve drive acts as a tank drive
            module.desire_azimuth_angle(Rotation2d(0))

            # Drive motors at desired voltage
            module.set_drive_voltage(volts)

    def _sysid_log(self, log: wpilib.sysid.SysIdRoutineLog, module_names: Optional[tuple[str, ...]] = None):
        """
        Log motor information for SysId

        :param log: The SysId log
        :param module_names: An optional ordered tuple of swerve module names to associate with recorded information
        """
        if module_names:
            # Associate each swerve module with a name
            modules = zip(module_names, self._modules)
        else:
            # Associate each swerve module with a number if no names were provided
            modules = enumerate(self._modules)

        for name, module in modules:  # type: str | int, SwerveModule
            (
                log.motor(str(name))
                .voltage(module.drive_voltage)
                .position(module.drive_distance)
                .velocity(module.drive_velocity)
            )

    def teleop_command(
        self,
        translation: Callable[[], float],
        strafe: Callable[[], float],
        rotation: Callable[[], float],
        field_relative: bool,
        drive_open_loop: bool,
    ) -> "_TeleOpCommand":
        """
        Construct a command that drives the robot using joystick (or other) inputs

        :param translation: A method that returns the desired +X (forward/backward) velocity as a percentage in [-1, 1]
        :param strafe: A method that returns the desired +Y (left/right) velocity as a percentage in [-1, 1]
        :param rotation: A method that returns the desired CCW+ rotational velocity as a percentage in [-1, 1]
        :param field_relative: If True, gyroscopic zero is used as the forward direction.
               Else, forward faces the front of the robot.
        :param drive_open_loop: Use open loop (True) or closed loop (False) velocity control for driving the wheel
        :return: The command
        """
        return _TeleOpCommand(self, translation, strafe, rotation, field_relative, drive_open_loop)

    def follow_trajectory_command(
        self,
        path: PathPlannerPath,
        parameters: "TrajectoryFollowerParameters",
        first_path: bool = False,
        drive_open_loop: bool = False,
        flip_path: Callable[[], bool] = lambda: False,
    ) -> commands2.Command:
        """
        Construct a command that follows a trajectory

        :param path: The path to follow
        :param parameters: Options that determine how the robot will follow the trajectory
        :param first_path: If True, the robot's pose will be reset to the trajectory's initial pose
        :param drive_open_loop: Use open loop control (True) or closed loop (False) to swerve module speeds. Closed-loop
               positional control will always be used for trajectory following
        :param flip_path: Method returning whether to flip the provided path.
               This will maintain a global blue alliance origin.
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
        command = FollowPathCommand(
            path,
            lambda: self.pose,
            lambda: self.robot_relative_speeds,
            lambda speeds: self.drive(speeds, drive_open_loop=drive_open_loop),
            controller,
            ReplanningConfig(),
            flip_path,
            self,
        )

        # If this is the first path in a sequence, reset the robot's pose so that it aligns with the start of the path
        if first_path:
            initial_pose = path.getPreviewStartingHolonomicPose()
            command = command.beforeStarting(commands2.InstantCommand(lambda: self.reset_odometry(initial_pose)))

        return command

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        """
        Run a quasistatic characterization test. The robot will move until this command is cancelled.

        In this test, the mechanism is gradually sped-up such that the voltage corresponding to
        acceleration is negligible (hence, “as if static”).

        :param direction: The direction the robot will drive
        """
        return self._sysid_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        """
        Run a dynamic characterization test. The robot will move until this command is cancelled.

        In this test, a constant ‘step voltage’ is given to the mechanism, so that the
        behavior while accelerating can be determined.

        :param direction: The direction the robot will drive
        """
        return self._sysid_routine.dynamic(direction)


class _TeleOpCommand(commands2.Command):
    def __init__(
        self,
        swerve: SwerveDrive,
        translation: Callable[[], float],
        strafe: Callable[[], float],
        rotation: Callable[[], float],
        field_relative: bool,
        drive_open_loop: bool,
    ):
        super().__init__()
        self.addRequirements(swerve)
        self.setName("TeleOp Command")

        self._swerve = swerve
        self.translation = translation
        self.strafe = strafe
        self.rotation = rotation
        self.field_relative = field_relative
        self.open_loop = drive_open_loop

    def execute(self):
        self._swerve.drive(
            Translation2d(self.translation(), self.strafe()) * self._swerve.max_velocity,
            self.rotation() * self._swerve.max_angular_velocity,
            self.field_relative,
            self.open_loop,
        )

    def initSendable(self, builder: SendableBuilder):
        builder.addBooleanProperty(
            "Field Relative", lambda: self.field_relative, lambda val: setattr(self, "field_relative", val)
        )
        builder.addBooleanProperty(
            "Open Loop", lambda: self.open_loop, lambda val: setattr(self, "drive_open_loop", val)
        )

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
