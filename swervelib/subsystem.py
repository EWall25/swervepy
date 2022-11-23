import math
from typing import Callable

import commands2
import ctre
import pathplannerlib as pp
import wpimath.controller
from astropy import units as u
from astropy.units import Quantity
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Odometry, ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState
from wpimath.trajectory import Trajectory

from .configs import SwerveModuleParameters, SwerveParameters, AutoParameters
from .mod import SwerveModule


class Swerve(commands2.SubsystemBase):
    __slots__ = "odometry", "swerve_modules", "gyro", "swerve_params"

    def __init__(
        self,
        module_params: tuple[
            SwerveModuleParameters, SwerveModuleParameters, SwerveModuleParameters, SwerveModuleParameters
        ],
        swerve_params: SwerveParameters,
    ):
        commands2.SubsystemBase.__init__(self)

        self.gyro = ctre.WPI_PigeonIMU(*swerve_params.gyro_id)
        self.gyro.configFactoryDefault()
        self.zero_heading()

        self.odometry = SwerveDrive4Odometry(swerve_params.kinematics, self.heading)

        # Create four swerve modules and pass each a unique set of parameters
        self.swerve_modules = tuple(SwerveModule(module_params[i], swerve_params) for i in range(4))

        self.swerve_params = swerve_params

    def periodic(self):
        # Unpack a tuple of swerve module states into four arguments using the * symbol
        self.odometry.update(self.heading, *self.module_states)

    def drive(self, translation: Translation2d, rotation: Quantity[u.rad / u.s], field_relative: bool, open_loop: bool):
        speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.x, translation.y, rotation.to_value(u.rad / u.s), self.heading
            )
            if field_relative
            else ChassisSpeeds(translation.x, translation.y, rotation.to_value(u.rad / u.s))
        )
        swerve_module_states = self.swerve_params.kinematics.toSwerveModuleStates(speeds)
        swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states, self.swerve_params.max_speed.to_value(u.m / u.s)
        )

        for i in range(4):
            mod: SwerveModule = self.swerve_modules[i]
            mod.desire_state(swerve_module_states[i], open_loop)

    def set_module_states(
        self, desired_states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]
    ):
        desired_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states, self.swerve_params.max_speed)

        for i in range(4):
            mod: SwerveModule = self.swerve_modules[i]
            mod.desire_state(desired_states[i], False)

    def zero_heading(self):
        self.gyro.setYaw(0)

    def reset_odometry(self, pose: Pose2d):
        self.odometry.resetPosition(pose, self.heading)

    @property
    def pose(self) -> Pose2d:
        return self.odometry.getPose()

    @property
    def module_states(self) -> tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        # noinspection PyTypeChecker
        return tuple(mod.state for mod in self.swerve_modules)

    @property
    def heading(self) -> Rotation2d:
        yaw = self.gyro.getYaw()

        # Because the encoder is absolute and always reports its actual rotation,
        # subtract the yaw from 360 degrees to invert.
        if self.swerve_params.invert_gyro:
            yaw = 360 - yaw

        return Rotation2d.fromDegrees(yaw)

    def teleop_command(
        self,
        translation: Callable[[], float],
        strafe: Callable[[], float],
        rotation: Callable[[], float],
        field_relative: bool,
        open_loop: bool,
    ):
        # TODO: Add deadbanding
        # TODO: Add 45deg stopping feature
        return commands2.RunCommand(
            lambda: self.drive(
                Translation2d(translation(), strafe()) * self.swerve_params.max_speed.to_value(u.m / u.s),
                rotation() * self.swerve_params.max_angular_velocity,
                field_relative,
                open_loop,
            ),
            self,
        )

    def follow_trajectory_command(
        self, trajectory: pp.PathPlannerTrajectory | Trajectory | str, first_path: bool, params: AutoParameters
    ):
        # If a string is passed in, load a PathPlanner path from storage
        if isinstance(trajectory, str):
            trajectory = pp.PathPlanner.loadPath(trajectory, params.max_speed, params.max_acceleration)

        # Change from the PathPlanner trajectory format to the WPILib format for the built-in controller
        if isinstance(trajectory, pp.PathPlannerTrajectory):
            trajectory = trajectory.asWPILibTrajectory()

        theta_controller = wpimath.controller.ProfiledPIDControllerRadians(1, 0, 0, params.theta_controller_constraints)
        theta_controller.enableContinuousInput(-math.pi, math.pi)

        command = commands2.Swerve4ControllerCommand(
            trajectory,
            lambda: self.pose,
            self.swerve_params.kinematics,
            wpimath.controller.PIDController(1, 0, 0),
            wpimath.controller.PIDController(1, 0, 0),
            theta_controller,
            self.set_module_states,
            self,
        )

        # If this is the first path in a sequence, reset the robot's pose so that it aligns with the start of the path
        if first_path:
            initial_pose = trajectory.getInitialState().pose
            command = command.beforeStarting(lambda: self.reset_odometry(initial_pose))

        return command

    def ski_stop_command(self):
        # Turn all swerve modules 45 degrees so the robot is harder to stop
        # fmt: off
        angles = (
            45, 315,  # Front Left, Front Right
            315, 45,  # Back Left, Back Right
        )
        # fmt: on

        # Can't assume the 0th index of the swerve_modules tuple will be the front-left module,
        # so access the appropriate angle by each module's position on the robot (e.g. front-left)
        # The RelativeModulePosition enum is ordered in the same way as the angles tuple
        states = tuple(
            SwerveModuleState(0, Rotation2d.fromDegrees(angles[mod.relative_position])) for mod in self.swerve_modules
        )

        # noinspection PyTypeChecker
        return commands2.InstantCommand(lambda: self.set_module_states(states))
