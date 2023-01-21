import math
import time
from typing import Callable

import commands2
import ctre
import wpilib

import wpimath.controller
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.kinematics import (
    SwerveDrive4Odometry,
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModuleState,
    SwerveModulePosition,
)
from wpimath.trajectory import Trajectory

from .configs import SwerveModuleParameters, SwerveParameters, AutoParameters
from .dummy import Dummy
from .mod import SwerveModule

SwerveModuleParameters4 = tuple[
    SwerveModuleParameters, SwerveModuleParameters, SwerveModuleParameters, SwerveModuleParameters
]
SwerveModuleState4 = tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]
SwerveModulePosition4 = tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]


class Swerve(commands2.SubsystemBase):
    __slots__ = "odometry", "swerve_modules", "gyro", "swerve_params", "kinematics"

    def __init__(self, module_params: SwerveModuleParameters4, swerve_params: SwerveParameters):
        commands2.SubsystemBase.__init__(self)

        # noinspection PyTypeChecker
        self.swerve_params: SwerveParameters = swerve_params.in_standard_units()

        # If a flag is set, create a NoOp object in place of the gyro for testing without hardware
        if swerve_params.fake_gyro:
            self.gyro = Dummy()
        else:
            self.gyro = ctre.WPI_PigeonIMU(swerve_params.gyro_id)
        self.gyro.configFactoryDefault()
        self.zero_heading()

        # Sort the module parameters list into front-left, front-right, back-left, back-right order
        # The RelativeModulePosition enum has maps 0 to front-left and 3 to back-right, so the items are
        # sorted in ascending order according to the enum value.
        module_params = sorted(module_params, key=lambda param: param.corner)

        # Create four swerve modules and pass each a unique set of parameters
        self.swerve_modules = tuple(SwerveModule(module_param, swerve_params) for module_param in module_params)

        # Pause for a second to allow the motors time to configure themselves before resetting their positions.
        # See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
        # NOTE: This locks up the main thread and makes robot startup slower.
        time.sleep(1)
        self.reset_modules_to_absolute()

        # Create kinematics in the same order as the swerve modules tuple
        self.kinematics = SwerveDrive4Kinematics(*[module_param.relative_position for module_param in module_params])

        # Reset the driven distance of each module before constructing odometry
        self.zero_module_distances()
        self.odometry = SwerveDrive4Odometry(self.kinematics, self.heading, self.module_positions)

    def periodic(self):
        # Unpack a tuple of swerve module positions into four arguments using the * symbol
        self.odometry.update(self.heading, *self.module_positions)

        self._update_dashboard()

    def drive(self, translation: Translation2d, rotation: float, field_relative: bool, open_loop: bool):
        speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, self.heading)
            if field_relative
            else ChassisSpeeds(translation.x, translation.y, rotation)
        )
        swerve_module_states = self.kinematics.toSwerveModuleStates(speeds)
        swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states, self.swerve_params.max_speed
        )

        for i in range(4):
            mod: SwerveModule = self.swerve_modules[i]
            mod.desire_state(swerve_module_states[i], open_loop)

    def set_module_states(self, desired_states: SwerveModuleState4):
        desired_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states, self.swerve_params.max_speed)

        for i in range(4):
            mod: SwerveModule = self.swerve_modules[i]
            mod.desire_state(desired_states[i], False)

    def zero_heading(self):
        self.gyro.setYaw(0)

    def reset_odometry(self, pose: Pose2d):
        self.odometry.resetPosition(pose, self.heading, *self.module_positions)

    def reset_modules_to_absolute(self):
        """Reset the azimuth motors' position readings to their absolute encoder's."""
        for mod in self.swerve_modules:
            mod.reset_to_absolute()

    def zero_module_distances(self):
        for mod in self.swerve_modules:
            mod.zero_distance()

    def _update_dashboard(self):
        # TODO: Fix loop overruns caused by this method
        wpilib.SmartDashboard.putNumber("Heading (deg)", self.heading.degrees())
        for mod in self.swerve_modules:
            wpilib.SmartDashboard.putNumber(
                f"{mod.corner.name} CANCoder Absolute Rotation (deg)",
                mod.absolute_encoder_rotation.degrees(),
            )
            wpilib.SmartDashboard.putNumber(f"{mod.corner.name} Drive Speed (mps)", mod.velocity)
            # wpilib.SmartDashboard.putNumber(f"{mod.corner.name} Drive Speed (falcon)", mod.drive_motor.getSelectedSensorVelocity())
            wpilib.SmartDashboard.putNumber(f"{mod.corner.name} Azimuth Angle (deg)", mod.angle.degrees())

    @property
    def pose(self) -> Pose2d:
        return self.odometry.getPose()

    @property
    def module_states(self) -> SwerveModuleState4:
        # noinspection PyTypeChecker
        return tuple(mod.state for mod in self.swerve_modules)

    @property
    def module_positions(self) -> SwerveModulePosition4:
        # noinspection PyTypeChecker
        return tuple(mod.position for mod in self.swerve_modules)

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
        return commands2.RunCommand(
            lambda: self.drive(
                # https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
                Translation2d(translation(), -strafe()) * self.swerve_params.max_speed,
                rotation() * self.swerve_params.max_angular_velocity,
                field_relative,
                open_loop,
            ),
            self,
        )

    def teleop_with_ski_stop_command(
        self,
        translation: Callable[[], float],
        strafe: Callable[[], float],
        rotation: Callable[[], float],
        field_relative: bool,
        open_loop: bool,
    ):
        def received_input():
            return translation() != 0 or strafe() != 0 or rotation() != 0

        return commands2.SequentialCommandGroup(
            self.teleop_command(translation, strafe, rotation, field_relative, open_loop)
            # When the driver stops moving joysticks, perform a ski stop (45 degree wheels)
            .until(lambda: not received_input()),
            commands2.SequentialCommandGroup(
                # Wait for a moment so that the wheels don't change angles if the driver stops and starts quickly
                commands2.WaitCommand(0.3),
                # If the command isn't run perpetually, the driver will regain control before he moves a joystick
                # and ski stop will essentially never run
                self.ski_stop_command().perpetually(),
            )
            # Give back control to the driver whenever he moves a joystick
            .until(received_input),
        )

    def follow_trajectory_command(self, trajectory: Trajectory, first_path: bool, params: AutoParameters):
        theta_controller = wpimath.controller.ProfiledPIDControllerRadians(1, 0, 0, params.theta_controller_constraints)
        theta_controller.enableContinuousInput(-math.pi, math.pi)

        command = commands2.Swerve4ControllerCommand(
            trajectory,
            lambda: self.pose,
            self.kinematics,
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

        # The swerve_modules tuple is already ordered in front-left, front-right, back-left, back-right order
        # Create a state with the proper angle for each module's position
        states = tuple(SwerveModuleState(0, Rotation2d.fromDegrees(angle)) for angle in angles)

        # noinspection PyTypeChecker
        return commands2.InstantCommand(lambda: self.set_module_states(states), self)
