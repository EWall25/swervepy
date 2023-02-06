import enum
import math
import time
from typing import Callable, Optional

import commands2
import ctre
import wpilib

import wpimath.controller
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModuleState,
    SwerveModulePosition,
)
from wpimath.trajectory import Trajectory

from .configs import (
    SwerveModuleParameters,
    SwerveParameters,
    AutoParameters,
    CTRESwerveModuleParameters,
    CTRESwerveParameters,
    REVSwerveParameters,
    REVSwerveModuleParameters,
    VisionParameters,
)
from .dummy import Dummy
from .mod import SwerveModule, CTRESwerveModule, REVSwerveModule
from .vision import AprilTagCameraCollection

SwerveModuleParameters4 = tuple[
    SwerveModuleParameters, SwerveModuleParameters, SwerveModuleParameters, SwerveModuleParameters
]
SwerveModuleState4 = tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]
SwerveModulePosition4 = tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]


class DrivetrainVendor(enum.Enum):
    CTRE = enum.auto()
    REV = enum.auto()


class Swerve(commands2.SubsystemBase):
    """
    This is a swerve subsystem that can be integrated into any Command-based robot code.
    It is designed for a 4-wheel swerve drive with Falcon 500 motors, CTRE CANCoders, and a CTRE Pigeon IMU.

    All methods and commands that take driver input use the WPILib cartesian coordinate system, where:
        * The positive x direction is the same direction that either the robot or the driver is facing
        * The positive y direction is perpendicular to the x-axis, facing to the left of the robot or the driver
        * Positive theta (rotation around the z-axis) is counter-clockwise, otherwise known as CCW+
    This coordinate system is detailed here:
    https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system.
    """

    __slots__ = "odometry", "swerve_modules", "gyro", "swerve_params", "kinematics", "vision_estimator", "field"

    def __init__(
        self,
        module_params: SwerveModuleParameters4,
        swerve_params: SwerveParameters,
        vision_params: Optional[VisionParameters] = None,
    ):
        """
        Constructor for a Swerve subsystem.

        :param module_params: A tuple of module-specific parameters that does not need to be ordered in any particular way
        :param swerve_params: General parameters describing the drivetrain's hardware
        :param vision_params: NetworkTables and position data for each camera. If equal to None, no vision is used
        """

        commands2.SubsystemBase.__init__(self)

        # Determine whether this is a CTRE or REV drive base
        if isinstance(module_params[0], CTRESwerveModuleParameters) and isinstance(swerve_params, CTRESwerveParameters):
            vendor = DrivetrainVendor.CTRE
        elif isinstance(module_params[0], REVSwerveModuleParameters) and isinstance(swerve_params, REVSwerveParameters):
            vendor = DrivetrainVendor.REV
        else:
            raise Exception("Swerve module parameters and swerve parameters are mismatched! Use either CTRE or REV.")

        # Convert the parameters to decimal values because doing unit conversions every iteration takes too long
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
        # The RelativeModulePosition enum maps 0 to front-left and 3 to back-right, so the items are
        # sorted in ascending order according to the enum value.
        # We do this to easily identify which module is in which position for i.e. the ski stop command.
        module_params = tuple(sorted(module_params, key=lambda param: param.corner))

        # Create four swerve modules and pass each a unique set of parameters. This tuple has the same swerve module
        # order as the module_params tuple above it.
        swerve_module_type = CTRESwerveModule if vendor is DrivetrainVendor.CTRE else REVSwerveModule
        # noinspection PyTypeChecker
        self.swerve_modules = tuple(swerve_module_type(module_param, swerve_params) for module_param in module_params)

        # Pause for a second to allow the motors time to configure themselves before resetting their positions.
        # See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
        # NOTE: This locks up the main thread and makes robot startup slower.
        time.sleep(1)
        self.reset_modules_to_absolute()

        # Kinematics are used to calculate individual wheel speeds from a desired robot velocity and direction.
        # This is required for controlling the robot with one flight stick, instead of a separate stick per wheel.
        self.kinematics = SwerveDrive4Kinematics(*[module_param.relative_position for module_param in module_params])

        # Set up an object that will estimate the robot's pose based on vision. This estimation will later be fused
        # with encoder and gyro data for a more accurate pose.
        if vision_params:
            self.vision_estimator = AprilTagCameraCollection(
                vision_params.camera_definitions, vision_params.field_layout
            )

        # Odometry tracks the robot's position using its wheel encoders and gyro, so that the robot
        # can follow paths in autonomous.
        self.zero_module_distances()
        self.odometry = SwerveDrive4PoseEstimator(self.kinematics, self.heading, self.module_positions, Pose2d(0, 0, 0))

        # A field element to visualize the robot's estimated pose on
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

    def periodic(self):
        # Odometry must be updated every iteration with the robot's heading and its wheels' driven distance and angle.
        # These parameters are used to calculate the robot's overall position.
        self.odometry.update(self.heading, self.module_positions)

        # Enhance the previous estimation by fusing it with vision data
        if hasattr(self, "vision_estimator"):
            self._update_odometry_from_vision()

        self._update_dashboard()

    def _update_odometry_from_vision(self):
        timestamp = wpilib.Timer.getFPGATimestamp()
        vision_pose = self.vision_estimator.estimate_pose(self.pose)
        self.odometry.addVisionMeasurement(vision_pose, timestamp)
        self.field.setRobotPose(vision_pose)

    def drive(self, translation: Translation2d, rotation: float, field_relative: bool, open_loop: bool):
        """
        Drive the robot.

        :param translation: The desired movement of the robot in metres
        :param rotation: The rotation of the robot in rads/sec, where CCW+
        :param field_relative: Is "forward" facing the front of the robot or the front of the driver?
        :param open_loop: If False, use velocity control. Else, use percent output
        """

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
        """
        Command each module to a specified state using closed-loop control.

        :param desired_states: The desired states of each module. This list must be in front-left, front-right,
        back-left, back-right order
        """

        desired_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states, self.swerve_params.max_speed)

        for i in range(4):
            mod: SwerveModule = self.swerve_modules[i]
            mod.desire_state(desired_states[i], False)

    def zero_heading(self):
        self.gyro.setYaw(0)

    def reset_odometry(self, pose: Pose2d):
        self.odometry.resetPosition(self.heading, self.module_positions, pose)

    def reset_modules_to_absolute(self):
        """Reset the azimuth motors' position readings to their absolute encoder's."""
        for mod in self.swerve_modules:
            mod.reset_to_absolute()

    def zero_module_distances(self):
        for mod in self.swerve_modules:
            mod.zero_distance()

    def _update_dashboard(self):
        wpilib.SmartDashboard.putNumber("Heading (deg)", self.heading.degrees())
        for mod in self.swerve_modules:
            wpilib.SmartDashboard.putNumber(
                f"{mod.corner.name} CANCoder Absolute Rotation (deg)",
                mod.absolute_encoder_rotation.degrees(),
            )
            wpilib.SmartDashboard.putNumber(f"{mod.corner.name} Drive Speed (mps)", mod.velocity)
            wpilib.SmartDashboard.putNumber(f"{mod.corner.name} Azimuth Angle (deg)", mod.angle.degrees())

    @property
    def pose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()

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
                Translation2d(translation(), strafe()) * self.swerve_params.max_speed,
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

        # TODO: Check if this command works with the current set_module_states method because set_module_states takes
        # parameters in FL, FR, BL, BR order
        command = commands2.Swerve4ControllerCommand(
            trajectory,
            lambda: self.pose,
            self.kinematics,
            wpimath.controller.PIDController(1, 0, 0),
            wpimath.controller.PIDController(1, 0, 0),
            theta_controller,
            self.set_module_states,
            [self],
        )

        # If this is the first path in a sequence, reset the robot's pose so that it aligns with the start of the path
        if first_path:
            initial_pose = trajectory.initialPose()
            command = command.beforeStarting(lambda: self.reset_odometry(initial_pose))

        return command

    def ski_stop_command(self):
        """Turn the wheels into an 'X' shape to make the robot more difficult to push."""

        # TODO: These angles currently make an "O" instead of an "X"
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
