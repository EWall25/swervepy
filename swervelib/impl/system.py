import time
from typing import Callable, Optional, TYPE_CHECKING

import commands2
import wpilib
import wpimath.kinematics
import wpimath.estimator
from pint import Quantity
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveModulePosition
from wpiutil import SendableBuilder

if TYPE_CHECKING:
    from wpimath.kinematics import SwerveDrive4Kinematics
    from wpimath.estimator import SwerveDrive4PoseEstimator

from .. import u
from ..abstract.motor import CoaxialDriveComponent, CoaxialAzimuthComponent
from ..abstract.sensor import Gyro
from ..abstract.system import SwerveModule


class SwerveDrive(commands2.SubsystemBase):
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

    def periodic(self):
        self._odometry.update(self._gyro.heading, self.module_positions)

        vision_pose = self._vision_pose_callback(self.pose)
        if vision_pose:
            self._odometry.addVisionMeasurement(vision_pose, wpilib.Timer.getFPGATimestamp())

    def drive(self, translation: Translation2d, rotation: float, field_relative: bool, open_loop: bool):
        speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, self._gyro.heading)
            if field_relative
            else ChassisSpeeds(translation.x, translation.y, rotation)
        )
        swerve_module_states = self._kinematics.toSwerveModuleStates(speeds)

        self.desire_module_states(swerve_module_states, open_loop, rotate_in_place=False)

    def desire_module_states(
        self, states: tuple[SwerveModuleState, ...], open_loop: bool = False, rotate_in_place: bool = True
    ):
        swerve_module_states = self._kinematics.desaturateWheelSpeeds(states, self.max_velocity)  # type: ignore

        for i in range(4):
            module: SwerveModule = self._modules[i]
            module.desire_state(swerve_module_states[i], open_loop, rotate_in_place)

    @property
    def module_positions(self) -> tuple[SwerveModulePosition, ...]:
        return tuple(module.module_position for module in self._modules)

    @property
    def pose(self) -> Pose2d:
        return self._odometry.getEstimatedPosition()

    # TODO: Add commands

    def reset_modules(self):
        for module in self._modules:
            module.reset()

    def zero_heading(self):
        self._gyro.zero_heading()

    def reset_odometry(self, pose: Pose2d):
        self._odometry.resetPosition(self._gyro.heading, self.module_positions, pose)  # type: ignore

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
                Translation2d(translation(), strafe()) * self.max_velocity,
                rotation() * self.max_angular_velocity,
                field_relative,
                open_loop,
            ),
            self,
        )


class CoaxialSwerveModule(SwerveModule):
    last_commanded_drive_velocity: float = 0
    last_commanded_azimuth_angle = Rotation2d.fromDegrees(0)

    def __init__(self, drive: CoaxialDriveComponent, azimuth: CoaxialAzimuthComponent, placement: Translation2d):
        super().__init__()

        self._drive = drive
        self._azimuth = azimuth
        self.placement = placement

    def desire_drive_velocity(self, velocity: float, open_loop: bool):
        self.last_commanded_drive_velocity = velocity
        if open_loop:
            self._drive.follow_velocity_open(velocity)
        else:
            self._drive.follow_velocity_closed(velocity)

    def desire_azimuth_angle(self, angle: Rotation2d):
        self.last_commanded_azimuth_angle = angle
        self._azimuth.follow_angle(angle)

    def reset(self):
        self._drive.reset()
        self._azimuth.reset()

    @property
    def drive_velocity(self) -> float:
        return self._drive.velocity

    @property
    def drive_distance(self) -> float:
        return self._drive.distance

    @property
    def azimuth_angle(self) -> Rotation2d:
        return self._azimuth.angle

    @property
    def azimuth_velocity(self) -> float:
        return self._azimuth.rotational_velocity

    def initSendable(self, builder: SendableBuilder):
        # fmt: off
        builder.setSmartDashboardType("CoaxialSwerveModule")
        builder.addDoubleProperty("Drive Velocity (mps)", lambda: self._drive.velocity, lambda _: None)
        builder.addDoubleProperty("Drive Distance (m)", lambda: self._drive.distance, lambda _: None)
        builder.addDoubleProperty("Azimuth Velocity (radps)", lambda: self._azimuth.rotational_velocity, lambda _: None)
        builder.addDoubleProperty("Azimuth Position (rad)", lambda: self._azimuth.angle.radians(), lambda _: None)
        builder.addDoubleProperty("Azimuth Position (deg)", lambda: self._azimuth.angle.degrees(), lambda _: None)
        builder.addDoubleProperty("Desired Drive Velocity (mps)", lambda: self.last_commanded_drive_velocity, lambda _: None)
        builder.addDoubleProperty("Desired Azimuth Position (rad)", lambda: self.last_commanded_azimuth_angle.radians(), lambda _: None)
        builder.addDoubleProperty("Desired Azimuth Position (deg)", lambda: self.last_commanded_azimuth_angle.degrees(), lambda _: None)
        # fmt: on
