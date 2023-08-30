import typing

import commands2
import ctre
import wpimath.kinematics
import wpimath.estimator
from pint import Quantity
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.kinematics import SwerveModulePosition, ChassisSpeeds

from . import u
from .abstracts import SwerveModule, CoaxialDriveMotor, CoaxialAzimuthMotor, Gyro

if typing.TYPE_CHECKING:
    from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry

class CoaxialSwerveModule(SwerveModule):
    def __init__(self, drive_motor: CoaxialDriveMotor, azimuth_motor: CoaxialAzimuthMotor):
        self._drive_motor = drive_motor
        self._azimuth_motor = azimuth_motor

    def desire_drive_velocity(self, velocity: float, open_loop: bool):
        if open_loop:
            self._drive_motor.follow_velocity_open(velocity)
        else:
            self._drive_motor.follow_velocity_closed(velocity)

    def desire_azimuth_angle(self, angle: Rotation2d):
        self._azimuth_motor.follow_angle(angle)

    @property
    def drive_velocity(self) -> float:
        return self._drive_motor.velocity

    @property
    def drive_distance(self) -> float:
        return self._drive_motor.position

    @property
    def azimuth_angle(self) -> Rotation2d:
        return self._azimuth_motor.angle

    @property
    def azimuth_velocity(self) -> Rotation2d:
        return self._azimuth_motor.rotational_velocity


class SwerveDrive(commands2.SubsystemBase):
    def __init__(self, modules: tuple[SwerveModule, ...], gyro: Gyro, max_velocity: Quantity, max_angular_velocity: Rotation2d):
        super().__init__()

        self._modules = modules
        self._gyro = gyro
        self.max_velocity: float = max_velocity.m_as(u.m / u.s)
        self.max_angular_velocity = max_angular_velocity

        # There are different classes for each number of swerve modules in a drive base,
        # so construct the class name from number of modules.
        self._kinematics: "SwerveDrive4Kinematics" = getattr(wpimath.kinematics, f"SwerveDrive{len(modules)}Kinematics")(
            *[module.placement for module in self._modules]
        )

        # TODO: Reset modules before constructing odometry
        self._odometry: "SwerveDrive4Odometry" = getattr(wpimath.estimator, f"SwerveDrive{len(modules)}PoseEstimator")(
            self.kinematics, self._gyro.heading, self.module_positions, Pose2d()
        )

    def drive(self, translation: Translation2d, rotation: Rotation2d, field_relative: bool, open_loop: bool):
        speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, self.heading)
            if field_relative
            else ChassisSpeeds(translation.x, translation.y, rotation)
        )
        swerve_module_states = self._kinematics.toSwerveModuleStates(speeds)
        swerve_module_states = self._kinematics.desaturateWheelSpeeds(swerve_module_states, self.max_velocity)

        for i in range(4):
            module: SwerveModule = self._modules[i]
            module.desire_state(swerve_module_states[i], open_loop)

    @property
    def module_positions(self) -> tuple[SwerveModulePosition, ...]:
        return tuple(module.module_position for module in self._modules)

    # TODO: Add utility methods
    # TODO: Impl dashboard
    # TODO: Add commands

class Falcon500CoaxialDriveMotor(CoaxialDriveMotor):
    def __init__(self, id_: int):
        # TODO: Accept CAN IDs on other busses
        self._motor = ctre.TalonFX(id_)
        # TODO: Configure motor

    def follow_velocity_open(self, velocity: float):
        pass

    def follow_velocity_closed(self, velocity: float):
        pass

    def reset(self):
        pass

    @property
    def velocity(self) -> float:
        pass

    @property
    def position(self) -> float:
        pass


class Falcon500CoaxialAzimuthMotor(CoaxialAzimuthMotor):
    def __init__(self, id_: int):
        # TODO: Accept CAN IDs on other busses
        self._motor = ctre.TalonFX(id_)
        # TODO: Configure motor

    def follow_angle(self, angle: Rotation2d):
        pass

    def reset(self):
        pass

    @property
    def rotational_velocity(self) -> Rotation2d:
        pass

    @property
    def angle(self) -> Rotation2d:
        pass