import math

import phoenix5
import wpilib
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState

from swervepy import u, SwerveDrive, TrajectoryFollowerParameters
from swervepy.impl import (
    PigeonGyro,
    CoaxialSwerveModule,
    Falcon500CoaxialDriveComponent,
    Falcon500CoaxialAzimuthComponent,
    AbsoluteCANCoder,
)


class RobotContainer:
    def __init__(self):
        field_relative = True
        open_loop = True

        track_width = (21.73 * u.inch).m_as(u.m)
        wheel_base = (21.73 * u.inch).m_as(u.m)
        max_speed = 4.5 * (u.m / u.s)
        max_angular_velocity = 11.5 * (u.rad / u.s)

        # Each component defines a Parameters dataclass with any options applicable to all instances of that component.
        # For example, wheel circumference is the same for all four modules on a SDS Mk4 drive base, so that is included
        # in the Parameters class. Motor IDs are different for each instance of a module, so those are not included.
        drive_params = Falcon500CoaxialDriveComponent.Parameters(
            wheel_circumference=4 * math.pi * u.inch,
            gear_ratio=6.75 / 1,  # SDS Mk4i L2
            max_speed=max_speed,
            open_loop_ramp_rate=0,
            closed_loop_ramp_rate=0,
            continuous_current_limit=40,
            peak_current_limit=60,
            peak_current_duration=0.01,
            neutral_mode=phoenix5.NeutralMode.Coast,
            kP=0,
            kI=0,
            kD=0,
            kS=0,
            kV=0,
            kA=0,
            invert_motor=False,
        )
        azimuth_params = Falcon500CoaxialAzimuthComponent.Parameters(
            gear_ratio=150 / 7,  # SDS Mk4i
            max_angular_velocity=max_angular_velocity,
            ramp_rate=0,
            continuous_current_limit=40,
            peak_current_limit=60,
            peak_current_duration=0.01,
            neutral_mode=phoenix5.NeutralMode.Brake,
            kP=0,
            kI=0,
            kD=0,
            invert_motor=False,
        )

        gyro = PigeonGyro(0, True)

        # When defining module positions for kinematics, +x values represent moving toward the front of the robot, and
        # +y values represent moving toward the left of the robot
        modules = (
            # Swerve module implementations are as general as possible (coaxial, differential) but take specific components
            # like Falcon or NEO drive motors as arguments.
            CoaxialSwerveModule(
                # Pass in general Parameters and module-specific options
                Falcon500CoaxialDriveComponent(0, drive_params),
                # The Azimuth component included the CANCOder (absolute encoder) because it needs to be able to
                # reset to absolute position
                Falcon500CoaxialAzimuthComponent(4, Rotation2d.fromDegrees(0), azimuth_params, AbsoluteCANCoder(0)),
                Translation2d(wheel_base / 2, track_width / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveComponent(1, drive_params),
                Falcon500CoaxialAzimuthComponent(5, Rotation2d.fromDegrees(0), azimuth_params, AbsoluteCANCoder(1)),
                Translation2d(wheel_base / 2, -track_width / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveComponent(2, drive_params),
                Falcon500CoaxialAzimuthComponent(6, Rotation2d.fromDegrees(0), azimuth_params, AbsoluteCANCoder(2)),
                Translation2d(-wheel_base / 2, track_width / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveComponent(3, drive_params),
                Falcon500CoaxialAzimuthComponent(7, Rotation2d.fromDegrees(0), azimuth_params, AbsoluteCANCoder(3)),
                Translation2d(-wheel_base / 2, -track_width / 2),
            ),
        )

        self.stick = wpilib.Joystick(0)

        # Define a swerve drive subsystem by passing in a list of SwerveModules and some options
        self.swerve = SwerveDrive(modules, gyro, max_speed, max_angular_velocity)

        self.swerve.setDefaultCommand(
            self.swerve.teleop_command(
                lambda: deadband(-self.stick.getRawAxis(1), 0.05),
                lambda: deadband(-self.stick.getRawAxis(0), 0.05),
                lambda: deadband(-self.stick.getRawAxis(4), 0.1),  # Invert for CCW+
                field_relative,
                open_loop,
            )
        )

    def get_autonomous_command(self):
        # Example path-following autonomous routine
        # Doesn't work yet for 2024 because RobotPy is missing some commands
        follower_params = TrajectoryFollowerParameters(
            max_drive_velocity=4.5 * (u.m / u.s),
            theta_kP=1,
            xy_kP=1,
        )

        bezier_points = PathPlannerPath.bezierFromPoses([
            Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)),
        ])

        path = PathPlannerPath(
            bezier_points,
            PathConstraints(3.0, 3.0, 2 * math.pi, 4 * math.pi),
            GoalEndState(0.0, Rotation2d.fromDegrees(-90)),     # Zero velocity and facing 90 degrees clockwise
        )

        return self.swerve.follow_trajectory_command(path, follower_params, True, True)


def deadband(value, band):
    return value if abs(value) > band else 0
