import math

import ctre
import wpilib
import wpimath.trajectory
from wpimath.geometry import Translation2d, Rotation2d, Transform3d, Pose2d
import robotpy_apriltag as apriltag

from swervepy import u, vision, SwerveDrive, TrajectoryFollowerParameters
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
            neutral_mode=ctre.NeutralMode.Coast,
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
            neutral_mode=ctre.NeutralMode.Brake,
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

        # camera_1 = vision.CameraDefinition("camera_1", Transform3d())
        # camera_collection = vision.AprilTagCameraCollection(
        #     [camera_1], apriltag.loadAprilTagLayoutField(apriltag.AprilTagField.k2023ChargedUp)
        # )

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
        follower_params = TrajectoryFollowerParameters(
            target_angular_velocity=math.pi * (u.rad / u.s),
            target_angular_acceleration=math.pi * (u.rad / (u.s * u.s)),
            theta_kP=1,
            x_kP=1,
            y_kP=1,
        )

        trajectory_config = wpimath.trajectory.TrajectoryConfig(maxVelocity=4.5, maxAcceleration=1)

        trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
            [
                Pose2d(0, 0, 0),  # Start at (0, 0)
                Pose2d(1, 0, 0),  # Move 1m forward
            ],
            trajectory_config,
        )

        return self.swerve.follow_trajectory_command(trajectory, follower_params, True)


def deadband(value, band):
    return value if abs(value) > band else 0
