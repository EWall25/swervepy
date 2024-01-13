"""
This file defines constants related to your robot.  These constants include:

 * Physical constants (exterior dimensions, wheelbase)

 * Mechanical constants (gear reduction ratios)

 * Electrical constants (current limits, CAN bus IDs, roboRIO slot numbers)

 * Operation constants (desired max velocity, max turning speed)

 * Software constants (USB ID for driver joystick)
"""

import math
from collections import namedtuple
import rev
import phoenix5
from swervepy import u

# Physical constants
phys_data = {
    "track_width": (21.73 * u.inch).m_as(u.m),
    "wheel_base": (21.73 * u.inch).m_as(u.m),
    "wheel_circumference": 4 * math.pi * u.inch,
}
PHYS = namedtuple("Data", phys_data.keys())(**phys_data)

# Mechanical constants
mech_data = {
    "swerve_module_propulsion_gearing_ratio": 6.75,  # SDS Mk4i L2
    "swerve_module_steering_gearing_ratio": 150 / 7,  # SDS Mk4i

    "propulsion_motor_inverted": False,
    "steering_motor_inverted": False,
}
MECH = namedtuple("Data", mech_data.keys())(**mech_data)

# Electrical constants
elec_data = {
    # These current limit parameters are per-motor in the swerve modules
    "drive_continuous_current_limit": 40,
    "azimuth_continuous_current_limit": 30,
    "drive_peak_current_limit": 60,
    "azimuth_peak_current_limit": 40,

    # Talon FX motor controllers can set peak_current_duration.
    # SparkMAX motor controllers can't.
    "drive_peak_current_duration": 0.01,
    "azimuth_peak_current_duration": 0.01,

    # time in seconds for propulsion motors to ramp up to full speed
    # reference: https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax
    "open_loop_ramp_rate": 0.5,
    "closed_loop_ramp_rate": 0.5,

    "RF_steer_CAN_ID": 11,
    "RF_drive_CAN_ID": 12,
    "RF_encoder_DIO": 0,
    "RB_steer_CAN_ID": 13,
    "RB_drive_CAN_ID": 14,
    "RB_encoder_DIO": 1,
    "LB_steer_CAN_ID": 15,
    "LB_drive_CAN_ID": 16,
    "LB_encoder_DIO": 2,
    "LF_steer_CAN_ID": 17,
    "LF_drive_CAN_ID": 18,
    "LF_encoder_DIO": 3,
}
ELEC = namedtuple("Data", elec_data.keys())(**elec_data)

JOYSTICK_AXES = {
    "LEFT_X": 0,
    "LEFT_Y": 1,
    "RIGHT_X": 4,
    "RIGHT_Y": 5,
}

# Operation constants
op_data = {
    # These maximum parameters reflect the maximum physically possible, not the
    # desired maximum limit.
    "max_speed": 4.5 * (u.m / u.s),
    "max_angular_velocity": 11.5 * (u.rad / u.s),

    # You can limit how fast your robot moves (e.g. during testing) using the
    # following parameters.  Setting to None is the same as setting to
    # max_speed/max_angular_velocity, and indicates no limit.
    #
    "speed_limit": 1.0 * (u.m / u.s),
    "angular_velocity_limit": 2.5 * (u.rad / u.s),

    # For NEO / SparkMAX, use the following and comment out the Falcon500 values
    # "propulsion_neutral": rev.CANSparkMax.IdleMode.kCoast,
    # "steering_neutral": rev.CANSparkMax.IdleMode.kBrake,
    # For Falcon500 / TalonFX, use the following and comment out the NEO values
    "propulsion_neutral": phoenix5.NeutralMode.Coast,
    "steering_neutral": phoenix5.NeutralMode.Brake,

    # Values to pass to stick.getRawAxis()
    # Set these according to your operator preferences
    "translation_joystick_axis": JOYSTICK_AXES["RIGHT_Y"],
    "strafe_joystick_axis": JOYSTICK_AXES["RIGHT_X"],
    "rotation_joystick_axis": JOYSTICK_AXES["LEFT_X"],
}
OP = namedtuple("Data", op_data.keys())(**op_data)

# Software constants
sw_data = {
    # field_relative: True if "forward" means "down the field"; False if
    # "forward" means "in the direction the robot is facing".  A True value
    # requires a (non-Dummy) gyro.
    "field_relative": False,

    # drive_open_loop: True if we're not using PID control *for velocity targeting*,
    # i.e. when a target velocity is calculated, do we use the corresponding
    # CoaxialDriveComponent's follow_velocity_open() method (set motor output
    # proportionally based on target and max velocities) or
    # follow_velocity_closed() method (put motor in PID control mode and set
    # target velocity).
    #
    "drive_open_loop": True,

    # Constants for PID control of the propulsion AND steering motors
    # (kP must be non-zero, or azimuth motors won't engage.)
    "kP": 0.3,  # representative value for Falcon500 motors
    # "kP": 0.01,   # representative value for NEO motors
    "kI": 0,
    "kD": 0,

    # Constants for feed-forward of propulsion motors
    "kS": 0,
    "kV": 0,
    "kA": 0,
}
SW = namedtuple("Data", sw_data.keys())(**sw_data)
