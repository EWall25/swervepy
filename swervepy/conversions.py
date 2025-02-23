"""
A collection of methods for converting between native Falcon 500 units and standard units, like metres.
"""
import math
from typing_extensions import deprecated

import wpimath.geometry

FALCON_CPR = 2048
DEGREES_PER_ROTATION = 360
RADS_PER_ROTATION = 2 * math.pi


def metres_to_rotations(distance: float, circumference: float) -> float:
    return distance / circumference


def rotations_to_metres(rotations: float, circumference: float) -> float:
    return rotations * circumference


@deprecated("Unnecessary for Phoenix 6")
def falcon_to_degrees(counts: float, gear_ratio: float) -> wpimath.geometry.Rotation2d:
    degrees = counts * (DEGREES_PER_ROTATION / (gear_ratio * FALCON_CPR))
    return wpimath.geometry.Rotation2d.fromDegrees(degrees)


@deprecated("Unnecessary for Phoenix 6")
def degrees_to_falcon(degrees: wpimath.geometry.Rotation2d, gear_ratio: float) -> float:
    ticks = degrees.degrees() / (DEGREES_PER_ROTATION / (gear_ratio * FALCON_CPR))
    return ticks


@deprecated("Unnecessary for Phoenix 6")
def falcon_to_rpm(velocity: float, gear_ratio: float) -> float:
    # Falcon velocity is in units/100ms, so multiplying by 600 changes to units/min.
    motor_rpm = velocity * (600 / FALCON_CPR)
    mechanism_rpm = motor_rpm / gear_ratio
    return mechanism_rpm


@deprecated("Unnecessary for Phoenix 6")
def rpm_to_falcon(rpm: float, gear_ratio: float) -> float:
    motor_rpm = rpm * gear_ratio
    # Falcon velocity is in units/100ms, so multiplying by 600 changes to units/min.
    ticks = motor_rpm * (FALCON_CPR / 600)
    return ticks


@deprecated("Unnecessary for Phoenix 6")
def falcon_to_mps(velocity: float, circumference: float, gear_ratio: float) -> float:
    wheel_rpm = falcon_to_rpm(velocity, gear_ratio)
    # Divide by 60 to change from m/min to m/s
    wheel_mps = (wheel_rpm * circumference) / 60
    return wheel_mps


@deprecated("Unnecessary for Phoenix 6")
def mps_to_falcon(velocity: float, circumference: float, gear_ratio: float) -> float:
    # Multiply by 60 to change m/s to m/min
    wheel_rpm = (velocity * 60) / circumference
    wheel_velocity = rpm_to_falcon(wheel_rpm, gear_ratio)
    return wheel_velocity


@deprecated("Unnecessary for Phoenix 6")
def falcon_to_metres(counts: float, circumference: float, gear_ratio: float) -> float:
    rotations = counts / (gear_ratio * FALCON_CPR)
    metres = rotations * circumference
    return metres


@deprecated("Unnecessary for Phoenix 6")
def metres_to_falcon(metres: float, circumference: float, gear_ratio: float) -> float:
    rotations = metres / circumference
    counts = rotations * (gear_ratio * FALCON_CPR)
    return counts


@deprecated("Unnecessary for Phoenix 6")
def falcon_to_dps(velocity: float, gear_ratio: float) -> float:
    return falcon_to_rpm(velocity, gear_ratio) * DEGREES_PER_ROTATION / 60


@deprecated("Unnecessary for Phoenix 6")
def dps_to_falcon(dps: float, gear_ratio: float) -> float:
    rpm = dps * 60 / DEGREES_PER_ROTATION
    return rpm_to_falcon(rpm, gear_ratio)


@deprecated("Unnecessary for Phoenix 6")
def falcon_to_radps(velocity: float, gear_ratio: float) -> float:
    return falcon_to_rpm(velocity, gear_ratio) * RADS_PER_ROTATION / 60


@deprecated("Unnecessary for Phoenix 6")
def radps_to_falcon(dps: float, gear_ratio: float) -> float:
    rpm = dps * 60 / RADS_PER_ROTATION
    return rpm_to_falcon(rpm, gear_ratio)


@deprecated("Unnecessary for Phoenix 6")
def units_per_100_ms_to_units_per_sec(velocity: float) -> float:
    return velocity * 10
