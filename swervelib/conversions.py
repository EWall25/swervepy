import astropy.units as u
from astropy.units import Quantity

FALCON_CPR = 2048
DEGREES_PER_ROTATION = 360


def falcon_to_degrees(counts: float, gear_ratio: float) -> Quantity[u.deg]:
    degrees = counts * (DEGREES_PER_ROTATION / (gear_ratio * FALCON_CPR))
    return degrees * u.deg


def degrees_to_falcon(degrees: Quantity[u.deg], gear_ratio: float) -> float:
    ticks = degrees.to_value(u.deg) / (DEGREES_PER_ROTATION / (gear_ratio * FALCON_CPR))
    return ticks


def falcon_to_rpm(velocity: float, gear_ratio: float) -> float:
    # Falcon velocity is in units/100ms, so multiplying by 600 changes to units/min.
    motor_rpm = velocity * (600 / FALCON_CPR)
    mechanism_rpm = motor_rpm / gear_ratio
    return mechanism_rpm


def rpm_to_falcon(rmp: float, gear_ratio: float) -> float:
    motor_rpm = rmp * gear_ratio
    # Falcon velocity is in units/100ms, so multiplying by 600 changes to units/min.
    ticks = motor_rpm * (FALCON_CPR / 600)
    return ticks


def falcon_to_mps(velocity: float, circumference: Quantity["length"], gear_ratio: float) -> Quantity[u.m / u.s]:
    wheel_rpm = falcon_to_rpm(velocity, gear_ratio)
    # Divide by 60 to change from m/min to m/s
    wheel_mps = (wheel_rpm * circumference.to_value(u.m)) / 60
    return wheel_mps * (u.m / u.s)


def mps_to_falcon(velocity: Quantity[u.m / u.s], circumference: Quantity["length"], gear_ratio: float) -> float:
    # Multiply by 60 to change m/s to m/min
    wheel_rpm = (velocity.to_value(u.m / u.s) * 60) / circumference.to_value(u.m)
    wheel_velocity = rpm_to_falcon(wheel_rpm, gear_ratio)
    return wheel_velocity


def falcon_to_metres(counts: float, circumference: Quantity["length"], gear_ratio: float) -> Quantity[u.m]:
    rotations = counts / (gear_ratio * FALCON_CPR)
    metres = rotations * circumference.to_value(u.m)
    return metres * u.m


def metres_to_falcon(metres: Quantity[u.m], circumference: Quantity["length"], gear_ratio: float) -> float:
    rotations = metres.to_value(u.m) / circumference.to_value(u.m)
    counts = rotations * (gear_ratio * FALCON_CPR)
    return counts
