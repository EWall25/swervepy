"""
A modular swerve drive library extendable to any swerve configuration.
Comes with support for coaxial SDS- and MAXSwerve-style drivetrains out of the box.
"""

__all__ = ["u", "SwerveDrive", "TrajectoryFollowerParameters"]

# fmt: off

# Initialize the unit registry before importing anything that relies on it
from pint import UnitRegistry
u = UnitRegistry()

from .subsystem import SwerveDrive, TrajectoryFollowerParameters
