"""
A swerve library for a four-module drivebase equipped with Falcon 500s, CTRE CANCoders, and a CTRE Pigeon IMU.
This is not template code, but rather a ready-to-go subsystem and configuration objects for it that are passed in
through dependency injection.
"""

__all__ = [
    "SwerveParameters",
    "SwerveModuleParameters",
    "SwerveModule",
    "Swerve",
    "ModuleCorner",
    "CanFDDeviceID",
    "u",
    "CTRESwerveParameters",
    "CTRESwerveModuleParameters",
    "REVSwerveModuleParameters",
    "REVSwerveParameters",
    "VisionParameters",
    "AutoParameters",
    "CameraDefinition",
]

from pint import UnitRegistry

u = UnitRegistry()

from .configs import (
    SwerveParameters,
    SwerveModuleParameters,
    ModuleCorner,
    CanFDDeviceID,
    CTRESwerveModuleParameters,
    CTRESwerveParameters,
    REVSwerveModuleParameters,
    REVSwerveParameters,
    VisionParameters,
    AutoParameters,
)
from .mod import SwerveModule
from .subsystem import Swerve
from .vision import CameraDefinition
