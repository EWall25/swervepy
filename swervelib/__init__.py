"""
A swerve library for a four-module drivebase equipped with Falcon 500s, CTRE CANCoders, and a CTRE Pigeon IMU.
This is not template code, but rather a ready-to-go subsystem and configuration objects for it that are passed in
through dependency injection.
"""

__all__ = [
    "CTREConfigs",
    "SwerveParameters",
    "SwerveModuleParameters",
    "SwerveModule",
    "Swerve",
    "ModuleCorner",
    "CANDeviceID",
]

from .configs import CTREConfigs, SwerveParameters, SwerveModuleParameters, ModuleCorner, CANDeviceID
from .mod import SwerveModule
from .subsystem import Swerve
