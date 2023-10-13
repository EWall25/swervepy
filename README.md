# swervelib

## Description
**swervelib** is a library for swerve drive bases in FRC. If unfamiliar, you can read about swerve drive 
[here](). This library is extendable to any drive base type (coaxial, differential) or module configuration.
Motors, sensors, and swerve modules are interchangeable and easy to develop.

## Installation
Include the `swervelib` folder in your project. Then, import it like any other Python module:
```python
import swervelib
```

## Building a Drive Base
Let's build a swerve drivetrain. For this example, we'll use SDS Mk4i modules running Falcon 500s.

To build our drive base, we'll eventually create an instance of `SwerveDrive`.

```python
import swervelib.impl

swerve = swervelib.impl.SwerveDrive(modules, gyro, max_velocity, max_angular_velocity)
```

But first, we need to define some parameters ubiquitous to all our modules.

```python
drive_params = swervelib.impl.Falcon500CoaxialDriveComponent.Parameters(
    wheel_circumference=4 * math.pi * u.inch,
    gear_ratio=6.75 / 1,  # SDS Mk4i L2
    max_speed=4.5 * (u.m / u.s),
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
azimuth_params = swervelib.impl.Falcon500CoaxialAzimuthComponent.Parameters(
    gear_ratio=150 / 7,  # SDS Mk4i
    max_angular_velocity=11.5 * (u.rad / u.s),
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
```

Now, we can begin creating a swerve module. 
To do so, we'll create the components making up a module: motors, encoders, etc. These components are implementations of
abstract classes that the user can also implement themselves. More on this later.

```python
import swervelib.impl

# Drive motor on Falcon ID 0
drive_component = swervelib.impl.Falcon500CoaxialDriveComponent(0, drive_params)

# CANCoder on ID 0
absolute_encoder = swervelib.impl.AbsoluteCANCoder(0)
# Azimuth module offset. This is the value reported by the absolute encoder when the wheel is pointed straight.
offset = Rotation2d.fromDegrees(0)
# Azimuth (turning) motor on ID 4. The azimuth component includes the absolute encoder because it needs to reset its
# recorded rotation to absolute.
azimuth_component = swervelib.impl.Falcon500CoaxialAzimuthComponent(4, offset, azimuth_params, absolute_encoder)
```

Next, create a swerve module from its components. 
We'll need to specify where the module is placed relative to the chassis center (`placement`).

```python
import swervelib.impl

module_0 = swervelib.impl.CoaxialSwerveModule(
    drive=drive_component,
    azimuth=azimuth_component,
    placement=Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
)
```

Repeat for as many modules as are on the drivetrain, and add them to a list...

```python
modules = [module_0, module_1, ...]
```

We're getting close now! Swerve drivetrains require a gyro, and like motors and encoders, gyros are implementations of
an interface `Gyro`. Let's create a CTRE Pigeon gyro:

```python
import swervelib.impl

# Pigeon IMU on ID 0
gyro = swervelib.impl.PigeonGyro(0, invert=False)
```

Finally, we can assemble the drive base.

```python
swerve = swervelib.impl.SwerveDrive(modules, gyro, MAX_SPEED, MAX_ANGULAR_VELOCITY)
```

For more info on assembling drive bases and using commands, check `example_robot`.

## Running in TeleOp
Because `SwerveDrive` is a subsystem, we can assign it a command to run in teleop period. If you're unfamiliar with the
command-based framework, you can read about it [here](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html).

`SwerveDrive` comes with a teleop command, so just instantiate a new instance and assign it as `swerve`'s
default command.

```python
import wpilib

# Create an XboxController to control the drive base
joystick = wpilib.XboxController(0)

teleop_command = swerve.teleop_command(
    translation=lambda: -joystick.getLeftY(),   # Invert input for positive forward
    strafe=lambda: -joystick.getLeftX(),        # Invert input for positive left
    rotation=lambda: -joystick.getRightX(),     # Invert input for CCW+
    field_relative=True,                        # Forward is always facing the opposing driver station
    open_loop=True,                             # Motors are not running feedback control
)

# The swerve subsystem will run the teleop command when no other commands are running (like during teleop)
swerve.setDefaultCommand(teleop_command)
```

## Defining Your Own Components
**swervelib** comes with some components, such as Falcon motors, a CANCoder, and a Pigeon IMU. In the modern FRC space,
there are many components from which swerve modules can be built. To define a component not included in **swervelib**,
implement from a class in `swervelib.abstract`. Then, you can use your implementation like any of the default ones when
creating a `SwerveModule`.

Check `swervelib.impl` for example implementations of motors, encoders, swerve modules, and gyros.

## WPILib Coordinate System
**swervelib** implements the standard WPILib coordinate system. The robot travels on the XY-plane, and rotates around
the Z-axis.

+X is always forward, and +Y is left. Counter-clockwise is the positive direction (CCW+). This is **different** than FLL,
where rotation is clockwise-positive (CW+). "Forward" depends on the frame of reference.

### Field Coordinate System
Forward faces the opposing alliance's driver station from the allied driver station wall. In this system,
the forward direction never changes.

![Field coordinate system showing +X pointing away from the driver station wall and the +Y axis perpendicular and to the left of the +X](https://docs.wpilib.org/en/stable/_images/field-system.svg)

### Robot Coordinate System
Forward faces the front of the robot, so "forward" changes as the chassis rotates.

![Robot coordinate system showing +X forward, +Y left, and CCW+](https://docs.wpilib.org/en/stable/_images/robot-system.svg)

## Units
**swervelib** uses [Pint](https://pint.readthedocs.io/en/stable/) to specify units whenever possible. Whenever a parameter
requires units, it is denoted with a `Quantity` type hint.

Take for example, a Falcon 500 drive motor's `Parameters`:

```python
from swervelib import u
import swervelib.impl

drive_params = swervelib.impl.Falcon500CoaxialDriveComponent.Parameters(
    wheel_circumference=(4 * math.pi) * u.inch, # 4π in
    max_speed=4.5 * (u.m / u.s),                # 4.5 m/s
    ...
)
```

Simply multiply a number by a unit to create a `Quantity`. Units can be multiplied/divided together to create new units.
For example, `u.m / u.s` creates metres/second, and `u.m / (u.s * u.s)` creates m/s².

## Contributing
Please, feel free to contribute any components or improvements you write for **swervelib**.

WIP

## Contact
* [EWall25 on GitHub](https://github.com/EWall25)
* [ethanhwall07@gmail.com](mailto:ethanhwall07@gmail.com)