# SwervePy: Swerve library for Python

## Description
**swervepy** is a library for swerve drive bases in FRC. If unfamiliar, you can read about swerve drive 
[here](). This library is extendable to any drive base type (coaxial, differential) or module configuration.
Motors, sensors, and swerve modules are interchangeable and easy to develop.

## Installation
Include the `swervepy` folder in your project. Then, import it like any other Python module:
```python
import swervepy
```

## Design Philosophy
Swerve drivetrains are made up of any number of swerve modules, and swerve modules can be made from different motors and
sensors. There are a nearly infinite number of configurations, so the library backing the drivetrain must be
infinitely configurable.

All swerve modules can drive and rotate a wheel, so **swervepy** defines an abstract `SwerveModule`
with those functions. And, because the basic functionality of all drivetrains is similar (move forward, back, left, right, etc.),
**swervepy** also provides a concrete `SwerveDrive` class.

Users may implement `SwerveModule` to define _how_ the swerve module drives and rotates. For example, **swervepy** includes a
`CoaxialSwerveModule` (the most common kind), in which one motor drives a wheel, and another turns it.

Each drivetrain and module is made up of a number of **components**. A **component** is any actuator or sensor with one 
defined purpose, such as a drive motor, turning motor, or azimuth encoder. Gyros and even swerve modules are **components**
that comprise a drive base.

**swervepy** defines abstract **components** for the user to implement. For example, `swervepy.abstract.motor`
contains `CoaxialDriveComponent`. This **component** can follow a velocity and report its velocity. In `swervepy.impl.motor`,
you will find an implementation of `CoaxialDriveComponent` called `Falcon500CoaxialDriveComponent`. This implementation
defines how a Falcon 500 should fulfill the purpose of a `CoaxialDriveComponent`. If the user has, for example, a NEO they
want to use as a drive motor, they can implement it in the same way as **swervepy** implements the Falcon 500.

Then, the user may create a `CoaxialSwerveModule` (or any other implementation of `SwerveModule`) by passing in a
drive and azimuth component. These components can be changed out like mechanical parts, accelerating
development.

## Building a Drive Base
Let's build a swerve drivetrain. For this example, we'll use SDS Mk4i modules running Falcon 500s.

To build our drive base, we'll eventually create an instance of `SwerveDrive`.

```python
import swervepy.subsystem

swerve = swervepy.subsystem.SwerveDrive(modules, gyro, MAX_VELOCITY, MAX_ANGULAR_VELOCITY)
```

But first, we need to define some parameters ubiquitous to all our modules.

```python
drive_params = swervepy.impl.Falcon500CoaxialDriveComponent.Parameters(
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
azimuth_params = swervepy.impl.Falcon500CoaxialAzimuthComponent.Parameters(
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
abstract classes that the user can also implement themselves.

```python
import swervepy.impl

# Drive motor on Falcon ID 0
drive_component = swervepy.impl.Falcon500CoaxialDriveComponent(0, drive_params)

# CANCoder on ID 0
absolute_encoder = swervepy.impl.AbsoluteCANCoder(0)
# Azimuth module offset. This is the value reported by the absolute encoder when the wheel is pointed straight.
offset = Rotation2d.fromDegrees(0)
# Azimuth (turning) motor on ID 4. The azimuth component includes the absolute encoder because it needs to reset its
# recorded rotation to absolute.
azimuth_component = swervepy.impl.Falcon500CoaxialAzimuthComponent(4, offset, azimuth_params, absolute_encoder)
```

Next, create a swerve module from its components. 
We'll need to specify where the module is placed relative to the chassis center (`placement`).

```python
import swervepy.impl

module_0 = swervepy.impl.CoaxialSwerveModule(
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
import swervepy.impl

# Pigeon IMU on ID 0
gyro = swervepy.impl.PigeonGyro(0, invert=False)
```

Finally, we can assemble the drive base.

```python
import swervepy.subsystem

swerve = swervepy.subsystem.SwerveDrive(modules, gyro, MAX_VELOCITY, MAX_ANGULAR_VELOCITY)
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
    translation=lambda: -joystick.getLeftY(),  # Invert input for positive forward
    strafe=lambda: -joystick.getLeftX(),  # Invert input for positive left
    rotation=lambda: -joystick.getRightX(),  # Invert input for CCW+
    field_relative=True,  # Forward is always facing the opposing driver station
    drive_open_loop=True,  # Motors are not running feedback control
)

# The swerve subsystem will run the teleop command when no other commands are running (like during teleop)
swerve.setDefaultCommand(teleop_command)
```

## Defining Your Own Components
**swervepy** comes with some components, such as Falcon motors, a CANCoder, and a Pigeon IMU. In the modern FRC space,
there are many components from which swerve modules can be built. To define a component not included in **swervepy**,
implement from a class in `swervepy.abstract`. Then, you can use your implementation like any of the default ones when
creating a `SwerveModule`.

Check `swervepy.impl` for example implementations of motors, encoders, swerve modules, and gyros.

## WPILib Coordinate System
**swervepy** implements the standard WPILib coordinate system. The robot travels on the XY-plane, and rotates around
the Z-axis.

+X is always forward, and +Y is left. Counter-clockwise is the positive direction (CCW+). This is **different** from FLL,
where rotation is clockwise-positive (CW+). "Forward" depends on the frame of reference.

### Field Coordinate System
Forward faces the opposing alliance's driver station from the allied driver station wall. In this system,
the forward direction never changes.

![Field coordinate system showing +X pointing away from the driver station wall and the +Y axis perpendicular and to the left of the +X](https://docs.wpilib.org/en/stable/_images/field-system.svg)

### Robot Coordinate System
Forward faces the front of the robot, so "forward" changes as the chassis rotates.

![Robot coordinate system showing +X forward, +Y left, and CCW+](https://docs.wpilib.org/en/stable/_images/robot-system.svg)

## Units
**swervepy** uses [Pint](https://pint.readthedocs.io/en/stable/) to specify units whenever possible. Whenever a parameter
requires units, it is denoted with a `Quantity` type hint.

Take for example, a Falcon 500 drive motor's `Parameters`:

```python
from swervepy import u
import swervepy.impl

drive_params = swervepy.impl.Falcon500CoaxialDriveComponent.Parameters(
    wheel_circumference=(4 * math.pi) * u.inch, # 4π in
    max_speed=4.5 * (u.m / u.s),                # 4.5 m/s
    ...
)
```

Simply multiply a number by a unit to create a `Quantity`. Units can be multiplied/divided together to create new units.
For example, `u.m / u.s` creates metres/second, and `u.m / (u.s * u.s)` creates m/s².

## Contributing
Please, feel free to contribute any components or improvements you write for **swervepy**.

WIP

## Contact
* [EWall25 on GitHub](https://github.com/EWall25)
* [ethanhwall07@gmail.com](mailto:ethanhwall07@gmail.com)
