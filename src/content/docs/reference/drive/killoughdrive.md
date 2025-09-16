---
title: KilloughDrive
slug: reference/drive/killoughdrive
---

# KilloughDrive

A class for driving Killough (Kiwi) drive platforms.

Killough drives are triangular with one omni wheel on each corner.

Drive Base Diagram::

      /_____\
     / \   / \
        \ /
        ---

Each `drive()` function provides different inverse kinematic relations for a Killough drive.
The default wheel vectors are parallel to their respective opposite sides, but can be overridden.
See the constructor for more information.

This library uses the NED axes convention (North-East-Down as external reference in the world
frame): http://www.nuclearprojects.com/ins/images/axis_big.png.

The positive X axis points ahead, the positive Y axis points right, and the positive Z axis
points down. Rotations follow the right-hand rule, so clockwise rotation around the Z axis is
positive.

## Methods

### __init__()

:param front_right_motor: Motor controller for the front right wheel
:param front_left_motor: Motor controller for the front left wheel
:param back_motor: Motor controller for the back wheel
:param angles: List of wheel angles (default: [120, -120, 0])

### _calculate_transform_matrix()

Precomputes the wheel transformation matrix based on configured angles.

### drive_cartesian()

Drive method for Killough platform.

Angles are measured clockwise from the positive X axis. The robot's speed is independent
from its angle or rotation rate.

:param ySpeed: The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
:param xSpeed: The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
:param omega: The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
:param gyro_angle: The current angle reading from the gyro in degrees around the Z axis.
                   Use this to implement field-oriented controls.

### drive_polar()

Drive method for Killough platform using polar coordinates.

Angles are measured counter-clockwise from straight ahead. The speed at which the robot
drives (translation) is independent from its angle or zRotation rate.

:param magnitude: The robot's speed at a given angle [-1.0..1.0]. Forward is positive.
:param angle: The angle around the Z axis at which the robot drives in degrees [-180..180].
:param zRotation: The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.

### _apply_field_oriented_control()

Applies field-oriented correction using the gyro.

### normalize()

Normalizes wheel speeds to keep them within [-1.0..1.0].

### _calculate_wheel_speeds()

Computes the wheel speeds and applies normalization.

### _update_odometry()

Updates the robot's estimated position on the field.

### get_position()

Returns the estimated position of the robot.

### initSendable()

Initializes the sendable interface for SmartDashboard integration.

