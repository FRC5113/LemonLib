import math
from typing import Any
from dataclasses import dataclass
from ..util import clamp
from wpilib.interfaces import MotorController
from wpiutil import Sendable, SendableBuilder
from wpilib.drive import RobotDriveBase
from wpimath.geometry import Pose2d

__all__ = ["KilloughDrive"]


class KilloughDrive:
    r"""A class for driving Killough (Kiwi) drive platforms.

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
    """

    def __init__(
        self,
        front_right_motor: MotorController,
        front_left_motor: MotorController,
        back_motor: MotorController,
        angles=None,
    ):
        """
        :param front_right_motor: Motor controller for the front right wheel
        :param front_left_motor: Motor controller for the front left wheel
        :param back_motor: Motor controller for the back wheel
        :param angles: List of wheel angles (default: [120, -120, 0])
        """
        self.front_right_motor = front_right_motor
        self.front_left_motor = front_left_motor
        self.back_motor = back_motor

        self.angles = angles if angles else [120, -120, 0]
        self._calculate_transform_matrix()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Heading in radians

    def _calculate_transform_matrix(self):
        """Precomputes the wheel transformation matrix based on configured angles."""
        self.transform = []
        for angle in self.angles:
            rad = math.radians(angle)
            self.transform.append([math.cos(rad), math.sin(rad), 1])

    def drive_cartesian(
        self, ySpeed: float, xSpeed: float, omega: float, gyro_angle: float = 0.0
    ):
        """Drive method for Killough platform.

        Angles are measured clockwise from the positive X axis. The robot's speed is independent
        from its angle or rotation rate.

        :param ySpeed: The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
        :param xSpeed: The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
        :param omega: The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
        :param gyro_angle: The current angle reading from the gyro in degrees around the Z axis.
                           Use this to implement field-oriented controls.
        """
        x, y = self._apply_field_oriented_control(xSpeed, ySpeed, gyro_angle)
        speeds = self._calculate_wheel_speeds(x, y, omega)

        self.front_left_motor.set(speeds[0])
        self.front_right_motor.set(speeds[1])
        self.back_motor.set(speeds[2])

    def drive_polar(self, magnitude: float, angle: float, zRotation: float) -> None:
        """Drive method for Killough platform using polar coordinates.

        Angles are measured counter-clockwise from straight ahead. The speed at which the robot
        drives (translation) is independent from its angle or zRotation rate.

        :param magnitude: The robot's speed at a given angle [-1.0..1.0]. Forward is positive.
        :param angle: The angle around the Z axis at which the robot drives in degrees [-180..180].
        :param zRotation: The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
        """
        magnitude = max(min(magnitude, 1), -1) * math.sqrt(2)

        self.drive_cartesian(
            magnitude * math.cos(math.radians(angle)),
            magnitude * math.sin(math.radians(angle)),
            zRotation,
            0,
        )

    def _apply_field_oriented_control(self, vx: float, vy: float, gyro_angle: float):
        """Applies field-oriented correction using the gyro."""
        robot_angle = math.radians(gyro_angle)
        temp_vx = vx * math.cos(robot_angle) - vy * math.sin(robot_angle)
        vy = vx * math.sin(robot_angle) + vy * math.cos(robot_angle)
        return temp_vx, vy

    def normalize(self, v1: float, v2: float, v3: float):
        """Normalizes wheel speeds to keep them within [-1.0..1.0]."""
        max_speed = max(abs(v1), abs(v2), abs(v3))
        if max_speed > 1:
            v1 /= max_speed
            v2 /= max_speed
            v3 /= max_speed
        return [v1, v2, v3]

    def _calculate_wheel_speeds(self, vx: float, vy: float, omega: float):
        """Computes the wheel speeds and applies normalization."""
        v1 = (
            self.transform[0][0] * vx
            + self.transform[0][1] * vy
            + self.transform[0][2] * omega
        )
        v2 = (
            self.transform[1][0] * vx
            + self.transform[1][1] * vy
            + self.transform[1][2] * omega
        )
        v3 = (
            self.transform[2][0] * vx
            + self.transform[2][1] * vy
            + self.transform[2][2] * omega
        )

        return self.normalize(v1, v2, v3)

    def _update_odometry(self, vx: float, vy: float, omega: float, dt: float):
        """Updates the robot's estimated position on the field."""
        self.theta += omega * dt
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt

    def get_position(self):
        """Returns the estimated position of the robot."""
        return Pose2d(self.x, self.y, self.theta)

    def initSendable(self, builder: SendableBuilder) -> None:
        """Initializes the sendable interface for SmartDashboard integration."""
        builder.setSmartDashboardType("KilloughDrive")
        builder.addDoubleProperty("X Position", lambda: self.x, lambda x: None)
        builder.addDoubleProperty("Y Position", lambda: self.y, lambda x: None)
        builder.addDoubleProperty(
            "Heading (deg)", lambda: math.degrees(self.theta), lambda x: None
        )
        builder.addDoubleProperty(
            "Left Motor Speed", self.front_left_motor.get, self.front_left_motor.set
        )
        builder.addDoubleProperty(
            "Right Motor Speed", self.front_right_motor.get, self.front_right_motor.set
        )
        builder.addDoubleProperty(
            "Back Motor Speed", self.back_motor.get, self.back_motor.set
        )
