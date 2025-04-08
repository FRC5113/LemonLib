import math
from typing import Any
from dataclasses import dataclass
from lemonlib.util import clamp
from wpilib.interfaces import MotorController
from wpiutil import Sendable, SendableBuilder
from wpilib.drive import RobotDriveBase
from wpimath.geometry import Pose2d

__all__ = ["KilloughDrive"]


class KilloughDrive(Sendable):
    """
    A class for driving Killough drive platforms with omni wheels.
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
        :param angles: List of wheel angles (default: [60, -60, 0])
        """
        self.front_right_motor = front_right_motor
        self.front_left_motor = front_left_motor
        self.back_motor = back_motor

        self.angles = angles if angles else [60, -60, 0]
        self._calculate_transform_matrix()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Heading in radians

        Sendable.__init__(self)

    def _calculate_transform_matrix(self):
        """Precomputes the wheel transformation matrix based on configured angles."""
        self.transform = []
        for angle in self.angles:
            rad = math.radians(angle)
            self.transform.append([math.cos(rad), math.sin(rad), 1])

    def drive_cartesian(self, x: float, y: float, omega: float, dt: float):
        """Drives the robot using standard Cartesian controls."""
        self._calculate_wheel_speeds(x, y, omega)
        self._update_odometry(x, y, omega, dt)

    def drive_field_oriented(
        self, x: float, y: float, omega: float, gyro_angle: float, dt: float
    ):
        """Drives the robot using field-oriented Cartesian controls."""
        if gyro_angle:
            x, y = self._apply_field_oriented_control(x, y, gyro_angle)
        self._calculate_wheel_speeds(x, y, omega)
        self._update_odometry(x, y, omega, dt)

    def _apply_field_oriented_control(self, vx: float, vy: float, gyro_angle: float):
        """Applies field-oriented correction using the gyro."""
        robot_angle = math.radians(gyro_angle)
        temp_vx = vx * math.cos(robot_angle) - vy * math.sin(robot_angle)
        vy = vx * math.sin(robot_angle) + vy * math.cos(robot_angle)
        return temp_vx, vy

    def _calculate_wheel_speeds(self, vx: float, vy: float, omega: float):
        """Computes the wheel speeds and sets motor power."""
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

        max_speed = max(abs(v1), abs(v2), abs(v3))
        if max_speed > 1:
            v1 /= max_speed
            v2 /= max_speed
            v3 /= max_speed

        self.front_right_motor.set(clamp(v1, -1, 1))
        self.front_left_motor.set(clamp(v2, -1, 1))
        self.back_motor.set(clamp(v3, -1, 1))

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
