import wpilib.drive
import math
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Twist2d
from ..drive import Vector2d, KilloughDrive


class KilloughDriveSim:
    def __init__(self, drive, mass=50.0, moment_of_inertia=10.0, wheel_force=100.0):
        """
        :param drive: An instance of KilloughDrive.
        :param mass: Robot mass in kg.
        :param moment_of_inertia: Rotational inertia (kg*m^2).
        :param wheel_force: Maximum force (N) per wheel at full output.
        :param dt: Simulation timestep (seconds).
        """
        self.drive = drive
        self.mass = mass
        self.moment_of_inertia = moment_of_inertia
        self.wheel_force = wheel_force

        # Represent the robot’s pose as a WPIMath Pose2d (in meters and radians).
        self.pose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))

        # Chassis velocities in the robot frame (m/s and rad/s).
        self.vx_robot = 0.0  # forward speed (m/s)
        self.vy_robot = 0.0  # sideways speed (m/s)
        self.omega = 0.0  # angular velocity (rad/s)

        # Assume the wheels are arranged in an equilateral triangle.
        self.R = 0.5  # Distance (meters) from robot center to each wheel.
        self.wheel_positions = [
            Vector2d(
                self.R * math.cos(math.radians(KilloughDrive.kDefaultLeftMotorAngle)),
                self.R * math.sin(math.radians(KilloughDrive.kDefaultLeftMotorAngle)),
            ),
            Vector2d(
                self.R * math.cos(math.radians(KilloughDrive.kDefaultRightMotorAngle)),
                self.R * math.sin(math.radians(KilloughDrive.kDefaultRightMotorAngle)),
            ),
            Vector2d(
                self.R * math.cos(math.radians(KilloughDrive.kDefaultBackMotorAngle)),
                self.R * math.sin(math.radians(KilloughDrive.kDefaultBackMotorAngle)),
            ),
        ]
        # Use the same wheel vectors defined in the drive (robot frame).
        self.wheel_directions = [
            self.drive.leftVec,
            self.drive.rightVec,
            self.drive.backVec,
        ]

    def update(self, dt=0.02):
        """
        Update the simulation by one timestep.
        Reads motor outputs, computes forces (in the robot frame), updates chassis speeds,
        applies damping, and integrates the pose using WPIMath’s twist (exponential map).
        """
        # Read motor outputs.
        left_output = self.drive.leftMotor.get()
        right_output = self.drive.rightMotor.get()
        back_output = self.drive.backMotor.get()
        self.dt = dt

        # Compute force from each wheel.
        forces = [
            left_output * self.wheel_force,
            right_output * self.wheel_force,
            back_output * self.wheel_force,
        ]

        net_force_robot_x = 0.0
        net_force_robot_y = 0.0
        net_torque = 0.0

        for i in range(3):
            direction = self.wheel_directions[i]
            force = forces[i]
            # Force components in robot frame.
            fx = force * direction.x
            fy = force * direction.y
            net_force_robot_x += fx
            net_force_robot_y += fy
            # Compute torque: torque = r_x * F_y - r_y * F_x
            r = self.wheel_positions[i]
            net_torque += r.x * fy - r.y * fx

        # Compute accelerations in the robot frame.
        ax_robot = net_force_robot_x / self.mass
        ay_robot = net_force_robot_y / self.mass
        alpha = net_torque / self.moment_of_inertia

        # Update chassis velocities.
        self.vx_robot += ax_robot * self.dt
        self.vy_robot += ay_robot * self.dt
        self.omega += alpha * self.dt

        # Apply damping (simulate friction/resistance).
        damping_factor = 0.98
        self.vx_robot *= damping_factor
        self.vy_robot *= damping_factor
        self.omega *= damping_factor

        # Create a twist (displacement in the robot frame over dt).
        twist = Twist2d(
            self.vx_robot * self.dt, self.vy_robot * self.dt, self.omega * self.dt
        )
        # Update the robot’s pose using the exponential map.
        self.pose = self.pose.exp(twist)

    def get_pose(self):
        """
        Returns the robot’s pose as (x, y, theta_degrees).
        """
        x = self.pose.translation().x
        y = self.pose.translation().y
        theta_deg = self.pose.rotation().radians()
        return Pose2d(x, y, theta_deg)
