import wpilib
import wpilib.drive
from wpilib.drive import RobotDriveBase,DifferentialDrive,MecanumDrive
from wpilib import SmartDashboard,RobotBase
from wpimath import applyDeadband
from lemonlib.preference import SmartPreference
import math
from wpiutil import Sendable,SendableBuilder
from wpilib import interfaces,Spark
from wpimath import geometry
from lemonlib.util import clamp
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Twist2d


class SwagDrive(Sendable):
    maxspeed = SmartPreference(0.8)
    defspeed = SmartPreference(0.5)
    swagadd = SmartPreference(1)
    maxswag = SmartPreference(9000)
    minswag = SmartPreference(0.1)
    swagmulti = SmartPreference(10)

    def __init__(self, leftMotor, rightMotor):
        Sendable.__init__(self)
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.robotDrive = DifferentialDrive(self.leftMotor, self.rightMotor)


        # Swag-related variables
        self.swagLevel = 0
        self.swagPeriod = 0
        self.oldMove = 0
        self.oldRotate = 0

    def Drive(self, moveValue, rotateValue):
        """Custom drive function that incorporates 'swag' logic."""

        SWAG_BARRIER = self.minswag
        SWAG_MULTIPLIER = self.swagmulti
        MAX_SWAG_LEVEL = self.maxswag
        SWAG_PERIOD = 500
        SWAG_ADD = self.swagadd

        moveToSend = moveValue
        rotateToSend = rotateValue



        if self.swagPeriod == 0:
            moveDiff = abs(moveValue - self.oldMove)
            rotateDiff = abs(rotateValue - self.oldRotate)

            if moveDiff < SWAG_BARRIER:
                moveToSend = (moveDiff * SWAG_MULTIPLIER) + moveValue
            else:
                self.swagLevel += SWAG_ADD

            if rotateDiff < SWAG_BARRIER:
                rotateToSend = (rotateDiff * SWAG_MULTIPLIER) + rotateValue
            else:
                self.swagLevel += SWAG_ADD

            if self.swagLevel > MAX_SWAG_LEVEL:
                self.swagPeriod = SWAG_PERIOD
                self.swagLevel = 0


        else:
            moveToSend = 0
            rotateToSend = 1.0
            self.swagPeriod -= 1
         
        self.rotatediff = rotateDiff
        self.movediff = moveDiff
        # Call arcadeDrive with modified move and rotate values
        self.robotDrive.arcadeDrive(moveToSend, rotateToSend)

        self.oldMove = moveValue
        self.oldRotate = rotateValue
    def initSendable(self, builder):
        builder.setSmartDashboardType("SwagDrive")
        builder.addDoubleProperty("Swag Level", lambda: self.swagLevel, lambda _: None)
        builder.addDoubleProperty("Swag Period", lambda: self.swagPeriod, lambda _: None)
        builder.addDoubleProperty("Rotate Diff", lambda: self.rotatediff, lambda _: None)
        builder.addDoubleProperty("Move Diff", lambda: self.movediff, lambda _: None)
        self.robotDrive.initSendable(builder)


class KilloughDrive(RobotDriveBase, Sendable):
    r"""A class for driving Killough drive platforms.

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

    kDefaultLeftMotorAngle = 60.0
    kDefaultRightMotorAngle = 120.0
    kDefaultBackMotorAngle = 270.0

    def __init__(self, leftMotor, rightMotor, backMotor,
                 leftMotorAngle=kDefaultLeftMotorAngle,
                 rightMotorAngle=kDefaultRightMotorAngle,
                 backMotorAngle=kDefaultBackMotorAngle):
        """
        Constructs a Killough drive.
        Angles are measured in degrees clockwise from the positive X axis.
        
        The default motor angles make the wheels on each corner parallel to their
        respective opposite sides.

        If a motor needs to be inverted, do so before passing it in.

        :param leftMotor: The motor on the left corner.
        :param rightMotor: The motor on the right corner.
        :param backMotor: The motor on the back corner.
        :param leftMotorAngle: The angle of the left wheel's forward direction of travel
        :param rightMotorAngle: The angle of the right wheel's forward direction of travel
        :param backMotorAngle: The angle of the back wheel's forward direction of travel
        """
        super().__init__()
        Sendable.__init__(self)

        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.backMotor = backMotor

        # Create a dummy MecanumDrive instance (for compatibility).
        self.mechDrive = MecanumDrive(self.leftMotor, self.backMotor, self.rightMotor, self.backMotor)

        # Compute each wheel’s unit driving vector (in the robot frame: x forward, y right)
        self.leftVec = Vector2d(math.cos(math.radians(leftMotorAngle)),
                                math.sin(math.radians(leftMotorAngle)))
        self.rightVec = Vector2d(math.cos(math.radians(rightMotorAngle)),
                                 math.sin(math.radians(rightMotorAngle)))
        self.backVec = Vector2d(math.cos(math.radians(backMotorAngle)),
                                math.sin(math.radians(backMotorAngle)))

    def driveCartesian(self, ySpeed, xSpeed, zRotation, gyroAngle=0.0):
        """Drive method for Killough platform.

        Angles are measured clockwise from the positive X axis. The robot's speed is independent
        from its angle or rotation rate.

        :param ySpeed: The robot's speed along the Y axis `[-1.0..1.0]`. Right is positive.
        :param xSpeed: The robot's speed along the X axis `[-1.0..1.0]`. Forward is positive.
        :param zRotation: The robot's rotation rate around the Z axis `[-1.0..1.0]`. Clockwise is positive.
        :param gyroAngle: The current angle reading from the gyro in degrees around the Z axis. Use
                          this to implement field-oriented controls.
        """
        # Clamp and apply deadband.
        ySpeed = clamp(ySpeed, -1.0, 1.0)
        ySpeed = applyDeadband(ySpeed, self._m_deadband)
        xSpeed = clamp(xSpeed, -1.0, 1.0)
        xSpeed = applyDeadband(xSpeed, self._m_deadband)

        # Create an input vector (x: forward, y: right).
        input_vec = Vector2d(xSpeed, ySpeed)
        input_vec.rotate(gyroAngle)  # Rotate input if using field-oriented control.

        # Compute wheel speeds by projecting the input vector onto each wheel vector and adding rotation.
        wheelSpeeds = [
            input_vec.scalarProject(self.leftVec) + zRotation,
            input_vec.scalarProject(self.rightVec) + zRotation,
            input_vec.scalarProject(self.backVec) + zRotation
        ]

        KilloughDrive.normalize(wheelSpeeds)

        # Set motor outputs.
        self.leftMotor.set(wheelSpeeds[0] * self._m_maxOutput)
        self.rightMotor.set(wheelSpeeds[1] * self._m_maxOutput)
        self.backMotor.set(wheelSpeeds[2] * self._m_maxOutput)

        self.feed()
        self.mechDrive.driveCartesian(ySpeed, xSpeed, zRotation)

    def drivePolar(self, magnitude, angle, zRotation):
        """Drive method for Killough platform.

        Angles are measured counter-clockwise from straight ahead. The speed at which the robot
        drives (translation) is independent from its angle or zRotation rate.

        :param magnitude: The robot's speed at a given angle `[-1.0..1.0]`. Forward is positive.
        :param angle: The angle around the Z axis at which the robot drives in degrees `[-180..180]`.
        :param zRotation: The robot's rotation rate around the Z axis `[-1.0..1.0]`. Clockwise is positive.
        """
        magnitude = clamp(magnitude, -1, 1) * math.sqrt(2)
        self.driveCartesian(magnitude * math.sin(math.radians(angle)),
                            magnitude * math.cos(math.radians(angle)),
                            zRotation, gyroAngle=0)

    def stopMotor(self):
        self.leftMotor.stopMotor()
        self.rightMotor.stopMotor()
        self.backMotor.stopMotor()
        self.feed()

    def getDescription(self):
        return "Killough Drive"

    def initSendable(self, builder):
        builder.setSmartDashboardType("KilloughDrive")
        builder.addDoubleProperty("Left Motor Speed", self.leftMotor.get, self.leftMotor.set)
        builder.addDoubleProperty("Right Motor Speed", self.rightMotor.get, self.rightMotor.set)
        builder.addDoubleProperty("Back Motor Speed", self.backMotor.get, self.backMotor.set)
        self.mechDrive.initSendable(builder)

    @staticmethod
    def normalize(wheelSpeeds):
        """Normalize all wheel speeds if the magnitude of any wheel is greater
        than 1.0.

        :param wheelSpeeds: Iterable of wheelspeeds to normalize
        """
        maxMagnitude = max(abs(x) for x in wheelSpeeds)
        if maxMagnitude > 1.0:
            for i in range(len(wheelSpeeds)):
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude
class Vector2d:
    def __init__(self, x=0.0, y=0.0):
        self.x = x  # forward component
        self.y = y  # right component

    def rotate(self, angle_deg):
        """Rotate this vector counter-clockwise by angle_deg degrees."""
        angle_rad = math.radians(angle_deg)
        cosA = math.cos(angle_rad)
        sinA = math.sin(angle_rad)
        x_new = self.x * cosA - self.y * sinA
        y_new = self.x * sinA + self.y * cosA
        self.x = x_new
        self.y = y_new

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def magnitude(self):
        return math.hypot(self.x, self.y)

    def scalarProject(self, other):
        """Returns the scalar projection of this vector onto 'other'."""
        mag = other.magnitude()
        if mag == 0:
            return 0.0
        return self.dot(other) / mag

# --- Killough Drive Simulation Using WPIMath Geometry ---
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
        self.omega = 0.0     # angular velocity (rad/s)

        # Assume the wheels are arranged in an equilateral triangle.
        self.R = 0.5  # Distance (meters) from robot center to each wheel.
        self.wheel_positions = [
            Vector2d(self.R * math.cos(math.radians(KilloughDrive.kDefaultLeftMotorAngle)),
                     self.R * math.sin(math.radians(KilloughDrive.kDefaultLeftMotorAngle))),
            Vector2d(self.R * math.cos(math.radians(KilloughDrive.kDefaultRightMotorAngle)),
                     self.R * math.sin(math.radians(KilloughDrive.kDefaultRightMotorAngle))),
            Vector2d(self.R * math.cos(math.radians(KilloughDrive.kDefaultBackMotorAngle)),
                     self.R * math.sin(math.radians(KilloughDrive.kDefaultBackMotorAngle)))
        ]
        # Use the same wheel vectors defined in the drive (robot frame).
        self.wheel_directions = [self.drive.leftVec,
                                 self.drive.rightVec,
                                 self.drive.backVec]

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
            back_output * self.wheel_force
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
            net_torque += (r.x * fy - r.y * fx)

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
        twist = Twist2d(self.vx_robot * self.dt, self.vy_robot * self.dt, self.omega * self.dt)
        # Update the robot’s pose using the exponential map.
        self.pose = self.pose.exp(twist)

    def get_pose(self):
        """
        Returns the robot’s pose as (x, y, theta_degrees).
        """
        x = self.pose.translation().x
        y = self.pose.translation().y
        theta_deg = self.pose.rotation().radians()
        return (Pose2d(x, y, theta_deg))