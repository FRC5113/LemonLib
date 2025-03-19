import wpilib.drive
from wpilib.drive import RobotDriveBase, MecanumDrive
from wpimath import applyDeadband
import math
from wpiutil import Sendable
from lemonlib.util import clamp
from ._vector2d import Vector2d

 
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

    def __init__(
        self,
        leftMotor,
        rightMotor,
        backMotor,
        leftMotorAngle=kDefaultLeftMotorAngle,
        rightMotorAngle=kDefaultRightMotorAngle,
        backMotorAngle=kDefaultBackMotorAngle,
    ):
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
        self.mechDrive = MecanumDrive(
            self.leftMotor, self.backMotor, self.rightMotor, self.backMotor
        )

        # Compute each wheel’s unit driving vector (in the robot frame: x forward, y right)
        self.leftVec = Vector2d(
            math.cos(math.radians(leftMotorAngle)),
            math.sin(math.radians(leftMotorAngle)),
        )
        self.rightVec = Vector2d(
            math.cos(math.radians(rightMotorAngle)),
            math.sin(math.radians(rightMotorAngle)),
        )
        self.backVec = Vector2d(
            math.cos(math.radians(backMotorAngle)),
            math.sin(math.radians(backMotorAngle)),
        )

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
            input_vec.scalarProject(self.backVec) + zRotation,
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
        self.driveCartesian(
            magnitude * math.sin(math.radians(angle)),
            magnitude * math.cos(math.radians(angle)),
            zRotation,
            gyroAngle=0,
        )

    def stopMotor(self):
        self.leftMotor.stopMotor()
        self.rightMotor.stopMotor()
        self.backMotor.stopMotor()
        self.feed()

    def getDescription(self):
        return "Killough Drive"

    def initSendable(self, builder):
        builder.setSmartDashboardType("KilloughDrive")
        builder.addDoubleProperty(
            "Left Motor Speed", self.leftMotor.get, self.leftMotor.set
        )
        builder.addDoubleProperty(
            "Right Motor Speed", self.rightMotor.get, self.rightMotor.set
        )
        builder.addDoubleProperty(
            "Back Motor Speed", self.backMotor.get, self.backMotor.set
        )
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
