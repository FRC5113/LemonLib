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

    instances = 0

    def __init__(self, leftMotor, rightMotor, backMotor, leftMotorAngle=kDefaultLeftMotorAngle,
                 rightMotorAngle=kDefaultRightMotorAngle, backMotorAngle=kDefaultBackMotorAngle):
        """Construct a Killough drive with the given motors and default motor angles.

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
       
        self.mechDrive = MecanumDrive(self.leftMotor, self.rightMotor, self.backMotor, Spark(19))
        

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

        ySpeed = Legacy.limit(ySpeed)
        ySpeed = Legacy.applyDeadband(ySpeed, self._m_deadband)

        xSpeed = Legacy.limit(xSpeed)
        xSpeed = Legacy.applyDeadband(xSpeed, self._m_deadband)

        # Compensate for gyro angle
        input = Vector2d(ySpeed, xSpeed)
        input.rotate(gyroAngle)

        wheelSpeeds = [input.scalarProject(self.leftVec) + zRotation,
                       input.scalarProject(self.rightVec) + zRotation,
                       input.scalarProject(self.backVec) + zRotation]

        Legacy.normalize(wheelSpeeds)

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

        magnitude = Legacy.limit(magnitude) * math.sqrt(2)

        self.driveCartesian(magnitude * math.cos(math.radians(angle)), magnitude * math.sin(math.radians(angle)),
                            zRotation, 0)
    

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
class Vector2d:
    """This is a 2D vector struct that supports basic operations"""
    def __init__(self, x=0.0, y=0.0):
        """Construct a 2D vector

        :param x: x component of the vector
        :param y: y component of the vector
        """
        self.x = x
        self.y = y

    def rotate(self, angle):
        """Rotate a vector in Cartesian space.

        :param angle: Angle in degrees by which to rotate vector counter-clockwise
        """
        angle = math.radians(angle)
        cosA = math.cos(angle)
        sinA = math.sin(angle)

        x = self.x * cosA - self.y * sinA
        y = self.x * sinA + self.y * cosA
        self.x = x
        self.y = y


    def dot(self, vec):
        """Returns dot product of this vector and argument

        :param vec: Vector with which to perform dot product
        :type vec: Vector2d
        """
        return self.x * vec.x + self.y * vec.y


    def magnitude(self):
        """ Returns magnitude of vector"""
        return math.hypot(self.x, self.y)


    def scalarProject(self, vec):
        """Returns scalar projection of this vector onto argument

        :param vec: Vector onto which to project this vector
        :type vec: Vector2d
        :return: scalar projection of this vector onto argument
        """
        return self.dot(vec) / vec.magnitude()

class Legacy:
    @staticmethod
    def limit(value):
        """Limit motor values to the -1.0 to +1.0 range."""
        if value > 1.0:
            return 1.0
        if value < -1.0:
            return -1.0
        return value
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
    @staticmethod
    def applyDeadband(value, deadband):
        """Returns 0.0 if the given value is within the specified range around zero. The remaining range
        between the deadband and 1.0 is scaled from 0.0 to 1.0.

        :param value: value to clip
        :param deadband: range around zero
        """
        if abs(value) > deadband:
            if value < 0.0:
                return (value - deadband) / (1.0 - deadband)
            else:
                return (value + deadband) / (1.0 - deadband)
        return 0.0
