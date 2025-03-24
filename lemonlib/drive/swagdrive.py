import wpilib.drive
from wpilib.drive import DifferentialDrive
from lemonlib.smart.preference import SmartPreference
from wpiutil import Sendable


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
        builder.addDoubleProperty(
            "Swag Period", lambda: self.swagPeriod, lambda _: None
        )
        builder.addDoubleProperty(
            "Rotate Diff", lambda: self.rotatediff, lambda _: None
        )
        builder.addDoubleProperty("Move Diff", lambda: self.movediff, lambda _: None)
        self.robotDrive.initSendable(builder)
