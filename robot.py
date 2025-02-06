import wpilib
from lemonlib.drivetrain import SwagDrive,KilloughDrive

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.leftMotor = wpilib.Talon(0)
        self.rightMotor = wpilib.Talon(1)
        self.backMotor = wpilib.Talon(2)
        self.swagDrive = SwagDrive(self.leftMotor, self.rightMotor)
        self.killoughDrive = KilloughDrive(self.leftMotor, self.rightMotor, self.backMotor)

    def teleopPeriodic(self):
        moveValue = wpilib.Joystick(0).getY()
        rotateValue = wpilib.Joystick(0).getZ()
        self.swagDrive.Drive(moveValue, rotateValue)
        self.killoughDrive.driveCartesian(moveValue, rotateValue, 0)
        self.killoughDrive.drivePolar(moveValue, rotateValue, 0)

if __name__ == "__main__":
    wpilib.run(MyRobot)