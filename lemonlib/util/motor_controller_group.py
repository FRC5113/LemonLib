from wpilib.interfaces import MotorController


class MotorControllerGroup(MotorController):
    isInverted: bool = False

    instances: int = 0

    def __init__(self, motorController: MotorController, *args: MotorController):
        """Create a new MotorControllerGroup with the provided MotorControllers.

        :param args: MotorControllers to add
        """

        MotorController.__init__(self)

        self._motor_controllers = [motorController] + list(args)

        self.isInverted = False

        MotorControllerGroup.instances += 1

    def set(self, speed: float):
        for mc in self._motor_controllers:
            mc.set(-speed if self.isInverted else speed)

    def setVoltage(self, outputVolts: float):
        for mc in self._motor_controllers:
            mc.setVoltage(-outputVolts if self.isInverted else outputVolts)

    def get(self):
        if len(self._motor_controllers) > 0:
            speed = self._motor_controllers[0].get()
            return -speed if self.isInverted else speed
        return 0.0

    def setInverted(self, isInverted: bool) -> None:
        self.isInverted = isInverted

    def getInverted(self) -> bool:
        return self.isInverted

    def disable(self) -> None:
        for mc in self._motor_controllers:
            mc.disable()

    def stopMotor(self) -> None:
        for mc in self._motor_controllers:
            mc.stopMotor()
