from wpilib.interfaces import GenericHID
from wpilib.simulation import GenericHIDSim
from ..control import LemonInput


class LemonInputSim(GenericHIDSim):
    def __init__(self, port: int):
        GenericHIDSim.__init__(self, port)
        if LemonInput(port).getType() == "Xbox":
            self.button_map = LemonInput.xbox_buttons
        else:
            self.button_map = LemonInput.ps5_buttons

    def setLeftBumper(self, value: bool) -> None:
        return self.setRawButton(self.button_map.kLeftBumper.value, value)

    def setRightBumper(self, value: bool) -> None:
        return self.setRawButton(self.button_map.kRightBumper.value, value)

    def setStartButton(self, value: bool) -> None:
        return self.setRawButton(self.button_map.kStart.value, value)

    def setBackButton(self, value: bool) -> None:
        return self.setRawButton(self.button_map.kBack.value, value)

    def setAButton(self, value: bool) -> None:
        return self.setRawButton(self.button_map.kA.value, value)

    def setBButton(self, value: bool) -> None:
        return self.setRawButton(self.button_map.kB.value, value)

    def setXButton(self, value: bool) -> None:
        return self.setRawButton(self.button_map.kX.value, value)

    def setYButton(self, value: bool) -> None:
        return self.setRawButton(self.button_map.kY.value, value)

    def setLeftStickButton(self, value: bool) -> None:
        return self.setRawButton(self.button_map.kLeftStick.value, value)

    def setRightStickButton(self, value: bool) -> None:
        return self.setRawButton(self.button_map.kRightStick.value, value)

    def setRightTriggerAxis(self, value: float) -> None:
        return self.setRawAxis(self.button_map.kRightTrigger.value, value)

    def setLeftTriggerAxis(self, value: float) -> None:
        return self.setRawAxis(self.button_map.kLeftTrigger.value, value)

    def setLeftX(self, value: float) -> None:
        return self.setRawAxis(self.button_map.kLeftX.value, value)

    def setLeftY(self, value: float) -> None:
        return self.setRawAxis(self.button_map.kLeftY.value, value)

    def setRightX(self, value: float) -> None:
        return self.setRawAxis(self.button_map.kRightX.value, value)

    def setRightY(self, value: float) -> None:
        return self.setRawAxis(self.button_map.kRightY.value, value)

    def setPov(self, value: int) -> None:
        return self.setPOV(value)
