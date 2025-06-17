from wpilib import DriverStation, RobotBase
from wpilib.interfaces import GenericHID
from wpilib.simulation import GenericHIDSim
from wpiutil import Sendable
from lemonlib.util import Alert, AlertType
from enum import IntEnum
import math

RIGHT_RUMBLE = GenericHID.RumbleType.kRightRumble
LEFT_RUMBLE = GenericHID.RumbleType.kLeftRumble


class LemonInput(GenericHID, Sendable):
    """
    LemonInput is a wrapper class for Xbox and PS5 controllers allowing automatic or manual detection and use in code.
    """

    class xbox_buttons(IntEnum):
        kLeftTrigger = 2
        kLeftX = 0
        kLeftY = 1
        kRightTrigger = 3
        kRightX = 4
        kRightY = 5
        kA = 1
        kB = 2
        kBack = 7
        kLeftBumper = 5
        kLeftStick = 9
        kRightBumper = 6
        kRightStick = 10
        kStart = 8
        kX = 3
        kY = 4

    class ps5_buttons(IntEnum):
        kLeftTrigger = 3
        kLeftX = 0
        kLeftY = 1
        kRightTrigger = 4
        kRightX = 2
        kRightY = 5
        kA = 2
        kB = 3
        kBack = 9
        kLeftBumper = 5
        kLeftStick = 11
        kRightBumper = 6
        kRightStick = 12
        kStart = 10
        kX = 1
        kY = 4

    class legion_buttons(IntEnum):
        kLeftTrigger = 2
        kLeftX = 0
        kLeftY = 1
        kRightTrigger = 3
        kRightX = 4
        kRightY = 5
        kA = 1
        kB = 2
        kBack = 7
        kLeftBumper = 5
        kLeftStick = 9
        kRightBumper = 6
        kRightStick = 10
        kStart = 8
        kX = 3
        kY = 4

    def __init__(self, port: int = None, type: str = "auto"):
        """
        Initializes the control object with the specified port number and type.
        Args:
            port (int, optional): The port number of the controller. If unset,
                chooses first controller matching type.
            type (str, optional): The type of the controller. Defaults to "auto".
                - "auto": Automatically detects the controller type.
                - "Xbox": Forces the controller type to Xbox.
                - "PS5": Forces the controller type to PS5.
        """
        Sendable.__init__(self)

        if port is None:
            port = 0
            while port < 5:
                # assumes DS gives empty string if no joystick at port
                if DriverStation.getJoystickName(port) != "":
                    if type == "auto":
                        break
                    if type == "Xbox" and DriverStation.getJoystickIsXbox(port):
                        break
                    if type == "PS5" and not DriverStation.getJoystickIsXbox(port):
                        break
                port += 1
            else:
                print(f"ERROR: No Joystick found matching type: {type}")
        GenericHID.__init__(self, port)

        if type == "auto":
            if RobotBase.isSimulation():
                self.button_map = self.xbox_buttons
                self.contype = "Xbox"

            elif DriverStation.getJoystickIsXbox(port):
                self.button_map = self.xbox_buttons
                self.contype = "Xbox"

            else:
                self.button_map = self.ps5_buttons
                self.contype = "PS5"

        elif type == "Xbox":
            if RobotBase.isSimulation():
                self.button_map = self.xbox_buttons
                self.contype = "Xbox"
            else:
                self.button_map = self.xbox_buttons
                self.contype = "Xbox"

        elif type == "PS5":
            if RobotBase.isSimulation():
                self.button_map = self.ps5_buttons
                self.contype = "PS5"
            else:
                self.button_map = self.ps5_buttons
                self.contype = "PS5"

    def getType(self):
        """Returns the type of controller (Xbox or PS5)."""
        return self.contype

    """Xbox funcs but still work with PS5 just for ease of use"""

    def getLeftBumper(self):
        """Returns the state of the left bumper button."""
        return self.getRawButton(self.button_map.kLeftBumper.value)

    def getRightBumper(self):
        """
        Returns the state of the right bumper button.

        Returns:
            bool: The state of the right bumper button (pressed or not).
        """
        return self.getRawButton(self.button_map.kRightBumper.value)

    def getStartButton(self):
        """
        Returns the state of the start button.

        Returns:
            bool: The state of the start button (pressed or not).
        """
        return self.getRawButton(self.button_map.kStart.value)

    def getBackButton(self):
        """
        Returns the state of the back button.

        Returns:
            bool: The state of the back button (pressed or not).
        """
        return self.getRawButton(self.button_map.kBack.value)

    def getAButton(self):
        """
        Returns the state of the 'A' button.

        Returns:
            bool: The state of the 'A' button (pressed or not).
        """
        return self.getRawButton(self.button_map.kA.value)

    def getBButton(self):
        """
        Returns the state of the 'B' button.

        Returns:
            bool: The state of the 'B' button (pressed or not).
        """
        return self.getRawButton(self.button_map.kB.value)

    def getXButton(self):
        """
        Returns the state of the 'X' button.

        Returns:
            bool: The state of the 'X' button (pressed or not).
        """
        return self.getRawButton(self.button_map.kX.value)

    def getYButton(self):
        """
        Returns the state of the 'Y' button.

        Returns:
            bool: The state of the 'Y' button (pressed or not).
        """
        return self.getRawButton(self.button_map.kY.value)

    def getLeftStickButton(self):
        """
        Returns the state of the left stick button.

        Returns:
            bool: The state of the left stick button (pressed or not).
        """
        return self.getRawButton(self.button_map.kLeftStick.value)

    def getRightStickButton(self):
        """
        Returns the state of the right stick button.

        Returns:
            bool: The state of the right stick button (pressed or not).
        """
        return self.getRawButton(self.button_map.kRightStick.value)

    def getRightTriggerAxis(self) -> float:
        """
        Returns the state of the right trigger button.

        Returns:
            float: The state of the right trigger button ranging from 0.0 to 1.0.
        """
        return self.getRawAxis(self.button_map.kRightTrigger.value)

    def getLeftTriggerAxis(self) -> float:
        """
        Returns the state of the left trigger button.

        Returns:
            float: The state of the left trigger button ranging from 0.0 to 1.0.
        """
        return self.getRawAxis(self.button_map.kLeftTrigger.value)

    """PS5 funcs still work with Xbox just for ease of use"""

    def getL1Button(self):
        """Returns the state of the L1 button."""
        return self.getRawButton(self.button_map.kLeftBumper.value)

    def getR1Button(self):
        """Returns the state of the R1 button."""
        return self.getRawButton(self.button_map.kRightBumper.value)

    def getOptionsButton(self):
        """Returns the state of the Options button."""
        return self.getRawButton(self.button_map.kStart.value)

    def getCreateButton(self):
        """Returns the state of the Create button."""
        return self.getRawButton(self.button_map.kBack.value)

    def getCrossButton(self):
        """Returns the state of the Cross (X) button."""
        return self.getRawButton(self.button_map.kA.value)

    def getCircleButton(self):
        """Returns the state of the Circle (O) button."""
        return self.getRawButton(self.button_map.kB.value)

    def getSquareButton(self):
        """Returns the state of the Square button."""
        return self.getRawButton(self.button_map.kX.value)

    def getTriangleButton(self):
        """Returns the state of the Triangle button."""
        return self.getRawButton(self.button_map.kY.value)

    def getL3(self):
        """Returns the state of the L3 (left stick) button."""
        return self.getRawButton(self.button_map.kLeftStick.value)

    def getR3(self):
        """Returns the state of the R3 (right stick) button."""
        return self.getRawButton(self.button_map.kRightStick.value)

    def getR2Axis(self) -> float:
        """Returns the state of the R2 trigger."""
        return self.getRawAxis(self.button_map.kRightTrigger.value)

    def getL2Axis(self) -> float:
        """Returns the state of the L2 trigger."""
        return self.getRawAxis(self.button_map.kLeftTrigger.value)

    """Both Xbox and PS5 funcs"""

    def setRumbleLeft(self, value: float):
        """
        Sets the rumble of the controller.

        Args:
            value (float): The value of the rumble to set.
        """
        self.setRumble(LEFT_RUMBLE, value)

    def setRumbleRight(self, value: float):
        """
        Sets the rumble of the controller.

        Args:
            value (float): The value of the rumble to set.
        """
        self.setRumble(RIGHT_RUMBLE, value)

    def getLeftX(self) -> float:
        """
        Returns the X-axis value of the left joystick.

        Returns:
            float: The X-axis value of the left joystick, ranging from -1.0 to 1.0.
        """
        return self.getRawAxis(self.button_map.kLeftX.value)

    def getLeftY(self) -> float:
        """
        Returns the Y-axis value of the left joystick.

        Returns:
            float: The Y-axis value of the left joystick, ranging from -1.0 to 1.0.
        """
        return self.getRawAxis(self.button_map.kLeftY.value)

    def getRightX(self) -> float:
        """
        Returns the X-axis value of the right joystick.

        Returns:
            float: The X-axis value of the right joystick, ranging from -1.0 to 1.0.
        """
        return self.getRawAxis(self.button_map.kRightX.value)

    def getRightY(self) -> float:
        """
        Returns the Y-axis value of the right joystick.

        Returns:
            float: The Y-axis value of the right joystick, ranging from -1.0 to 1.0.
        """
        return self.getRawAxis(self.button_map.kRightY)

    def __pov_xy(self):
        """
        Returns the X and Y values of the POV as a tuple using sin and cos,
        or (0, 0) if the POV is not pressed (-1).

        Returns:
            tuple: The X and Y values of the POV as a tuple.
        """
        pov_value = self.getPOV()

        # If POV is -1 (not pressed), return (0, 0)
        if pov_value == -1:
            return (0, 0)

        # Convert POV value to radians
        radians = math.radians(pov_value)

        # Calculate X and Y using sin and cos
        x = math.cos(radians)
        y = -math.sin(
            radians
        )  # Negative because POV values are typically flipped vertically

        # Return the calculated values
        return (x, y)

    def getPovX(self) -> float:
        """
        Returns the X-axis value of the POV (Point of View) of a joystick.

        Example:
        ```
        controller = SmartController(0)

        if controller.pov() >= 0:
            left_joy_x = controller.pov_x()
            left_joy_y = controller.pov_y()
        ```

        Returns:
            float: The X-axis value of the POV.
        """
        return self.__pov_xy()[0]

    def getPovY(self) -> float:
        """
        Returns the Y-axis value of the POV (Point of View) of a joystick.

        Example:
        ```
        controller = SmartController(0)

        if controller.pov() >= 0:
            left_joy_x = controller.pov_x()
            left_joy_y = controller.pov_y()
        ```

        Returns:
            float: The Y-axis value of the POV.
        """
        return self.__pov_xy()[1]

    def initSendable(self, builder):
        """
        Initializes the sendable for the LemonInput class.

        Args:
            builder: The sendable builder.
        """
        builder.setSmartDashboardType("LemonInput")
        builder.addStringProperty("Type", lambda: self.contype, lambda: None)
        builder.addBooleanProperty(
            "LeftBumper", lambda: self.getLeftBumper(), lambda: None
        )
        builder.addBooleanProperty(
            "RightBumper", lambda: self.getRightBumper(), lambda: None
        )
        builder.addBooleanProperty(
            "StartButton", lambda: self.getStartButton(), lambda: None
        )
        builder.addBooleanProperty(
            "BackButton", lambda: self.getBackButton(), lambda: None
        )
        builder.addBooleanProperty("AButton", lambda: self.getAButton(), lambda: None)
        builder.addBooleanProperty("BButton", lambda: self.getBButton(), lambda: None)
        builder.addBooleanProperty("XButton", lambda: self.getXButton(), lambda: None)
        builder.addBooleanProperty("YButton", lambda: self.getYButton(), lambda: None)
        builder.addBooleanProperty(
            "LStickButton", lambda: self.getLeftStickButton(), lambda: None
        )
        builder.addBooleanProperty(
            "RStickButton", lambda: self.getRightStickButton(), lambda: None
        )
        builder.addDoubleProperty("LeftX", lambda: self.getLeftX(), lambda: None)
        builder.addDoubleProperty("LeftY", lambda: self.getLeftY(), lambda: None)
        builder.addDoubleProperty("RightX", lambda: self.getRightX(), lambda: None)
        builder.addDoubleProperty("RightY", lambda: self.getRightY(), lambda: None)
        builder.addDoubleProperty(
            "RightTrigger", lambda: self.getRightTriggerAxis(), lambda: None
        )
        builder.addDoubleProperty(
            "LeftTrigger", lambda: self.getLeftTriggerAxis(), lambda: None
        )
        builder.addDoubleProperty("POV_X", lambda: self.getPovX(), lambda: None)
        builder.addDoubleProperty("POV_Y", lambda: self.getPovY(), lambda: None)
