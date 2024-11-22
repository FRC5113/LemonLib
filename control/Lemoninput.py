from wpilib import DriverStation, RobotBase, XboxController, PS5Controller
from wpilib.interfaces import GenericHID


class LemonInput:
    """Wrapper class for Xbox and PS5 controllers allowing automatic detection and use in code."""

    xbox_buttons = {
        "kLeftTrigger": 2,
        "kLeftX": 0,
        "kLeftY": 1,
        "kRightTrigger": 3,
        "kRightX": 4,
        "kRightY": 5,
        "kA": 1,
        "kB": 2,
        "kBack": 7,
        "kLeftBumper": 5,
        "kLeftStick": 9,
        "kRightBumper": 6,
        "kRightStick": 10,
        "kStart": 8,
        "kX": 3,
        "kY": 4,
    }

    ps5_buttons = {
        "kLeftTrigger": 3,
        "kLeftX": 0,
        "kLeftY": 1,
        "kRightTrigger": 4,
        "kRightX": 2,
        "kRightY": 5,
        "kA": 2,
        "kB": 3,
        "kBack": 9,
        "kLeftBumper": 5,
        "kLeftTrigger": 7,
        "kLeftStick": 11,
        "kStart": 10,
        "kPS": 13,
        "kRightBumper": 6,
        "kRightTrigger": 8,
        "kRightStick": 12,
        "kX": 1,
        "kTouchpad": 14,
        "kY": 4,
    }

    def __init__(self, port_number: int):
        self.con = GenericHID(port_number)
        if DriverStation.getJoystickIsXbox(port_number) or RobotBase.isSimulation():
            self.button_map = self.xbox_buttons
            self.contype = "Xbox"
        else:
            self.button_map = self.ps5_buttons
            self.contype = "PS5"

    def type(self):
        """Returns the type of controller (Xbox or PS5)."""
        return self.contype

    def leftbumper(self):
        """Returns the state of the left bumper button."""
        return self.con.getRawButton(self.button_map["kLeftBumper"])

    def rightbumper(self):
        """
        Returns the state of the right bumper button.

        Returns:
            bool: The state of the right bumper button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kRightBumper"])

    def startbutton(self):
        """
        Returns the state of the start button.

        Returns:
            bool: The state of the start button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kStart"])

    def backbutton(self):
        """
        Returns the state of the back button.

        Returns:
            bool: The state of the back button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kBack"])

    def abutton(self):
        """
        Returns the state of the 'A' button.

        Returns:
            bool: The state of the 'A' button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kA"])

    def bbutton(self):
        """
        Returns the state of the 'B' button.

        Returns:
            bool: The state of the 'B' button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kB"])

    def xbutton(self):
        """
        Returns the state of the 'X' button.

        Returns:
            bool: The state of the 'X' button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kX"])

    def ybutton(self):
        """
        Returns the state of the 'Y' button.

        Returns:
            bool: The state of the 'Y' button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kY"])

    def lstickbutton(self):
        """
        Returns the state of the left stick button.

        Returns:
            bool: The state of the left stick button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kLeftStick"])

    def rstickbutton(self):
        """
        Returns the state of the right stick button.

        Returns:
            bool: The state of the right stick button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kRightStick"])

    def leftx(self) -> float:
        """
        Returns the X-axis value of the left joystick.

        Returns:
            float: The X-axis value of the left joystick, ranging from -1.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kLeftX"])

    def lefty(self) -> float:
        """
        Returns the Y-axis value of the left joystick.

        Returns:
            float: The Y-axis value of the left joystick, ranging from -1.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kLeftY"])

    def rightx(self) -> float:
        """
        Returns the X-axis value of the right joystick.

        Returns:
            float: The X-axis value of the right joystick, ranging from -1.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kRightX"])

    def righty(self) -> float:
        """
        Returns the Y-axis value of the right joystick.

        Returns:
            float: The Y-axis value of the right joystick, ranging from -1.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kRightY"])

    def pov(self) -> int:
        """
        Returns the Point of View (POV) value as an integer.

        Returns:
            int: The current POV value. Returns -1 if no POV is pressed.
        """
        return self.con.getPOV()

    def righttrigger(self) -> float:
        """
        Returns the state of the right trigger button.

        Returns:
            float: The state of the right trigger button ranging from 0.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kRightTrigger"])

    def lefttrigger(self) -> float:
        """
        Returns the state of the left trigger button.

        Returns:
            float: The state of the left trigger button ranging from 0.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kLeftTrigger"])

    def __pov_xy(self):
        """
        Returns the X and Y values of the POV as a tuple.

        Returns:
            tuple: The X and Y values of the POV as a tuple.
        """
        pov_value = self.pov()
        pov_mapping = {
            0: (1, 0),
            45: (0.707, -0.707),
            90: (0, -1),
            135: (-0.707, -0.707),
            180: (-1, 0),
            225: (-0.707, 0.707),
            270: (0, 1),
            315: (0.707, 0.707),
        }
        return pov_mapping.get(
            pov_value, (0, 0)
        )  # Return (0, 0) for unmapped POV values

    def pov_x(self) -> float:
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

    def pov_y(self) -> float:
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
