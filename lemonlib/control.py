import math
from enum import IntEnum

from wpilib import DriverStationBackend, Gamepad, GenericHID, POVDirection

RIGHT_RUMBLE = GenericHID.RumbleType.RIGHT_RUMBLE
LEFT_RUMBLE = GenericHID.RumbleType.LEFT_RUMBLE

# POV directions as angles in degrees (clockwise from up), -1 when centered
POV_ANGLES = {
    POVDirection.CENTER: -1,
    POVDirection.UP: 0,
    POVDirection.UP_RIGHT: 45,
    POVDirection.RIGHT: 90,
    POVDirection.DOWN_RIGHT: 135,
    POVDirection.DOWN: 180,
    POVDirection.DOWN_LEFT: 225,
    POVDirection.LEFT: 270,
    POVDirection.UP_LEFT: 315,
}
POV_DIRECTIONS = {angle: direction for direction, angle in POV_ANGLES.items()}

_XBOX_TYPES = {GenericHID.HIDType.XBOX_360.value, GenericHID.HIDType.XBOX_ONE.value}
_PS_TYPES = {GenericHID.HIDType.PS4.value, GenericHID.HIDType.PS5.value}
_UNRECOGNIZED_TYPES = {
    GenericHID.HIDType.UNKNOWN.value,
    GenericHID.HIDType.STANDARD.value,
}


class LemonInput:
    """
    LemonInput is a wrapper class for Xbox
    and PS5 controllers allowing automatic
    or manual detection and use in code.
    """

    class xbox_buttons(IntEnum):
        """Raw (0-indexed) layout of an Xbox controller on the NI Driver Station."""

        kLeftTrigger = 2
        kLeftX = 0
        kLeftY = 1
        kRightTrigger = 3
        kRightX = 4
        kRightY = 5
        kA = 0
        kB = 1
        kBack = 6
        kLeftBumper = 4
        kLeftStick = 8
        kRightBumper = 5
        kRightStick = 9
        kStart = 7
        kX = 2
        kY = 3

    class ps5_buttons(IntEnum):
        """Raw (0-indexed) layout of a PS5 controller on the NI Driver Station."""

        kLeftTrigger = 3
        kLeftX = 0
        kLeftY = 1
        kRightTrigger = 4
        kRightX = 2
        kRightY = 5
        kA = 1
        kB = 2
        kBack = 8
        kLeftBumper = 4
        kLeftStick = 10
        kRightBumper = 5
        kRightStick = 11
        kStart = 9
        kX = 0
        kY = 3

    def __init__(self, port: int | None = None, variant: str = "DriverStation"):
        """
        Initializes the control object with the specified port number and type.
        Args:
            port (int, optional): The port number of the controller.
                If unset, chooses first controller matching type not using driverstation maps.
            variant (str, optional): The type of the controller. Defaults to "DriverStation".
                - "auto": Automatically detects the controller type.
                - "DriverStation": Uses the Driver Station's standard gamepad mapping.
                - "Xbox": Forces the controller type to Xbox not using driverstation maps.
                - "PS5": Forces the controller type to PS5 not using driverstation maps.
        """

        self._using_driverstation_maps = False
        self.button_map = self.xbox_buttons

        if port is None:
            if variant == "DriverStation":
                raise ValueError(
                    "DriverStation type selected but no port specified. Please specify a port."
                )
            port = self._find_port(variant)

        match variant:
            case "auto":
                self._auto_type(port)
            case "DriverStation":
                self.contype = "DriverStation"
                self._using_driverstation_maps = True
            case "Xbox":
                self.contype = "Xbox"
                self.button_map = self.xbox_buttons
            case "PS5":
                self.contype = "PS5"
                self.button_map = self.ps5_buttons
            case _:
                self.contype = "Unknown"
                self.button_map = self.xbox_buttons

        self.port = port
        self.gamepad = Gamepad(port)

    def _find_port(self, variant: str) -> int:
        """Returns the first connected port matching the variant."""
        for port in range(DriverStationBackend.JOYSTICK_PORTS):
            if not DriverStationBackend.is_joystick_connected(port):
                continue
            if (
                (variant == "auto")
                or (variant == "Xbox" and self._is_Xbox(port))
                or (variant == "PS5" and self._is_PS5(port))
            ):
                return port
        print(f"ERROR: No Joystick found matching type: {variant}")
        return 0

    def _auto_type(self, port: int):
        if self._is_Xbox(port):
            self.contype = "Xbox"
            self.button_map = self.xbox_buttons
        elif self._is_PS5(port):
            self.contype = "PS5"
            self.button_map = self.ps5_buttons
        else:
            self.contype = "Unknown"
            self.button_map = self.xbox_buttons

        # A Driver Station that recognizes the controller type remaps it to
        # the standard gamepad layout, so the raw maps no longer apply
        gamepad_type = DriverStationBackend.get_joystick_gamepad_type(port)
        if gamepad_type not in _UNRECOGNIZED_TYPES:
            self._using_driverstation_maps = True

    def _is_Xbox(self, port: int) -> bool:
        """
        Checks if the controller at the specified port is an Xbox controller.
        Args:
            port (int): The port number to check.
        """
        if DriverStationBackend.get_joystick_gamepad_type(port) in _XBOX_TYPES:
            return True
        joystick_name = DriverStationBackend.get_joystick_name(port).lower()
        xbox_variants = ["xbox", "x-box", "360", "series x", "series s"]
        return any(variant in joystick_name for variant in xbox_variants)

    def _is_PS5(self, port: int) -> bool:
        """
        Checks if the controller at the specified port is a PS5 controller.
        Args:
            port (int): The port number to check.
        """
        if DriverStationBackend.get_joystick_gamepad_type(port) in _PS_TYPES:
            return True
        joystick_name = DriverStationBackend.get_joystick_name(port).lower()
        ps5_variants = ["ps5", "playstation 5", "dualsense"]
        return any(variant in joystick_name for variant in ps5_variants)

    def getType(self):
        """Returns the type of controller (Xbox or PS5)."""
        return self.contype

    @property
    def uses_driverstation_maps(self) -> bool:
        """True if inputs are read using the Driver Station's gamepad mapping."""
        return self._using_driverstation_maps

    def getRawButton(self, button: int) -> bool:
        """Returns the state of a raw (0-indexed) button."""
        return self.gamepad.get_hid().get_raw_button(button)

    def getRawAxis(self, axis: int) -> float:
        """Returns the value of a raw (0-indexed) axis."""
        return self.gamepad.get_hid().get_raw_axis(axis)

    def getPOV(self) -> int:
        """Returns the POV angle in degrees (clockwise from up), or -1 if not pressed."""
        return POV_ANGLES.get(self.gamepad.get_hid().get_pov(), -1)

    def setRumble(self, type: GenericHID.RumbleType, value: float):
        """Sets the rumble output of the controller."""
        self.gamepad.set_rumble(type, value)

    def _button(self, ds_button: Gamepad.Button, key: str) -> bool:
        if self._using_driverstation_maps:
            return self.gamepad.get_button(ds_button)
        return self.getRawButton(self.button_map[key].value)

    def _axis(self, ds_axis: Gamepad.Axis, key: str) -> float:
        if self._using_driverstation_maps:
            return self.gamepad.get_axis(ds_axis)
        return self.getRawAxis(self.button_map[key].value)

    """Xbox funcs but still work with PS5 just for ease of use"""

    def getLeftBumper(self):
        """Returns the state of the left bumper button."""
        return self._button(Gamepad.Button.LEFT_BUMPER, "kLeftBumper")

    def getRightBumper(self):
        """
        Returns the state of the right bumper button.

        Returns:
            bool: The state of the right bumper button (pressed or not).
        """
        return self._button(Gamepad.Button.RIGHT_BUMPER, "kRightBumper")

    def getStartButton(self):
        """
        Returns the state of the start button.

        Returns:
            bool: The state of the start button (pressed or not).
        """
        return self._button(Gamepad.Button.START, "kStart")

    def getBackButton(self):
        """
        Returns the state of the back button.

        Returns:
            bool: The state of the back button (pressed or not).
        """
        return self._button(Gamepad.Button.BACK, "kBack")

    def getAButton(self):
        """
        Returns the state of the 'A' button.

        Returns:
            bool: The state of the 'A' button (pressed or not).
        """
        return self._button(Gamepad.Button.FACE_DOWN, "kA")

    def getBButton(self):
        """
        Returns the state of the 'B' button.

        Returns:
            bool: The state of the 'B' button (pressed or not).
        """
        return self._button(Gamepad.Button.FACE_RIGHT, "kB")

    def getXButton(self):
        """
        Returns the state of the 'X' button.

        Returns:
            bool: The state of the 'X' button (pressed or not).
        """
        return self._button(Gamepad.Button.FACE_LEFT, "kX")

    def getYButton(self):
        """
        Returns the state of the 'Y' button.

        Returns:
            bool: The state of the 'Y' button (pressed or not).
        """
        return self._button(Gamepad.Button.FACE_UP, "kY")

    def getLeftStickButton(self):
        """
        Returns the state of the left stick button.

        Returns:
            bool: The state of the left stick button (pressed or not).
        """
        return self._button(Gamepad.Button.LEFT_STICK, "kLeftStick")

    def getRightStickButton(self):
        """
        Returns the state of the right stick button.

        Returns:
            bool: The state of the right stick button (pressed or not).
        """
        return self._button(Gamepad.Button.RIGHT_STICK, "kRightStick")

    def getRightTriggerAxis(self) -> float:
        """
        Returns the state of the right trigger button.

        Returns:
            float: The state of the right trigger button ranging from 0.0 to 1.0.
        """
        return self._axis(Gamepad.Axis.RIGHT_TRIGGER, "kRightTrigger")

    def getLeftTriggerAxis(self) -> float:
        """
        Returns the state of the left trigger button.

        Returns:
            float: The state of the left trigger button ranging from 0.0 to 1.0.
        """
        return self._axis(Gamepad.Axis.LEFT_TRIGGER, "kLeftTrigger")

    """PS5 funcs still work with Xbox just for ease of use"""

    def getL1Button(self):
        """Returns the state of the L1 button."""
        return self.getLeftBumper()

    def getR1Button(self):
        """Returns the state of the R1 button."""
        return self.getRightBumper()

    def getOptionsButton(self):
        """Returns the state of the Options button."""
        return self.getStartButton()

    def getCreateButton(self):
        """Returns the state of the Create button."""
        return self.getBackButton()

    def getCrossButton(self):
        """Returns the state of the Cross (X) button."""
        return self.getAButton()

    def getCircleButton(self):
        """Returns the state of the Circle (O) button."""
        return self.getBButton()

    def getSquareButton(self):
        """Returns the state of the Square button."""
        return self.getXButton()

    def getTriangleButton(self):
        """Returns the state of the Triangle button."""
        return self.getYButton()

    def getL3(self):
        """Returns the state of the L3 (left stick) button."""
        return self.getLeftStickButton()

    def getR3(self):
        """Returns the state of the R3 (right stick) button."""
        return self.getRightStickButton()

    def getR2Axis(self) -> float:
        """Returns the state of the R2 trigger."""
        return self.getRightTriggerAxis()

    def getL2Axis(self) -> float:
        """Returns the state of the L2 trigger."""
        return self.getLeftTriggerAxis()

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
        return self._axis(Gamepad.Axis.LEFT_X, "kLeftX")

    def getLeftY(self) -> float:
        """
        Returns the Y-axis value of the left joystick.

        Returns:
            float: The Y-axis value of the left joystick, ranging from -1.0 to 1.0.
        """
        return self._axis(Gamepad.Axis.LEFT_Y, "kLeftY")

    def getRightX(self) -> float:
        """
        Returns the X-axis value of the right joystick.

        Returns:
            float: The X-axis value of the right joystick, ranging from -1.0 to 1.0.
        """
        return self._axis(Gamepad.Axis.RIGHT_X, "kRightX")

    def getRightY(self) -> float:
        """
        Returns the Y-axis value of the right joystick.

        Returns:
            float: The Y-axis value of the right joystick, ranging from -1.0 to 1.0.
        """
        return self._axis(Gamepad.Axis.RIGHT_Y, "kRightY")

    def _pov_xy(self):
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
        controller = LemonInput(0)

        if controller.getPOV() >= 0:
            pov_x = controller.getPovX()
            pov_y = controller.getPovY()
        ```

        Returns:
            float: The X-axis value of the POV.
        """
        return self._pov_xy()[0]

    def getPovY(self) -> float:
        """
        Returns the Y-axis value of the POV (Point of View) of a joystick.

        Example:
        ```
        controller = LemonInput(0)

        if controller.getPOV() >= 0:
            pov_x = controller.getPovX()
            pov_y = controller.getPovY()
        ```

        Returns:
            float: The Y-axis value of the POV.
        """
        return self._pov_xy()[1]
