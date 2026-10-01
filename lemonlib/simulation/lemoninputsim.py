from wpilib import Gamepad, POVDirection
from wpilib.simulation import GenericHIDSim

from ..control import POV_DIRECTIONS, LemonInput


class LemonInputSim(GenericHIDSim):
    """Simulated controller that drives a `LemonInput` on the same port,
    using the same button mapping as `LemonInput(port, variant)`.
    Call `notify_new_data()` after setting values to publish them."""

    def __init__(self, port: int, variant: str = "DriverStation"):
        GenericHIDSim.__init__(self, port)
        lemon_input = LemonInput(port, variant)
        self._using_driverstation_maps = lemon_input.uses_driverstation_maps
        self.button_map = lemon_input.button_map
        self.set_buttons_maximum_index(len(Gamepad.Button.__members__))
        self.set_axes_maximum_index(len(Gamepad.Axis.__members__))
        self.set_povs_maximum_index(1)

    def _set_button(self, ds_button: Gamepad.Button, key: str, value: bool) -> None:
        if self._using_driverstation_maps:
            self.set_raw_button(ds_button.value, value)
        else:
            self.set_raw_button(self.button_map[key].value, value)

    def _set_axis(self, ds_axis: Gamepad.Axis, key: str, value: float) -> None:
        if self._using_driverstation_maps:
            self.set_raw_axis(ds_axis.value, value)
        else:
            self.set_raw_axis(self.button_map[key].value, value)

    def setLeftBumper(self, value: bool) -> None:
        self._set_button(Gamepad.Button.LEFT_BUMPER, "kLeftBumper", value)

    def setRightBumper(self, value: bool) -> None:
        self._set_button(Gamepad.Button.RIGHT_BUMPER, "kRightBumper", value)

    def setStartButton(self, value: bool) -> None:
        self._set_button(Gamepad.Button.START, "kStart", value)

    def setBackButton(self, value: bool) -> None:
        self._set_button(Gamepad.Button.BACK, "kBack", value)

    def setAButton(self, value: bool) -> None:
        self._set_button(Gamepad.Button.FACE_DOWN, "kA", value)

    def setBButton(self, value: bool) -> None:
        self._set_button(Gamepad.Button.FACE_RIGHT, "kB", value)

    def setXButton(self, value: bool) -> None:
        self._set_button(Gamepad.Button.FACE_LEFT, "kX", value)

    def setYButton(self, value: bool) -> None:
        self._set_button(Gamepad.Button.FACE_UP, "kY", value)

    def setLeftStickButton(self, value: bool) -> None:
        self._set_button(Gamepad.Button.LEFT_STICK, "kLeftStick", value)

    def setRightStickButton(self, value: bool) -> None:
        self._set_button(Gamepad.Button.RIGHT_STICK, "kRightStick", value)

    def setRightTriggerAxis(self, value: float) -> None:
        self._set_axis(Gamepad.Axis.RIGHT_TRIGGER, "kRightTrigger", value)

    def setLeftTriggerAxis(self, value: float) -> None:
        self._set_axis(Gamepad.Axis.LEFT_TRIGGER, "kLeftTrigger", value)

    def setLeftX(self, value: float) -> None:
        self._set_axis(Gamepad.Axis.LEFT_X, "kLeftX", value)

    def setLeftY(self, value: float) -> None:
        self._set_axis(Gamepad.Axis.LEFT_Y, "kLeftY", value)

    def setRightX(self, value: float) -> None:
        self._set_axis(Gamepad.Axis.RIGHT_X, "kRightX", value)

    def setRightY(self, value: float) -> None:
        self._set_axis(Gamepad.Axis.RIGHT_Y, "kRightY", value)

    def setPov(self, value: int | POVDirection) -> None:
        """Sets the POV from an angle in degrees (-1 for centered) or a POVDirection."""
        if not isinstance(value, POVDirection):
            value = POV_DIRECTIONS[value]
        self.set_pov(value)
