from typing import Optional

from ..control import LemonInput
from wpilib.event import EventLoop

from commands2 import CommandScheduler
from commands2.button import CommandGenericHID
from commands2.button import Trigger


class CommandLemonInput(CommandGenericHID):
    """
    A version of LemonLib with Trigger factories for command-based.
    """

    _hid: LemonInput

    def __init__(self, port: int):
        """
        Construct an instance of a controller.

        :param port: The port index on the Driver Station that the controller is plugged into.
        """
        super().__init__(port)
        self._hid = LemonInput(port)

    def getHID(self) -> LemonInput:
        """
        Get the underlying GenericHID object.

        :returns: the wrapped GenericHID object
        """
        return self._hid

    def leftBumper(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the left bumper's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the right bumper's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getLeftBumper())

    def rightBumper(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the right bumper's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the left bumper's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getRightBumper())

    def leftStick(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the left stick button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the left stick button's digital signal attached to the
                  given loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getLeftStickButton())

    def rightStick(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the right stick button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the right stick button's digital signal attached to the
                  given loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getRightStickButton())

    def a(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the A button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the A button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getAButton())

    def b(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the B button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the B button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getBButton())

    def x(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the X button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the X button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getXButton())

    def y(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the Y button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the Y button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getYButton())

    def start(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the start button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the start button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getStartButton())

    def back(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the back button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the back button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getBackButton())

    def L2(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the L2 button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the L2 button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getL2Button())

    def R2(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the R2 button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the R2 button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getR2Button())

    def L1(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the L1 button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the L1 button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getL1Button())

    def R1(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the R1 button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the R1 button's digital signal attached to the given
            loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getR1Button())

    def L3(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the L3 button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the L3 button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getL3Button())

    def R3(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the R3 button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the R3 button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getR3Button())

    def square(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the square button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the square button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getSquareButton())

    def cross(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the cross button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the cross button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getCrossButton())

    def triangle(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the triangle button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the triangle button's digital signal attached to the
                  given loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getTriangleButton())

    def circle(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the circle button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the circle button's digital signal attached to the given
            loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getCircleButton())

    def share(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the share button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the share button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getShareButton())

    def PS(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the PS button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the PS button's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getPSButton())

    def options(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the options button's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the options button's digital signal attached to the
                  given loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getOptionsButton())

    def touchpad(self, loop: Optional[EventLoop] = None) -> Trigger:
        """
        Constructs an event instance around the touchpad's digital signal.

        :param loop: the event loop instance to attach the event to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: an event instance representing the touchpad's digital signal attached to the given
                  loop.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getTouchpad())

    def leftTrigger(
        self, threshold: float = 0.5, loop: Optional[EventLoop] = None
    ) -> Trigger:
        """
        Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
        will be true when the axis value is greater than {@code threshold}.

        :param threshold: the minimum axis value for the returned Trigger to be true. This value
                          should be in the range [0, 1] where 0 is the unpressed state of the axis.
        :param loop: the event loop instance to attach the Trigger to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: a Trigger instance that is true when the left trigger's axis exceeds the provided
            threshold, attached to the given event loop
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getLeftTriggerAxis() > threshold)

    def rightTrigger(
        self, threshold: float = 0.5, loop: Optional[EventLoop] = None
    ) -> Trigger:
        """
        Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
        will be true when the axis value is greater than {@code threshold}.

        :param threshold: the minimum axis value for the returned Trigger to be true. This value
                          should be in the range [0, 1] where 0 is the unpressed state of the axis.
        :param loop: the event loop instance to attach the Trigger to, defaults
                     to :func:`commands2.CommandScheduler.getDefaultButtonLoop`

        :returns: a Trigger instance that is true when the right trigger's axis exceeds the provided
                  threshold, attached to the given event loop
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self._hid.getRightTriggerAxis() > threshold)

    def getLeftX(self) -> float:
        """
        Get the X axis value of left side of the controller. Right is positive.

        :returns: The axis value.
        """
        return self._hid.getLeftX()

    def getRightX(self) -> float:
        """
        Get the X axis value of right side of the controller. Right is positive.

        :returns: The axis value.
        """
        return self._hid.getRightX()

    def getLeftY(self) -> float:
        """
        Get the Y axis value of left side of the controller. Back is positive.

        :returns: The axis value.
        """
        return self._hid.getLeftY()

    def getRightY(self) -> float:
        """
        Get the Y axis value of right side of the controller. Back is positive.

        :returns: The axis value.
        """
        return self._hid.getRightY()

    def getLeftTriggerAxis(self) -> float:
        """
        Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the
        range of [0, 1] as opposed to the usual [-1, 1].

        :returns: The axis value.
        """
        return self._hid.getLeftTriggerAxis()

    def getRightTriggerAxis(self) -> float:
        """
        Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the
        range of [0, 1] as opposed to the usual [-1, 1].

        :returns: The axis value.
        """
        return self._hid.getRightTriggerAxis()
