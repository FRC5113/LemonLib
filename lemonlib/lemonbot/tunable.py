import functools
import inspect
from magicbot import MagicRobot, tunable, feedback
from wpilib import DriverStation
from typing import Optional, Callable


def is_fms_attached() -> bool:
    """
    Check if the robot is connected to a Field Management System (FMS).
    :return: True if FMS is attached, False otherwise.
    """
    return DriverStation.isFMSAttached()


def fms_feedback(f=None, *, key: Optional[str] = None) -> Callable:
    if f is None:
        return functools.partial(fms_feedback, key=key)

    if not callable(f):
        raise TypeError(f"Illegal use of fms_feedback decorator on non-callable {f!r}")

    @functools.wraps(f)
    def wrapper(self):
        if not is_fms_attached():
            return f(self)
        return 0.0  # Safe default for numerical values

    wrapper._magic_feedback = True
    wrapper._magic_feedback_key = key
    return wrapper
