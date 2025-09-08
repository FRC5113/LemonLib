import functools
import inspect
from magicbot import MagicRobot, tunable, feedback
from wpilib import DriverStation
from typing import Optional, Callable


def fms_feedback(f=None, *, key: Optional[str] = None) -> Callable:
    if f is None:
        return functools.partial(fms_feedback, key=key)

    if not callable(f):
        raise TypeError(f"Illegal use of fms_feedback decorator on non-callable {f!r}")

    @functools.wraps(f)
    def wrapper(self):
        return f(self)
    
    if not DriverStation.isFMSAttached():
        wrapper._magic_feedback = True
        wrapper._magic_feedback_key = key
    return wrapper
