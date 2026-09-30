import functools
from collections.abc import Callable

from wpilib import DriverStation


def fms_feedback(f=None, *, key: str | None = None) -> Callable:
    if f is None:
        return functools.partial(fms_feedback, key=key)

    if not callable(f):
        raise TypeError(f"Illegal use of fms_feedback decorator on non-callable {f!r}")

    @functools.wraps(f)
    def wrapper(self):
        return f(self)

    if not DriverStation.isFMSAttached():
        wrapper._magic_feedback = (key, None)
    return wrapper
