__version__ = "2027.0.0a7"

from collections.abc import Callable

from phoenix6.status_code import StatusCode

from .control import LemonInput


def try_until_ok(attempts: int, command: Callable[[], StatusCode]):
    """
    Utility function to repeatedly attempt a Phoenix 6 command until it returns an OK status code.
    """
    for _ in range(attempts):
        code = command()
        if code.is_ok():
            break


__all__ = [
    "LemonInput",
    "try_until_ok",
]
