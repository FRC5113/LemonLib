from collections.abc import Callable

from phoenix6.status_code import StatusCode

from .pigeon import LemonPigeon
from .talonfx import LemonTalonFX

__all__ = ["LemonPigeon", "LemonTalonFX"]


def tryUntilOk(attempts: int, command: Callable[[], StatusCode]):
    """
    Utility function to repeatedly attempt a Phoenix 6 command until it returns an OK status code.
    """
    for _ in range(attempts):
        code = command()
        if code.is_ok():
            break
