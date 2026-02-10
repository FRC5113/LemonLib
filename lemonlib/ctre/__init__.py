from .pigeon import LemonPigeon
from .talonfx import LemonTalonFX

from typing import Callable

from phoenix6.status_code import StatusCode

__all__ = ["LemonPigeon", "LemonTalonFX"]


def tryUntilOk(attempts: int, command: Callable[[], StatusCode]):
    for _ in range(attempts):
        code = command()
        if code.is_ok():
            break
