from typing import Callable
from wpilib import DriverStation
from magicbot import feedback
from pathlib import Path


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Restrict value between min_value and max_value."""
    return max(min(value, max_value), min_value)


def is_red() -> bool:
    return DriverStation.getAlliance() == DriverStation.Alliance.kRed


def get_file(path: str) -> str:
    """Get a file from directory."""
    return Path(__file__).parent.resolve() / f"{path}"


def curve(
    mapping: Callable[[float], float],
    offset: float,
    deadband: float,
    max_mag: float,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    """Return a function that applies a curve to an input.

    Arguments:
    mapping -- maps input to output
    offset -- added to output, even if the input is deadbanded
    deadband -- when the input magnitude is less than this,
        the input is treated as zero
    max_mag -- restricts the output magnitude to a maximum.
        If this is 0, no restriction is applied.
    absolute_offset -- If true, applies offset always (even when deadbanded),
        If false, adds sign(input_val) * offset or 0 in the deadband
    """

    def f(input_val: float) -> float:
        """Apply a curve to an input. Be sure to call this function to get an output, not curve."""
        if abs(input_val) < deadband:
            return offset if absolute_offset else 0
        applied_offset = (1 if absolute_offset else abs(input_val) / input_val) * offset
        output_val = mapping(input_val) + applied_offset
        if max_mag == 0:
            return output_val
        else:
            return clamp(output_val, -max_mag, max_mag)

    return f


def linear_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(lambda x: scalar * x, offset, deadband, max_mag, absolute_offset)


def ollie_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(
        lambda x: scalar * x * abs(x), offset, deadband, max_mag, absolute_offset
    )


def cubic_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(lambda x: scalar * x**3, offset, deadband, max_mag, absolute_offset)


def SnapX(x, y) -> float:
    if abs(x) > abs(y):
        return x
    return 0.0


def SnapY(x, y) -> float:
    if abs(y) > abs(x):
        return y
    return 0.0
