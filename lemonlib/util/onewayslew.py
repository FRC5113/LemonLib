from typing import overload
from wpilib import RobotController

class OneWaySlewRateLimiter:
    """
    A one-direction slew-rate limiter that only limits the rate of decrease.
    Increases are applied immediately without any limiting.

    This mirrors the API style of wpimath.filter.SlewRateLimiter but with
    the following behavioral differences:
        - Only negative direction (falling) is limited.
        - Positive direction (rising) is unlimited.

    Suitable for controlling values where overshoot on the positive side
    is acceptable, but large negative steps must be smoothed.
    """

    __slots__ = ("fallRate", "prev")


    def __init__(self, rateLimit: float) -> None:
        """
        Creates a new OneWaySlewRateLimiter where only the negative rate limit
        is used. The provided rateLimit must be positive, and internally the
        negative rate limit becomes -rateLimit.

        :param rateLimit: Positive rate limit magnitude.
        """
        if rateLimit <= 0:
            raise ValueError("rateLimit must be positive")
        self.fallRate = -abs(rateLimit)  # negative per second
        self.prev = 0.0

    def calculate(self, input: float) -> float:
        """
        Filters the input value, limiting only the negative (falling)
        slew rate. Positive changes are applied immediately.

        :param input: Input value.
        :param dt:    Time step in seconds.

        :returns: The filtered value.
        """
        prev = self.prev
        time = RobotController.getFPGATime() * 1e-6
        if input >= prev:
            self.prev = input
            return input

        max_neg_delta = self.fallRate * time 
        candidate = prev + max_neg_delta    

        if candidate > input:
            self.prev = candidate
            return candidate

        self.prev = input
        return input

    def lastValue(self) -> float:
        """
        Returns the last output value produced by calculate().

        :returns: The last value.
        """
        return self.prev

    def reset(self, value: float) -> None:
        """
        Resets the limiter's output to the given value. No rate limiting
        is applied during a reset.

        :param value: New stored value.
        """
        self.prev = float(value)
