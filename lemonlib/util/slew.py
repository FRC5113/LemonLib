from wpilib import RobotController


class AsymmetricSlewLimiter:
    """
    slew rate limiter with separate rising and falling rates.
    """

    def __init__(self, rising_rate, falling_rate, initial_value=0.0):
        """
        Args:
            rising_rate (float): Maximum rate of change when signal is increasing (units/sample)
            falling_rate (float): Maximum rate of change when signal is decreasing (units/sample)
            initial_value (float, optional): Starting value of the limiter (default: 0.
        """
        self.rising_rate = abs(rising_rate)
        self.falling_rate = abs(falling_rate)
        self.prev_value = initial_value
        self.prev_time = RobotController.getTime() / 1e6

    def get_time_seconds(self):
        """Get the time in seconds"""
        return RobotController.getTime() / 1e6

    def calculate(self, input_signal):
        """
        Process input signal through slew rate limiter.

        Args:
            input_signal (float): float Input signal value(s)

        Returns:
            Rate-limited output signal
        """
        current_time = self.get_time_seconds()
        time_diff = current_time - self.prev_time
        diff = input_signal - self.prev_value
        if diff > 0:
            change = min(diff, self.rising_rate * time_diff)
        else:
            change = max(diff, -self.falling_rate * time_diff)
        self.prev_value += change
        return self.prev_value

    def lastValue(self):
        """Get the last output value of the limiter."""
        return self.prev_value

    def reset(self, value=0.0):
        """Reset the limiter to a specific value."""
        self.prev_value = value
