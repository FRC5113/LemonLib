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

        Uses rising_rate when the magnitude of the signal is increasing
        (accelerating) and falling_rate when it is decreasing (decelerating),
        so that the limiter behaves correctly for both positive and negative
        velocity commands.

        Args:
            input_signal (float): Input signal value

        Returns:
            Rate-limited output signal
        """
        current_time = self.get_time_seconds()
        time_diff = current_time - self.prev_time
        self.prev_time = current_time

        diff = input_signal - self.prev_value

        # Pick rate based on whether we are accelerating (magnitude increasing)
        # or decelerating (magnitude decreasing).
        accelerating = abs(input_signal) > abs(self.prev_value)
        rate = self.rising_rate if accelerating else self.falling_rate

        max_change = rate * time_diff
        if diff > max_change:
            change = max_change
        elif diff < -max_change:
            change = -max_change
        else:
            change = diff

        self.prev_value += change
        return self.prev_value

    def lastValue(self):
        """Get the last output value of the limiter."""
        return self.prev_value

    def reset(self, value=0.0):
        """Reset the limiter to a specific value."""
        self.prev_value = value
