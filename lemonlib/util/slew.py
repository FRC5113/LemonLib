import numpy as np


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

    def calculate(self, input_signal):
        """
        Process input signal through slew rate limiter.

        Args:
            input_signal (float): float Input signal value(s)

        Returns:
            Rate-limited output signal
        """
        # Handle scalar input
        if np.isscalar(input_signal):
            diff = input_signal - self.prev_value
            if diff > 0:
                change = min(diff, self.rising_rate)
            else:
                change = max(diff, -self.falling_rate)
            self.prev_value += change
            return self.prev_value

        # Vectorized processing for arrays (much faster)
        input_signal = np.asarray(input_signal)
        output = np.empty_like(input_signal)

        prev = self.prev_value
        for i, inp in enumerate(input_signal):
            diff = inp - prev
            if diff > 0:
                change = min(diff, self.rising_rate)
            else:
                change = max(diff, -self.falling_rate)
            prev += change
            output[i] = prev

        self.prev_value = prev
        return output

    def lastValue(self):
        """Get the last output value of the limiter."""
        return self.prev_value

    def reset(self, value=0.0):
        """Reset the limiter to a specific value."""
        self.prev_value = value
