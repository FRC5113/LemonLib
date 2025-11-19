from wpilib import AddressableLED, LEDPattern, Color, RobotController, Timer
import colorsys
from typing import Tuple
import wpimath.units


class LEDController:
    def __init__(self, pwm_port: int, length: int):
        """
        Initializes the LED controller.

        :param pwm_port: The PWM port the LED strip is connected to.
        :param length: Number of LEDs in the strip.
        """
        self.led = AddressableLED(pwm_port)
        self.length = length
        # Create a list of LEDData objects, one per LED
        self.buffer = [AddressableLED.LEDData(0, 0, 0) for _ in range(length)]
        self.led.setLength(length)
        self.led.setData(self.buffer)
        self.led.start()
        self.solid_color = None

    def apply_pattern(self, pattern: LEDPattern):
        """Applies a wpilib.LEDPattern to the LED buffer and updates the strip."""
        pattern.applyTo(self.buffer, self._write_data)
        self.led.setData(self.buffer)

    def _write_data(self, index: int, color: Color):
        self.buffer[index].setLED(color)

    def set_solid_color(self, color: Tuple[int, int, int]):
        """Sets the entire LED strip to a solid color."""
        if color == self.solid_color:
            return
        self.solid_color = color
        r, g, b = color
        for i in range(self.length):
            self.buffer[i].setRGB(r, g, b)
        self.led.setData(self.buffer)

    def set_pixel(self, index: int, color: Tuple[int, int, int]):
        """Sets the color of a single LED pixel."""
        r, g, b = color
        self.buffer[index].setRGB(r, g, b)
        self.led.setData(self.buffer)

    def set_gradient(
        self, start_color: Tuple[int, int, int], end_color: Tuple[int, int, int]
    ):
        """Custom preset that Sets a gradient from start_color to end_color across the LED strip."""
        start_r, start_g, start_b = start_color
        end_r, end_g, end_b = end_color
        for i in range(self.length):
            factor = i / (self.length - 1) if self.length > 1 else 0
            r = int(start_r + factor * (end_r - start_r))
            g = int(start_g + factor * (end_g - start_g))
            b = int(start_b + factor * (end_b - start_b))
            self.buffer[i].setRGB(r, g, b)
        self.led.setData(self.buffer)
        self.solid_color = None

    def static_rainbow(self, offset: int = 0):
        """Custom preset that Creates a rainbow effect across the LED strip.

        The offset parameter (in degrees) can be used to animate the rainbow.
        """
        for i in range(self.length):
            # Normalize index to [0,1] and add the offset (converted from degrees)
            hue = ((i / self.length) + (offset / 360.0)) % 1.0
            # Convert HSV to RGB; using full saturation and 50% brightness
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 0.5)
            self.buffer[i].setRGB(int(r * 255), int(g * 255), int(b * 255))
        self.led.setData(self.buffer)
        self.solid_color = None

    def scolling_rainbow(self, speed: float = 1):
        """Custom preset that Creates a rainbow effect across the LED strip.

        The offset parameter (in degrees) can be used to animate the rainbow.
        """

        for i in range(self.length):
            # Normalize index to [0,1] and add the offset (converted from degrees)
            hue = (
                (i / self.length)
                + (((RobotController.getTime() / 100000) * speed) / 360.0)
            ) % 1.0
            # Convert HSV to RGB; using full saturation and 50% brightness
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 0.5)
            self.buffer[i].setRGB(int(r * 255), int(g * 255), int(b * 255))
        self.led.setData(self.buffer)
        self.solid_color = None

    def move_across(
        self, color: Tuple[int, int, int], size: int = 1, hertz: wpimath.units.hertz = 1
    ):
        """Moves a block of LEDs across the strip using RobotController.getTime() for timing."""
        r, g, b = color

        # Get the current time
        current_time = Timer.getFPGATimestamp()

        # Compute the current position with slower movement
        position = int(current_time * hertz) % self.length

        # Clear buffer first
        for j in range(self.length):
            self.buffer[j].setRGB(0, 0, 0)

        # Light up the section
        for j in range(size):
            index = (position - j) % self.length
            self.buffer[index].setRGB(r, g, b)

        self.led.setData(self.buffer)
        self.solid_color = None

    def clear(self):
        """Turns off all LEDs."""
        self.solid_color = None
        self.set_solid_color((0, 0, 0))
