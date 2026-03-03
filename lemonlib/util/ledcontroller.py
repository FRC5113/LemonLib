from typing import Tuple

import wpimath.units
from wpilib import AddressableLED, Color, LEDPattern, RobotController, Timer


def _hsv_to_rgb255(h: float, s: float, v: float) -> tuple[int, int, int]:
    """Fast inline HSV→RGB returning integer 0–255 values, no colorsys overhead."""
    if s == 0.0:
        vi = int(v * 255)
        return vi, vi, vi
    h6 = h * 6.0
    i = int(h6) % 6
    f = h6 - int(h6)
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    v255, p255, q255, t255 = int(v * 255), int(p * 255), int(q * 255), int(t * 255)
    if i == 0:
        return v255, t255, p255
    if i == 1:
        return q255, v255, p255
    if i == 2:
        return p255, v255, t255
    if i == 3:
        return p255, q255, v255
    if i == 4:
        return t255, p255, v255
    return v255, p255, q255


class LEDController:
    def __init__(self, pwm_port: int, length: int, start_index: int = 0):
        """
        Initializes the LED controller.

        :param pwm_port: The PWM port the LED strip is connected to.
        :param length: Number of LEDs in the strip.
        """
        self.led = AddressableLED(pwm_port)
        self.length = length
        self._inv_length = 1.0 / length if length > 1 else 0.0
        self._inv_length_m1 = 1.0 / (length - 1) if length > 1 else 0.0
        # Create a list of LEDData objects, one per LED
        self.buffer = [AddressableLED.LEDData(0, 0, 0) for _ in range(length)]
        # Cache bound method references to avoid per-iteration __getattr__ lookups
        self._setrgb = [self.buffer[i].setRGB for i in range(length)]
        self._set_data = self.led.setData
        self.led.setLength(length)
        self._set_data(self.buffer)
        self.solid_color = None

    def apply_pattern(self, pattern: LEDPattern):
        """Applies a wpilib.LEDPattern to the LED buffer and updates the strip."""
        pattern.applyTo(self.buffer, self._write_data)
        self._set_data(self.buffer)

    def _write_data(self, index: int, color: Color):
        self.buffer[index].setLED(color)

    def set_solid_color(self, color: Tuple[int, int, int]):
        """Sets the entire LED strip to a solid color."""
        if color == self.solid_color:
            return
        self.solid_color = color
        r, g, b = color
        setrgb = self._setrgb
        for i in range(self.length):
            setrgb[i](r, g, b)
        self._set_data(self.buffer)

    def set_pixel(self, index: int, color: Tuple[int, int, int]):
        """Sets the color of a single LED pixel."""
        r, g, b = color
        self._setrgb[index](r, g, b)
        self._set_data(self.buffer)

    def set_gradient(
        self, start_color: Tuple[int, int, int], end_color: Tuple[int, int, int]
    ):
        """Custom preset that Sets a gradient from start_color to end_color across the LED strip."""
        start_r, start_g, start_b = start_color
        dr = end_color[0] - start_r
        dg = end_color[1] - start_g
        db = end_color[2] - start_b
        inv = self._inv_length_m1
        setrgb = self._setrgb
        for i in range(self.length):
            f = i * inv
            setrgb[i](
                int(start_r + f * dr), int(start_g + f * dg), int(start_b + f * db)
            )
        self._set_data(self.buffer)
        self.solid_color = None

    def static_rainbow(self, offset: int = 0):
        """Custom preset that Creates a rainbow effect across the LED strip.

        The offset parameter (in degrees) can be used to animate the rainbow.
        """
        inv = self._inv_length
        offset_norm = offset / 360.0
        setrgb = self._setrgb
        for i in range(self.length):
            hue = (i * inv + offset_norm) % 1.0
            setrgb[i](*_hsv_to_rgb255(hue, 1.0, 0.5))
        self._set_data(self.buffer)
        self.solid_color = None

    def scolling_rainbow(self, speed: float = 1):
        """Custom preset that Creates a scrolling rainbow effect across the LED strip."""
        inv = self._inv_length
        time_offset = ((RobotController.getTime() / 100000) * speed) / 360.0
        setrgb = self._setrgb
        for i in range(self.length):
            hue = (i * inv + time_offset) % 1.0
            setrgb[i](*_hsv_to_rgb255(hue, 1.0, 0.5))
        self._set_data(self.buffer)
        self.solid_color = None

    def move_across(
        self, color: Tuple[int, int, int], size: int = 1, hertz: wpimath.units.hertz = 1
    ):
        """Moves a block of LEDs across the strip using Timer.getFPGATimestamp() for timing."""
        length = self.length
        setrgb = self._setrgb
        r, g, b = color
        position = int(Timer.getFPGATimestamp() * hertz) % length

        # Clear buffer
        for j in range(length):
            setrgb[j](0, 0, 0)

        # Light up the moving block
        for j in range(size):
            setrgb[(position - j) % length](r, g, b)

        self._set_data(self.buffer)

    def move_across_multi(
        self,
        colors: list[tuple[int, int, int]] | tuple[int, int, int],
        size: int = 1,
        hertz: wpimath.units.hertz = 1,
    ):
        """Moves a fixed-size multicolor block across the strip using Timer.getFPGATimestamp() for timing."""
        length = self.length
        setrgb = self._setrgb

        # Handle single color input
        if isinstance(colors[0], int):
            colors = [colors]

        num_colors = len(colors)
        inv_size = 1.0 / size if size > 0 else 0.0
        position = int((Timer.getFPGATimestamp() * hertz) % length)

        # Clear all LEDs
        for j in range(length):
            setrgb[j](0, 0, 0)

        # Fill the moving block with a color pattern distributed across its size
        for i in range(size):
            color_index = int(i * inv_size * num_colors) % num_colors
            r, g, b = colors[color_index]
            setrgb[(position + i) % length](r, g, b)

        self._set_data(self.buffer)
        self.solid_color = None

    def clear(self):
        """Turns off all LEDs."""
        self.solid_color = None
        self.set_solid_color((0, 0, 0))
