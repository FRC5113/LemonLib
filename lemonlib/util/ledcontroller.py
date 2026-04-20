import colorsys
from typing import Tuple

import wpimath.units
from wpilib import AddressableLED, Color, LEDPattern, RobotController, Timer


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
        self.led.setStart(0)
        self.solid_color = None
        self._move_frame_initialized = False
        self._last_lit_indices: list[int] = []

    def _reset_move_cache(self):
        self._move_frame_initialized = False
        self._last_lit_indices.clear()

    def _clear_all(self):
        for led in self.buffer:
            led.setRGB(0, 0, 0)

    def apply_pattern(self, pattern: LEDPattern):
        """Applies a wpilib.LEDPattern to the LED buffer and updates the strip."""
        self._reset_move_cache()
        self.solid_color = None
        pattern.applyTo(self.buffer, self._write_data)
        self.led.setData(self.buffer)

    def _write_data(self, index: int, color: Color):
        self.buffer[index].setLED(color)

    def set_solid_color(self, color: Tuple[int, int, int]):
        """Sets the entire LED strip to a solid color."""
        if color == self.solid_color:
            return
        self._reset_move_cache()
        self.solid_color = color
        r, g, b = color
        for led in self.buffer:
            led.setRGB(r, g, b)
        self.led.setData(self.buffer)

    def set_pixel(self, index: int, color: Tuple[int, int, int]):
        """Sets the color of a single LED pixel."""
        self._reset_move_cache()
        self.solid_color = None
        r, g, b = color
        self.buffer[index].setRGB(r, g, b)
        self.led.setData(self.buffer)

    def set_gradient(
        self, start_color: Tuple[int, int, int], end_color: Tuple[int, int, int]
    ):
        """Custom preset that Sets a gradient from start_color to end_color across the LED strip."""
        self._reset_move_cache()
        start_r, start_g, start_b = start_color
        end_r, end_g, end_b = end_color
        length = self.length
        buffer = self.buffer
        for i in range(length):
            factor = i / (length - 1) if length > 1 else 0
            r = int(start_r + factor * (end_r - start_r))
            g = int(start_g + factor * (end_g - start_g))
            b = int(start_b + factor * (end_b - start_b))
            buffer[i].setRGB(r, g, b)
        self.led.setData(buffer)
        self.solid_color = None

    def static_rainbow(self, offset: int = 0):
        """Custom preset that Creates a rainbow effect across the LED strip.

        The offset parameter (in degrees) can be used to animate the rainbow.
        """
        self._reset_move_cache()
        length = self.length
        inv_length = 1.0 / length
        hue_offset = offset / 360.0
        buffer = self.buffer
        for i in range(length):
            # Normalize index to [0,1] and add the offset (converted from degrees)
            hue = ((i * inv_length) + hue_offset) % 1.0
            # Convert HSV to RGB; using full saturation and 50% brightness
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 0.5)
            buffer[i].setRGB(int(r * 255), int(g * 255), int(b * 255))
        self.led.setData(buffer)
        self.solid_color = None

    def scolling_rainbow(self, speed: float = 1):
        """Custom preset that Creates a rainbow effect across the LED strip.

        The offset parameter (in degrees) can be used to animate the rainbow.
        """
        self._reset_move_cache()
        length = self.length
        inv_length = 1.0 / length
        time_offset = (((RobotController.getTime() / 100000) * speed) / 360.0) % 1.0
        buffer = self.buffer
        for i in range(length):
            # Normalize index to [0,1] and add the offset (converted from degrees)
            hue = ((i * inv_length) + time_offset) % 1.0
            # Convert HSV to RGB; using full saturation and 50% brightness
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 0.5)
            buffer[i].setRGB(int(r * 255), int(g * 255), int(b * 255))
        self.led.setData(buffer)
        self.solid_color = None

    def move_across(
        self,
        colors: list[tuple[int, int, int]] | tuple[int, int, int],
        size: int = 1,
        hertz: wpimath.units.hertz = 1,
    ):
        """Moves a block of LEDs across the strip using RobotController.getTime() for timing."""
        if isinstance(colors[0], int):
            colors = [colors]

        # Get the current time
        current_time = Timer.getFPGATimestamp()
        length = self.length
        buffer = self.buffer

        # Compute the current position with slower movement
        position = int(current_time * hertz) % length

        if not self._move_frame_initialized:
            self._clear_all()
            self._move_frame_initialized = True
        else:
            for idx in self._last_lit_indices:
                buffer[idx].setRGB(0, 0, 0)

        num_colors = len(colors)
        segment_len = length // num_colors if num_colors > 0 else length
        current_lit: list[int] = []
        # Light up the section
        for i, (r, g, b) in enumerate(colors):
            offset = i * segment_len
            for j in range(size):
                index = (position - j + offset) % length
                buffer[index].setRGB(r, g, b)
                current_lit.append(index)

        self._last_lit_indices = current_lit
        self.solid_color = None
        self.led.setData(buffer)

    def move_across_multi(
        self,
        colors: list[tuple[int, int, int]] | tuple[int, int, int],
        size: int = 1,
        hertz: wpimath.units.hertz = 1,
    ):
        """Moves a fixed-size multicolor block across the strip using RobotController.getTime() for timing."""
        current_time = Timer.getFPGATimestamp()
        length = self.length
        buffer = self.buffer

        # Handle single color input
        if isinstance(colors[0], int):
            colors = [colors]

        num_colors = len(colors)

        # Current LED position (loops around)
        position = int((current_time * hertz) % length)

        if not self._move_frame_initialized:
            self._clear_all()
            self._move_frame_initialized = True
        else:
            for idx in self._last_lit_indices:
                buffer[idx].setRGB(0, 0, 0)

        # Fill the moving block with a color pattern distributed across its size
        current_lit: list[int] = []
        for i in range(size):
            # Determine which color to use for this LED in the block
            color_index = int((i / size) * num_colors) % num_colors
            r, g, b = colors[color_index]

            index = (position + i) % length
            buffer[index].setRGB(r, g, b)
            current_lit.append(index)

        self._last_lit_indices = current_lit
        self.led.setData(buffer)
        self.solid_color = None

    def blink(
        self,
        color1: Tuple[int, int, int],
        color2: Tuple[int, int, int] = (0, 0, 0),
        hertz: wpimath.units.hertz = 2,
    ):
        """Blinks the entire strip between two colors at the given frequency.

        :param color1: The first RGB color.
        :param color2: The second RGB color. Defaults to off (0, 0, 0).
        :param hertz: Blink frequency in Hz (full cycles per second). Default is 2 Hz.
        """
        current_time = Timer.getFPGATimestamp()
        first = (int(current_time * hertz * 2) % 2) == 0
        if first:
            self.set_solid_color(color1)
        else:
            self.set_solid_color(color2)

    def clear(self):
        """Turns off all LEDs."""
        self._reset_move_cache()
        self.solid_color = None
        self.set_solid_color((0, 0, 0))
