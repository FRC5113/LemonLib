---
title: LEDController
slug: reference/util/ledcontroller
---

# LEDController

## Methods

### __init__()

Initializes the LED controller.

:param pwm_port: The PWM port the LED strip is connected to.
:param length: Number of LEDs in the strip.

### apply_pattern()

Applies a wpilib.LEDPattern to the LED buffer and updates the strip.

### set_solid_color()

Sets the entire LED strip to a solid color.

### set_pixel()

Sets the color of a single LED pixel.

### set_gradient()

Custom preset that Sets a gradient from start_color to end_color across the LED strip.

### static_rainbow()

Custom preset that Creates a rainbow effect across the LED strip.

The offset parameter (in degrees) can be used to animate the rainbow.

### scolling_rainbow()

Custom preset that Creates a rainbow effect across the LED strip.

The offset parameter (in degrees) can be used to animate the rainbow.

### move_across()

Moves a block of LEDs across the strip using RobotController.getTime() for timing.

### clear()

Turns off all LEDs.

