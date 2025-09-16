---
title: LemonInput
slug: reference/root/lemoninput
---

# LemonInput

LemonInput is a wrapper class for Xbox
and PS5 controllers allowing automatic
or manual detection and use in code.

## Methods

### __init__()

Initializes the control object with the specified port number and type.
Args:
    port (int, optional): The port number of the controller. If unset,
        chooses first controller matching type.
    type (str, optional): The type of the controller. Defaults to "auto".
        - "auto": Automatically detects the controller type.
        - "Xbox": Forces the controller type to Xbox.
        - "PS5": Forces the controller type to PS5.

### getType()

Returns the type of controller (Xbox or PS5).

### getLeftBumper()

Returns the state of the left bumper button.

### getRightBumper()

Returns the state of the right bumper button.

Returns:
    bool: The state of the right bumper button (pressed or not).

### getStartButton()

Returns the state of the start button.

Returns:
    bool: The state of the start button (pressed or not).

### getBackButton()

Returns the state of the back button.

Returns:
    bool: The state of the back button (pressed or not).

### getAButton()

Returns the state of the 'A' button.

Returns:
    bool: The state of the 'A' button (pressed or not).

### getBButton()

Returns the state of the 'B' button.

Returns:
    bool: The state of the 'B' button (pressed or not).

### getXButton()

Returns the state of the 'X' button.

Returns:
    bool: The state of the 'X' button (pressed or not).

### getYButton()

Returns the state of the 'Y' button.

Returns:
    bool: The state of the 'Y' button (pressed or not).

### getLeftStickButton()

Returns the state of the left stick button.

Returns:
    bool: The state of the left stick button (pressed or not).

### getRightStickButton()

Returns the state of the right stick button.

Returns:
    bool: The state of the right stick button (pressed or not).

### getRightTriggerAxis()

Returns the state of the right trigger button.

Returns:
    float: The state of the right trigger button ranging from 0.0 to 1.0.

### getLeftTriggerAxis()

Returns the state of the left trigger button.

Returns:
    float: The state of the left trigger button ranging from 0.0 to 1.0.

### getL1Button()

Returns the state of the L1 button.

### getR1Button()

Returns the state of the R1 button.

### getOptionsButton()

Returns the state of the Options button.

### getCreateButton()

Returns the state of the Create button.

### getCrossButton()

Returns the state of the Cross (X) button.

### getCircleButton()

Returns the state of the Circle (O) button.

### getSquareButton()

Returns the state of the Square button.

### getTriangleButton()

Returns the state of the Triangle button.

### getL3()

Returns the state of the L3 (left stick) button.

### getR3()

Returns the state of the R3 (right stick) button.

### getR2Axis()

Returns the state of the R2 trigger.

### getL2Axis()

Returns the state of the L2 trigger.

### setRumbleLeft()

Sets the rumble of the controller.

Args:
    value (float): The value of the rumble to set.

### setRumbleRight()

Sets the rumble of the controller.

Args:
    value (float): The value of the rumble to set.

### getLeftX()

Returns the X-axis value of the left joystick.

Returns:
    float: The X-axis value of the left joystick, ranging from -1.0 to 1.0.

### getLeftY()

Returns the Y-axis value of the left joystick.

Returns:
    float: The Y-axis value of the left joystick, ranging from -1.0 to 1.0.

### getRightX()

Returns the X-axis value of the right joystick.

Returns:
    float: The X-axis value of the right joystick, ranging from -1.0 to 1.0.

### getRightY()

Returns the Y-axis value of the right joystick.

Returns:
    float: The Y-axis value of the right joystick, ranging from -1.0 to 1.0.

### __pov_xy()

Returns the X and Y values of the POV as a tuple using sin and cos,
or (0, 0) if the POV is not pressed (-1).

Returns:
    tuple: The X and Y values of the POV as a tuple.

### getPovX()

Returns the X-axis value of the POV (Point of View) of a joystick.

Example:
```
controller = SmartController(0)

if controller.pov() >= 0:
    left_joy_x = controller.pov_x()
    left_joy_y = controller.pov_y()
```

Returns:
    float: The X-axis value of the POV.

### getPovY()

Returns the Y-axis value of the POV (Point of View) of a joystick.

Example:
```
controller = SmartController(0)

if controller.pov() >= 0:
    left_joy_x = controller.pov_x()
    left_joy_y = controller.pov_y()
```

Returns:
    float: The Y-axis value of the POV.

### initSendable()

Initializes the sendable for the LemonInput class.

Args:
    builder: The sendable builder.

