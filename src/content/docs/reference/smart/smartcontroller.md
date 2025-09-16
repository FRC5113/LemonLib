---
title: SmartController
slug: reference/smart/smartcontroller
---

# SmartController

Used as a general wrapper for a variety of controllers that may
optionally report values to NetworkTables. It is recommended to
create these using a `SmartProfile`.

## Methods

### setTolerance()

Sets the error tolerance for the controller.

### at_setpoint()

Checks if the controller is at the setpoint within the tolerance.

### getError()

Returns the current error of the controller.

### getOutput()

Returns the current output of the controller.

### getReference()

Returns the current reference value of the controller.

### getMeasurement()

Returns the current measurement value of the controller.

