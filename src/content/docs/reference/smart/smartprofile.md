---
title: SmartProfile
slug: reference/smart/smartprofile
---

# SmartProfile

Used to store multiple gains and configuration values for several
different types of controllers. This can optionally interface with
NetworkTables so that the gains may be dynamically updated without
needing to redeploy code. This class has several helper methods
which can be used to create `SmartController` objects already supplied
with the necessary gains. Please note that gains will NOT be
dynamically updated in `SmartController` objects: therefore it is
recommended to create a new `SmartController` object on enable.

## Methods

### __init__()

Creates a SmartProfile.
Recommended gain keys (for use with `SmartController`):
kP: Proportional Gain
kI: Integral Gain
kD: Derivative Gain
kS: Static Gain
kV: Velocity Gain
kG: Gravity Gain
kA: Acceleration Gain
kMinInput: Minimum expected measurement value (used for continuous input)
kMaxInput: Maximum expected measurement value (used for continuous input)

Q1, Q2, Q3, Q4, Q5: State weighting for LTV controllers
R1, R2: Input weighting for LTV controllers

:param str profile_key: Prefix for associated NetworkTables keys
:param dict[str, float] gains: Dictionary containing gain_key: value pairs
:param bool tuning_enabled: Specify whether or not to send and retrieve
    data from NetworkTables. If true, values from NetworkTables
    are given precedence over values set in code.

### create_pid_controller()

Creates a PID controller.
Requires kP, kI, kD, [kMinInput, kMaxInput optional]

### create_wpi_pid_controller()

Creates a wpilib PIDController. Use `create_pid_controller()`
instead if possible.
Requires kP, kI, kD, [kMinInput, kMaxInput optional]

### create_ctre_pid_controller()

Creates a CTRE PIDController. Use `create_pid_controller()`
instead if possible.

### create_ctre_turret_controller()

Creates a CTRE PIDController. Use `create_pid_controller()`
instead if possible.
Requires kP, kI, kD,kS, kV,kA

### create_ctre_flywheel_controller()

Creates a CTRE PIDController. Use `create_pid_controller()`
instead if possible.
Requires kP, kI, kD,kS, kV,kA

### create_profiled_pid_controller()

Creates a profiled PID controller.
Requires kP, kI, kD, kMaxV, kMaxA, [kMinInput, kMaxInput optional]

### create_wpi_profiled_pid_controller_radians()

Creates a wpilib ProfiledPIDControllerRadians.
Requires kP, kI, kD, kMaxV, kMaxA, kMinInput, kMaxInput

### create_ltv_unicycle_controller()

Creates a wpilib LTVUnicyvleController.
Requires Qelems tuple(5 elements of SupportsFloat),
Relems tuple(2 elements of SupportsFloat)

### create_simple_feedforward()

Creates a simple DC motor feedforward controller.
Requires kS, kV, [kA optional]

### create_flywheel_controller()

Creates a PID controller combined with a DC motor feedforward controller.
Requires kP, kI, kD, kS, kV, [kA optional]

### create_turret_controller()

Creates a profiled PID controller combined with a DC motor feedforward controller.
Requires kP, kI, kD, kS, kV, [kA, kMinInput, kMaxInput optional]

### create_elevator_controller()

Creates a profiled PID controller combined with an elevator feedforward controller.
Requires kP, kI, kD, kS, kV, kG, [kA optional]

### create_arm_controller()

Creates a profiled PID controller combined with an arm feedforward controller.
Requires kP, kI, kD, kS, kV, kG, [kA optional]

