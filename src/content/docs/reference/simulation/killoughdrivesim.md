---
title: KilloughDriveSim
slug: reference/simulation/killoughdrivesim
---

# KilloughDriveSim

## Methods

### __init__()

:param drive: An instance of KilloughDrive.
:param mass: Robot mass in kg.
:param moment_of_inertia: Rotational inertia (kg*m^2).
:param wheel_force: Maximum force (N) per wheel at full output.
:param dt: Simulation timestep (seconds).

### update()

Update the simulation by one timestep.
Reads motor outputs, computes forces (in the robot frame), updates chassis speeds,
applies damping, and integrates the pose using WPIMath’s twist (exponential map).

### get_pose()

Returns the robot’s pose as (x, y, theta_degrees).

