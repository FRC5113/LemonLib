---
title: LemonCameraSim
slug: reference/simulation/lemoncamerasim
---

# LemonCameraSim

Simulated version of a LemonCamera. This class functions exactly
the same in code except for the following:
1. Must be initialized with an `AprilTagFieldLayout` and an FOV
2. `set_robot_pose()` must be called periodically to update the pose
of the robot. This should not be taken from a pose estimator that
uses vision updates, but rather a pose simulated in physics.py
3. This simulation assumes that the camera is at the center of the
robot looking directly forward, but the difference should be negligible

## Methods

### __init__()

Args:
field_layout (AprilTagFieldLayout): layout of the tags on the field, such as
    `AprilTagField.k2024Crescendo`
fov (float): horizontal range of vision (degrees)

