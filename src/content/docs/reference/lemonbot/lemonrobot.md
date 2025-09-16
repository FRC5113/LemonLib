---
title: LemonRobot
slug: reference/lemonbot/lemonrobot
---

# LemonRobot

Wrapper for the magicbot robot class to allow for command-based
functionality. This class is used to create a robot that can be
controlled using commands, while still using the magicbot framework.

## Methods

### autonomousPeriodic()

Periodic code for autonomous mode should go here.
Runs when not enabled for trajectory display.

Users should override this method for code which will be called
periodically at a regular rate while the robot is in autonomous mode.

This code executes before the ``execute`` functions of all
components are called.

### enabledperiodic()

Periodic code for when the bot is enabled should go here.
Runs when not enabled for trajectory display.

Users should override this method for code which will be called

### _enabled_periodic()

Run components and all periodic methods.

### get_period()

Get the period of the robot loop in seconds.

