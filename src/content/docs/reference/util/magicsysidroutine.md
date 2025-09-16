---
title: MagicSysIdRoutine
slug: reference/util/magicsysidroutine
---

# MagicSysIdRoutine

Magicbot implementation of SysIdRoutine from commands2.
To use this in a magicbot project, three things must be done:
1. Within the component that is being sysid'ed, create two methods
for driving the mechanism and logging values. The drive method
should take one parameter: volts (float), and the log method should
take one parameter: log (SysIdRoutineLog). Example log method:
```
def sysid_log(self, log: SysIdRoutineLog) -> None:
    log.motor("drive").voltage(
        self.drive_motor.getVoltage()
    ).position(
        self.drive_motor.getPosition()
    ).velocity(
        self.drive_motor.getVelocity()
    )
```
2. Create a new component that inherits from `MagicSysIdRoutine`.
This should be a high-level component that is injected lower-level
components that it controls. Use the `setup()` method (not
`__init__()`!) to call `setup_sysid()`.
```
class SysIdDrive(MagicSysIdRoutine):
    drive: Drive

    def setup(self):
        self.setup_sysid(
            SysIdRoutine.Config(rampRate=0.2, stepVoltage=7.0),
            SysIdRoutine.Mechanism(
                self.drive.sysid_drive, self.drive.sysid_log, self.drive, "Drive",
            ),
        )
```
3. In robot.py, annotate the sysid component (above the low-level
components), and call the `quasistatic_forwards()`,
`quasistatic_reverse()`, `dynamic_forward()`, and `dynamic_reverse()`
methods (eg. bound to controller buttons)
```

