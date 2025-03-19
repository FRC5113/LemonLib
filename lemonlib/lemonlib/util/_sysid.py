from wpilib import Timer
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog, State
from magicbot import will_reset_to


class MagicSysIdRoutine:
    """Magicbot implementation of SysIdRoutine from commands2.
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
    """

    enabled = will_reset_to(False)
    output_volts = will_reset_to(0)

    def __init__(self):
        self.timer = Timer()
        self.timed_out = False
        self.was_enabled = False
        self.state = State.kNone

    def setup_sysid(
        self, config: SysIdRoutine.Config, mechanism: SysIdRoutine.Mechanism
    ):
        self.log = SysIdRoutineLog(mechanism.name)
        self.config = config
        self.mechanism = mechanism
        self.record_state = config.recordState or self.log.recordState

    def quasistatic_forward(self):
        self.enabled = True
        self.state = State.kQuasistaticForward
        self.outputVolts = self.timer.get() * self.config.rampRate

    def quasistatic_reverse(self):
        self.enabled = True
        self.state = State.kQuasistaticReverse
        self.outputVolts = -self.timer.get() * self.config.rampRate

    def dynamic_forward(self):
        self.enabled = True
        self.state = State.kDynamicForward
        self.outputVolts = self.config.stepVoltage

    def dynamic_reverse(self):
        self.enabled = True
        self.state = State.kDynamicReverse
        self.outputVolts = -self.config.stepVoltage

    def on_start(self):
        self.timer.restart()
        self.timed_out = False
        self.was_enabled = True

    def on_end(self):
        self.was_enabled = False
        self.mechanism.drive(0.0)
        self.record_state(State.kNone)
        self.timer.stop()

    def execute(self):
        if self.was_enabled:
            if self.timed_out:
                return
            if not self.enabled:
                self.on_end()
                return
        else:
            if not self.enabled:
                return
            self.on_start()

        if self.timer.get() > self.config.timeout:
            self.on_end()
            self.timed_out = True
            return

        self.mechanism.drive(self.outputVolts)
        self.mechanism.log(self.log)
        self.record_state(self.state)
