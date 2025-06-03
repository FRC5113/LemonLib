import magicbot
import commands2
from wpilib import DriverStation


class CommandMagicRobot(magicbot.MagicRobot):
    """
    Wrapper for the magicbot robot class to allow for command-based
    functionality. This class is used to create a robot that can be
    controlled using commands, while still using the magicbot framework.
    """

    def autonomousPeriodic(self):
        """
        Periodic code for autonomous mode should go here.
        Runs when not enabled for trajectory display.

        Users should override this method for code which will be called
        periodically at a regular rate while the robot is in autonomous mode.

        This code executes before the ``execute`` functions of all
        components are called.
        """
        pass

    def _do_periodics(self):
        super()._do_periodics()
        commands2.CommandScheduler.getInstance().run()
        self.period = max(self.control_loop_wait_time, self.watchdog.getTime())
        if DriverStation.isAutonomous():
            self.autonomousPeriodic()

    def _enabled_periodic(self) -> None:
        """Run components and all periodic methods."""
        watchdog = self.watchdog

        for name, component in self._components:
            if commands2.Subsystem.getCurrentCommand(component) is None:
                try:
                    component.execute()

                except Exception:
                    self.onException()
            watchdog.addEpoch(name)

        self._do_periodics()

        for reset_dict, component in self._reset_components:
            component.__dict__.update(reset_dict)
