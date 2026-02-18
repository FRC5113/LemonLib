from typing import Callable, List, Tuple

import magicbot
from wpilib import DriverStation, Notifier,Timer


class LemonRobot(magicbot.MagicRobot):
    """
    Wrapper for the magicbot robot class to allow for command-based
    functionality. This class is used to create a robot that can be
    controlled using commands, while still using the magicbot framework.
    """

    low_bandwidth = DriverStation.isFMSAttached()

    def __init__(self):
        super().__init__()
        self._periodic_callbacks: List[List] = []

        self.loop_time = self.control_loop_wait_time

    def add_periodic(self, callback: Callable[[], None], period: float):
        now = Timer.getFPGATimestamp()
        self._periodic_callbacks.append([callback, period, now])

    def _run_periodics(self):
        now = Timer.getFPGATimestamp()
        for entry in self.__periodics:
            callback, period, last = entry
            if now - last >= period:
                entry[2] = now
                callback()

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

    def autonomous(self):
        super().autonomous()
        self.autonomousPeriodic()

    def enabledperiodic(self) -> None:
        """Periodic code for when the bot is enabled should go here.
        Runs when not enabled for trajectory display.

        Users should override this method for code which will be called"""
        pass

    def _on_mode_enable_components(self):
        super()._on_mode_enable_components()
        self.on_enable()

    def on_enable(self):
        pass

    def _enabled_periodic(self) -> None:
        """Run components and all periodic methods."""
        watchdog = self.watchdog

        for name, component in self._components:
            try:
                component.execute()

            except Exception:
                self.onException()
            watchdog.addEpoch(name)

        self.enabledperiodic()
        watchdog.addEpoch("enabledperiodic")

        self._do_periodics()
        watchdog.addEpoch("periodics")

        for reset_dict, component in self._reset_components:
            component.__dict__.update(reset_dict)

    def _do_periodics(self):
        super()._do_periodics()

        self.loop_time = max(self.control_loop_wait_time, self.watchdog.getTime())

    def get_period(self) -> float:
        """Get the period of the robot loop in seconds."""
        return self.loop_time
