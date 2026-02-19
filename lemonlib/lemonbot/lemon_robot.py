from typing import Callable, List

import magicbot
from wpilib import DriverStation, SmartDashboard, Timer

from lemonlib.smart import SmartPreference


class LemonRobot(magicbot.MagicRobot):
    """
    Wrapper for the magicbot robot class to allow for command-based
    functionality. This class is used to create a robot that can be
    controlled using commands, while still using the magicbot framework.
    """

    low_bandwidth = DriverStation.isFMSAttached()

    watchdog_profile = SmartPreference(False)
    watchdog_profile_period = SmartPreference(0.25)

    def __init__(self):
        super().__init__()
        self._periodic_callbacks: List[List] = []

        self.loop_time = self.control_loop_wait_time
        self._last_watchdog_profile_time = 0.0
        self._overrun_count = 0
        self._last_overrun_epochs: dict = {}

    def add_periodic(self, callback: Callable[[], None], period: float):
        now = Timer.getFPGATimestamp()
        self._periodic_callbacks.append([callback, period, now])

    def _run_periodics(self):
        now = Timer.getFPGATimestamp()
        for entry in self._periodic_callbacks:
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

        is_overrun = self.watchdog.getTime() > self.control_loop_wait_time
        if is_overrun:
            self._overrun_count += 1
            # Capture epoch data for overrun
            prev = self.watchdog._startTime
            now = self.watchdog._get_time()
            for key, value in self.watchdog._epochs:
                self._last_overrun_epochs[key] = (value - prev) / 1e6
                prev = value

        if self.watchdog_profile:
            now_fpga = Timer.getFPGATimestamp()
            if (
                now_fpga - self._last_watchdog_profile_time
                >= self.watchdog_profile_period
            ):
                self._last_watchdog_profile_time = now_fpga

                now = self.watchdog._get_time()
                self._lastEpochsPrintTime = now
                prev = self.watchdog._startTime
                max_epoch_time = 0.0
                max_epoch_key = ""
                for key, value in self.watchdog._epochs:
                    time = (value - prev) / 1e6
                    prev = value
                    if time > max_epoch_time:
                        max_epoch_time = time
                        max_epoch_key = key
                    SmartDashboard.putNumber(f"Watchdog Epochs/{key}", time)

                total_time = (now - self.watchdog._startTime) / 1e6
                SmartDashboard.putNumber("Watchdog Epochs/Total", total_time)
                SmartDashboard.putNumber("Watchdog Epochs/Max", max_epoch_time)
                SmartDashboard.putString("Watchdog Epochs/MaxKey", max_epoch_key)

                SmartDashboard.putNumber("Watchdog/LoopTime", self.watchdog.getTime())
                SmartDashboard.putNumber(
                    "Watchdog/ControlPeriod", self.control_loop_wait_time
                )
                SmartDashboard.putBoolean(
                    "Watchdog/Overrun",
                    self.watchdog.getTime() > self.control_loop_wait_time,
                )
                SmartDashboard.putNumber("Watchdog/OverrunCount", self._overrun_count)

                # Display last overrun epochs
                if self._last_overrun_epochs:
                    max_overrun_time = max(self._last_overrun_epochs.values())
                    total_overrun_time = sum(self._last_overrun_epochs.values())
                    SmartDashboard.putNumber(
                        "Watchdog LastOverrun/Max", max_overrun_time
                    )
                    SmartDashboard.putNumber(
                        "Watchdog LastOverrun/Total", total_overrun_time
                    )
                    for key, time in self._last_overrun_epochs.items():
                        SmartDashboard.putNumber(f"Watchdog LastOverrun/{key}", time)

            self.watchdog.addEpoch("watchdog_profile")
        self.loop_time = max(self.control_loop_wait_time, self.watchdog.getTime())

    def get_period(self) -> float:
        """Get the period of the robot loop in seconds."""
        return self.loop_time
