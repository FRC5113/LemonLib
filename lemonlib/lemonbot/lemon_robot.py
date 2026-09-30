from collections.abc import Callable

from wpilib import DriverStationBackend, Timer

from lemonlib.smart import SmartNT, SmartPreference
from modified_libs import magicbot


class LemonRobot(magicbot.MagicRobot):
    """
    Wrapper for the magicbot robot class to allow for command-based
    functionality. This class is used to create a robot that can be
    controlled using commands, while still using the magicbot framework.
    """

    low_bandwidth = DriverStationBackend.isFMSAttached()

    watchdog_profile = SmartPreference(False)
    watchdog_profile_period = SmartPreference(0.25)

    # EMA coefficient (0 < alpha <= 1)
    watchdog_ema_alpha = SmartPreference(0.25)

    def __init__(self):
        super().__init__()

        self._periodic_callbacks: list[list] = []

        self.loop_time = self.control_loop_wait_time
        self._last_watchdog_profile_time = 0.0
        self._overrun_count = 0

        # Profiling storage
        self._last_overrun_epochs: dict[str, float] = {}

        self._epoch_ema_all: dict[str, float] = {}
        self._epoch_ema_overrun: dict[str, float] = {}

        self._smart_nt = SmartNT("LemonRobot")

    def add_periodic(self, callback: Callable[[], None], period: float):
        now = Timer.getMonotonicTimestamp()
        self._periodic_callbacks.append([callback, period, now])

    def _run_periodics(self):
        now = Timer.getMonotonicTimestamp()
        for entry in self._periodic_callbacks:
            callback, period, last = entry
            if now - last >= period:
                entry[2] = now
                callback()

    def autonomousPeriodic(self):
        pass

    def autonomous(self):
        super().autonomous()
        self.autonomousPeriodic()

    def enabledperiodic(self) -> None:
        pass

    def _on_mode_enable_components(self):
        super()._on_mode_enable_components()
        self.on_enable()

    def on_enable(self):
        pass

    def _enabled_periodic(self) -> None:
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

    def _do_periodics(self):
        super()._do_periodics()

        wd = self.watchdog
        self.loop_time = wd.getTime()

    def get_period(self) -> float:
        """Get the period of the robot loop in seconds."""
        return self.loop_time
