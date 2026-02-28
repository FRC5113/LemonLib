from typing import Callable, Dict, List

import magicbot
from wpilib import DriverStation, Timer

from lemonlib.smart import SmartNT, SmartPreference


class LemonRobot(magicbot.MagicRobot):
    """
    Wrapper for the magicbot robot class to allow for command-based
    functionality. This class is used to create a robot that can be
    controlled using commands, while still using the magicbot framework.
    """

    low_bandwidth = DriverStation.isFMSAttached()

    watchdog_profile = SmartPreference(False)
    watchdog_profile_period = SmartPreference(0.25)

    # EMA coefficient (0 < alpha <= 1)
    watchdog_ema_alpha = SmartPreference(0.25)

    def __init__(self):
        super().__init__()

        self._periodic_callbacks: List[List] = []

        self.loop_time = self.control_loop_wait_time
        self._last_watchdog_profile_time = 0.0
        self._overrun_count = 0

        # Profiling storage
        self._last_overrun_epochs: Dict[str, float] = {}

        self._epoch_ema_all: Dict[str, float] = {}
        self._epoch_ema_overrun: Dict[str, float] = {}

        self._smart_nt = SmartNT("LemonRobot")

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

        if not self.watchdog_profile:
            self.loop_time = max(self.control_loop_wait_time, self.watchdog.getTime())
            return

        wd = self.watchdog
        loop_time = wd.getTime()
        is_overrun = loop_time > self.control_loop_wait_time

        epochs = wd._epochs
        prev = wd._startTime

        alpha = float(self.watchdog_ema_alpha)
        one_minus_alpha = 1.0 - alpha

        ema_all = self._epoch_ema_all
        ema_overrun = self._epoch_ema_overrun
        last_overrun = self._last_overrun_epochs

        for key, value in epochs:
            delta = (value - prev) * 1e-6

            prev_ema = ema_all.get(key)
            if prev_ema is None:
                ema_all[key] = delta
            else:
                ema_all[key] = alpha * delta + one_minus_alpha * prev_ema

            prev = value

        if is_overrun:
            self._overrun_count += 1

            prev = wd._startTime
            for key, value in epochs:
                delta = (value - prev) * 1e-6
                last_overrun[key] = delta

                prev_ema = ema_overrun.get(key)
                if prev_ema is None:
                    ema_overrun[key] = delta
                else:
                    ema_overrun[key] = alpha * delta + one_minus_alpha * prev_ema

                prev = value

        now_fpga = Timer.getFPGATimestamp()
        if now_fpga - self._last_watchdog_profile_time >= self.watchdog_profile_period:
            self._last_watchdog_profile_time = now_fpga

            start = wd._startTime
            now = wd._get_time()
            total_time = (now - start) * 1e-6

            self._smart_nt.put_number("Watchdog/LoopTime", round(loop_time, 6))
            self._smart_nt.put_number(
                "Watchdog/ControlPeriod", round(self.control_loop_wait_time, 6)
            )
            self._smart_nt.put_boolean("Watchdog/Overrun", is_overrun)
            self._smart_nt.put_number("Watchdog/OverrunCount", self._overrun_count)
            self._smart_nt.put_number("Watchdog Epochs/Total", round(total_time, 6))

            if last_overrun:
                max_last = 0.0
                total_last = 0.0
                for v in last_overrun.values():
                    total_last += v
                    if v > max_last:
                        max_last = v

                self._smart_nt.put_number(
                    "Watchdog LastOverrun/Max", round(max_last, 6)
                )
                self._smart_nt.put_number(
                    "Watchdog LastOverrun/Total", round(total_last, 6)
                )

                for k, v in last_overrun.items():
                    self._smart_nt.put_number(f"Watchdog LastOverrun/{k}", round(v, 6))

            if ema_all:
                max_all = 0.0
                total_all = 0.0
                for v in ema_all.values():
                    total_all += v
                    if v > max_all:
                        max_all = v

                self._smart_nt.put_number("Watchdog EMA/Max", round(max_all, 6))
                self._smart_nt.put_number("Watchdog EMA/Total", round(total_all, 6))

                for k, v in ema_all.items():
                    self._smart_nt.put_number(f"Watchdog EMA/{k}", round(v, 6))

            if ema_overrun:
                max_or = 0.0
                total_or = 0.0
                for v in ema_overrun.values():
                    total_or += v
                    if v > max_or:
                        max_or = v

                self._smart_nt.put_number("Watchdog EMAOverrun/Max", round(max_or, 6))
                self._smart_nt.put_number(
                    "Watchdog EMAOverrun/Total", round(total_or, 6)
                )

                for k, v in ema_overrun.items():
                    self._smart_nt.put_number(f"Watchdog EMAOverrun/{k}", round(v, 6))

        wd.addEpoch("watchdog_profile")
        self.loop_time = max(self.control_loop_wait_time, loop_time)

    def get_period(self) -> float:
        """Get the period of the robot loop in seconds."""
        return self.loop_time
