import magicbot
import commands2
from wpilib import DriverStation
from wpilib import RobotController, Timer, reportError
from wpilib import SmartDashboard
from robotpy_ext.autonomous import AutonomousModeSelector
from .commandcomponent import LemonComponent
from lemonlib.util import AlertManager, AlertType
from lemonlib.smart import SmartNT
import heapq


class _PeriodicCallback:
    def __init__(self, func, start_time, period, offset):
        self.func = func
        self.period = period
        # Calculate aligned expiration time
        self.expiration = (
            start_time
            + offset
            + period
            + int((RobotController.getFPGATime() - start_time) * 1e-6 / period) * period
        )

    def __lt__(self, other):
        return self.expiration < other.expiration


class LemonRobot(magicbot.MagicRobot):
    """
    Wrapper for the magicbot robot class to allow for command-based
    functionality. This class is used to create a robot that can be
    controlled using commands, while still using the magicbot framework.
    """

    low_bandwidth = DriverStation.isFMSAttached()

    commandscheduler = commands2.CommandScheduler.getInstance()

    def __init__(self):
        super().__init__()
        self._periodic_callbacks = []
        self._start_time = RobotController.getFPGATime()

        self.loop_time = self.control_loop_wait_time
        SmartDashboard.putData("CommandScheduler", self.commandscheduler)

    def addPeriodic(self, callback, periodSeconds: float, offsetSeconds: float = 0.0):
        """Register a callback to run periodically.

        Mimics the behavior of TimedRobot.addPeriodic().

        Args:
            callback: Function to call.
            periodSeconds: Interval between calls.
            offsetSeconds: Time offset for staggered execution.
        """
        callback_obj = _PeriodicCallback(
            callback, self._start_time, periodSeconds, offsetSeconds
        )
        heapq.heappush(self._periodic_callbacks, callback_obj)

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

    # def endCompetition(self) -> None:
    #     self.__done = True
    #     if self._automodes:
    #         self._automodes.endCompetition()

    def robotPeriodic(self):
        super().robotPeriodic()

        now = Timer.getFPGATimestamp()

        while (
            self._periodic_callbacks and self._periodic_callbacks[0].expiration <= now
        ):
            callback_obj = heapq.heappop(self._periodic_callbacks)
            try:
                callback_obj.func()
            except Exception as e:
                reportError(f"Exception in periodic callback: {e}", False)

            # Advance expiration to next aligned time
            missed_intervals = int(
                (now - callback_obj.expiration) / callback_obj.period
            )
            callback_obj.expiration += callback_obj.period * (1 + missed_intervals)
            heapq.heappush(self._periodic_callbacks, callback_obj)

    def autonomous(self):
        super().autonomous()
        self.autonomousPeriodic()

    def enabledperiodic(self) -> None:
        """Periodic code for when the bot is enabled should go here.
        Runs when not enabled for trajectory display.

        Users should override this method for code which will be called"""
        pass

    def _enabled_periodic(self) -> None:
        """Run components and all periodic methods."""
        watchdog = self.watchdog
        self.commandscheduler.run()

        for name, component in self._components:
            if commands2.Subsystem.getCurrentCommand(component) is None and issubclass(
                component.__class__, LemonComponent
            ):
                try:
                    component.execute()

                except Exception:
                    self.onException()
            else:
                try:
                    component.execute()

                except Exception:
                    self.onException()
            watchdog.addEpoch(name)

        self.enabledperiodic()

        self._do_periodics()

        for reset_dict, component in self._reset_components:
            component.__dict__.update(reset_dict)

    def _do_periodics(self):
        super()._do_periodics()

        self.loop_time = max(self.control_loop_wait_time, self.watchdog.getTime())

    def get_period(self) -> float:
        """Get the period of the robot loop in seconds."""
        return self.loop_time
