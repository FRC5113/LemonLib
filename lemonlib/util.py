from enum import Enum
from logging import Logger
from typing import List, Dict
from wpilib import SmartDashboard, Timer
from wpiutil import Sendable, SendableBuilder
from ntcore import NetworkTableInstance, PubSubOptions
import json

from typing import Callable

from wpilib import Timer
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog, State
from magicbot import will_reset_to


class AlertType(Enum):
    """
    Enum representing the severity level of an alert.
    """

    ERROR = 0
    WARNING = 1
    INFO = 2


class Alert:
    """
    Represents an individual alert with text, type, and optional timeout.

    Alerts can be activated, deactivated, or updated with new text.
    """

    def __init__(
        self, text: str, type: AlertType, timeout: float = 0.0, elasticnoti: bool = True
    ):
        """
        Initialize an alert instance.

        Args:
            text (str): The message text for the alert.
            type (AlertType): The severity level of the alert.
            timeout (float): Duration in seconds after which the alert auto-deactivates.
            elasticnoti (bool): Whether to send the alert to the Elastic dashboard. defaults to True.
        """
        self.text = text
        self.type = type
        self.timeout = timeout
        self.active = False
        self.active_start_time = 0.0
        self.last_log = 0.0
        AlertManager.alerts.append(self)
        self.elasticnoti = elasticnoti

    def set(self, active: bool):
        """
        Activate or deactivate the alert.

        Args:
            active (bool): True to activate, False to deactivate.
        """
        if active and not self.active:
            self.active_start_time = Timer.getFPGATimestamp()

            # Log the alert based on its type.
            match self.type:
                case AlertType.ERROR:
                    AlertManager.logger.error(self.text)
                case AlertType.WARNING:
                    AlertManager.logger.warning(self.text)
                case AlertType.INFO:
                    AlertManager.logger.info(self.text)

            # Send notification to Elastic dashboard.
            notification = ElasticNotification(
                level=self.type.name,
                title="Robot Alert",
                description=self.text,
                display_time=int(self.timeout * 1000) if self.timeout > 0 else 3000,
            )
            if self.elasticnoti:
                Elastic.send_alert(notification)

        self.active = active

    def enable(self):
        """
        Enable the alert.
        """
        self.set(True)

    def disable(self):
        """
        Disable the alert.
        """
        self.set(False)

    def set_text(self, text: str):
        """
        Update the alert's text and log the change if it is active.

        Args:
            text (str): New text for the alert.
        """
        if (
            self.active
            and self.text != text
            and Timer.getFPGATimestamp() - self.last_log > 1.0
        ):
            self.last_log = Timer.getFPGATimestamp()
            match self.type:
                case AlertType.ERROR:
                    AlertManager.logger.error(text)
                case AlertType.WARNING:
                    AlertManager.logger.warning(text)
                case AlertType.INFO:
                    AlertManager.logger.info(text)
        self.text = text


class AlertManager(Sendable):
    """
    Manages a collection of alerts and integrates with the SmartDashboard.
    """

    alerts: List[Alert] = []
    logger: Logger = None

    def __init__(self, logger):
        """
        Initialize the AlertManager and add it to the SmartDashboard.

        Args:
            logger (Logger): Logger instance for logging alert messages.
        """
        Sendable.__init__(self)
        AlertManager.logger = logger
        SmartDashboard.putData("Alerts", self)

    def initSendable(self, builder: SendableBuilder) -> None:
        """
        Configure the SmartDashboard properties for the alerts.

        Args:
            builder (SendableBuilder): The builder to configure.
        """
        builder.setSmartDashboardType("Alerts")
        builder.addStringArrayProperty(
            "errors", lambda: AlertManager.get_strings(AlertType.ERROR), lambda _: None
        )
        builder.addStringArrayProperty(
            "warnings",
            lambda: AlertManager.get_strings(AlertType.WARNING),
            lambda _: None,
        )
        builder.addStringArrayProperty(
            "infos", lambda: AlertManager.get_strings(AlertType.INFO), lambda _: None
        )

    @staticmethod
    def get_strings(type: AlertType) -> List[str]:
        """
        Retrieve active alerts of a specified type as strings.

        Args:
            type (AlertType): The type of alerts to retrieve.

        Returns:
            List[str]: List of alert messages.
        """
        alerts = []
        timestamp = Timer.getFPGATimestamp()
        for alert in AlertManager.alerts:
            if not alert.active:
                continue
            if (
                alert.timeout > 0.0
                and timestamp - alert.active_start_time >= alert.timeout
            ):
                alert.set(False)
                continue
            if alert.type == type:
                alerts.append(alert)
        return [
            alert.text
            for alert in sorted(alerts, key=lambda alert: alert.active_start_time)
        ]

    @staticmethod
    def instant_alert(text: str, type: AlertType, timeout: float = 0.0):
        """
        Create and immediately enable a new alert.

        Args:
            text (str): The alert message.
            type (AlertType): The severity level of the alert.
            timeout (float): The timeout in seconds for the alert.
        """
        alert = Alert(text, type, timeout)
        alert.enable()


class ElasticNotification:
    """
    Represents a notification object to be sent to the Elastic dashboard.
    """

    class NotificationLevel:
        INFO = "INFO"
        WARNING = "WARNING"
        ERROR = "ERROR"

    def __init__(
        self,
        level=NotificationLevel.INFO,
        title: str = "",
        description: str = "",
        display_time: int = 3000,
        width: float = 350,
        height: float = -1,
    ):
        """
        Initialize an ElasticNotification object.

        Args:
            level (str): Severity level of the notification.
            title (str): Title of the notification.
            description (str): Description of the notification.
            display_time (int): Display duration in milliseconds.
            width (float): Width of the notification display.
            height (float): Height of the notification display.
        """
        self.level = level
        self.title = title
        self.description = description
        self.display_time = display_time
        self.width = width
        self.height = height

    def to_dict(self) -> Dict[str, str | float | int]:
        """
        Convert the notification to a dictionary for JSON serialization.

        Returns:
            Dict: Dictionary representation of the notification.
        """
        return {
            "level": self.level,
            "title": self.title,
            "description": self.description,
            "displayTime": self.display_time,
            "width": self.width,
            "height": self.height,
        }

    # Chained methods for modifying properties omitted for brevity...


class Elastic:
    """
    Handles sending notifications to the Elastic dashboard using NetworkTables.
    """

    _topic = NetworkTableInstance.getDefault().getStringTopic(
        "/Elastic/RobotNotifications"
    )
    _publisher = _topic.publish(PubSubOptions(sendAll=True, keepDuplicates=True))

    @staticmethod
    def send_alert(alert: ElasticNotification):
        """
        Send an alert notification to the Elastic dashboard.

        Args:
            alert (ElasticNotification): The notification object.
        """
        try:
            Elastic._publisher.set(json.dumps(alert.to_dict()))
        except Exception as e:
            print(f"Error serializing alert: {e}")




def clamp(value: float, min_value: float, max_value: float) -> float:
    """Restrict value between min_value and max_value."""
    return max(min(value, max_value), min_value)


def curve(
    mapping: Callable[[float], float],
    offset: float,
    deadband: float,
    max_mag: float,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    """Return a function that applies a curve to an input.

    Arguments:
    mapping -- maps input to output
    offset -- added to output, even if the input is deadbanded
    deadband -- when the input magnitude is less than this,
        the input is treated as zero
    max_mag -- restricts the output magnitude to a maximum.
        If this is 0, no restriction is applied.
    absolute_offset -- If true, applies offset always (even when deadbanded),
        If false, adds sign(input_val) * offset or 0 in the deadband
    """

    def f(input_val: float) -> float:
        """Apply a curve to an input. Be sure to call this function to get an output, not curve."""
        if abs(input_val) < deadband:
            return offset if absolute_offset else 0
        applied_offset = (1 if absolute_offset else abs(input_val) / input_val) * offset
        output_val = mapping(input_val) + applied_offset
        if max_mag == 0:
            return output_val
        else:
            return clamp(output_val, -max_mag, max_mag)

    return f


def linear_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(lambda x: scalar * x, offset, deadband, max_mag, absolute_offset)


def ollie_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(
        lambda x: scalar * x * abs(x), offset, deadband, max_mag, absolute_offset
    )


def cubic_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(lambda x: scalar * x**3, offset, deadband, max_mag, absolute_offset)



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
