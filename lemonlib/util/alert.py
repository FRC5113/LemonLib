from enum import Enum
from logging import Logger

from wpilib import DriverStationBackend, Timer, RobotController
import tunables
import telemetry

from .elastic import Notification, NotificationLevel, send_notification


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
            self.active_start_time = Timer.get_monotonic_timestamp()
            self._log(self.text)

            # Send notification to Elastic dashboard (display time is in ms).
            notification = Notification(
                level=NotificationLevel[self.type.name],
                title="Robot Alert",
                description=self.text,
                display_time=int(self.timeout * 1000) if self.timeout > 0 else 3000,
            )
            if self.elasticnoti:
                send_notification(notification)

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
            and Timer.get_monotonic_timestamp() - self.last_log > 1.0
        ):
            self.last_log = Timer.get_monotonic_timestamp()
            self._log(text)
        self.text = text

    def _log(self, text: str):
        """Log text to the AlertManager logger at this alert's severity."""
        logger = AlertManager.logger
        if logger is None:
            return
        match self.type:
            case AlertType.ERROR:
                logger.error(text)
            case AlertType.WARNING:
                logger.warning(text)
            case AlertType.INFO:
                logger.info(text)


class AlertManager:
    """
    Manages a collection of alerts and integrates with the SmartDashboard.
    """

    alerts: list[Alert] = []
    logger: Logger = None

    def __init__(self, logger, enabled: bool = True):
        """
        Initialize the AlertManager.

        Args:
            logger (Logger): Logger instance for logging alert messages.
            enabled (bool): Whether to publish alerts to dashboard.
        """
        AlertManager.logger = logger
        if enabled and not DriverStationBackend.is_fms_attached():
            table = tunables.get_table("Alerts")
            table.publish_string_array(
                "errors",
                lambda: AlertManager.get_strings(AlertType.ERROR),
                lambda _: None,
            )
            table.publish_string_array(
                "warnings",
                lambda: AlertManager.get_strings(AlertType.WARNING),
                lambda _: None,
            )
            table.publish_string_array(
                "infos",
                lambda: AlertManager.get_strings(AlertType.INFO),
                lambda _: None,
            )

    @staticmethod
    def get_strings(type: AlertType) -> list[str]:
        """
        Retrieve active alerts of a specified type as strings.

        Args:
            type (AlertType): The type of alerts to retrieve.

        Returns:
            List[str]: List of alert messages.
        """
        alerts = []
        timestamp = Timer.get_monotonic_timestamp()
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
