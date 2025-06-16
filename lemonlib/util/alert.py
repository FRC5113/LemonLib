from enum import Enum
from logging import Logger
from typing import List, Dict
from wpilib import SmartDashboard, Timer
from wpiutil import Sendable, SendableBuilder
from ntcore import NetworkTableInstance, PubSubOptions
import json
from .elastic import Notification, send_notification


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
                case AlertType.ERROR.value:
                    AlertManager.logger.error(self.text)
                case AlertType.WARNING.value:
                    AlertManager.logger.warning(self.text)
                case AlertType.INFO.value:
                    AlertManager.logger.info(self.text)

            # Send notification to Elastic dashboard.
            notification = Notification(
                level=self.type.name,
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
