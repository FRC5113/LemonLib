from enum import Enum
from typing import List
from logging import Logger

from wpiutil import Sendable, SendableBuilder
from wpilib import Timer, SmartDashboard


class AlertType(Enum):
    ERROR = 0
    WARNING = 1
    INFO = 2


class Alert:
    def __init__(self, text: str, type: AlertType, timeout: float = 0.0):
        self.text = text
        self.type = type
        self.timeout = timeout
        self.active = False
        self.active_start_time = 0.0
        self.last_log = 0.0
        AlertManager.alerts.append(self)

    def set(self, active: bool):
        if active and not self.active:
            self.active_start_time = Timer.getFPGATimestamp()
            match self.type:
                case AlertType.ERROR:
                    AlertManager.logger.error(self.text)
                case AlertType.WARNING:
                    AlertManager.logger.warning(self.text)
                case AlertType.INFO:
                    AlertManager.logger.info(self.text)
        self.active = active

    def enable(self):
        self.set(True)

    def disable(self):
        self.set(False)

    def set_text(self, text: str):
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
    alerts: List[Alert] = []
    logger: Logger = None

    def __init__(self, logger):
        Sendable.__init__(self)
        AlertManager.logger = logger
        SmartDashboard.putData("Alerts", self)

    def initSendable(self, builder: SendableBuilder) -> None:
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

    def get_strings(type: AlertType):
        alerts = []
        timestamp = Timer.getFPGATimestamp()
        for alert in AlertManager.alerts:
            if not alert.active:
                continue
            if alert.timeout > 0.0:
                if timestamp - alert.active_start_time >= alert.timeout:
                    alert.set(False)
                    continue
            if alert.type == type:
                alerts.append(alert)
        return [
            alert.text
            for alert in sorted(alerts, key=lambda alert: alert.active_start_time)
        ]

    def instant_alert(text: str, type: AlertType, timeout: float = 0.0):
        """Add new alert and instantly enable it"""
        alert = Alert(text, type, timeout)
        alert.enable()
