from wpilib import Preferences, SmartDashboard
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpiutil import Sendable, SendableBuilder

class SmartGain:
    """Used internally by SmartProfile and SmartController"""

    def __init__(self, key, value, updater, low_bandwidth):
        if not low_bandwidth:
            Preferences.initDouble(key, value)
        self.key = key
        self.value = value if low_bandwidth else Preferences.getDouble(key, value)
        self.updater = updater
        self.low_bandwidth = low_bandwidth

    def set(self, value):
        if value != self.value:
            self.value = value
            if self.low_bandwidth:
                return
            Preferences.setDouble(self.key, self.value)

    def get(self):
        if self.low_bandwidth:
            return self.value
        from_preferences = Preferences.getDouble(self.key, self.value)
        if self.value != from_preferences:
            self.value = from_preferences
        return self.value

    def update_controller(self, controller):
        self.updater(controller, self.value)