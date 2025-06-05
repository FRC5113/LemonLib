from wpilib import SmartDashboard
from wpiutil import Sendable, SendableBuilder
from wpilib import SmartDashboard
from wpiutil import Sendable, SendableBuilder
from .nettables import SmartNT


class SmartController:
    """Used as a general wrapper for a variety of controllers that may
    optionally report values to NetworkTables. It is recommended to
    create these using a `SmartProfile`.
    """

    def __init__(self, key: str, calculate_method, feedback_enabled):
        self._calculate_method = calculate_method
        self.reference = 0
        self.measurement = 0
        self.error = 0
        self.output = 0
        self.nt = SmartNT(f"SmartController/{key}_controller", verbose=False)

        if feedback_enabled:
            self.initSendable()

    def initSendable(self):
        self.nt.put("Reference", self.reference)
        self.nt.put("Measurement", self.measurement)
        self.nt.put("Error", self.error)
        self.nt.put("Output", self.output)

    def calculate(self, measurement: float, reference: float):
        self.reference = reference
        self.measurement = measurement
        self.error = reference - measurement
        self.output = self._calculate_method(measurement, reference)
        return self.output
