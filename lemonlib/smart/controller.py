import telemetry


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
        self.tolerance = 0.0

        self.controller_table = (
            telemetry.get_table(f"SmartController/{key}") if feedback_enabled else None
        )

    def setTolerance(self, error_tolerance: float):
        """Sets the error tolerance for the controller."""
        self.tolerance = error_tolerance

    def at_setpoint(self) -> bool:
        """Checks if the controller is at the setpoint within the tolerance."""
        return abs(self.error) < self.tolerance

    def getError(self) -> float:
        """Returns the current error of the controller."""
        return self.error

    def getOutput(self) -> float:
        """Returns the current output of the controller."""
        return self.output

    def getReference(self) -> float:
        """Returns the current reference value of the controller."""
        return self.reference

    def getMeasurement(self) -> float:
        """Returns the current measurement value of the controller."""
        return self.measurement

    def calculate(self, measurement: float, reference: float):
        self.reference = reference
        self.measurement = measurement
        self.error = reference - measurement

        if abs(self.error) < self.tolerance:
            self.output = 0.0
        else:
            self.output = self._calculate_method(measurement, reference)

        if self.controller_table is not None:
            self.controller_table.log("Reference", self.reference)
            self.controller_table.log("Measurement", self.measurement)
            self.controller_table.log("Error", self.error)
            self.controller_table.log("Output", self.output)
        return self.output
