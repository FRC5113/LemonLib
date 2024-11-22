from wpilib import Preferences, SmartDashboard
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpiutil import Sendable, SendableBuilder

class SmartController(ProfiledPIDController, Sendable):
    """Wraps a `ProfiledPIDController` and a `SimpleMotorFeedforward`
    together and uses Preferences to allow for dynamic gain setting.
    This should **only** be created from the `create_controller()`
    method in `SmartProfile`
    """

    def __init__(self, key, gains, low_bandwidth) -> None:
        ProfiledPIDController.__init__(
            self, 0, 0, 0, TrapezoidProfile.Constraints(0, 0), 0.02
        )
        self.feedforward = SimpleMotorFeedforwardMeters(0, 0, 0)
        Sendable.__init__(self)
        self._gains = gains
        for gain in self._gains:
            gain.update_controller(self)
        self._measurement = None
        self._output = 0
        if not low_bandwidth:
            SmartDashboard.putData(f"{key}_controller", self)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("SmartController")
        builder.addDoubleProperty(
            "Setpoint", lambda: self.getSetpoint().position, lambda _: None
        )
        builder.addDoubleProperty(
            "Goal", lambda: self.getGoal().position, lambda _: None
        )
        builder.addDoubleProperty(
            "Measurement", lambda: self.getMeasurement(), lambda _: None
        )
        builder.addDoubleProperty(
            "Error", lambda: self.getPositionError(), lambda _: None
        )
        builder.addDoubleProperty("Output", lambda: self.getOutput(), lambda _: None)

    def _update(self):
        for gain in self._gains:
            gain.update_controller(self)

    def calculate(self, measurement: float, goal: float = None) -> float:
        """Overridden. Get output based on a provided measurement and a
        goal.

        Args:
            measurement (float): measurement of the process variable
            goal (float, optional): goal position. Defaults to None.

        Returns:
            float: output from feedforward and profiledpid
        """
        self._measurement = measurement
        self._update()
        self._output = self.feedforward.calculate(self.getSetpoint().position)
        if goal is None:
            self._output += super().calculate(measurement)
        else:
            self.setGoal(goal)
            self._output += super().calculate(measurement)
        return self._output

    def getMeasurement(self) -> float:
        """Returns measurement most recently passed to `calculate()`"""
        if self._measurement is None:
            return 0
        return self._measurement

    def getOutput(self) -> float:
        """Returns most recent output from `calculate()`"""
        return self._output

    def setS(self, value: float) -> None:
        """Set feedfoward kS to `value`"""
        self.feedforward = SimpleMotorFeedforwardMeters(
            value, self.feedforward.kV, self.feedforward.kA
        )

    def setV(self, value: float) -> None:
        """Set feedfoward kV to `value`"""
        self.feedforward = SimpleMotorFeedforwardMeters(
            self.feedforward.kS, value, self.feedforward.kA
        )

    def setA(self, value: float) -> None:
        """Set feedfoward kA to `value`"""
        self.feedforward = SimpleMotorFeedforwardMeters(
            self.feedforward.kS, self.feedforward.kV, value
        )

    def setMaxV(self, value: float) -> None:
        """Set max velocity to `value`"""
        constraints = self.getConstraints()
        self.setConstraints(
            TrapezoidProfile.Constraints(value, constraints.maxAcceleration)
        )

    def setMaxA(self, value: float) -> None:
        """Set max acceleration to `value`"""
        constraints = self.getConstraints()
        self.setConstraints(
            TrapezoidProfile.Constraints(constraints.maxVelocity, value)
        )

