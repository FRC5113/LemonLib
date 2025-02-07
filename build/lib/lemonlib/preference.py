from wpilib import Preferences, SmartDashboard
from wpimath.trajectory import TrapezoidProfile
from wpiutil import Sendable, SendableBuilder
from wpilib import Preferences, SmartDashboard
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import (
    PIDController,
    ProfiledPIDController,
    SimpleMotorFeedforwardMeters,
    ElevatorFeedforward,
)
from wpiutil import Sendable, SendableBuilder


class SmartController(Sendable):
    """Used as a general wrapper for a variety of controllers that may
    optionally report values to NetworkTables. It is recommended to
    create these using a `SmartProfile`.
    """

    def __init__(self, key: str, calculate_method, feedback_enabled):
        Sendable.__init__(self)
        self._calculate_method = calculate_method
        self.reference = 0
        self.measurement = 0
        self.error = 0
        self.output = 0
        if feedback_enabled:
            SmartDashboard.putData(f"{key}_controller", self)

    def initSendable(self, builder: SendableBuilder):
        builder.setSmartDashboardType("SmartController")
        builder.addDoubleProperty("Reference", lambda: self.reference, lambda _: None)
        builder.addDoubleProperty(
            "Measurement", lambda: self.measurement, lambda _: None
        )
        builder.addDoubleProperty("Error", lambda: self.error, lambda _: None)
        builder.addDoubleProperty("Output", lambda: self.output, lambda _: None)

    def calculate(self, measurement: float, reference: float):
        self.reference = reference
        self.measurement = measurement
        self.error = reference - measurement
        self.output = self._calculate_method(measurement, reference)
        return self.output


class SmartProfile(Sendable):
    """Used to store multiple gains and configuration values for several
    different types of controllers. This can optionally interface with
    NetworkTables so that the gains may be dynamically updated without
    needing to redeploy code. This class has several helper methods
    which can be used to create `SmartController` objects already supplied
    with the necessary gains. Please note that gains will NOT be
    dynamically updated in `SmartController` objects: therefore it is
    recommended to create a new `SmartController` object on enable.
    """

    def __init__(self, profile_key: str, gains: dict[str, float], tuning_enabled: bool):
        """Creates a SmartProfile.
        Recommended gain keys (for use with `SmartController`):
        kP: Proportional Gain
        kI: Integral Gain
        kD: Derivative Gain
        kS: Static Gain
        kV: Velocity Gain
        kG: Gravity Gain
        kA: Acceleration Gain
        kMinInput: Minimum expected measurement value (used for continuous input)
        kMaxInput: Maximum expected measurement value (used for continuous input)

        :param str profile_key: Prefix for associated NetworkTables keys
        :param dict[str, float] gains: Dictionary containing gain_key: value pairs
        :param bool tuning_enabled: Specify whether or not to send and retrieve
            data from NetworkTables. If true, values from NetworkTables
            are given precedence over values set in code.
        """
        Sendable.__init__(self)
        self.profile_key = profile_key
        self.tuning_enabled = tuning_enabled
        self.gains = gains
        if tuning_enabled:
            for gain in gains:
                Preferences.initDouble(f"{profile_key}_{gain}", gains[gain])
                self.gains[gain] = Preferences.getDouble(
                    f"{profile_key}_{gain}", gains[gain]
                )
            SmartDashboard.putData(f"{profile_key}_profile", self)

    def initSendable(self, builder: SendableBuilder):
        builder.setSmartDashboardType("SmartProfile")
        for gain_key in self.gains:
            builder.addDoubleProperty(
                gain_key,
                # optional arguments used to hackily avoid late binding
                (lambda key=gain_key: self.gains[key]),
                (lambda value, key=gain_key: self._set_gain(key, value)),
            )

    def _set_gain(self, key: str, value: float):
        self.gains[key] = value
        if self.tuning_enabled:
            Preferences.setDouble(f"{self.profile_key}_{key}", value)

    def _requires(requirements: set[str]):
        def inner(func):
            def wrapper(self, key, feedback_enabled=None):
                missing_reqs = requirements - set(self.gains.keys())
                assert (
                    len(missing_reqs) == 0
                ), f"Requires gains: {', '.join(missing_reqs)}"
                return func(self, key, feedback_enabled)

            return wrapper

        return inner

    @_requires({"kP", "kI", "kD"})
    def create_pid_controller(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a PID controller.
        Requires kP, kI, kD, [kMinInput, kMaxInput optional]
        """
        controller = PIDController(self.gains["kP"], self.gains["kI"], self.gains["kD"])
        if "kMinInput" in self.gains.keys() and "kMaxInput" in self.gains.keys():
            controller.enableContinuousInput(
                self.gains["kMinInput"], self.gains["kMaxInput"]
            )
        return SmartController(
            key,
            (lambda y, r: controller.calculate(y, r)),
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )

    @_requires({"kP", "kI", "kD", "kMaxV", "kMaxA"})
    def create_profiled_pid_controller(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a profiled PID controller.
        Requires kP, kI, kD, kMaxV, kMaxA, [kMinInput, kMaxInput optional]
        """
        controller = ProfiledPIDController(
            self.gains["kP"],
            self.gains["kI"],
            self.gains["kD"],
            TrapezoidProfile.Constraints(self.gains["kMaxV"], self.gains["kMaxA"]),
        )
        if "kMinInput" in self.gains.keys() and "kMaxInput" in self.gains.keys():
            controller.enableContinuousInput(
                self.gains["kMinInput"], self.gains["kMaxInput"]
            )
        return SmartController(
            key,
            (lambda y, r: controller.calculate(y, r)),
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )

    @_requires({"kS", "kV"})
    def create_simple_feedforward(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a simple DC motor feedforward controller.
        Requires kS, kV, [kA optional]
        """
        controller = SimpleMotorFeedforwardMeters(
            self.gains["kS"],
            self.gains["kV"],
            self.gains["kA"] if "kA" in self.gains else 0,
        )
        return SmartController(
            key,
            (lambda y, r: controller.calculate(r)),
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )

    @_requires({"kP", "kI", "kD", "kS", "kV"})
    def create_flywheel_controller(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a PID controller combined with a DC motor feedforward controller.
        Requires kP, kI, kD, kS, kV, [kA optional]
        """
        pid = PIDController(self.gains["kP"], self.gains["kI"], self.gains["kD"])
        if "kMinInput" in self.gains.keys() and "kMaxInput" in self.gains.keys():
            pid.enableContinuousInput(self.gains["kMinInput"], self.gains["kMaxInput"])
        feedforward = SimpleMotorFeedforwardMeters(
            self.gains["kS"],
            self.gains["kV"],
            self.gains["kA"] if "kA" in self.gains else 0,
        )
        return SmartController(
            key,
            (lambda y, r: pid.calculate(y, r) + feedforward.calculate(r)),
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )

    @_requires({"kP", "kI", "kD", "kS", "kV", "kMaxV", "kMaxA"})
    def create_turret_controller(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a profiled PID controller combined with a DC motor feedforward controller.
        Requires kP, kI, kD, kS, kV, [kA, kMinInput, kMaxInput optional]
        """
        pid = ProfiledPIDController(
            self.gains["kP"],
            self.gains["kI"],
            self.gains["kD"],
            TrapezoidProfile.Constraints(self.gains["kMaxV"], self.gains["kMaxA"]),
        )
        if "kMinInput" in self.gains.keys() and "kMaxInput" in self.gains.keys():
            pid.enableContinuousInput(self.gains["kMinInput"], self.gains["kMaxInput"])
        feedforward = SimpleMotorFeedforwardMeters(
            self.gains["kS"],
            self.gains["kV"],
            self.gains["kA"] if "kA" in self.gains else 0,
        )

        def calculate(y, r):
            pid_output = pid.calculate(y, r)
            setpoint = pid.getSetpoint()
            # add acceleration eventually
            feedforward_output = feedforward.calculate(setpoint.velocity)
            return pid_output + feedforward_output

        return SmartController(
            key,
            calculate,
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )

    @_requires({"kP", "kI", "kD", "kS", "kG", "kV", "kMaxV", "kMaxA"})
    def create_elevator_controller(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a profiled PID controller combined with an elevator feedforward controller.
        Requires kP, kI, kD, kS, kV, kG, [kA optional]
        """
        pid = ProfiledPIDController(
            self.gains["kP"],
            self.gains["kI"],
            self.gains["kD"],
            TrapezoidProfile.Constraints(self.gains["kMaxV"], self.gains["kMaxA"]),
        )
        feedforward = ElevatorFeedforward(
            self.gains["kS"],
            self.gains["kG"],
            self.gains["kV"],
            self.gains["kA"] if "kA" in self.gains else 0,
        )

        def calculate(y, r):
            pid_output = pid.calculate(y, r)
            setpoint = pid.getSetpoint()
            # add acceleration eventually
            feedforward_output = feedforward.calculate(setpoint.velocity)
            return pid_output + feedforward_output

        return SmartController(
            key,
            calculate,
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )


class SmartPreference(object):
    """Wrapper for wpilib Preferences that improves it in a few ways:
    1. Previous values from NetworkTables are remembered if connection
    is lost instead of defaulting to the values set in code
    2. Everything is done dynamically so there is no need to specify
    type. However, because of NT limitations, the type must stay the
    same throughout the entirety of the code
    3. Including `low_bandwidth = True` as a class attribute will stop
    the `SmartPreference` from referencing NT and simply use defaults
    3. Initializing, getting, and setting Preferences is made much
    easier and enables this class to be a drop-in replacement for normal
    values. For example:
    ```
    class MyComponent:

        # initialize a preference with NT key "foo" and default value True
        # SmartPreferences MUST be class attributes (ie. initialized under the header)
        # Values must be of type int, float, str, or bool
        foo = SmartPreference(True)

        def execute(self):

            # retrieve the preference from NT (defaults to previous value)
            foo = self.foo

            # set the preference in NT
            self.foo = False
    ```
    """

    _changed_flag = False

    def __init__(self, value) -> None:
        self._value = value
        self._type = type(value)
        if self._type not in (int, float, str, bool):
            raise TypeError(
                f"SmartPreference must be int, float, str, or bool (not {self._type})"
            )

    def __set_name__(self, obj, name):
        try:
            self._low_bandwidth = obj.low_bandwidth
        except:
            self._low_bandwidth = False
        self._key = name
        if self._low_bandwidth:
            return
        if self._type == int or self._type == float:
            Preferences.initDouble(self._key, self._value)
        elif self._type == str:
            Preferences.initString(self._key, self._value)
        elif self._type == bool:
            Preferences.initBoolean(self._key, self._value)

    def __get__(self, obj, objtype=None):
        if self._low_bandwidth:
            return self._value
        new = None
        if self._type == int or self._type == float:
            new = Preferences.getDouble(self._key, self._value)
        elif self._type == str:
            new = Preferences.getString(self._key, self._value)
        elif self._type == bool:
            new = Preferences.getBoolean(self._key, self._value)
        if new != self._value:
            SmartPreference._changed_flag = True
            self._value = new
        return self._value

    def __set__(self, obj, value):
        if type(value) != self._type:
            raise TypeError(
                f"Set value type ({type(value)} does not match original ({self._type}))"
            )
        self._value = value
        self._type = type(value)
        if self._low_bandwidth:
            return
        if self._type == int or self._type == float:
            self._value = Preferences.setDouble(self._key, self._value)
        elif self._type == str:
            self._value = Preferences.setString(self._key, self._value)
        elif self._type == bool:
            self._value = Preferences.setBoolean(self._key, self._value)

    def has_changed() -> bool:
        """Returns if any SmartPreference has changed since checked.
        Only works if called statically."""
        if SmartPreference._changed_flag:
            SmartPreference._changed_flag = False
            return True
        return False
