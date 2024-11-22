from wpilib import Preferences, SmartDashboard
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpiutil import Sendable, SendableBuilder

from preference.smartgain import SmartGain
from preference.smartcontroller import SmartController


class SmartProfile(Sendable):
    """Stores several gains that are commonly used for control. This
    class allows the use of NetworkTables to change the gains on the
    fly, and uses Preferences so that the gains are stored locally on
    the robot. Use the `create_controller()` method to create a
    `SmartController` with synchronized gains.
    """

    def __init__(
        self,
        key: str,
        kP=0.0,
        kI=0.0,
        kD=0.0,
        kS=0.0,
        kV=0.0,
        kA=0.0,
        kMaxV=0.0,
        kMaxA=0.0,
        continuous_range: tuple[float] = None,
        low_bandwidth: bool = False,
    ) -> None:
        """Create a SmartProfile with the designated gains. There will
        only be a trapezoidal profile if `kMaxV` and `kMaxA` are set.
        The input will only be continuous if `continuous_range` is set.

        Args:
            key (str): Prefix of gains in SmartDashboard
            kP (float, optional): Proportional term. Defaults to 0.0.
            kI (float, optional): Integral term. Defaults to 0.0.
            kD (float, optional): Derivative term. Defaults to 0.0.
            kS (float, optional): Static voltage. Defaults to 0.0.
            kV (float, optional): Cruise voltage. Defaults to 0.0.
            kA (float, optional): Acceleration voltage. Defaults to 0.0.
            kMaxV (float, optional): Maximum velocity. Defaults to 0.0.
            kMaxA (float, optional): Maximum acceleration. Defaults to 0.0.
            continuous_range (tuple[float], optional): Tuple containing
                minimum and maximum value. Use to specify continuous
                input. Defaults to None.
        """
        Sendable.__init__(self)
        self._gains = (
            SmartGain(
                f"{key}_kP",
                kP,
                (lambda controller, value: controller.setP(value)),
                low_bandwidth,
            ),
            SmartGain(
                f"{key}_kI",
                kI,
                (lambda controller, value: controller.setI(value)),
                low_bandwidth,
            ),
            SmartGain(
                f"{key}_kD",
                kD,
                (lambda controller, value: controller.setD(value)),
                low_bandwidth,
            ),
            SmartGain(
                f"{key}_kS",
                kS,
                (lambda controller, value: controller.setS(value)),
                low_bandwidth,
            ),
            SmartGain(
                f"{key}_kV",
                kV,
                (lambda controller, value: controller.setV(value)),
                low_bandwidth,
            ),
            SmartGain(
                f"{key}_kA",
                kA,
                (lambda controller, value: controller.setA(value)),
                low_bandwidth,
            ),
            SmartGain(
                f"{key}_kMaxV",
                kMaxV,
                (lambda controller, value: controller.setMaxV(value)),
                low_bandwidth,
            ),
            SmartGain(
                f"{key}_kMaxA",
                kMaxA,
                (lambda controller, value: controller.setMaxA(value)),
                low_bandwidth,
            ),
        )
        self.continuous_range = continuous_range
        self.low_bandwidth = low_bandwidth
        if not low_bandwidth:
            SmartDashboard.putData(f"{key}_profile", self)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("SmartProfile")
        for i in range(len(self._gains)):
            builder.addDoubleProperty(
                self._gains[i].key, self._gains[i].get, self._create_setter(i)
            )

    def _create_setter(self, index):
        # used to avoid late binding
        return lambda x: self._gains[index].set(x)

    def create_controller(self, key) -> SmartController:
        """Creates new `SmartController` with synchronized gains. This
        should be the only way that SmartControllers are created.

        Args:
            period_getter: function that returns period

        Returns:
            SmartController: Created SmartController
        """
        controller = SmartController(key, self._gains, self.low_bandwidth)
        if self.continuous_range is not None:
            controller.enableContinuousInput(
                self.continuous_range[0], self.continuous_range[1]
            )
        return controller


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
