from wpilib import Preferences


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
        if self._type is int or self._type is float:
            Preferences.initDouble(self._key, self._value)
        elif self._type is str:
            Preferences.initString(self._key, self._value)
        elif self._type is bool:
            Preferences.initBoolean(self._key, self._value)

    def __get__(self, obj, objtype=None):
        if self._low_bandwidth:
            return self._value
        new = None
        if self._type is int or self._type is float:
            new = Preferences.getDouble(self._key, self._value)
        elif self._type is str:
            new = Preferences.getString(self._key, self._value)
        elif self._type is bool:
            new = Preferences.getBoolean(self._key, self._value)
        if new != self._value:
            SmartPreference._changed_flag = True
            self._value = new
        return self._value

    def __set__(self, obj, value):
        if type(value) is not self._type:
            raise TypeError(
                f"Set value type ({type(value)} does not match original ({self._type}))"
            )
        self._value = value
        self._type = type(value)
        if self._low_bandwidth:
            return
        if self._type is int or self._type is float:
            self._value = Preferences.setDouble(self._key, self._value)
        elif self._type is str:
            self._value = Preferences.setString(self._key, self._value)
        elif self._type is bool:
            self._value = Preferences.setBoolean(self._key, self._value)

    def has_changed() -> bool:
        """Returns if any SmartPreference has changed since checked.
        Only works if called statically."""
        if SmartPreference._changed_flag:
            SmartPreference._changed_flag = False
            return True
        return False
