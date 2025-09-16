---
title: SmartPreference
slug: reference/smart/smartpreference
---

# SmartPreference

Wrapper for wpilib Preferences that improves it in a few ways:
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

## Methods

### has_changed()

Returns if any SmartPreference has changed since checked.
Only works if called statically.

