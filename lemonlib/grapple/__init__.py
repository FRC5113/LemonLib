from libgrapplefrc import (
    LaserCAN as _LaserCAN,
)
from libgrapplefrc import (
    LaserCanMeasurement as _LaserCanMeasurement,
)
from libgrapplefrc import (
    LaserCanRangingMode,
    LaserCanTimingBudget,
    can_bridge_tcp,
)
from libgrapplefrc import (
    LaserCanRoi as _LaserCanRoi,
)
from libgrapplefrc import (
    MitoCANdria as _MitoCANdria,
)

__all__ = [
    "can_bridge_tcp",
    "LaserCAN",
    "LaserCanMeasurement",
    "LaserCanRangingMode",
    "LaserCanRoi",
    "LaserCanTimingBudget",
    "MitoCANdria",
]


class LaserCAN(_LaserCAN):
    """Wrapper for LaserCAN sensor."""

    pass


class LaserCanMeasurement(_LaserCanMeasurement):
    """Measurement result from LaserCAN."""

    pass


class LaserCanRoi(_LaserCanRoi):
    """Region of interest for LaserCAN."""

    pass


class MitoCANdria(_MitoCANdria):
    """CAN communication abstraction."""

    pass
