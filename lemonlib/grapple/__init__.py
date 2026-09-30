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
    "LaserCAN",
    "LaserCanMeasurement",
    "LaserCanRangingMode",
    "LaserCanRoi",
    "LaserCanTimingBudget",
    "MitoCANdria",
    "can_bridge_tcp",
]


class LaserCAN(_LaserCAN):
    """Wrapper for LaserCAN sensor."""


class LaserCanMeasurement(_LaserCanMeasurement):
    """Measurement result from LaserCAN."""


class LaserCanRoi(_LaserCanRoi):
    """Region of interest for LaserCAN."""


class MitoCANdria(_MitoCANdria):
    """CAN communication abstraction."""
