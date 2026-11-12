from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

import ntcore
import robotpy_fields
from wpimath import ChassisVelocities, Pose2d, Pose3d, Rotation3d, Transform3d, units

from photonlibpy.targeting import PhotonPipelineResult

__all__ = ["LemonCamera", "Lemonlight", "photonname"]


@dataclass
class MegaTagResult:
    pose: Pose3d = field(default_factory=Pose3d)
    latency: float = 0.0
    tag_count: float = 0.0
    tag_span: float = 0.0
    avg_tag_distance: float = 0.0
    avg_tag_area: float = 0.0
    ts: float = 0.0


@dataclass
class MegaTagStdDevs:
    x: units.meters = 0.0
    y: units.meters = 0.0
    z: units.meters = 0.0
    rot: Rotation3d = field(default_factory=Rotation3d)
    ts: float = 0.0


@dataclass
class LimelightResults:
    megatag1: list[MegaTagResult] = field(default_factory=list)
    megatag2: list[MegaTagResult] = field(default_factory=list)
    megatag1_std_devs: list[MegaTagStdDevs] = field(default_factory=list)
    megatag2_std_devs: list[MegaTagStdDevs] = field(default_factory=list)


@staticmethod
def _transform3d_to_list(transform: Transform3d) -> list[float]:
    arr = []
    rot = transform.rotation()

    arr.append(transform.x)
    arr.append(transform.y)
    arr.append(transform.z)
    arr.append(rot.x)
    arr.append(rot.y)
    arr.append(rot.z)
    return arr


class NullCamera:
    def __init__(
        self,
        camera_name: str,
        robot_to_camera: Transform3d,
        field: robotpy_fields.Field,
    ) -> None:
        pass

    def is_connected(self) -> bool:
        return False

    def get_std_devs(self) -> tuple[float, float, float]:
        return (0.0, 0.0, 0.0)

    def estimate_multi_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
        drivetrain_velocities: ChassisVelocities | None = None,
    ) -> Pose2d | None:
        return None

    def estimate_single_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
        drivetrain_velocities: ChassisVelocities | None = None,
    ) -> Pose2d | None:
        return None

    def get_latest_results(self) -> Any:
        return None

    def has_target(self) -> bool:
        return False

    def set_std_baselines(
        self,
        linear_baseline: units.meters = 0.10,
        angular_baseline: units.radians = 0.174,
    ):
        pass

    def camera_exists(self, camera_name: str) -> bool:
        return False

    def get_timestamp(self) -> float:
        return 0.0

    def estimate_pose_debug(self):
        return None
