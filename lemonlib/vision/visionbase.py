from abc import ABC, abstractmethod

import robotpy_fields
from wpimath import ChassisVelocities, Pose2d, Transform3d, units

from photonlibpy.targeting import PhotonPipelineResult

from . import LimelightResults


class VisionBase(ABC):
    @abstractmethod
    def __init__(
        self,
        camera_name: str,
        robot_to_camera: Transform3d,
        field: robotpy_fields.Field,
    ) -> None: ...

    @abstractmethod
    def is_connected(self) -> bool: ...

    @abstractmethod
    def get_std_devs(self) -> tuple[float, float, float]: ...

    @abstractmethod
    def estimate_multi_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
        drivetrain_velocities: ChassisVelocities | None = None,
    ) -> Pose2d | None: ...

    @abstractmethod
    def estimate_single_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
        drivetrain_velocities: ChassisVelocities | None = None,
    ) -> Pose2d | None: ...

    @abstractmethod
    def get_latest_results(self) -> list[PhotonPipelineResult] | LimelightResults: ...

    @abstractmethod
    def has_target(self) -> bool: ...

    @abstractmethod
    def get_timestamp(self) -> float: ...

    @abstractmethod
    def set_std_baselines(
        self,
        linear_baseline: units.meters = 0.10,
        angular_baseline: units.radians = 0.174,
    ): ...

    @abstractmethod
    def camera_exists(self, camera_name: str) -> bool: ...
