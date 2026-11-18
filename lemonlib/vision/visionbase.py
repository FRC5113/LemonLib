from abc import ABC, abstractmethod
from typing import overload

import robotpy_fields
from photonlibpy.targeting import PhotonPipelineResult
from wpimath import Pose2d, Rotation2d, Transform3d, units

from ._util import PoseEstimates
from .limelightutils import LimelightResults


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

    @overload
    def estimate_multi_tag_pose(self, robot_pose: Pose2d) -> PoseEstimates: ...

    @overload
    def estimate_multi_tag_pose(self) -> PoseEstimates: ...

    @abstractmethod
    def estimate_multi_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
    ) -> PoseEstimates: ...

    @overload
    def estimate_single_tag_pose(
        self,
        heading: Rotation2d,
    ) -> PoseEstimates: ...

    @overload
    def estimate_single_tag_pose(
        self,
        heading: Rotation2d,
        robot_pose: Pose2d,
    ) -> PoseEstimates: ...

    @abstractmethod
    def estimate_single_tag_pose(
        self,
        heading: Rotation2d,
        robot_pose: Pose2d | None = None,
    ) -> PoseEstimates: ...

    @abstractmethod
    def get_latest_results(
        self,
    ) -> list[PhotonPipelineResult] | list[LimelightResults]: ...

    @abstractmethod
    def has_target(self) -> bool: ...

    @abstractmethod
    def set_std_baselines(
        self,
        linear_baseline: units.meters = 0.10,
        angular_baseline: units.radians = 0.174,
    ): ...

    @abstractmethod
    def camera_exists(self, camera_name: str) -> bool: ...
