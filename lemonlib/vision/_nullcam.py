from typing import Any, overload

import robotpy_fields
from wpimath import (
    Pose2d,
    Rotation2d,
    Transform3d,
    units,
)

from ._util import PoseEstimates


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

    @overload
    def estimate_multi_tag_pose(self, robot_pose: Pose2d) -> PoseEstimates: ...

    @overload
    def estimate_multi_tag_pose(self, robot_pose: None) -> PoseEstimates: ...

    @overload
    def estimate_multi_tag_pose(self) -> PoseEstimates: ...

    def estimate_multi_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
    ) -> PoseEstimates:
        return PoseEstimates()

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

    @overload
    def estimate_single_tag_pose(
        self,
        heading: Rotation2d,
        robot_pose: None,
    ) -> PoseEstimates: ...

    def estimate_single_tag_pose(
        self,
        heading: Rotation2d,
        robot_pose: Pose2d | None = None,
    ) -> PoseEstimates:
        return PoseEstimates()

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

    def update(self):
        pass
