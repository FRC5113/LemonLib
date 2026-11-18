from enum import IntEnum
from typing import overload

import robotpy_fields
import wpilib
from photonlibpy.targeting import PhotonPipelineResult
from wpimath import Pose2d, Rotation2d, Transform3d, units

from ._nullcam import NullCamera
from ._util import PoseEstimates
from .limelight import Lemonlight
from .limelightutils import LimelightResults
from .photon import photonname
from .visionbase import VisionBase


class CameraType(IntEnum):
    PHOTONVISION = 0
    LIMELIGHT = 1


class LemonCamera(VisionBase):
    def __init__(
        self,
        camera_name: str,
        robot_to_camera: Transform3d,
        field: robotpy_fields.Field,
        camera_type: CameraType = CameraType.PHOTONVISION,
    ):
        self.camera_name = camera_name
        self.robot_to_camera = robot_to_camera
        self.field = field
        self._report_camera_not_found = f"Camera {camera_name} not found, make sure it and its coprocessor are connected"

        if camera_type == CameraType.PHOTONVISION:
            self._camera = photonname(camera_name, robot_to_camera, field)

        elif camera_type == CameraType.LIMELIGHT:
            self._camera = Lemonlight(camera_name, robot_to_camera, field)

        else:
            self._camera = NullCamera(
                self.camera_name, self.robot_to_camera, self.field
            )
            wpilib.report_warning(self._report_camera_not_found)

    def is_connected(self) -> bool:
        return self._camera.is_connected()

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
        return self._camera.estimate_multi_tag_pose(robot_pose)

    @overload
    def estimate_single_tag_pose(self) -> PoseEstimates: ...

    @overload
    def estimate_single_tag_pose(
        self,
        robot_pose: Pose2d,
    ) -> PoseEstimates: ...

    @overload
    def estimate_single_tag_pose(
        self,
        robot_pose: None,
    ) -> PoseEstimates: ...

    def estimate_single_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
    ) -> PoseEstimates:
        return self._camera.estimate_single_tag_pose(robot_pose)

    def get_latest_results(self) -> list[PhotonPipelineResult] | list[LimelightResults]:
        return self._camera.get_latest_results()

    def has_target(self) -> bool:
        return self._camera.has_target()

    def set_std_baselines(
        self,
        linear_baseline: units.meters = 0.10,
        angular_baseline: units.radians = 0.174,
    ):
        return self._camera.set_std_baselines(linear_baseline, angular_baseline)

    def add_heading_data(self, timestamp: units.seconds, heading: Rotation2d) -> None:
        self._camera.add_heading_data(timestamp, heading)
