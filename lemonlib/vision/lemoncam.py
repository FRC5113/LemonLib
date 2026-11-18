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

_lastVersionTimeCheck = 0.0
_CHECK_EXIST = True


class LemonCamera(VisionBase):
    def __init__(
        self,
        camera_name: str,
        robot_to_camera: Transform3d,
        field: robotpy_fields.Field,
    ):
        self.camera_name = camera_name
        self.robot_to_camera = robot_to_camera
        self.field = field
        self._report_camera_not_found = f"Camera {camera_name} not found, make sure it and its coprocessor are connected"

        if photonname.camera_exists(self.camera_name):
            self._camera = photonname(
                self.camera_name, self.robot_to_camera, self.field
            )
        elif Lemonlight.camera_exists(self.camera_name):
            self._camera = Lemonlight(
                self.camera_name, self.robot_to_camera, self.field
            )
        else:
            self._camera = NullCamera(
                self.camera_name, self.robot_to_camera, self.field
            )
            wpilib.report_warning(self._report_camera_not_found)

    def is_connected(self) -> bool:

        self._check_camera_exist()
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

        self._check_camera_exist()
        return self._camera.estimate_multi_tag_pose(robot_pose)

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

        self._check_camera_exist()
        return self._camera.estimate_single_tag_pose(heading, robot_pose)

    def get_latest_results(self) -> list[PhotonPipelineResult] | list[LimelightResults]:

        self._check_camera_exist()
        return self._camera.get_latest_results()

    def has_target(self) -> bool:

        self._check_camera_exist()
        return self._camera.has_target()

    def set_std_baselines(
        self,
        linear_baseline: units.meters = 0.10,
        angular_baseline: units.radians = 0.174,
    ):
        self._check_camera_exist()
        return self._camera.set_std_baselines(linear_baseline, angular_baseline)

    def camera_exists(self, camera_name: str) -> bool:
        self._check_camera_exist()
        return self._camera.camera_exists(camera_name)

    def _check_camera_exist(self):
        global _lastVersionTimeCheck

        if not _CHECK_EXIST:
            return

        if (wpilib.Timer.get_monotonic_timestamp() - _lastVersionTimeCheck) < 5.0:
            return

        _lastVersionTimeCheck = wpilib.Timer.get_monotonic_timestamp()

        if photonname.camera_exists(self.camera_name):
            self._camera = photonname(
                self.camera_name, self.robot_to_camera, self.field
            )
        elif Lemonlight.camera_exists(self.camera_name):
            self._camera = Lemonlight(
                self.camera_name, self.robot_to_camera, self.field
            )
        else:
            self._camera = NullCamera(
                self.camera_name, self.robot_to_camera, self.field
            )
            wpilib.report_warning(self._report_camera_not_found)
