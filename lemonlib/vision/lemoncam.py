import robotpy_fields
import wpilib
from wpimath import ChassisVelocities, Pose2d, Transform3d, units

from photonlibpy.targeting import PhotonPipelineResult

from . import LimelightResults, NullCamera, limelight, photon
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

        if photon.photonname.camera_exists(self.camera_name):
            self._camera = photon.photonname(
                self.camera_name, self.robot_to_camera, self.field
            )
        elif limelight.Lemonlight.camera_exists(self.camera_name):
            self._camera = limelight.Lemonlight(
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

    def get_std_devs(self) -> tuple[float, float, float]:

        self._check_camera_exist()
        return self._camera.get_std_devs()

    def estimate_pose_debug(self):
        self._check_camera_exist()
        return self._camera.estimate_pose_debug()

    def estimate_multi_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
        drivetrain_velocities: ChassisVelocities | None = None,
    ) -> Pose2d | None:

        self._check_camera_exist()
        return self._camera.estimate_multi_tag_pose(robot_pose, drivetrain_velocities)

    def estimate_single_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
        drivetrain_velocities: ChassisVelocities | None = None,
    ) -> Pose2d | None:

        self._check_camera_exist()
        return self._camera.estimate_single_tag_pose(robot_pose, drivetrain_velocities)

    def get_latest_results(self) -> list[PhotonPipelineResult] | LimelightResults:

        self._check_camera_exist()
        return self._camera.get_latest_results()

    def has_target(self) -> bool:

        self._check_camera_exist()
        return self._camera.has_target()

    def get_timestamp(self) -> float:

        self._check_camera_exist()
        return self._camera.get_timestamp()

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

        if photon.photonname(
            self.camera_name, self.robot_to_camera, self.field
        ).is_connected():
            self._camera = photon.photonname(
                self.camera_name, self.robot_to_camera, self.field
            )
        elif limelight.Lemonlight.camera_exists(self.camera_name):
            self._camera = limelight.Lemonlight(
                self.camera_name, self.robot_to_camera, self.field
            )
        else:
            self._camera = NullCamera(
                self.camera_name, self.robot_to_camera, self.field
            )
            wpilib.report_warning(self._report_camera_not_found)
