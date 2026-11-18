from typing import overload

import ntcore
import robotpy_fields
from wpimath import (
    Pose2d,
    Rotation2d,
    TimeInterpolatableRotation2dBuffer,
    Transform3d,
    units,
)

from ._util import PoseEstimates
from .limelightutils import LimelightResults, decode
from .visionbase import VisionBase
from .visionutil import VisionUtils


class Lemonlight(VisionBase):
    def __init__(
        self,
        camera_name: str,
        robot_to_camera: Transform3d,
        field: robotpy_fields.Field,
    ) -> None:
        self._camera_name = camera_name
        self._robot_to_camera = robot_to_camera
        self._field = field

        self.vision_util = VisionUtils()

        table = ntcore.NetworkTableInstance.get_default().get_table(camera_name)
        self._msgpack = table.get_raw_topic("results_msgpack").subscribe(
            "msgpack",
            b"",
            ntcore.PubSubOptions(periodic=0.005, send_all=True, poll_storage=20),
        )

        table.get_entry("camerapose_robotspace_set").set_double_array(
            [
                robot_to_camera.x,
                robot_to_camera.y,
                robot_to_camera.z,
                units.radians_to_degrees(robot_to_camera.rotation().x),
                units.radians_to_degrees(robot_to_camera.rotation().y),
                units.radians_to_degrees(robot_to_camera.rotation().z),
            ]
        )

        self._std_devs: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_timestamp = 0.0

        self._latest_results: list[LimelightResults] = []

        self._headingBuffer = TimeInterpolatableRotation2dBuffer(1)

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
        latest_results = self._latest_results

        poses: list[Pose2d] = []
        std_devs: list[tuple[float, float, float]] = []
        timestamps: list[float] = []

        for result in latest_results:
            targets = result.fiducial_targets

            pose = result.get_robot_pose_mt2()
            if pose is None:
                pose = result.get_robot_pose_mt1()
                if pose is None:
                    continue

            ts = result.timestamp_micros

            # Compute std devs
            tag_count = len(targets)
            if tag_count <= 0:
                continue

            total_dist = 0
            for t in targets:
                cpts = t.get_camera_pose_target_space()
                if cpts is not None:
                    total_dist += cpts.translation().norm()

            avg_dist = total_dist / tag_count

            # if VisionUtils.reject_measurement(
            #     pose, ts, self._last_timestamp, robot_pose
            # ):
            #     continue

            self._last_timestamp = ts

            std_devs.append(VisionUtils.compute_multitag_std_devs(avg_dist, tag_count))
            poses.append(pose.to_pose2d())
            timestamps.append(ts)

        if not poses:
            return self.estimate_single_tag_pose()

        return PoseEstimates(poses, std_devs, timestamps)

    @overload
    def estimate_single_tag_pose(
        self,
    ) -> PoseEstimates: ...

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
        latest_results = self._latest_results

        poses: list[Pose2d] = []
        std_devs: list[tuple[float, float, float]] = []
        timestamps: list[float] = []

        for result in latest_results:
            if (target := result.get_best_target()) is None:
                continue

            pose = (
                target.get_robot_pose_field_space_mt2()
                or target.get_robot_pose_field_space()
            )
            if pose is None:
                continue

            ts = result.timestamp_micros

            if (cpts := target.get_camera_pose_target_space()) is None:
                continue

            total_dist = cpts.translation().norm()

            # if VisionUtils.reject_measurement(
            #     pose, ts, self._last_timestamp, robot_pose
            # ):
            #     continue

            self._last_timestamp = ts

            std_devs.append(VisionUtils.compute_multitag_std_devs(total_dist, 1))
            poses.append(pose.to_pose2d())
            timestamps.append(ts)

        return PoseEstimates(poses, std_devs, timestamps)

    def get_latest_results(self) -> list[LimelightResults]:
        """
        must call every loop
        """

        self._latest_results = decode(self._msgpack.read_queue())
        return self._latest_results

    def has_target(self) -> bool:
        return self._latest_results[-1].get_target_count() > 0

    def set_std_baselines(
        self, linear_baseline: float = 0.1, angular_baseline: float = 0.174
    ):
        self.linear_baseline_std = linear_baseline
        self.angular_baseline_std = angular_baseline

    def is_connected(self) -> bool:
        # not really gonna work but this gonna get replaced when limelight lib comes out sooo
        return self._latest_results[-1].nt_connected

    def add_heading_data(
        self, timestampSeconds: units.seconds, heading: Rotation2d
    ) -> None:

        self._headingBuffer.add_sample(timestampSeconds, heading)
