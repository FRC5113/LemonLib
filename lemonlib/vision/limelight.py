import math
from typing import overload

import ntcore
import robotpy_fields
from wpimath import (
    Pose2d,
    Rotation2d,
    Transform2d,
    Transform3d,
    Translation2d,
    Translation3d,
    units,
)

from ._util import PoseEstimates
from .limelightutils import FiducialTarget, LimelightResults, decode
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

        return PoseEstimates(poses, std_devs, timestamps)

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
        # not worth doing atm
        return PoseEstimates()

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

    @staticmethod
    def camera_exists(camera_name: str) -> bool:
        instance = ntcore.NetworkTableInstance.get_default()

        return instance.get_table(camera_name).contains_key("hb")

    @staticmethod
    def _estimate_field_to_robot(
        camera_height_meters: float,
        target_height_meters: float,
        camera_pitch_radians: float,
        target_pitch_radians: float,
        target_yaw: Rotation2d,
        gyro_angle: Rotation2d,
        field_to_target: Pose2d,
        camera_to_robot: Transform2d,
    ) -> Pose2d:
        # Calculate distance to target using camera and target elevation
        distance = (target_height_meters - camera_height_meters) / math.tan(
            camera_pitch_radians + target_pitch_radians
        )

        # Calculate camera-relative target translation
        camera_to_target = Translation2d(
            distance * target_yaw.cos(),
            distance * target_yaw.sin(),
        )

        # Calculate camera-to-target transform
        camera_to_target_transform = Transform2d(
            camera_to_target,
            -gyro_angle - (field_to_target.rotation()),
        )

        # Calculate target-to-camera transform
        target_to_camera = camera_to_target_transform.inverse()

        # Calculate field-relative camera pose
        field_to_camera = field_to_target.transform_by(target_to_camera)

        # Calculate field-relative robot pose
        return field_to_camera.transform_by(camera_to_robot)

    @staticmethod
    def _estimate_pnp_distance_trig_solve_pose(
        targets: list[FiducialTarget],
        gyro_angle: Rotation2d,
        robot_to_camera: Transform3d,
        field_layout: robotpy_fields.Field,
    ) -> Pose2d | None:

        # Select the target with the lowest ambiguity
        valid_targets = [
            target
            for target in targets
            if target.fiducial_id > 0
            and target.ambiguity < 1.0
            and len(target.target_pose_camera_space) >= 6
        ]

        if not valid_targets:
            return None

        best_target = min(valid_targets, key=lambda target: target.ambiguity)

        # Get the known AprilTag field pose
        tag_pose = field_layout.get_tag_pose(best_target.fiducial_id)

        if tag_pose is None:
            return None

        tag_pose = tag_pose.to_pose2d()

        # Extract camera-to-tag translation
        target = best_target.target_pose_camera_space

        camera_to_tag = Translation3d(
            target[0],
            target[1],
            target[2],
        )

        # Rotate camera-to-tag into robot orientation
        camera_to_tag = camera_to_tag.rotate_by(robot_to_camera.rotation())

        # Convert camera-to-tag into field coordinates
        camera_to_tag_field = camera_to_tag.to_translation2d().rotate_by(gyro_angle)

        # Calculate field-relative camera position
        field_to_camera = tag_pose.translation() - camera_to_tag_field

        # Calculate camera-to-robot translation
        camera_to_robot = (-robot_to_camera.translation().to_translation2d()).rotate_by(
            gyro_angle
        )

        # Calculate final robot pose
        return Pose2d(
            field_to_camera + camera_to_robot,
            gyro_angle,
        )
