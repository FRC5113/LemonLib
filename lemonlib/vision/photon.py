import math

import ntcore
import robotpy_fields
from wpimath import ChassisVelocities, Pose2d, Transform3d

import photonlibpy
from photonlibpy.targeting import PhotonPipelineResult

from .visionbase import VisionBase
from .visionutil import VisionUtils


class photonname(VisionBase):
    def __init__(
        self,
        camera_name: str,
        robot_to_camera: Transform3d,
        field: robotpy_fields.Field,
    ) -> None:

        self.photon_camera = photonlibpy.photon_camera.PhotonCamera(camera_name)
        self.pose_estimator = photonlibpy.photon_pose_estimator.PhotonPoseEstimator(
            field, robot_to_camera
        )
        self.vision_util = VisionUtils()

        self.linear_baseline_std = 0.10
        self.angular_baseline_std = math.radians(10.0)

        self._std_devs: tuple[float, float, float] = (0.0, 0.0, 0.0)

        self._timestamp = -1e-6

    def estimate_pose_debug(self):
        latest_results = self.get_latest_results()
        if len(latest_results) == 0:
            return None

        test = self.pose_estimator.estimateLowestAmbiguityPose(latest_results[-1])
        if test is not None:
            return test.estimatedPose.to_pose2d()
        else:
            return None

    def estimate_multi_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
        drivetrain_velocities: ChassisVelocities | None = None,
    ) -> Pose2d | None:
        latest_results = self.get_latest_results()

        valid_estimates: list[tuple[Pose2d, float, tuple[float, float, float]]] = []

        for res in latest_results:
            targets = res.getTargets()
            if not targets:
                continue

            pose_estimate = self.pose_estimator.estimateCoprocMultiTagPose(res)

            if pose_estimate is None:
                continue

            ts = pose_estimate.timestampSeconds
            pose3d = pose_estimate.estimatedPose
            twod_pose = pose3d.to_pose2d()

            # Compute std devs
            tag_count = len(targets)
            total_dist = sum(
                t.getBestCameraToTarget().translation().norm() for t in targets
            )
            avg_dist = total_dist / tag_count

            if self.vision_util.reject_measurement(pose3d, twod_pose, ts, robot_pose):
                continue

            std_devs = self.vision_util.compute_std_devs(avg_dist, tag_count)

            valid_estimates.append((twod_pose, ts, std_devs))

        if valid_estimates:
            if len(valid_estimates) == 1:
                pose, self._timestamp, self._std_devs = valid_estimates[0]
                return pose
            else:
                fused = self.vision_util.fuse_estimates(
                    valid_estimates, drivetrain_velocities
                )
                if fused is not None:
                    pose, self._timestamp, self._std_devs = fused
                    return pose

        return None

    def estimate_single_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
        drivetrain_velocities: ChassisVelocities | None = None,
    ) -> Pose2d | None:
        latest_results = self.get_latest_results()

        valid_estimates: list[tuple[Pose2d, float, tuple[float, float, float]]] = []

        for res in latest_results:
            targets = res.getTargets()
            if not targets:
                continue

            pose_estimate = self.pose_estimator.estimateLowestAmbiguityPose(res)

            if pose_estimate is None:
                continue

            ts = pose_estimate.timestampSeconds
            pose3d = pose_estimate.estimatedPose
            twod_pose = pose3d.to_pose2d()

            # Compute std devs
            tag_count = len(targets)
            total_dist = sum(
                t.getBestCameraToTarget().translation().norm() for t in targets
            )
            avg_dist = total_dist / tag_count

            if self.vision_util.reject_measurement(pose3d, twod_pose, ts, robot_pose):
                continue

            std_devs = self.vision_util.compute_std_devs(avg_dist, tag_count)

            valid_estimates.append((twod_pose, ts, std_devs))

        if valid_estimates:
            if len(valid_estimates) == 1:
                pose, self._timestamp, self._std_devs = valid_estimates[0]
                return pose
            else:
                fused = self.vision_util.fuse_estimates(
                    valid_estimates, drivetrain_velocities
                )
                if fused is not None:
                    pose, self._timestamp, self._std_devs = fused
                    return pose

        return None

    def get_latest_results(self) -> list[PhotonPipelineResult]:
        return self.photon_camera.getAllUnreadResults()

    def has_target(self) -> bool:
        latest_results = self.get_latest_results()

        if len(latest_results) > 0:
            for res in latest_results:
                if res.hasTargets():
                    return True
        return False

    def get_std_devs(self) -> tuple[float, float, float]:
        return self._std_devs

    def get_timestamp(self) -> float:
        return self._timestamp

    def set_std_baselines(
        self, linear_baseline: float = 0.1, angular_baseline: float = 0.174
    ):
        self.linear_baseline_std = linear_baseline
        self.angular_baseline_std = angular_baseline

    def is_connected(self) -> bool:
        return self.photon_camera.isConnected()

    @staticmethod
    def camera_exists(camera_name: str) -> bool:
        instance = ntcore.NetworkTableInstance.get_default()
        instance.flush()
        table = instance.get_table("photonvision")
        return table.contains_sub_table(camera_name)
