import math
from typing import overload

import photonlibpy
import robotpy_fields
from photonlibpy.targeting import PhotonPipelineResult
from wpimath import Pose2d, Rotation2d, Transform3d, units

from ._util import PoseEstimates
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

        self.linear_baseline_std = 0.10
        self.angular_baseline_std = math.radians(10.0)

        self._last_timestamp = 0.0

        self._latest_results: list[PhotonPipelineResult] = []

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
            targets = result.getTargets()

            pose_estimate = self.pose_estimator.estimateCoprocMultiTagPose(result)

            if pose_estimate is None:
                pose_estimate = self.pose_estimator.estimateLowestAmbiguityPose(result)

                if pose_estimate is None:
                    continue

            ts = pose_estimate.timestampSeconds
            pose = pose_estimate.estimatedPose

            # Compute std devs
            tag_count = len(targets)
            if tag_count <= 0:
                continue

            total_dist = sum(
                t.getBestCameraToTarget().translation().norm() for t in targets
            )
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
            return self.estimate_single_tag_pose(robot_pose)

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
            targets = result.getTargets()

            # pnp returns none use lowest ambigutiy
            pose_estimate = self.pose_estimator.estimatePnpDistanceTrigSolvePose(
                result
            ) or self.pose_estimator.estimateLowestAmbiguityPose(result)

            if pose_estimate is None:
                continue

            ts = pose_estimate.timestampSeconds
            pose = pose_estimate.estimatedPose

            # Compute std devs
            tag_count = len(targets)
            if tag_count <= 0:
                continue

            total_dist = sum(
                t.getBestCameraToTarget().translation().norm() for t in targets
            )
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

    def get_latest_results(self) -> list[PhotonPipelineResult]:
        """
        must call every loop
        """
        self._latest_results = self.photon_camera.getAllUnreadResults()
        return self._latest_results

    def has_target(self) -> bool:
        latest_results = self._latest_results

        if len(latest_results) > 0:
            for res in latest_results:
                if res.hasTargets():
                    return True
        return False

    def set_std_baselines(
        self, linear_baseline: float = 0.1, angular_baseline: float = 0.174
    ):
        self.linear_baseline_std = linear_baseline
        self.angular_baseline_std = angular_baseline

    def add_heading_data(
        self, timestampSeconds: units.seconds, heading: Rotation2d
    ) -> None:
        self.pose_estimator.addHeadingData(timestampSeconds, heading)

    def is_connected(self) -> bool:
        return self.photon_camera.isConnected()
