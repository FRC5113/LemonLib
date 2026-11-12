import math
from typing import SupportsFloat, SupportsIndex, cast

import ntcore
import robotpy_fields
from wpilib import Timer
from wpimath import ChassisVelocities, Pose2d, Pose3d, Rotation3d, Transform3d

from . import LimelightResults, MegaTagResult, MegaTagStdDevs, _transform3d_to_list
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

        self.vision_util = VisionUtils()

        table = ntcore.NetworkTableInstance.get_default().get_table(camera_name)
        self._latest_results = table.get_double_array_topic("t2d").subscribe([])
        self._megatag1 = table.get_double_array_topic("botpose").subscribe([])
        self._megatag2 = table.get_double_array_topic("botpose_orb").subscribe([])
        self._std_devs_mt = table.get_double_array_topic("stddevs").subscribe([])
        self._heartbeat = table.get_double_topic("hb").subscribe(-1.0)

        robot_to_cam_list = cast(
            list[SupportsFloat | SupportsIndex],
            _transform3d_to_list(robot_to_camera),
        )

        _camerapose_robotspace_set = table.get_double_array_topic(
            "camerapose_robotspace_set"
        ).get_entry(robot_to_cam_list)
        _camerapose_robotspace_set.set(robot_to_cam_list)

        self.linear_baseline_std = 0.10
        self.angular_baseline_std = math.radians(10.0)

        self._std_devs: tuple[float, float, float] = (0.0, 0.0, 0.0)

        self._timestamp = -1e-6
        self._prevHeartbeat: int = 0
        self._prevHeartbeatChangeTime = Timer.get_monotonic_timestamp()

    def estimate_pose_debug(self):
        latest_results = self.get_latest_results()
        if len(latest_results.megatag1) == 0:
            return None

        test = latest_results.megatag1[-1]
        if test is not None:
            return test.pose
        else:
            return None

    def estimate_multi_tag_pose(
        self,
        robot_pose: Pose2d | None = None,
        drivetrain_velocities: ChassisVelocities | None = None,
    ) -> Pose2d | None:
        """
        Mega tag 2
        """
        latest_results = self.get_latest_results()
        megatag = latest_results.megatag2

        valid_estimates: list[tuple[Pose2d, float, tuple[float, float, float]]] = []

        for res in megatag:
            targets = res.tag_count
            if targets < 0:
                continue

            ts = res.ts
            pose3d = res.pose
            twod_pose = pose3d.to_pose2d()

            # Compute std devs
            tag_count = int(res.tag_count)
            avg_dist = res.avg_tag_distance

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
        """
        Mega tag 1
        """
        latest_results = self.get_latest_results()
        megatag = latest_results.megatag1

        valid_estimates: list[tuple[Pose2d, float, tuple[float, float, float]]] = []

        for res in megatag:
            targets = res.tag_count
            if targets < 0:
                continue

            ts = res.ts
            pose3d = res.pose
            twod_pose = pose3d.to_pose2d()

            # Compute std devs
            tag_count = int(res.tag_count)
            avg_dist = res.avg_tag_distance

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

    def get_latest_results(self) -> LimelightResults:
        results = LimelightResults()

        results.megatag1 = self._get_megatag1_results()
        results.megatag2 = self._get_megatag2_results()
        results.megatag1_std_devs = self._get_megatag1_std_devs()
        results.megatag2_std_devs = self._get_megatag2_std_devs()

        return results

    def has_target(self) -> bool:
        latest_results = self.get_latest_results().megatag2

        if len(latest_results) > 0:
            for res in latest_results:
                if res.tag_count > 0:
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
        curHeartbeat = int(self._heartbeat.get())
        now = Timer.get_monotonic_timestamp()

        if curHeartbeat != self._prevHeartbeat:
            self._prevHeartbeat = curHeartbeat
            self._prevHeartbeatChangeTime = now

        return (now - self._prevHeartbeatChangeTime) < 0.5

    def _get_megatag1_results(self) -> list[MegaTagResult]:
        queue = self._megatag1.read_queue()

        results: list[MegaTagResult] = []

        for result in queue:
            nt = result.value

            mt = MegaTagResult()
            mt.pose = Pose3d(
                x=nt[0],
                y=nt[1],
                z=nt[2],
                rotation=Rotation3d(
                    roll=math.radians(nt[3]),
                    pitch=math.radians(nt[4]),
                    yaw=math.radians(nt[5]),
                ),
            )
            mt.latency = nt[6]
            mt.tag_count = nt[7]
            mt.tag_span = nt[8]
            mt.avg_tag_distance = nt[9]
            mt.avg_tag_area = nt[10]
            mt.ts = result.time

            results.append(mt)

        return results

    def _get_megatag2_results(self) -> list[MegaTagResult]:
        queue = self._megatag2.read_queue()

        results: list[MegaTagResult] = []

        for result in queue:
            nt = result.value

            mt = MegaTagResult()
            mt.pose = Pose3d(
                x=nt[0],
                y=nt[1],
                z=nt[2],
                rotation=Rotation3d(
                    roll=math.radians(nt[3]),
                    pitch=math.radians(nt[4]),
                    yaw=math.radians(nt[5]),
                ),
            )
            mt.latency = nt[6]
            mt.tag_count = nt[7]
            mt.tag_span = nt[8]
            mt.avg_tag_distance = nt[9]
            mt.avg_tag_area = nt[10]
            mt.ts = result.time

            results.append(mt)

        return results

    def _get_megatag1_std_devs(self) -> list[MegaTagStdDevs]:
        queue = self._std_devs_mt.read_queue()

        results: list[MegaTagStdDevs] = []

        for result in queue:
            nt = result.value
            std_devs = MegaTagStdDevs()
            std_devs.x = nt[0]
            std_devs.y = nt[1]
            std_devs.z = nt[2]
            std_devs.rot = Rotation3d(roll=nt[3], pitch=nt[4], yaw=nt[5])
            std_devs.ts = result.time

            results.append(std_devs)

        return results

    def _get_megatag2_std_devs(self) -> list[MegaTagStdDevs]:
        queue = self._std_devs_mt.read_queue()

        results: list[MegaTagStdDevs] = []

        for result in queue:
            nt = result.value
            std_devs = MegaTagStdDevs()
            std_devs.x = nt[6]
            std_devs.y = nt[7]
            std_devs.z = nt[8]
            std_devs.rot = Rotation3d(roll=nt[9], pitch=nt[10], yaw=nt[11])
            std_devs.ts = result.time

            results.append(std_devs)

        return results

    @staticmethod
    def camera_exists(camera_name: str) -> bool:
        instance = ntcore.NetworkTableInstance.get_default()

        return instance.get_table(camera_name).contains_key("hb")
