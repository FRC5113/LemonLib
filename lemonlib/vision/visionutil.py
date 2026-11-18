from wpilib import DriverStationBackend
from wpimath import Pose2d, Pose3d


class VisionUtils:
    def __init__(self):
        self._last_timestamp = 0.0

    @staticmethod
    def compute_multitag_std_devs(
        avg_dist: float,
        tag_count: int,
        linear_baseline_std: float = 0.1,
        angular_baseline_std: float = 0.174,
    ) -> tuple[float, float, float]:
        """Compute standard deviations for a vision measurement."""
        if tag_count == 0:
            return (0.0, 0.0, 0.0)

        std_factor = (avg_dist**2) / tag_count
        std_xy = linear_baseline_std * std_factor
        std_rot = angular_baseline_std * std_factor

        if DriverStationBackend.is_disabled():
            # Vision should never be more trusted than wheel odometry.
            # Without this floor, close-range tags produce std devs near
            # or below the state stds (0.01), letting vision noise
            # dominate the pose estimate and cause drift.
            std_xy = max(std_xy, 0.05)
            std_rot = max(std_rot, 0.5)
        return (std_xy, std_xy, std_rot)

    @staticmethod
    def reject_measurement(
        estimated_pose: Pose3d,
        ts: float,
        last_timestamp: float,
        robot_pose: Pose2d | None = None,
    ) -> bool:
        """Return True if the measurement should be rejected."""
        # Z-height check: reject poses that claim robot is far off ground
        if abs(estimated_pose.z) > 0.2:
            return True

        # Stale timestamp check
        if ts <= last_timestamp:
            return True

        last_timestamp = ts

        # Skip the check if its not given
        if robot_pose is not None:
            # Distance from current pose check (existing)
            dist = (
                robot_pose.relative_to(estimated_pose.to_pose2d()).translation().norm()
            )
            return dist > 4.0 and not DriverStationBackend.is_disabled()
        return False
