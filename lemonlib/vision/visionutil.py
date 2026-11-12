import math

from wpilib import DriverStationBackend
from wpimath import ChassisVelocities, Pose2d, Pose3d, Rotation2d


class VisionUtils:
    def __init__(self) -> None:
        self.linear_baseline_std = 0.10
        self.angular_baseline = math.radians(10.0)
        self._last_timestamp = 0.0

    def compute_std_devs(
        self, avg_dist: float, tag_count: int
    ) -> tuple[float, float, float]:
        """Compute standard deviations for a vision measurement."""
        if tag_count == 0:
            return (0.0, 0.0, 0.0)

        std_factor = (avg_dist**2) / tag_count
        std_xy = self.linear_baseline_std * std_factor
        std_rot = self.angular_baseline * std_factor

        if DriverStationBackend.is_disabled():
            # Vision should never be more trusted than wheel odometry.
            # Without this floor, close-range tags produce std devs near
            # or below the state stds (0.01), letting vision noise
            # dominate the pose estimate and cause drift.
            std_xy = max(std_xy, 0.05)
            std_rot = max(std_rot, 0.5)
        return (std_xy, std_xy, std_rot)

    def reject_measurement(
        self,
        pose3d: Pose3d,
        twod_pose: Pose2d,
        ts: float,
        robot_pose: Pose2d | None = None,
    ) -> bool:
        """Return True if the measurement should be rejected."""
        # Z-height check: reject poses that claim robot is far off ground
        if abs(pose3d.z) > 0.2:
            return True

        # Stale timestamp check
        if ts <= self._last_timestamp:
            return True

        self._last_timestamp = ts

        if robot_pose is not None:
            # Distance from current pose check (existing)
            dist = robot_pose.relative_to(twod_pose).translation().norm()
            return dist > 4.0 and not DriverStationBackend.is_disabled()
        return False

    def fuse_estimates(
        self,
        estimates: list[tuple[Pose2d, float, tuple[float, float, float]]],
        drivetrain_velocities: ChassisVelocities | None = None,
    ) -> tuple[Pose2d, float, tuple[float, float, float]] | None:
        """Fuse multiple vision estimates using inverse-variance weighting.

        Each estimate is (pose2d, timestamp, (std_x, std_y, std_rot)).
        Returns fused (pose2d, timestamp, std_devs).
        """
        if not estimates:
            return None

        if len(estimates) == 1:
            return estimates[0]

        # Use the most recent timestamp
        estimates.sort(key=lambda e: e[1])
        newest_ts = estimates[-1][1]

        # Time-align older estimates to newest using odometry delta
        aligned: list[tuple[Pose2d, tuple[float, float, float]]] = []

        for pose, ts, stds in estimates:
            if (drivetrain_velocities is not None) and ts < newest_ts:
                # Approximate alignment: use the pose estimator's odometry
                # The time difference should be small (< 1 frame), so we use
                # the raw pose offset as a simple approximation
                dt = newest_ts - ts
                dx = drivetrain_velocities.vx * dt
                dy = drivetrain_velocities.vy * dt
                dtheta = drivetrain_velocities.omega * dt
                heading = pose.rotation().radians()
                cos_t = math.cos(heading)
                sin_t = math.sin(heading)
                new_x = pose.x + dx * cos_t - dy * sin_t
                new_y = pose.y + dx * sin_t + dy * cos_t
                new_rot = pose.rotation().radians() + dtheta
                pose = Pose2d(new_x, new_y, Rotation2d(new_rot))

            aligned.append((pose, stds))

        # Inverse-variance weighting for X and Y independently
        sum_inv_var_x = 0.0
        sum_inv_var_y = 0.0
        sum_inv_var_rot = 0.0
        weighted_x = 0.0
        weighted_y = 0.0
        weighted_sin = 0.0
        weighted_cos = 0.0

        for pose, stds in aligned:
            inv_var_x = 1.0 / (stds[0] ** 2) if stds[0] > 0 else 1e6
            inv_var_y = 1.0 / (stds[1] ** 2) if stds[1] > 0 else 1e6
            inv_var_rot = 1.0 / (stds[2] ** 2) if stds[2] > 0 else 1e6

            sum_inv_var_x += inv_var_x
            sum_inv_var_y += inv_var_y
            sum_inv_var_rot += inv_var_rot

            weighted_x += pose.x * inv_var_x
            weighted_y += pose.y * inv_var_y
            rot = pose.rotation().radians()
            weighted_sin += math.sin(rot) * inv_var_rot
            weighted_cos += math.cos(rot) * inv_var_rot

        fused_x = weighted_x / sum_inv_var_x
        fused_y = weighted_y / sum_inv_var_y
        fused_rot = math.atan2(weighted_sin, weighted_cos)

        fused_std_x = math.sqrt(1.0 / sum_inv_var_x)
        fused_std_y = math.sqrt(1.0 / sum_inv_var_y)
        fused_std_rot = math.sqrt(1.0 / sum_inv_var_rot)

        fused_pose = Pose2d(fused_x, fused_y, Rotation2d(fused_rot))
        return (fused_pose, newest_ts, (fused_std_x, fused_std_y, fused_std_rot))
