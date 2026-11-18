from dataclasses import dataclass, field

import wpimath


@dataclass
class PoseEstimates:
    poses: list[wpimath.Pose2d] = field(default_factory=list[wpimath.Pose2d])
    stdevs: list[tuple[float, float, float]] = field(
        default_factory=list[tuple[float, float, float]]
    )
    timestamps: list[float] = field(default_factory=list[float])

    def get_latest_pose(self) -> wpimath.Pose2d | None:
        if self.poses:
            return self.poses[-1]
        return

    def get_latest_std_devs(self) -> tuple[float, float, float] | None:
        if self.stdevs:
            return self.stdevs[-1]
        return

    def get_latest_timestamp(self) -> float | None:
        if self.timestamps:
            return self.timestamps[-1]
        return
