from photonlibpy.photonCamera import PhotonCamera
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose2d, Pose3d, Transform3d
from wpimath import units
from pyfrc.physics.visionsim import VisionSimTarget


class LemonCamera(PhotonCamera):
    """Wrapper for photonlibpy PhotonCamera"""

    def __init__(
        self,
        name: str,
        camera_to_bot: Transform3d,
    ):
        """Parameters:
        camera_name -- name of camera in PhotonVision
        camera_transform -- Transform3d that maps camera space to robot space
        window -- number of ticks until a tag is considered lost.
        """
        PhotonCamera.__init__(self, name)
        self.camera_to_bot = camera_to_bot
        self.tag_poses = {}
        self.tag_ambiguities = {}
        self.latency = 0

    def update(self) -> None:
        """Call this every loop"""
        result = self.getLatestResult()
        self.tag_poses = {}
        self.tag_ambiguities = {}
        self.latency = result.getLatencyMillis() / 1000
        if result.hasTargets():
            targets = result.getTargets()
            for target in targets:
                self.tag_ambiguities[target.getFiducialId()] = target.getPoseAmbiguity()
                # this probably won't work
                self.tag_poses[target.getFiducialId()] = (
                    Pose3d()
                    # transform origin in tag space to camera space
                    .transformBy(target.getBestCameraToTarget())
                    # transform tag pose in camera space to robot space
                    .relativeTo(Pose3d().transformBy(self.camera_to_bot))
                    # flatten to 2d space
                    .toPose2d()
                )

    def has_targets(self) -> bool:
        return len(self.tag_ambiguities) > 0

    def has_tag(self, id: int) -> bool:
        return id in self.tag_ambiguities.keys()

    def getLatency(self) -> float | None:
        return self.latency if self.has_targets() else None

    def get_best_id(self) -> int | None:
        if self.has_targets():
            return min(self.tag_ambiguities, key=self.tag_ambiguities.get)
        else:
            return None

    def get_ambiguity(self, id: int | None = None) -> float | None:
        """Return ambiguity of tag with given id. If id is not
        specified, uses tag with least ambiguity."""
        if id is not None:
            return self.tag_ambiguities[id] if self.has_tag(id) else None
        return self.tag_ambiguities[self.get_best_id()] if self.has_targets() else None

    def get_pose(self, id: int | None = None, robot_pose: Pose2d = None) -> Pose2d:
        """Return pose of tag with given id. If id is not specified,
        uses tag with least ambiguity. If `robot_pose` is specified,
        pose will be field-relative. Otherwise, it will be
        robot-relative."""
        if id is None:
            if not self.has_targets():
                return
            id = self.get_best_id()
        if self.has_tag(id):
            if robot_pose is None:
                return self.tag_poses[id]
            return self.tag_poses[id].relativeTo(Pose2d().relativeTo(robot_pose))
        return None

    def get_transform(self) -> Transform3d:
        return self.camera_to_bot
