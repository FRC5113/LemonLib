from wpimath.geometry import Pose2d, Pose3d, Transform3d
from photonlibpy.photonCamera import PhotonCamera
from robotpy_apriltag import AprilTagFieldLayout


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
                    .transformBy(self.camera_to_bot)
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


class LemonCameraSim(LemonCamera):
    """Simulated version of a LemonCamera. This class functions exactly
    the same in code except for the following:
    1. Must be initialized with an `AprilTagFieldLayout` and an FOV
    2. `set_robot_pose()` must be called periodically to update the pose
    of the robot. This should not be taken from a pose estimator that
    uses vision updates, but rather a pose simulated in physics.py
    3. This simulation assumes that the camera is at the center of the
    robot looking directly forward, but the difference should be negligible
    """

    def __init__(
        self,
        field_layout: AprilTagFieldLayout,
        fov: float,
    ):
        """Args:
        field_layout (AprilTagFieldLayout): layout of the tags on the field, such as
            `AprilTagField.k2024Crescendo`
        fov (float): horizontal range of vision (degrees)
        """
        LemonCamera.__init__(self, "Sim", Transform3d())
        self.field_layout = field_layout
        self.fov = fov
        self.robot_pose = None
        self.tag_poses = {}
        self.tag_ambiguities = {}
        self.latency = 0

    def set_robot_pose(self, pose: Pose2d):
        self.robot_pose = pose

    def update(self):
        if self.robot_pose is None:
            return
        self.tag_poses = {}
        self.tag_ambiguities = {}
        self.latency = 0.02
        for tag in self.field_layout.getTags():
            tag_pose = tag.pose.toPose2d()
            relative_pose = tag_pose.relativeTo(self.robot_pose)
            dist = relative_pose.translation().norm()
            # calculate estimated tag ambiguity based on distance and angle
            ambiguity = (
                -dist * dist / (tag_pose.rotation() - self.robot_pose.rotation()).cos()
            )
            # check that robot can "see" tag
            if (
                abs(relative_pose.translation().angle().degrees()) < self.fov / 2
                and ambiguity > 0
            ):
                self.tag_ambiguities[tag.ID] = ambiguity
                self.tag_poses[tag.ID] = relative_pose
