from photonlibpy.photonCamera import PhotonCamera
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField, AprilTagPoseEstimator
from wpimath.geometry import Pose2d, Pose3d, Transform3d
from wpimath import units
import math


class LemonCamera(PhotonCamera):
    """Wrapper for photonlibpy PhotonCamera"""

    def __init__(
        self,
        name: str,
        camera_to_bot: Transform3d,
        april_tag_field: AprilTagFieldLayout,
    ):
        """Parameters:
        camera_name -- name of camera in PhotonVision
        camera_transform -- Transform3d that maps camera space to robot space
        window -- number of ticks until a tag is considered lost.
        """
        PhotonCamera.__init__(self, name)
        self.camera_to_bot = camera_to_bot
        self.april_tag_field = april_tag_field

    def update(self):
        self.results = self.getAllUnreadResults()

    def has_target(self):
        return len(self.results) > 0 and self.results[-1].hasTargets()

    def get_best_tag(self) -> int:
        if self.results:
            result = self.results[-1]
            best_target = result.getBestTarget()
            if best_target is not None:
                self._last_valid_tag = best_target.getFiducialId()
                return self._last_valid_tag
        return getattr(self, "_last_valid_tag", None)

    def get_tag_pose(self, ID: int, twod: bool):
        if twod:
            return self.april_tag_field.getTagPose(ID).toPose2d()
        else:
            return self.april_tag_field.getTagPose(ID)

    def get_best_pose(self, twod: bool = True):
        best_tag = self.get_best_tag()
        if best_tag is None:
            return None
        tag_pose = self.april_tag_field.getTagPose(best_tag)
        if tag_pose is None:
            return None
        if twod:
            return tag_pose.toPose2d()
        return tag_pose
