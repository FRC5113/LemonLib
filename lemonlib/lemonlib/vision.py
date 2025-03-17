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

    def get_best_tag(self) -> int:
        results = self.getAllUnreadResults()
        if len(results) > 0:
            result = results[-1]
            best_tag = result.getBestTarget().getFiducialId()

            return best_tag

    def get_tag_pose(self, ID: int):
        return self.april_tag_field.getTagPose(ID)

    def get_best_pose(self):
        return self.april_tag_field.getTagPose(self.get_best_tag())
