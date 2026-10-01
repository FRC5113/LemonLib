from robotpy_fields import FieldId, get_field
from wpimath import Transform3d

from photonlibpy import PhotonCamera


class LemonCamera(PhotonCamera):
    """Wrapper for photonlibpy PhotonCamera"""

    def __init__(
        self,
        name: str,
        camera_to_bot: Transform3d,
        april_tag_field_id: FieldId,
    ):
        """Parameters:
        camera_name -- name of camera in PhotonVision
        camera_transform -- Transform3d that maps camera space to robot space
        window -- number of ticks until a tag is considered lost.
        """
        PhotonCamera.__init__(self, name)
        self.camera_to_bot = camera_to_bot
        self.april_tag_field = get_field(april_tag_field_id)
        self.results = []
        self._last_valid_tag: int | None = None

    def update(self):
        self.results = self.getAllUnreadResults()

    def has_target(self):
        return len(self.results) > 0 and self.results[-1].hasTargets()

    def get_best_tag(self) -> int | None:
        if self.results:
            result = self.results[-1]
            best_target = result.getBestTarget()
            if best_target is not None:
                self._last_valid_tag = best_target.getFiducialId()
        return self._last_valid_tag

    def get_tag_pose(self, ID: int, twod: bool):
        tag_pose = self.april_tag_field.get_tag_pose(ID)
        if tag_pose is None:
            return None
        if twod:
            return tag_pose.to_pose2d()
        return tag_pose

    def get_best_pose(self, twod: bool = True):
        best_tag = self.get_best_tag()
        if best_tag is None:
            return None
        return self.get_tag_pose(best_tag, twod)
