from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim
from robotpy_fields import Field
from wpimath import Pose2d, Rotation2d

from ..vision import LemonCamera


class LemonVisionSim:
    """Shared VisionSystemSim that manages all simulated cameras.
    Create one instance and register all cameras with it, then call
    `update()` once per physics tick to process all cameras together.
    """

    def __init__(self, field_layout: Field):
        self.vision_sim = VisionSystemSim("vision_sim")
        self.vision_sim.addAprilTags(field_layout)

    def add_camera(self, camera_sim: "LemonCameraSim") -> None:
        self.vision_sim.addCamera(camera_sim, camera_sim.camera.camera_to_bot)

    def update(self, pose: Pose2d) -> None:
        self.vision_sim.update(pose)


class LemonCameraSim(PhotonCameraSim):
    """Simulated version of a LemonCamera. This class functions exactly
    the same in code except for the following:
    1. Must be initialized with a `robotpy_fields.Field` and an FOV
    2. Must be registered with a shared `LemonVisionSim` instance
    3. `LemonVisionSim.update()` must be called once per tick to update
    all cameras together with the robot pose
    """

    def __init__(
        self,
        camera: LemonCamera,
        field_layout: Field,
        fov: float = 100.0,
        fps: float = 20.0,
        avg_latency: float = 0.035,
        latency_std_dev: float = 0.005,
    ):
        """Args:
        field_layout (Field): layout of the tags on the field, such as
            `get_field(FieldId.FRC_2026_REBUILT_WELDED)`
        fov (float): diagonal range of vision (degrees)
        """
        self.field_layout = field_layout
        self.fov = Rotation2d.from_degrees(fov)
        self.camera = camera

        self.camera_props = SimCameraProperties()
        self.camera_props.setCalibrationFromFOV(640, 480, self.fov)
        self.camera_props.setFPS(fps)
        self.camera_props.setAvgLatency(avg_latency)
        self.camera_props.setLatencyStdDev(latency_std_dev)
        PhotonCameraSim.__init__(
            self, self.camera, self.camera_props, self.field_layout
        )
