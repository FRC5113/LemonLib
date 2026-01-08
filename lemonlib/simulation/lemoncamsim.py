from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose2d, Transform3d, Rotation2d
from photonlibpy.simulation.photonCameraSim import PhotonCameraSim
from photonlibpy.simulation.simCameraProperties import SimCameraProperties
from photonlibpy.simulation.visionSystemSim import VisionSystemSim
from ..vision import LemonCamera


class LemonCameraSim(PhotonCameraSim):
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
        camera: LemonCamera,
        field_layout: AprilTagFieldLayout,
        fov: float = 100.0,
        fps: int = 20.0,
        avg_latency: float = 0.035,
        latency_std_dev: float = 0.005,
    ):
        """Args:
        field_layout (AprilTagFieldLayout): layout of the tags on the field, such as
            `AprilTagField.k2024Crescendo`
        fov (float): horizontal range of vision (degrees)
        """
        self.field_layout = field_layout
        self.fov = Rotation2d.fromDegrees(fov)
        self.camera = camera

        # Vision Simulation
        self.vision_sim = VisionSystemSim("vision_sim")
        self.vision_sim.addAprilTags(self.field_layout)
        self.vision_sim.resetRobotPose
        self.camera_props = SimCameraProperties()
        self.camera_props.setCalibrationFromFOV(640, 480, self.fov)
        self.camera_props.setFPS(fps)
        self.camera_props.setAvgLatency(avg_latency)
        self.camera_props.setLatencyStdDev(latency_std_dev)
        PhotonCameraSim.__init__(
            self, self.camera, self.camera_props, self.field_layout
        )
        self.vision_sim.addCamera(self, self.camera.camera_to_bot)

    def update(self, pose: Pose2d) -> None:
        self.vision_sim.update(pose)
