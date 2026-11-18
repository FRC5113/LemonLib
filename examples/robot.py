import robotpy_fields
import telemetry
import wpilib
import wpimath
from lemonlib.vision import CameraType, LemonCamera, Lemonlight, photonname


class Robot(wpilib.TimedRobot):
    def __init__(self) -> None:
        super().__init__()

        rtc = wpimath.Transform3d()
        field = robotpy_fields.get_field(robotpy_fields.FieldId.FRC_2026_REBUILT_WELDED)
        self.lime = Lemonlight("limelightsc3", rtc, field)
        self.photon = photonname("Uno", rtc, field)

        self.lemon = LemonCamera("Uno", rtc, field, CameraType.PHOTONVISION)
        self.lemon = LemonCamera("limelightsc3", rtc, field, CameraType.LIMELIGHT)

    def teleop_periodic(self) -> None:
        self.lime.get_latest_results()
        self.photon.get_latest_results()
        telemetry.log(
            "lime",
            f"{self.lime.estimate_multi_tag_pose().get_latest_pose()!s}",
        )

        telemetry.log(
            "photon",
            f"{self.photon.estimate_multi_tag_pose().get_latest_pose()!s}",
        )
