import math
from dataclasses import dataclass, field

import msgpack
import ntcore
from wpimath import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Transform3d,
    Translation2d,
    Translation3d,
    units,
)


@dataclass
class FiducialTarget:
    # Fiducial/AprilTag ID.
    fiducial_id: int = 0

    # Fiducial family (e.g. "36h11")
    family: str = ""

    # Target skew values
    skew: list[float] = field(default_factory=list)

    # Pose ambiguity. Lower is better. Values above 0.7 are usually not reliable
    ambiguity: float = 1.0

    # True if this tag contributed to pose estimation. The tag matched the field map and the ID
    # filters did not exclude it.
    fielded: bool = False

    # Camera pose in target space [x, y, z, roll, pitch, yaw] (meters, degrees).
    camera_pose_target_space: list[float] = field(default_factory=lambda: [0.0] * 6)

    # Target pose in camera space.
    target_pose_camera_space: list[float] = field(default_factory=lambda: [0.0] * 6)

    # Target pose in robot space.
    target_pose_robot_space: list[float] = field(default_factory=lambda: [0.0] * 6)

    # Robot pose in target space.
    robot_pose_target_space: list[float] = field(default_factory=lambda: [0.0] * 6)

    # Robot pose in field space (MegaTag1, this tag only).
    robot_pose_field_space: list[float] = field(default_factory=lambda: [0.0] * 6)

    # Robot pose in field space (MegaTag2, this tag only).
    robot_pose_field_space_mt2: list[float] = field(default_factory=lambda: [0.0] * 6)

    def get_distance_to_camera(self) -> units.meters:
        """
        Returns 3D distance from the camera to this tag in meters.
        """
        return self._distance3d(self.target_pose_camera_space)

    def get_distance_to_robot(self) -> units.meters:
        """
        Returns 3D distance from the robot center to this tag in meters.
        """
        return self._distance3d(self.target_pose_robot_space)

    def get_camera_pose_target_space(self) -> Pose3d | None:
        """
        Returns camera pose in target space as a Pose3d.
        """
        return to_Pose3d(self.camera_pose_target_space)

    def get_target_pose_camera_space(self) -> Pose3d | None:
        """
        Returns target pose in camera space as a Pose3d.
        """
        return to_Pose3d(self.target_pose_camera_space)

    def get_target_pose_robot_space(self) -> Pose3d | None:
        """
        Returns target pose in robot space as a Pose3d.
        """
        return to_Pose3d(self.target_pose_robot_space)

    def get_robot_pose_target_space(self) -> Pose3d | None:
        """
        Returns robot pose in target space as a Pose3d.
        """
        return to_Pose3d(self.robot_pose_target_space)

    def get_robot_pose_field_space(self) -> Pose3d | None:
        """
        Returns megaTag1 robot pose in field space from this tag only, as a Pose3d.
        """
        return to_Pose3d(self.robot_pose_field_space)

    def get_robot_pose_field_space_mt2(self) -> Pose3d | None:
        """
        Returns megaTag2 robot pose in field space from this tag only, as a Pose3d.
        """
        return to_Pose3d(self.robot_pose_field_space_mt2)

    @staticmethod
    def _distance3d(pose: list[float]) -> float:
        if len(pose) < 3:
            return 0

        return math.sqrt(pose[0] ** 2 + pose[1] ** 2 + pose[2] ** 2)


@dataclass
class LimelightResults:
    """Results from a Limelight vision frame."""

    # Frame validity and timestamps
    valid: bool = False
    timestamp_millis: units.milliseconds = 0.0
    timestamp_micros: int = 0
    system_timestamp_micros: int = 0
    nt_timestamp_micros: int = 0
    frame_index: int = -1

    # Pipeline information
    targeting_latency_millis: units.milliseconds = 0.0
    capture_latency_millis: units.milliseconds = 0.0
    pipeline_index: float = -1.0
    pipeline_type: str = ""
    focus_metric: float = 0.0

    # Camera-reported standard deviations [x, y, z, roll, pitch, yaw]
    std_devs_mt1: list[float] = field(default_factory=lambda: [0.0] * 6)
    std_devs_mt2: list[float] = field(default_factory=lambda: [0.0] * 6)

    # MegaTag1 robot poses
    robot_pose_mt1: list[float] = field(default_factory=lambda: [0.0] * 6)
    robot_pose_mt1_wpi_red: list[float] = field(default_factory=lambda: [0.0] * 6)
    robot_pose_mt1_wpi_blue: list[float] = field(default_factory=lambda: [0.0] * 6)

    # MegaTag2 robot poses
    robot_pose_mt2: list[float] = field(default_factory=lambda: [0.0] * 6)
    robot_pose_mt2_wpi_red: list[float] = field(default_factory=lambda: [0.0] * 6)
    robot_pose_mt2_wpi_blue: list[float] = field(default_factory=lambda: [0.0] * 6)

    # AprilTag information
    reported_tag_count: int = 0
    tag_span_meters: units.meters = 0.0
    avg_tag_distance_meters: units.meters = 0.0
    avg_tag_area_percent: units.percent = 0.0

    # Camera pose in robot space
    camera_pose_robot_space: list[float] = field(default_factory=lambda: [0.0] * 6)

    # Python SnapScript output
    python_output: list[float] = field(default_factory=list)

    # Primary target information
    tx_degrees: units.degrees = 0.0
    ty_degrees: units.degrees = 0.0
    tx_degrees_no_crosshair: float = 0.0
    ty_degrees_no_crosshair: float = 0.0
    target_area_percent: units.percent = 0.0
    target_distance_meters: units.meters = 0.0

    # Targets
    fiducial_targets: list[FiducialTarget] = field(default_factory=list[FiducialTarget])

    # Hardware and image information
    image_source: int = 0
    hardware_type: int = 0
    ui_refresh: int = 0

    # NetworkTables state
    ignore_nt: bool = False
    nt_connected: bool = False

    # Pipeline configuration override
    pipeline_configuration_override_active: bool = False

    # Shared field map
    shared_map_active: bool = False

    # Parsing and networking
    parse_latency_millis: units.milliseconds = 0.0
    receive_timestamp_seconds: units.seconds = 0.0

    # Error information
    error: str = ""

    def get_age_seconds(self) -> units.seconds:
        return ntcore._now() / 1e9 - self.receive_timestamp_seconds

    def get_target_count(self) -> int:
        return len(self.fiducial_targets)

    def get_robot_pose_mt1(self) -> Pose3d | None:
        return to_Pose3d(self.robot_pose_mt1)

    def get_robot_pose_mt2(self) -> Pose3d | None:
        return to_Pose3d(self.robot_pose_mt2)

    def get_camera_pose_robot_space(self) -> Pose3d | None:
        return to_Pose3d(self.camera_pose_robot_space)


@staticmethod
def _transform3d_to_list(transform: Transform3d) -> list[float]:
    arr = []
    rot = transform.rotation()

    arr.append(transform.x)
    arr.append(transform.y)
    arr.append(transform.z)
    arr.append(rot.x)
    arr.append(rot.y)
    arr.append(rot.z)
    return arr


@staticmethod
def to_Pose3d(in_data: list[float]) -> Pose3d | None:
    if (in_data is None) or (len(in_data) < 6):
        return None

    return Pose3d(
        Translation3d(in_data[0], in_data[1], in_data[2]),
        Rotation3d(
            units.degrees_to_radians(in_data[3]),
            units.degrees_to_radians(in_data[4]),
            units.degrees_to_radians(in_data[5]),
        ),
    )


@staticmethod
def to_Pose2d(in_data: list[float]) -> Pose2d | None:
    if (in_data is None) or (len(in_data) < 6):
        return None

    return Pose2d(
        Translation2d(in_data[0], in_data[1]),
        Rotation2d(units.degrees_to_radians(in_data[3])),
    )


@staticmethod
def decode(data: list[ntcore.TimestampedRaw]) -> list[LimelightResults]:
    arr: list[LimelightResults] = []
    for raw in data:
        try:
            results = msgpack.unpackb(
                raw.value,
                raw=False,
                strict_map_key=False,
            )
        except Exception as e:
            print(f"Failed to decode Limelight data: {e}")
            continue

        arr.append(_decode_results(results))

    return arr


@staticmethod
def _decode_results(results: dict) -> LimelightResults:
    out = LimelightResults()
    for key, item in results.items():
        match key:
            case "v":
                out.valid = item
            case "ts":
                out.timestamp_millis = item
            case "ts_us":
                out.timestamp_micros = item
            case "ts_sys":
                out.system_timestamp_micros = item
            case "ts_nt":
                out.nt_timestamp_micros = item
            case "fidx":
                out.frame_index = item
            case "tl":
                out.targeting_latency_millis = item
            case "cl":
                out.capture_latency_millis = item
            case "pID":
                out.pipeline_index = item
            case "pType":
                out.pipeline_type = item
            case "focus_metric":
                out.focus_metric = item
            case "stdev_mt1":
                out.std_devs_mt1 = item
            case "stdev_mt2":
                out.std_devs_mt2 = item
            case "botpose":
                out.robot_pose_mt1 = item
            case "botpose_wpired":
                out.robot_pose_mt1_wpi_red = item
            case "botpose_wpiblue":
                out.robot_pose_mt1_wpi_blue = item
            case "botpose_orb":
                out.robot_pose_mt2 = item
            case "botpose_orb_wpired":
                out.robot_pose_mt2_wpi_red = item
            case "botpose_orb_wpiblue":
                out.robot_pose_mt2_wpi_blue = item
            case "botpose_tagcount":
                out.reported_tag_count = int(item)
            case "botpose_span":
                out.tag_span_meters = item
            case "botpose_avgdist":
                out.avg_tag_distance_meters = item
            case "botpose_avgarea":
                out.avg_tag_area_percent = item
            case "t6c_rs":
                out.camera_pose_robot_space = item
            case "PythonOut":
                out.python_output = item
            case "tx":
                out.tx_degrees = item
            case "ty":
                out.ty_degrees = item
            case "txnc":
                out.tx_degrees_no_crosshair = item
            case "tync":
                out.ty_degrees_no_crosshair = item
            case "ta":
                out.target_area_percent = item
            case "tdist":
                out.target_distance_meters = item
            case "Retro":
                continue
            case "Fiducial":
                out.fiducial_targets = _decode_fiducial_targets(item)
            case "Detector":
                continue
            case "Classifier":
                continue
            case "ClassifierRejected":
                continue
            case "Barcode":
                continue
            case "Counter":
                continue
            case "OCR":
                continue
            case "imu":
                continue
            case "hw":
                continue
            case "rewind":
                continue
            case "botorient":
                continue
            case "intrinsics":
                continue
            case "imgsrc":
                continue
            case "hwtype":
                continue
            case "uirefresh":
                out.ui_refresh = item
                break
            case "ignorent":
                out.ignore_nt = item
            case "ntconnected":
                out.nt_connected = item != 0
            case "codepipeline":
                out.pipeline_configuration_override_active = item != 0
            case "codepipelinestate":
                continue
            case "codemap":
                out.shared_map_active = item != 0
            case "codemapstate":
                continue
            case _:
                continue
    return out


@staticmethod
def _decode_fiducial_targets(data) -> list[FiducialTarget]:
    targets = [FiducialTarget() for _ in data]

    for index, target in enumerate(data):
        t = targets[index]
        for key, item in target.items():
            match key:
                case "fID":
                    t.fiducial_id = int(item)
                case "fam":
                    t.family = item
                case "skew":
                    t.skew = item
                case "ambig":
                    t.ambiguity = item
                case "fielded":
                    t.fielded = item != 0
                case "t6c_ts":
                    t.camera_pose_target_space = item
                case "t6t_cs":
                    t.target_pose_camera_space = item
                case "t6t_rs":
                    t.target_pose_robot_space = item
                case "t6r_ts":
                    t.target_pose_robot_space = item
                case "t6r_fs":
                    t.robot_pose_field_space = item
                case "t6r_fs_orb":
                    t.robot_pose_field_space_mt2 = item
                case _:
                    continue

    return targets
