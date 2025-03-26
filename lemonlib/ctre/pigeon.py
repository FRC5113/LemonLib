import math
from ntcore import NetworkTableInstance
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.hardware.pigeon2 import Pigeon2
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import FeedbackSensorSourceValue
from phoenix6.configs.config_groups import Slot0Configs, Slot1Configs, FeedbackConfigs
from phoenix6.signals.spn_enums import GravityTypeValue, SensorDirectionValue


class LemonPigeon:
    "Gyro class that creates an instance of the Pigeon 2.0 Gyro"

    def __init__(self, can_id: int):
        self.gyro = Pigeon2(can_id)
        self.gyro.reset()

    def getAngleCCW(self):

        return self.gyro.get_yaw().value

    def getRoll(self):
        return self.gyro.get_roll().value

    def getPitch(self):
        return self.gyro.get_pitch().value

    def getDegreesPerSecCCW(self):
        return self.gyro.get_angular_velocity_z_world().value

    def getRadiansPerSecCCW(self):
        return math.radians(self.getDegreesPerSecCCW())

    def getRotation2d(self):
        return self.gyro.getRotation2d()

    def setAngleAdjustment(self, angle):
        self.gyro.set_yaw(angle)
