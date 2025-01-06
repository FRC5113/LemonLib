from phoenix6.hardware import Pigeon2
from wpimath.geometry import Rotation2d, Rotation3d
from wpilib import SmartDashboard
from wpiutil import SendableBuilder, Sendable

from wpilib.interfaces import MotorController
import phoenix6

class LemonPigeon(Pigeon2, Sendable):
    """
    Wrapper for the Pigeon2 that makes it easier to use and some fetures in
    beta and back ports them.
    Acts like a normal Pigeon2 object but with some extra features.
    """

    def __init__(self, device_id, can_bus="rio"):
        """
        Initialize the Pigeon Wrapper.

        :param device_id: The CAN ID of the Pigeon device.
        :param can_bus: The CAN bus name (default is "rio").
        """
        super().__init__(device_id, can_bus)
        Sendable.__init__(self)
        SmartDashboard.putData("Pigeon", self)
       

    def reset(self):
        """
        Reset the yaw angle to zero.
        """
        self.set_yaw(0)

    def getRotation2d(self) -> Rotation2d:
        """
        Get the yaw angle as a Rotation2d object.

        :return: A Rotation2d object representing the yaw.
        """
        yaw_degrees = self.get_yaw().value
        return Rotation2d.fromDegrees(yaw_degrees)

    def get_rotation3d(self):
        """
        Get the yaw, pitch, and roll angles as a Rotation3d object.

        :return: A Rotation3d object representing the yaw, pitch, and roll.
        """
        yaw_degrees = self.get_yaw()
        pitch_degrees = self.get_pitch()
        roll_degrees = self.get_roll()
        return Rotation3d.fromDegrees(yaw_degrees, pitch_degrees, roll_degrees)

    def sim_states_add_yaw(self, angle):
        return self.sim_state.set_raw_yaw(angle)

    def sim_states_voltage(self, voltage):
        return self.sim_state.set_supply_voltage(voltage)
    
    def initSendable(self, builder: SendableBuilder):
        """
        Initialize the sendable builder to connect with the Gyro widget.

        :param builder: The SendableBuilder instance.
        """
        
        builder.setSmartDashboardType("Gyro")
        builder.addDoubleProperty(
            "Value",
            lambda: self.get_yaw().value_as_double,
            lambda _: None,
        )



class LemonTalonFX(phoenix6.hardware.TalonFX, MotorController):
    """Wrapper for the phoenix6 TalonFX that implements
    the wpilib MotorController interface, making it possible
    to use TalonFX controllers in, for example, MotorControllerGroup
    and DifferentialDrive
    """

    def __init__(self, device_id: int, canbus: str = "", enable_foc: bool = False):
        phoenix6.hardware.TalonFX.__init__(self, device_id, canbus=canbus)
        MotorController.__init__(self)
        self.config = phoenix6.configs.TalonFXConfiguration()
        self.duty_cycle_out = phoenix6.controls.DutyCycleOut(0, enable_foc=enable_foc)
        self.voltage_out = phoenix6.controls.VoltageOut(0, enable_foc=enable_foc)
        self.is_disabled = False

    def disable(self):
        self.stopMotor()
        self.is_disabled = True

    def get(self) -> float:
        return self.duty_cycle_out.output

    def getInverted(self) -> bool:
        return (
            self.config.motor_output.inverted
            == phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

    def set(self, speed: float):
        if not self.is_disabled:
            self.duty_cycle_out.output = speed
            self.set_control(self.duty_cycle_out)

    def setIdleMode(self, mode: phoenix6.signals.NeutralModeValue):
        """Set the idle mode setting

        Arguments:
        mode -- Idle mode (coast or brake)
        """
        self.config.motor_output.neutral_mode = mode
        self.configurator.apply(self.config)

    def setInverted(self, isInverted: bool):
        if isInverted:
            self.config.motor_output.inverted = (
                phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
            )
        else:
            self.config.motor_output.inverted = (
                phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
        self.configurator.apply(self.config)

    def setVoltage(self, volts: float):
        if not self.is_disabled:
            self.voltage_out.output = volts
            self.set_control(self.voltage_out)

    def stopMotor(self):
        self.set(0)
