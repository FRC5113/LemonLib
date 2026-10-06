from phoenix6.hardware.talon_fx import TalonFX
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId


class _TalonFXSim:
    """Simulates a single TalonFX-driven motor as a DC motor mechanism and
    feeds the result back into the TalonFX's sim state."""

    def __init__(self, motor: TalonFX, gearbox: DCMotor, moi: float, gearing: float):
        self.gearbox = gearbox
        self.plant = LinearSystemId.DCMotorSystem(self.gearbox, moi, gearing)
        self.gearing = gearing
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.gearbox)

    def getSetpoint(self) -> float:
        return self.sim_state.motor_voltage

    def update(self, dt: float):
        voltage = self.sim_state.motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        self.sim_state.set_raw_rotor_position(
            self.motor_sim.getAngularPositionRotations() * self.gearing
        )
        self.sim_state.set_rotor_velocity(
            self.motor_sim.getAngularVelocityRPM() / 60 * self.gearing
        )


class FalconSim(_TalonFXSim):
    def __init__(self, motor: TalonFX, moi: float, gearing: float):
        super().__init__(motor, DCMotor.falcon500(1), moi, gearing)


class FalconSimFOC(_TalonFXSim):
    def __init__(self, motor: TalonFX, moi: float, gearing: float):
        super().__init__(motor, DCMotor.falcon500FOC(1), moi, gearing)


class KrakenSim(_TalonFXSim):
    def __init__(self, motor: TalonFX, moi: float, gearing: float):
        super().__init__(motor, DCMotor.krakenX60(1), moi, gearing)


class KrakenSimFOC(_TalonFXSim):
    def __init__(self, motor: TalonFX, moi: float, gearing: float):
        super().__init__(motor, DCMotor.krakenX60FOC(1), moi, gearing)
