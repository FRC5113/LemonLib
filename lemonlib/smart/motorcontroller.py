# TODO: Replace stubs + testing

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Generic, List, Optional, Tuple, TypeVar

#from wpilib import DriverStation, SmartDashboard
from wpimath.controller import (
    ArmFeedforward,
    ElevatorFeedforward,
    PIDController,
    SimpleMotorFeedforwardMeters,
)
from wpimath.filter import Debouncer
from wpimath.system.plant import DCMotor
from wpimath.trajectory import TrapezoidProfile
from wpimath.units import rotationsToRadians
import ntcore

from .motorcontrollerconfig import *
from lemonlib.util.notifier import Notifier

# Telemetry stubs

class SmartMotorControllerTelemetry:
    """Stub — replace with real telemetry implementation."""

    class BooleanTelemetryField:
        pass

    class DoubleTelemetryField:
        pass

    def setupTelemetry(self, controller, telemetry_table, tuning_table, config) -> None:
        pass

    def tuningEnabled(self) -> bool:
        return False

    def publish(self, controller) -> None:
        pass

    def applyTuningValues(self, controller) -> None:
        pass

    def close(self) -> None:
        pass


class SmartMotorControllerTelemetryConfig:
    def withTelemetryVerbosity(self, verbosity) -> "SmartMotorControllerTelemetryConfig":
        return self


class SmartMotorControllerCommandRegistry:
    @staticmethod
    def addCommand(group: str, subsystem, command_fn: Callable) -> None:
        pass

# SimSupplier stub

class SimSupplier:
    pass

class SmartMotorController(ABC):
    """
    Python port of yams.motorcontrollers.SmartMotorController.

    Unit conventions (all floats):
      - Angles / positions: rotations
      - Angular velocities: rps
      - Distances: m
      - Linear velocities: m/s
      - Voltage: V
      - Current: amps
      - Temperature: C
      - Time: s
    """

    def __init__(self) -> None:
        self.telemetry: SmartMotorControllerTelemetry = SmartMotorControllerTelemetry()
        self.config: SmartMotorControllerConfig = SmartMotorControllerConfig()

        # Motion profiles
        self.trapezoid_profile: Optional[TrapezoidProfile] = None
        self.trap_state: Optional[TrapezoidProfile.State] = None

        # Controllers
        self.pid: Optional[PIDController] = None
        # self.lqr: Optional[LQRController] = None

        # Setpoints (rotations / rotations-per-second)
        self.setpoint_position: Optional[float] = None   # rotations
        self.setpoint_velocity: Optional[float] = None   # rotations/second

        # Notifier thread
        self.closed_loop_controller_thread: Optional[Notifier] = None
        self._closed_loop_controller_running: bool = False

        # NetworkTables
        self.parent_table = None
        self.telemetry_table = None
        self.tuning_table = None
        self.telemetry_config = None

        # Simulation
        self.sim_supplier: Optional[SimSupplier] = None

        # Loose followers
        self.loose_followers: Optional[List["SmartMotorController"]] = None

    # Factory

    @staticmethod
    def create(motor_controller: object, motor_sim: DCMotor,
               cfg: SmartMotorControllerConfig) -> Optional["SmartMotorController"]:
        """
        Instantiate the correct SmartMotorController subclass based on the
        motor_controller object type. Returns None until subclasses are registered.
        """
        return None

    # Motor identification

    def is_motor(self, a: DCMotor, b: DCMotor) -> bool:
        """Return True if both DCMotor specs describe the same motor type."""
        return (
            a.stallTorqueNewtonMeters == b.stallTorqueNewtonMeters
            and a.stallCurrentAmps == b.stallCurrentAmps
            and a.freeCurrentAmps == b.freeCurrentAmps
            and a.freeSpeedRadPerSec == b.freeSpeedRadPerSec
            and a.KtNMPerAmp == b.KtNMPerAmp
            and a.KvRadPerSecPerVolt == b.KvRadPerSecPerVolt
            and a.nominalVoltageVolts == b.nominalVoltageVolts
        )

    # Safety check

    def check_config_safety(self) -> None:
        """
        Validate config for motor-specific safe current limits.
        Raises SmartMotorControllerConfigurationException on violation.
        """
        dc = self.get_dc_motor()

        if self.is_motor(dc, DCMotor.getNeo550(1)):
            limit = self.config.getStatorStallCurrentLimit()
            if limit is None:
                raise SmartMotorControllerConfigurationException(
                    "Stator current limit is not defined for NEO550!",
                    "Safety check failed.",
                    "withStatorCurrentLimit(current_amps)"
                )
            if limit > 40:
                raise SmartMotorControllerConfigurationException(
                    "Stator current limit is too high for NEO550!",
                    "Safety check failed.",
                    "withStatorCurrentLimit(current_amps) where current < 40A"
                )

        if self.is_motor(dc, DCMotor.getNEO(1)):
            limit = self.config.getStatorStallCurrentLimit()
            if limit is None:
                raise SmartMotorControllerConfigurationException(
                    "Stator current limit is not defined for NEO!",
                    "Safety check failed.",
                    "withStatorCurrentLimit(current_amps)"
                )
            if limit > 60:
                raise SmartMotorControllerConfigurationException(
                    "Stator current limit is too high for NEO!",
                    "Safety check failed.",
                    "withStatorCurrentLimit(current_amps) where current < 60A"
                )

    # Simulation supplier

    def get_sim_supplier(self) -> Optional[SimSupplier]:
        return self.sim_supplier

    def set_sim_supplier(self, supplier: SimSupplier) -> None:
        self.sim_supplier = supplier

    # Profile state helpers

    def _get_trapezoidal_profile_state(self) -> Optional[TrapezoidProfile.State]:
        """Current encoder state as a TrapezoidProfile.State."""
        if self.trapezoid_profile is None:
            return None
        if self.config.getLinearClosedLoopControllerUse():
            return TrapezoidProfile.State(
                self.get_measurement_position(),   # meters
                self.get_measurement_velocity()    # m/s
            )
        return TrapezoidProfile.State(
            self.get_mechanism_position(),         # rotations
            self.get_mechanism_velocity()          # rot/s
        )

    # Closed-loop controller lifecycle

    def stop_closed_loop_controller(self) -> None:
        if self.closed_loop_controller_thread is not None:
            self.closed_loop_controller_thread.stop()
            self._closed_loop_controller_running = False

    def start_closed_loop_controller(self) -> None:
        if (self.closed_loop_controller_thread is not None
                and self.config.getMotorControllerMode()
                == SmartMotorControllerConfig.ControlMode.CLOSED_LOOP):

            if self.pid is not None:
                self.pid.reset()

            self.trap_state = self._get_trapezoidal_profile_state()

            # if self.lqr is not None:
            #     if self.config.getLinearClosedLoopControllerUse():
            #         self.lqr.reset(self.get_measurement_position(),
            #                          self.get_measurement_velocity())
            #     else:
            #         self.lqr.reset(self.get_mechanism_position(),
            #                          self.get_mechanism_velocity())

            self.closed_loop_controller_thread.stop()
            period_s = self.config.getClosedLoopControlPeriod() or 0.020
            self.closed_loop_controller_thread.startPeriodic(period_s)
            self._closed_loop_controller_running = True

    # Core closed-loop iteration

    def iterate_closed_loop_controller(self) -> None:
        """
        Run one iteration of the closed-loop controller.
        Called periodically by the Notifier thread.

        All angles in rotations, velocities in rot/s (or meters / m/s when
        linear mode is active).
        """
        velocity_trapezoidal_profile = False
        next_trap_state = TrapezoidProfile.State(0.0, 0.0)
        pid_output_voltage = 0.0
        feedforward = 0.0

        mech_lower_limit = self.config.getMechanismLowerLimit()   # rotations | None
        mech_upper_limit = self.config.getMechanismUpperLimit()   # rotations | None
        arm_ff = self.config.getArmFeedforward()
        elevator_ff = self.config.getElevatorFeedforward()
        simple_ff = self.config.getSimpleFeedforwardMeters()
        temp_cutoff = self.config.getTemperatureCutoff()          # celsius | None
        max_voltage = self.config.getClosedLoopControllerMaximumVoltage()  # volts | None

        self.synchronize_relative_encoder()

        if not self._closed_loop_controller_running:
            return

        # --- Clamp setpoint to mechanism limits ---
        if self.setpoint_position is not None:
            if mech_lower_limit is not None and self.setpoint_position < mech_lower_limit:
                name = self.config.getTelemetryName() or "Unnamed smart motor"
                DriverStation.reportWarning(
                    f"[WARNING] Setpoint is lower than Mechanism {name} lower limit, "
                    "changing setpoint to lower limit.", False
                )
                self.setpoint_position = mech_lower_limit

            if mech_upper_limit is not None and self.setpoint_position > mech_upper_limit:
                name = self.get_name()
                DriverStation.reportWarning(
                    f"[WARNING] Setpoint is higher than Mechanism {name} upper limit, "
                    "changing setpoint to upper limit.", False
                )
                self.setpoint_position = mech_upper_limit

        # --- Motion profile setpoints ---
        loop_time = self.config.getClosedLoopControlPeriod() or 0.020
        linear = self.config.getLinearClosedLoopControllerUse()

        if self.setpoint_position is not None:
            setpoint = self.setpoint_position                       # rotations
            position = self.get_mechanisposition()               # rotations
            velocity = self.get_mechanisvelocity()               # rot/s

            if linear:
                position = self.get_measurement_position()         # meters
                velocity = self.get_measurement_velocity()         # m/s
                setpoint = self.config.convertFromMechanism(self.setpoint_position)  # meters

            if self.trapezoid_profile is not None:
                fallback = self.trap_state or TrapezoidProfile.State(position, velocity)
                next_trap_state = self.trapezoid_profile.calculate(
                    loop_time, fallback,
                    TrapezoidProfile.State(setpoint, 0.0)
                )

        elif self.setpoint_velocity is not None:
            setpoint = self.setpoint_velocity                      # rot/s
            velocity = self.get_mechanism_velocity()               # rot/s

            if linear:
                velocity = self.get_measurement_velocity()         # m/s
                setpoint = self.config.convertFromMechanism(self.setpoint_velocity)  # m/s

            if self.trapezoid_profile is not None:
                # TODO (2027): derive acceleration from SMCs
                fallback = self.trap_state or TrapezoidProfile.State(velocity, 0.0)
                next_trap_state = self.trapezoid_profile.calculate(
                    loop_time, fallback,
                    TrapezoidProfile.State(setpoint, 0.0)
                )
                velocity_trapezoidal_profile = True

        # --- PID output ---
        if self.setpoint_position is not None:
            measured = self.get_mechanism_position()               # rotations
            setpoint = self.setpoint_position                      # rotations
            velocity_profile = 0.0

            if linear:
                measured = self.get_measurement_position()
                setpoint = self.config.convertFromMechanism(self.setpoint_position)

            if self.expo_profile is not None:
                setpoint = next_expo_state.position
                velocity_profile = next_expo_state.velocity
            elif (self.trapezoid_profile is not None
                  and not self.config.getVelocityTrapezoidalProfileInUse()):
                setpoint = next_trap_state.position
                velocity_profile = next_trap_state.velocity

            if self.pid is not None:
                pid_output_voltage = self.pid.calculate(measured, setpoint)

            # if self.lqr is not None:
            #     if linear:
            #         pid_output_voltage = self.lqr.calculate(
            #             measured, setpoint, velocity_profile)
            #     else:
            #         pid_output_voltage = self.lqr.calculate(
            #             measured, setpoint, velocity_profile)

        elif self.setpoint_velocity is not None:
            setpoint = self.setpoint_velocity                      # rot/s
            velocity = self.get_mechanism_velocity()               # rot/s

            if linear:
                velocity = self.get_measurement_velocity()
                setpoint = self.config.convertFromMechanism(self.setpoint_velocity)

            if (self.trapezoid_profile is not None
                    and self.config.getVelocityTrapezoidalProfileInUse()):
                # In velocity mode: .position holds the velocity setpoint (poorly named in original)
                setpoint = next_trap_state.position
                # .velocity holds the acceleration setpoint (also poorly named)
                _acceleration = next_trap_state.velocity  # noqa: F841 — available for FF use

            if self.pid is not None:
                pid_output_voltage = self.pid.calculate(velocity, setpoint)

            # if self.lqr is not None:
            #     pid_output_voltage = self.lqr.calculate(velocity, setpoint)

        # --- Feedforward ---

        if arm_ff is not None:
            profiled = (self.expo_profile is not None
                        or self.trapezoid_profile is not None)
            if profiled and not velocity_trapezoidal_profile:
                cur_vel_rps = (
                    self.trap_state.velocity if self.trap_state is not None
                    else (self.expo_state.velocity if self.expo_state is not None else 0.0)
                )
                nxt_vel_rps = (
                    next_trap_state.velocity if self.trapezoid_profile is not None
                    else (next_expo_state.velocity if self.expo_profile is not None else 0.0)
                )
                feedforward = arm_ff.calculateWithVelocities(
                    rotationsToRadians(self.get_mechanism_position()),
                    rotationsToRadians(cur_vel_rps),
                    rotationsToRadians(nxt_vel_rps)
                )
            else:
                nxt_vel_rps = (
                    next_trap_state.position
                    if velocity_trapezoidal_profile
                    else (self.setpoint_velocity or 0.0)
                )
                arm_ff.calculateWithVelocities(
                    rotationsToRadians(self.get_mechanism_position()),
                    rotationsToRadians(self.get_mechanism_velocity()),
                    rotationsToRadians(nxt_vel_rps)
                )
                # Note: Java original forgets to store this result — preserved as-is.

        if elevator_ff is not None:
            profiled = ((self.expo_profile is not None
                         or self.trapezoid_profile is not None)
                        and self.setpoint_position is not None)
            if profiled and not velocity_trapezoidal_profile:
                cur_vel_ms = (
                    self.trap_state.velocity if self.trap_state is not None
                    else (self.expo_state.velocity if self.expo_state is not None else 0.0)
                )
                nxt_vel_ms = (
                    next_trap_state.velocity if self.trapezoid_profile is not None
                    else (next_expo_state.velocity if self.expo_profile is not None else 0.0)
                )
                feedforward = elevator_ff.calculateWithVelocities(cur_vel_ms, nxt_vel_ms)
            else:
                # TODO: Implement velocity profile for elevator FF
                feedforward = elevator_ff.calculateWithVelocities(
                    self.get_measurement_velocity(), 0.0)

        if simple_ff is not None:
            profiled = (self.expo_profile is not None
                        or self.trapezoid_profile is not None)
            if profiled and not velocity_trapezoidal_profile:
                cur_vel_rps = (
                    self.trap_state.velocity if self.trap_state is not None
                    else (self.expo_state.velocity if self.expo_state is not None else 0.0)
                )
                nxt_vel_rps = (
                    next_trap_state.velocity if self.trapezoid_profile is not None
                    else (next_expo_state.velocity if self.expo_profile is not None else 0.0)
                )
                feedforward = simple_ff.calculateWithVelocities(cur_vel_rps, nxt_vel_rps)
            else:
                nxt_vel_rps = (
                    next_trap_state.position
                    if velocity_trapezoidal_profile
                    else (self.setpoint_velocity or 0.0)
                )
                feedforward = simple_ff.calculateWithVelocities(
                    self.get_mechanism_velocity(), nxt_vel_rps)

        # --- Advance profile states ---
        if self.expo_profile is not None:
            self.expo_state = next_expo_state
        if self.trapezoid_profile is not None:
            self.trap_state = next_trap_state

        # --- Boundary / safety clamps ---
        if mech_upper_limit is not None:
            if (self.get_mechanism_position() > mech_upper_limit
                    and (pid_output_voltage + feedforward) > 0):
                feedforward = 0.0
                pid_output_voltage = 0.0

        if mech_lower_limit is not None:
            if (self.get_mechanism_position() < mech_lower_limit
                    and (pid_output_voltage + feedforward) < 0):
                feedforward = 0.0
                pid_output_voltage = 0.0

        if temp_cutoff is not None:
            if self.get_temperature() >= temp_cutoff:
                feedforward = 0.0
                pid_output_voltage = 0.0

        output_voltage = pid_output_voltage + feedforward
        if max_voltage is not None:
            output_voltage = max(-max_voltage, min(max_voltage, output_voltage))

        self.set_voltage(output_voltage)

    # Abstract methods: implement in subclass per motor controller type

    @abstractmethod
    def setup_simulation(self) -> None:
        """Configure simulation for this motor controller."""

    @abstractmethod
    def seed_relative_encoder(self) -> None:
        """Seed the relative encoder from the absolute encoder position."""

    @abstractmethod
    def synchronize_relative_encoder(self) -> None:
        """Re-sync the relative encoder if it has drifted from the absolute encoder."""

    @abstractmethod
    def sim_iterate(self) -> None:
        """Advance the simulation by one timestep."""

    @abstractmethod
    def set_idle_mode(self, mode: str) -> None:
        """Set COAST or BRAKE idle mode (use MotorMode constants)."""

    @abstractmethod
    def set_encoder_velocity_angular(self, velocity_rps: float) -> None:
        """Set encoder velocity (rotations/second)."""

    @abstractmethod
    def set_encoder_velocity_linear(self, velocity_ms: float) -> None:
        """Set encoder velocity (meters/second)."""

    @abstractmethod
    def set_encoder_position_angular(self, angle_rotations: float) -> None:
        """Set encoder position (rotations)."""

    @abstractmethod
    def set_encoder_position_linear(self, distance_meters: float) -> None:
        """Set encoder position (meters)."""

    @abstractmethod
    def set_position_angular(self, angle_rotations: float) -> None:
        """Command the mechanism to an angle setpoint (rotations)."""

    @abstractmethod
    def set_position_linear(self, distance_meters: float) -> None:
        """Command the mechanism to a distance setpoint (meters)."""

    @abstractmethod
    def set_velocity_linear(self, velocity_ms: float) -> None:
        """Command the mechanism to a linear velocity setpoint (m/s)."""

    @abstractmethod
    def set_velocity_angular(self, velocity_rps: float) -> None:
        """Command the mechanism to an angular velocity setpoint (rot/s)."""

    @abstractmethod
    def apply_config(self, config: SmartMotorControllerConfig) -> bool:
        """Apply a SmartMotorControllerConfig. Returns True on success."""

    @abstractmethod
    def get_duty_cycle(self) -> float:
        """Return duty cycle in [-1, 1]."""

    @abstractmethod
    def set_duty_cycle(self, duty_cycle: float) -> None:
        """Set duty cycle output in [-1, 1]."""

    @abstractmethod
    def get_supply_current(self) -> Optional[float]:
        """Supply current in amps, or None if unavailable."""

    @abstractmethod
    def get_stator_current(self) -> float:
        """Stator current in amps."""

    @abstractmethod
    def get_voltage(self) -> float:
        """Motor output voltage in volts."""

    @abstractmethod
    def set_voltage(self, voltage_volts: float) -> None:
        """Set motor output voltage in volts."""

    @abstractmethod
    def get_dc_motor(self) -> DCMotor:
        """Return the DCMotor model for this motor controller."""

    @abstractmethod
    def get_measurement_velocity(self) -> float:
        """Linear velocity of the mechanism in meters/second (post-gearing)."""

    @abstractmethod
    def get_measurement_position(self) -> float:
        """Linear position of the mechanism in meters (post-gearing)."""

    @abstractmethod
    def get_mechanism_velocity(self) -> float:
        """Angular velocity of the mechanism in rotations/second (post-gearing)."""

    @abstractmethod
    def get_mechanism_position(self) -> float:
        """Angular position of the mechanism in rotations (post-gearing)."""

    @abstractmethod
    def get_rotor_velocity(self) -> float:
        """Raw rotor angular velocity in rotations/second."""

    @abstractmethod
    def get_rotor_position(self) -> float:
        """Raw rotor position in rotations (since power-on)."""

    @abstractmethod
    def set_motor_inverted(self, inverted: bool) -> None:
        """Invert the motor output direction."""

    @abstractmethod
    def set_encoder_inverted(self, inverted: bool) -> None:
        """Invert the encoder phase."""

    @abstractmethod
    def set_motion_profile_max_velocity_linear(self, max_velocity_ms: float) -> None:
        """Max velocity for trapezoidal profile (m/s)."""

    @abstractmethod
    def set_motion_profile_max_acceleration_linear(self, max_accel_ms2: float) -> None:
        """Max acceleration for trapezoidal profile (m/s²)."""

    @abstractmethod
    def set_motion_profile_max_velocity_angular(self, max_velocity_rps: float) -> None:
        """Max velocity for trapezoidal profile (rot/s)."""

    @abstractmethod
    def set_motion_profile_max_acceleration_angular(self, max_accel_rps2: float) -> None:
        """Max acceleration for trapezoidal profile (rot/s²)."""

    @abstractmethod
    def set_motion_profile_max_jerk(self, max_jerk: float) -> None:
        """Max jerk for the motion profile (rot/s³)."""

    @abstractmethod
    def set_kp(self, kP: float) -> None: ...

    @abstractmethod
    def set_ki(self, kI: float) -> None: ...

    @abstractmethod
    def set_kd(self, kD: float) -> None: ...

    @abstractmethod
    def set_feedback(self, kP: float, kI: float, kD: float) -> None: ...

    @abstractmethod
    def set_ks(self, kS: float) -> None: ...

    @abstractmethod
    def set_kv(self, kV: float) -> None: ...

    @abstractmethod
    def set_ka(self, kA: float) -> None: ...

    @abstractmethod
    def set_kg(self, kG: float) -> None: ...

    @abstractmethod
    def set_feedforward(self, kS: float, kV: float, kA: float, kG: float) -> None: ...

    @abstractmethod
    def set_stator_current_limit(self, current_amps: float) -> None: ...

    @abstractmethod
    def set_supply_current_limit(self, current_amps: float) -> None: ...

    @abstractmethod
    def set_closed_loop_ramp_rate(self, ramp_rate_seconds: float) -> None: ...

    @abstractmethod
    def set_open_loop_ramp_rate(self, ramp_rate_seconds: float) -> None: ...

    @abstractmethod
    def set_measurement_upper_limit(self, upper_limit_meters: float) -> None: ...

    @abstractmethod
    def set_measurement_lower_limit(self, lower_limit_meters: float) -> None: ...

    @abstractmethod
    def set_mechanism_upper_limit(self, upper_limit_rotations: float) -> None: ...

    @abstractmethod
    def set_mechanism_lower_limit(self, lower_limit_rotations: float) -> None: ...

    @abstractmethod
    def get_temperature(self) -> float:
        """Motor temperature in celsius."""

    @abstractmethod
    def get_config(self) -> SmartMotorControllerConfig: ...

    @abstractmethod
    def get_motor_controller(self) -> object: ...

    @abstractmethod
    def get_motor_controller_config(self) -> object: ...

    @abstractmethod
    def get_unsupported_telemetry_fields(
        self
    ) -> Tuple[
        Optional[List[SmartMotorControllerTelemetry.BooleanTelemetryField]],
        Optional[List[SmartMotorControllerTelemetry.DoubleTelemetryField]]
    ]: ...

    # Telemetry

    def setup_telemetry(self, telemetry_table=None, tuning_table=None) -> None:
        """
        Setup NetworkTables telemetry.
        If tables are not provided, defaults to /Mechanisms and /Tuning.
        """
        if telemetry_table is None or tuning_table is None:
            nt = ntcore.NetworkTableInstance.getDefault()
            telemetry_table = nt.getTable("Mechanisms")
            tuning_table = nt.getTable("Tuning")

        print("=" * 52 + "\nSETUP TELEMETRY\n" + "=" * 52)

        if self.parent_table is None:
            self.parent_table = telemetry_table
            name = self.config.getTelemetryName()
            if name is not None:
                self.telemetry_table = telemetry_table.getSubTable(self.get_name())
                self.tuning_table = tuning_table.getSubTable(self.get_name())

                cfg = self.config.getSmartControllerTelemetryConfig()
                if cfg is not None:
                    self.telemetry.setupTelemetry(
                        self, self.telemetry_table, self.tuning_table, cfg)
                else:
                    self.telemetry.setupTelemetry(
                        self, self.telemetry_table, self.tuning_table,
                        SmartMotorControllerTelemetryConfig().withTelemetryVerbosity(
                            self.config.getVerbosity()
                            or SmartMotorControllerConfig.TelemetryVerbosity.HIGH
                        )
                    )

                self.update_telemetry()

                if self.telemetry.tuningEnabled():
                    path_parts = self.telemetry_table.getPath()[1:].split("/")
                    telemetry_path_str = (path_parts[0] + "/Commands/"
                                         + path_parts[-1])

                    # Zero encoder command
                    def _zero_encoder():
                        print("=" * 52 + "\nSET ENCODER TO ZERO\n" + "=" * 52)
                        print(f"Current Mechanism Position: "
                              f"{self.get_mechanism_position() * 360.0}° "
                              f"Current Velocity: {self.get_rotor_velocity()}")
                        self.set_encoder_position_angular(0.0)

                    SmartDashboard.putData(
                        telemetry_path_str + "/ZeroEncoder", _zero_encoder)

                    SmartMotorControllerCommandRegistry.addCommand(
                        "Live Tuning",
                        self.config.getSubsystem(),
                        lambda: self.telemetry.applyTuningValues(self)
                    )

    def update_telemetry(self) -> None:
        if self.telemetry_table is not None and self.config.getVerbosity() is not None:
            self.telemetry.publish(self)
        elif self.config.getVerbosity() is not None:
            self.setup_telemetry()

    # Setpoint accessors

    def get_mechanism_position_setpoint(self) -> Optional[float]:
        """Mechanism setpoint in rotations."""
        return self.setpoint_position

    def get_mechanism_setpoint_velocity(self) -> Optional[float]:
        """Mechanism velocity setpoint in rotations/second."""
        return self.setpoint_velocity

    # Name

    def get_name(self) -> str:
        return self.config.getTelemetryName() or "SmartMotorController"

    # Cleanup

    def close(self) -> None:
        if self.closed_loop_controller_thread is not None:
            self.closed_loop_controller_thread.stop()
            self.closed_loop_controller_thread.close()
            self.closed_loop_controller_thread = None
        self.telemetry.close()

    def __str__(self) -> str:
        return self.get_name()

    def __repr__(self) -> str:
        return f"SmartMotorController(name={self.get_name()!r})"

