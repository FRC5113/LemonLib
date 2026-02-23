from __future__ import annotations

import math
from copy import deepcopy
from enum import Enum, auto
from typing import List, Optional, Tuple
from wpilib import DriverStation

# Simulation flag — set to True in your robot sim entry point
IS_SIMULATION: bool = False

# Stubs for YAMS modules

class SmartMotorControllerConfigurationException(Exception):
    def __init__(self, message: str, context: str, fix: str):
        super().__init__(f"{message} | Context: {context} | Fix: {fix}")


class MechanismGearing:
    """
    Stub gearing class.  May just get rid of since its not implemented.
    reduction_ratio > 1 means the mechanism spins slower than the rotor (e.g. 1.5:1 gearbox).
    """
    def __init__(self, reduction_ratio: float = 1.0):
        self._ratio = reduction_ratio

    def get_mechanism_to_rotor_ratio(self) -> float:
        return self._ratio

    def get_rotor_to_mechanism_ratio(self) -> float:
        return 1.0 / self._ratio

# SmartMotorControllerTelemetryConfig stub
class SmartMotorControllerTelemetryConfig:
    def with_telemetry_verbosity(self, v: "TelemetryVerbosity") -> "SmartMotorControllerTelemetryConfig":
        return self

# Enums

class TelemetryVerbosity(Enum):
    LOW = auto()
    MID = auto()
    HIGH = auto()


class MotorMode(Enum):
    BRAKE = auto()
    COAST = auto()


class ControlMode(Enum):
    OPEN_LOOP = auto()
    CLOSED_LOOP = auto()


class SmartMotorControllerOptions(Enum):
    MOTOR_INVERTED = auto()
    SUPPLY_CURRENT_LIMIT = auto()
    STATOR_CURRENT_LIMIT = auto()


class _BasicOptions(Enum):
    ControlMode = auto()
    FeedbackSynchronizationThreshold = auto()
    ClosedLoopControllerMaximumVoltage = auto()
    StartingPosition = auto()
    EncoderInverted = auto()
    MotorInverted = auto()
    TemperatureCutoff = auto()
    DiscontinuityPoint = auto()
    ClosedLoopTolerance = auto()
    UpperLimit = auto()
    LowerLimit = auto()
    IdleMode = auto()
    VoltageCompensation = auto()
    Followers = auto()
    LooselyCoupledFollowers = auto()
    StatorCurrentLimit = auto()
    SupplyCurrentLimit = auto()
    ClosedLoopRampRate = auto()
    OpenLoopRampRate = auto()
    ExternalEncoder = auto()
    Gearing = auto()
    ClosedLoopControlPeriod = auto()
    SimpleFeedforward = auto()
    ArmFeedforward = auto()
    ElevatorFeedforward = auto()
    PID = auto()
    TrapezoidalProfile = auto()
    ExponentialProfile = auto()


class _ExternalEncoderOptions(Enum):
    ZeroOffset = auto()
    UseExternalFeedbackEncoder = auto()
    ExternalGearing = auto()
    ExternalEncoderInverted = auto()

class SmartMotorControllerConfig:
    """
    Python port of yams.motorcontrollers.SmartMotorControllerConfig.

    All unit conventions (plain floats):
      - Angles / positions : rotations
      - Angular velocities : r/s
      - Angular accel      : r/s^2
      - Distances          : m
      - Linear velocities  : m/s
      - Linear accel       : m/s^2
      - Jerk               : m/s^3
      - Voltage            : V
      - Current            : amps
      - Temperature        : C
      - Time               : s
      - Mass               : kg
      - MOI                : kg*m^2
    """

    def __init__(self, subsystem=None):
        # HAL.report stub — no-op in Python
        self._subsystem = subsystem

        # Validation sets
        self._basic_options: set = set(_BasicOptions)
        self._external_encoder_options: set = set(_ExternalEncoderOptions)

        # Vendor config
        self._vendor_config: Optional[object] = None

        # Encoder / motor inversion
        self._encoder_inverted: bool = False
        self._motor_inverted: bool = False
        self._external_encoder_inverted: bool = False
        self._use_external_encoder: bool = True

        # Followers
        self._followers: Optional[List[Tuple[object, bool]]] = None  # (motor_obj, inverted)
        self._loosely_coupled_followers: Optional[List] = None  # List[SmartMotorController]

        # Feedforward (real + sim variants)
        self._simple_feedforward: Optional[SimpleMotorFeedforward] = None
        self._elevator_feedforward: Optional[ElevatorFeedforward] = None
        self._arm_feedforward: Optional[ArmFeedforward] = None
        self._sim_simple_feedforward: Optional[SimpleMotorFeedforward] = None
        self._sim_elevator_feedforward: Optional[ElevatorFeedforward] = None
        self._sim_arm_feedforward: Optional[ArmFeedforward] = None

        # Motion profiles (real + sim variants)
        # Stored as TrapezoidProfile.Constraints or ExponentialProfile.Constraints objects
        self._exponential_profile: Optional[object] = None
        self._trapezoid_profile: Optional[object] = None
        self._sim_exponential_profile: Optional[object] = None
        self._sim_trapezoid_profile: Optional[object] = None

        # Controllers (real + sim)
        self._pid: Optional[PIDController] = None
        # self._lqr: Optional[LQRController] = None
        self._sim_pid: Optional[PIDController] = None
        # self._sim_lqr: Optional[LQRController] = None

        # Gearing
        self._gearing: Optional[MechanismGearing] = None
        self._external_encoder_gearing: MechanismGearing = MechanismGearing(1.0)

        # Mechanism circumference (meters), None if not set
        self._mechanism_circumference: Optional[float] = None

        # Timing
        self._control_period: Optional[float] = None   # seconds
        self._open_loop_ramp_rate: float = 0.0          # seconds
        self._close_loop_ramp_rate: float = 0.0         # seconds

        # Current limits (amps as int, or None)
        self._stator_stall_current_limit: Optional[int] = None
        self._supply_stall_current_limit: Optional[int] = None

        # Voltage
        self._voltage_compensation: Optional[float] = None  # volts

        # Mode
        self._idle_mode: Optional[MotorMode] = None
        self._motor_controller_mode: ControlMode = ControlMode.CLOSED_LOOP

        # Soft limits (rotations)
        self._mechanism_lower_limit: Optional[float] = None
        self._mechanism_upper_limit: Optional[float] = None

        # Telemetry
        self._telemetry_name: Optional[str] = None
        self._verbosity: Optional[TelemetryVerbosity] = None
        self._specified_telemetry_config: Optional[SmartMotorControllerTelemetryConfig] = None

        # Zero offset, temperature cutoff, starting pos (all in rotations / celsius)
        self._zero_offset: Optional[float] = None             # rotations
        self._temperature_cutoff: Optional[float] = None      # celsius
        self._starting_position: Optional[float] = None       # rotations

        # Closed loop extras
        self._closed_loop_max_voltage: Optional[float] = None  # volts
        self._feedback_sync_threshold: Optional[float] = None  # rotations
        self._max_discontinuity_point: Optional[float] = None  # rotations
        self._min_discontinuity_point: Optional[float] = None  # rotations
        self._closed_loop_tolerance: Optional[float] = None    # rotations

        # Moment of inertia (kg·m²)
        self._moi: float = 0.02

        # Flags
        self._linear_closed_loop_controller: bool = False
        self._velocity_trapezoidal_profile: bool = False

        # External encoder
        self._external_encoder: Optional[object] = None

    # Copy / clone

    def clone(self) -> "SmartMotorControllerConfig":
        """Return a deep copy of this config."""
        return deepcopy(self)

    # Vendor config

    def with_vendor_config(self, vendor_config: object) -> "SmartMotorControllerConfig":
        """
        Set vendor-specific config as a base. SmartMotorControllerConfig options
        always take precedence and will overwrite the vendor config.
        """
        self._vendor_config = vendor_config
        return self

    def get_vendor_config(self) -> Optional[object]:
        return self._vendor_config

    # Subsystem

    def with_subsystem(self, subsystem) -> "SmartMotorControllerConfig":
        if self._subsystem is not None:
            raise SmartMotorControllerConfigurationException(
                "Subsystem has already been set",
                "Cannot set subsystem",
                "with_subsystem() should only be called once"
            )
        self._subsystem = subsystem
        return self

    def get_subsystem(self):
        if self._subsystem is None:
            raise SmartMotorControllerConfigurationException(
                "Subsystem is undefined",
                "Subsystem cannot be created.",
                "with_subsystem(subsystem)"
            )
        return self._subsystem

    # Motor / encoder inversion

    def with_motor_inverted(self, inverted: bool) -> "SmartMotorControllerConfig":
        self._motor_inverted = inverted
        return self

    def get_motor_inverted(self) -> bool:
        self._basic_options.discard(_BasicOptions.MotorInverted)
        if IS_SIMULATION:
            return False
        return self._motor_inverted

    def with_encoder_inverted(self, inverted: bool) -> "SmartMotorControllerConfig":
        self._encoder_inverted = inverted
        return self

    def get_encoder_inverted(self) -> bool:
        self._basic_options.discard(_BasicOptions.EncoderInverted)
        return self._encoder_inverted

    def with_external_encoder_inverted(self, inverted: bool) -> "SmartMotorControllerConfig":
        self._external_encoder_inverted = inverted
        return self

    def get_external_encoder_inverted(self) -> bool:
        self._external_encoder_options.discard(_ExternalEncoderOptions.ExternalEncoderInverted)
        if IS_SIMULATION:
            return False
        return self._external_encoder_inverted

    def with_use_external_feedback_encoder(self, use: bool) -> "SmartMotorControllerConfig":
        self._use_external_encoder = use
        return self

    def get_use_external_feedback(self) -> bool:
        self._external_encoder_options.discard(_ExternalEncoderOptions.UseExternalFeedbackEncoder)
        return self._use_external_encoder

    # Control mode

    def with_control_mode(self, mode: ControlMode) -> "SmartMotorControllerConfig":
        self._motor_controller_mode = mode
        return self

    def get_motor_controller_mode(self) -> ControlMode:
        self._basic_options.discard(_BasicOptions.ControlMode)
        return self._motor_controller_mode

    # Followers

    def with_followers(self, *followers: Tuple[object, bool]) -> "SmartMotorControllerConfig":
        """
        followers: variable number of (motor_object, inverted) pairs.
        Motor objects must be the raw vendor type (not SmartMotorController).
        """
        self._followers = list(followers)
        return self

    def get_followers(self) -> Optional[List[Tuple[object, bool]]]:
        self._basic_options.discard(_BasicOptions.Followers)
        return self._followers

    def clear_followers(self) -> None:
        self._followers = None

    def with_loosely_coupled_followers(self, *followers) -> "SmartMotorControllerConfig":
        """followers: SmartMotorController instances (only position/velocity requests forwarded)."""
        self._loosely_coupled_followers = list(followers)
        return self

    def get_loosely_coupled_followers(self) -> Optional[List]:
        self._basic_options.discard(_BasicOptions.LooselyCoupledFollowers)
        return self._loosely_coupled_followers

    # Current limits

    def with_stator_current_limit(self, current_amps: Optional[float]) -> "SmartMotorControllerConfig":
        self._stator_stall_current_limit = int(current_amps) if current_amps is not None else None
        return self

    def get_stator_stall_current_limit(self) -> Optional[int]:
        self._basic_options.discard(_BasicOptions.StatorCurrentLimit)
        return self._stator_stall_current_limit

    def with_supply_current_limit(self, current_amps: Optional[float]) -> "SmartMotorControllerConfig":
        self._supply_stall_current_limit = int(current_amps) if current_amps is not None else None
        return self

    def get_supply_stall_current_limit(self) -> Optional[int]:
        self._basic_options.discard(_BasicOptions.SupplyCurrentLimit)
        return self._supply_stall_current_limit

    # Voltage compensation

    def with_voltage_compensation(self, volts: Optional[float]) -> "SmartMotorControllerConfig":
        self._voltage_compensation = volts
        return self

    def get_voltage_compensation(self) -> Optional[float]:
        self._basic_options.discard(_BasicOptions.VoltageCompensation)
        return self._voltage_compensation

    # Idle mode

    def with_idle_mode(self, mode: Optional[MotorMode]) -> "SmartMotorControllerConfig":
        self._idle_mode = mode
        return self

    def get_idle_mode(self) -> Optional[MotorMode]:
        self._basic_options.discard(_BasicOptions.IdleMode)
        return self._idle_mode

    # Ramp rates

    def with_open_loop_ramp_rate(self, seconds: float) -> "SmartMotorControllerConfig":
        self._open_loop_ramp_rate = seconds
        return self

    def get_open_loop_ramp_rate(self) -> float:
        self._basic_options.discard(_BasicOptions.OpenLoopRampRate)
        return self._open_loop_ramp_rate

    def with_closed_loop_ramp_rate(self, seconds: float) -> "SmartMotorControllerConfig":
        self._close_loop_ramp_rate = seconds
        return self

    def get_closed_loop_ramp_rate(self) -> float:
        self._basic_options.discard(_BasicOptions.ClosedLoopRampRate)
        return self._close_loop_ramp_rate

    # Gearing

    def with_gearing(self, gear) -> "SmartMotorControllerConfig":
        """
        gear: MechanismGearing instance OR a float reduction ratio
        (e.g. 3.0 for a 3:1 gearbox).
        """
        if isinstance(gear, (int, float)):
            self._gearing = MechanismGearing(float(gear))
        else:
            self._gearing = gear
        return self

    def get_gearing(self) -> Optional[MechanismGearing]:
        self._basic_options.discard(_BasicOptions.Gearing)
        return self._gearing

    def with_external_encoder_gearing(self, gear) -> "SmartMotorControllerConfig":
        ratio = gear if isinstance(gear, (int, float)) else gear.get_rotor_to_mechanism_ratio()
        if ratio > 1:
            DriverStation.reportWarning(
                "[IMPORTANT] Your gearing is set in a way that the external encoder will exceed "
                "the maximum reading, this WILL result in multiple angle's being read as the same "
                "'angle.\n\tIgnore this warning IF your mechanism will never travel outside of the "
                "slice you are reading, adjust the offset accordingly.\n\tYou have been warned! "
                "(^.^) - Rivet"
            )
        self._external_encoder_gearing = (
            MechanismGearing(float(gear)) if isinstance(gear, (int, float)) else gear
        )
        return self

    def get_external_encoder_gearing(self) -> MechanismGearing:
        self._external_encoder_options.discard(_ExternalEncoderOptions.ExternalGearing)
        return self._external_encoder_gearing

    # Mechanism circumference / wheel helpers

    def with_mechanism_circumference(self, circumference_meters: Optional[float]) -> "SmartMotorControllerConfig":
        self._mechanism_circumference = circumference_meters
        return self

    def with_wheel_radius(self, radius_meters: float) -> "SmartMotorControllerConfig":
        self._mechanism_circumference = 2.0 * math.pi * radius_meters
        return self

    def with_wheel_diameter(self, diameter_meters: float) -> "SmartMotorControllerConfig":
        self._mechanism_circumference = math.pi * diameter_meters
        return self

    def get_mechanism_circumference(self) -> Optional[float]:
        return self._mechanism_circumference

    # Linear / distance based closed loop flag

    def with_linear_closed_loop_controller(self, enabled: bool) -> "SmartMotorControllerConfig":
        self._linear_closed_loop_controller = enabled
        return self

    def get_linear_closed_loop_controller_use(self) -> bool:
        return self._linear_closed_loop_controller and self._mechanism_circumference is not None

    def with_velocity_trapezoidal_profile(self, enabled: bool) -> "SmartMotorControllerConfig":
        self._velocity_trapezoidal_profile = enabled
        return self

    def get_velocity_trapezoidal_profile_in_use(self) -> bool:
        return self._velocity_trapezoidal_profile

    # Closed loop period

    def with_closed_loop_control_period(self, seconds: float) -> "SmartMotorControllerConfig":
        """Period in seconds (or pass a frequency in Hz — use 1/hz)."""
        self._control_period = seconds
        return self

    def get_closed_loop_control_period(self) -> Optional[float]:
        self._basic_options.discard(_BasicOptions.ClosedLoopControlPeriod)
        return self._control_period

    # Soft limits

    def with_soft_limit_angle(self, low_rotations: Optional[float],
                               high_rotations: Optional[float]) -> "SmartMotorControllerConfig":
        """Angle-based soft limits in rotations."""
        self._mechanism_lower_limit = low_rotations
        self._mechanism_upper_limit = high_rotations
        return self

    def with_soft_limit_distance(self, low_meters: Optional[float],
                                  high_meters: Optional[float]) -> "SmartMotorControllerConfig":
        """Distance-based soft limits in meters (requires mechanism circumference)."""
        if self._mechanism_circumference is None:
            raise SmartMotorControllerConfigurationException(
                "Mechanism circumference is undefined",
                "Cannot set soft limits.",
                "with_mechanism_circumference(meters)"
            )
        self._mechanism_lower_limit = (
            low_meters / self._mechanism_circumference if low_meters is not None else None
        )
        self._mechanism_upper_limit = (
            high_meters / self._mechanism_circumference if high_meters is not None else None
        )
        return self

    def get_mechanism_lower_limit(self) -> Optional[float]:
        """Returns lower soft limit in rotations."""
        self._basic_options.discard(_BasicOptions.LowerLimit)
        return self._mechanism_lower_limit

    def get_mechanism_upper_limit(self) -> Optional[float]:
        """Returns upper soft limit in rotations."""
        self._basic_options.discard(_BasicOptions.UpperLimit)
        return self._mechanism_upper_limit

    # Moment of inertia

    def with_moment_of_inertia(self, moi) -> "SmartMotorControllerConfig":
        """
        moi: float (kg·m²), or pass (length_meters, mass_kg) as a tuple to estimate.
        The double-arg form mirrors withMomentOfInertia(Distance, Mass).
        """
        if isinstance(moi, tuple):
            length_m, mass_kg = moi
            if length_m is None or mass_kg is None:
                raise SmartMotorControllerConfigurationException(
                    "Length or Weight cannot be null!",
                    "MOI is necessary for standalone SmartMotorController simulation!",
                    "with_moment_of_inertia((length_m, mass_kg))"
                )
            # SingleJointedArmSim.estimateMOI approximation: (1/3) * m * L²
            self._moi = (1.0 / 3.0) * mass_kg * (length_m ** 2)
        else:
            self._moi = float(moi)
        return self

    def get_moi(self) -> float:
        """Moment of inertia in kg·m²."""
        return self._moi

    # Continuous wrapping

    def with_continuous_wrapping(self, bottom_rotations: float,
                                  top_rotations: float) -> "SmartMotorControllerConfig":
        if self._mechanism_upper_limit is not None or self._mechanism_lower_limit is not None:
            raise SmartMotorControllerConfigurationException(
                "Soft limits set while configuring continuous wrapping",
                "Cannot set continuous wrapping",
                "with_soft_limit_angle() should be removed"
            )
        if self._linear_closed_loop_controller:
            raise SmartMotorControllerConfigurationException(
                "Distance based mechanism used with continuous wrapping",
                "Cannot set continuous wrapping",
                "with_mechanism_circumference() should be removed"
            )
        if self._pid is None:
            raise SmartMotorControllerConfigurationException(
                "No PID controller used",
                "Cannot set continuous wrapping!",
                "with_closed_loop_controller_pid(kP, kI, kD)"
            )
        self._pid.enable_continuous_input(bottom_rotations, top_rotations)
        self._max_discontinuity_point = top_rotations
        self._min_discontinuity_point = bottom_rotations
        return self

    def get_max_discontinuity_point(self) -> Optional[float]:
        self._check_discontinuity_bounds()
        self._basic_options.discard(_BasicOptions.DiscontinuityPoint)
        return self._max_discontinuity_point

    def get_min_discontinuity_point(self) -> Optional[float]:
        self._check_discontinuity_bounds()
        return self._min_discontinuity_point

    def _check_discontinuity_bounds(self) -> None:
        if (self._max_discontinuity_point is not None
                and self._min_discontinuity_point is not None):
            expected_min = self._max_discontinuity_point - 1.0
            if abs(self._min_discontinuity_point - expected_min) > 1e-9:
                raise SmartMotorControllerConfigurationException(
                    "Bounds are not correct!",
                    "Cannot get the discontinuity point.",
                    f"with_continuous_wrapping({expected_min:.4f}, "
                    f"{self._max_discontinuity_point:.4f}) instead"
                )

    # Closed loop tolerance

    def with_closed_loop_tolerance_angle(self, tolerance_rotations: Optional[float]) -> "SmartMotorControllerConfig":
        self._closed_loop_tolerance = tolerance_rotations
        if tolerance_rotations is not None:
            if self._pid is None:
                raise SmartMotorControllerConfigurationException(
                    "No PID controller used",
                    "Cannot set tolerance!",
                    "with_closed_loop_controller_pid(kP, kI, kD)"
                )
            self._pid.set_tolerance(tolerance_rotations)
        return self

    def with_closed_loop_tolerance_distance(self, tolerance_meters: Optional[float]) -> "SmartMotorControllerConfig":
        if not self._linear_closed_loop_controller:
            raise SmartMotorControllerConfigurationException(
                "Linear closed loop controller used with distance tolerance.",
                "Closed loop tolerance cannot be set.",
                "with_linear_closed_loop_controller(True)"
            )
        if tolerance_meters is not None:
            angle_tol = self.convert_to_mechanism_from_distance(tolerance_meters)
            self._closed_loop_tolerance = angle_tol
            if self._pid is None:
                raise SmartMotorControllerConfigurationException(
                    "No PID controller used",
                    "Cannot set tolerance!",
                    "with_closed_loop_controller_pid(kP, kI, kD)"
                )
            self._pid.set_tolerance(self.convert_from_mechanism_to_distance(
                self._closed_loop_tolerance))
        return self

    def get_closed_loop_tolerance(self) -> Optional[float]:
        """Closed loop tolerance in rotations."""
        self._basic_options.discard(_BasicOptions.ClosedLoopTolerance)
        return self._closed_loop_tolerance

    # Temperature cutoff

    def with_temperature_cutoff(self, cutoff_celsius: Optional[float]) -> "SmartMotorControllerConfig":
        self._temperature_cutoff = cutoff_celsius
        return self

    def get_temperature_cutoff(self) -> Optional[float]:
        self._basic_options.discard(_BasicOptions.TemperatureCutoff)
        return self._temperature_cutoff

    # Zero offset

    def with_external_encoder_zero_offset_angle(self, angle_rotations: Optional[float]) -> "SmartMotorControllerConfig":
        self._zero_offset = angle_rotations
        return self

    def with_external_encoder_zero_offset_distance(self, distance_meters: float) -> "SmartMotorControllerConfig":
        if self._mechanism_circumference is None:
            raise SmartMotorControllerConfigurationException(
                "Mechanism circumference is undefined",
                "Cannot set zero offset.",
                "with_mechanism_circumference(meters)"
            )
        self._zero_offset = self.convert_to_mechanism_from_distance(distance_meters)
        return self

    def get_zero_offset(self) -> Optional[float]:
        """Zero offset in rotations."""
        self._external_encoder_options.discard(_ExternalEncoderOptions.ZeroOffset)
        return self._zero_offset

    # Starting position

    def with_starting_position_angle(self, rotations: Optional[float]) -> "SmartMotorControllerConfig":
        self._starting_position = rotations
        return self

    def with_starting_position_distance(self, meters: float) -> "SmartMotorControllerConfig":
        self._starting_position = self.convert_to_mechanism_from_distance(meters)
        return self

    def get_starting_position(self) -> Optional[float]:
        """Starting position in rotations."""
        self._basic_options.discard(_BasicOptions.StartingPosition)
        return self._starting_position

    # Closed loop max voltage

    def with_closed_loop_controller_maximum_voltage(self, volts: Optional[float]) -> "SmartMotorControllerConfig":
        self._closed_loop_max_voltage = volts
        return self

    def get_closed_loop_controller_maximum_voltage(self) -> Optional[float]:
        self._basic_options.discard(_BasicOptions.ClosedLoopControllerMaximumVoltage)
        return self._closed_loop_max_voltage

    # Feedback sync threshold

    def with_feedback_synchronization_threshold(self, angle_rotations: Optional[float]) -> "SmartMotorControllerConfig":
        if self._mechanism_circumference is not None:
            raise SmartMotorControllerConfigurationException(
                "Auto-synchronization is unavailable when using distance based mechanisms",
                "Cannot set synchronization threshold.",
                "with_mechanism_circumference() should be removed."
            )
        self._feedback_sync_threshold = angle_rotations
        return self

    def get_feedback_synchronization_threshold(self) -> Optional[float]:
        self._basic_options.discard(_BasicOptions.FeedbackSynchronizationThreshold)
        return self._feedback_sync_threshold

    # Telemetry

    def with_telemetry(self, name_or_verbosity,
                       verbosity_or_config=None) -> "SmartMotorControllerConfig":
        """
        Overloaded forms:
          with_telemetry(verbosity)                   -> name = "motor"
          with_telemetry(name, verbosity)             -> named telemetry
          with_telemetry(name, telemetry_config)      -> named + custom config
        """
        if isinstance(name_or_verbosity, TelemetryVerbosity):
            # with_telemetry(verbosity)
            self._telemetry_name = "motor"
            self._verbosity = name_or_verbosity
        elif isinstance(verbosity_or_config, TelemetryVerbosity):
            # with_telemetry(name, verbosity)
            self._telemetry_name = name_or_verbosity
            self._verbosity = verbosity_or_config
        elif isinstance(verbosity_or_config, SmartMotorControllerTelemetryConfig):
            # with_telemetry(name, config)
            self._telemetry_name = name_or_verbosity
            self._verbosity = TelemetryVerbosity.HIGH
            self._specified_telemetry_config = verbosity_or_config
        else:
            raise ValueError(
                "with_telemetry requires (TelemetryVerbosity,) or (str, TelemetryVerbosity) "
                "or (str, SmartMotorControllerTelemetryConfig)"
            )
        return self

    def get_telemetry_name(self) -> Optional[str]:
        return self._telemetry_name

    def get_verbosity(self) -> Optional[TelemetryVerbosity]:
        return self._verbosity

    def get_smart_controller_telemetry_config(self) -> Optional[SmartMotorControllerTelemetryConfig]:
        return self._specified_telemetry_config

    # External encoder

    def with_external_encoder(self, encoder: Optional[object]) -> "SmartMotorControllerConfig":
        self._external_encoder = encoder
        return self

    def get_external_encoder(self) -> Optional[object]:
        self._basic_options.discard(_BasicOptions.ExternalEncoder)
        return self._external_encoder

    # Feedforward

    # --- Real feedforward ---
    def with_feedforward_arm(self, ff: Optional[ArmFeedforward]) -> "SmartMotorControllerConfig":
        if ff is None:
            self._arm_feedforward = None
        else:
            self._elevator_feedforward = None
            self._simple_feedforward = None
            self._arm_feedforward = ff
        return self

    def with_feedforward_elevator(self, ff: Optional[ElevatorFeedforward]) -> "SmartMotorControllerConfig":
        if ff is None:
            self._elevator_feedforward = None
        else:
            self._linear_closed_loop_controller = True
            self._arm_feedforward = None
            self._simple_feedforward = None
            self._elevator_feedforward = ff
        return self

    def with_feedforward_simple(self, ff: Optional[SimpleMotorFeedforward]) -> "SmartMotorControllerConfig":
        if ff is None:
            self._simple_feedforward = None
        else:
            self._arm_feedforward = None
            self._elevator_feedforward = None
            self._simple_feedforward = ff
        return self

    def get_arm_feedforward(self) -> Optional[ArmFeedforward]:
        self._basic_options.discard(_BasicOptions.ArmFeedforward)
        if IS_SIMULATION and self._sim_arm_feedforward is not None:
            return self._sim_arm_feedforward
        return self._arm_feedforward

    def get_elevator_feedforward(self) -> Optional[ElevatorFeedforward]:
        self._basic_options.discard(_BasicOptions.ElevatorFeedforward)
        if IS_SIMULATION and self._sim_elevator_feedforward is not None:
            return self._sim_elevator_feedforward
        return self._elevator_feedforward

    def get_simple_feedforward(self) -> Optional[SimpleMotorFeedforward]:
        self._basic_options.discard(_BasicOptions.SimpleFeedforward)
        if IS_SIMULATION and self._sim_simple_feedforward is not None:
            return self._sim_simple_feedforward
        return self._simple_feedforward

    # --- Sim feedforward ---
    def with_sim_feedforward_arm(self, ff: Optional[ArmFeedforward]) -> "SmartMotorControllerConfig":
        if ff is None:
            self._sim_arm_feedforward = None
        else:
            self._sim_elevator_feedforward = None
            self._sim_simple_feedforward = None
            self._sim_arm_feedforward = ff
        return self

    def with_sim_feedforward_elevator(self, ff: Optional[ElevatorFeedforward]) -> "SmartMotorControllerConfig":
        if ff is None:
            self._sim_elevator_feedforward = None
        else:
            self._linear_closed_loop_controller = True
            self._sim_arm_feedforward = None
            self._sim_simple_feedforward = None
            self._sim_elevator_feedforward = ff
        return self

    def with_sim_feedforward_simple(self, ff: Optional[SimpleMotorFeedforward]) -> "SmartMotorControllerConfig":
        if ff is None:
            self._sim_simple_feedforward = None
        else:
            self._sim_arm_feedforward = None
            self._sim_elevator_feedforward = None
            self._sim_simple_feedforward = ff
        return self

    # Motion profiles

    def with_profile_trap_constraints(self, constraints) -> "SmartMotorControllerConfig":
        """
        Set a TrapezoidProfile.Constraints directly.
        For linear controllers: max_vel in m/s, max_accel in m/s².
        For rotational: max_vel in rot/s, max_accel in rot/s².
        """
        DriverStation.reportWarning(
            "Trapezoidal profile will be given rotations/s and rotations/s^2 for rotational "
            "closed loop controllers."
        )
        DriverStation.reportWarning(
            "Trapezoidal profile will be given meters/s and meters/s^2 for linear closed loop "
            "controllers."
        )
        self._exponential_profile = None
        self._trapezoid_profile = constraints
        return self

    def with_trapezoidal_profile_linear(self, max_vel_ms: float,
                                         max_accel_ms2: float) -> "SmartMotorControllerConfig":
        self._linear_closed_loop_controller = True
        self._exponential_profile = None
        self._trapezoid_profile = TrapezoidProfile.Constraints(max_vel_ms, max_accel_ms2)
        return self

    def with_trapezoidal_profile_angular(self, max_vel_rps: float,
                                          max_accel_rps2: float) -> "SmartMotorControllerConfig":
        self._exponential_profile = None
        self._trapezoid_profile = TrapezoidProfile.Constraints(max_vel_rps, max_accel_rps2)
        return self

    def with_trapezoidal_profile_angular_velocity(self, max_accel_rps2: float,
                                                   max_jerk_rps3: float) -> "SmartMotorControllerConfig":
        """Velocity-mode trapezoidal profile (angular)."""
        self._exponential_profile = None
        self._trapezoid_profile = TrapezoidProfile.Constraints(max_accel_rps2, max_jerk_rps3)
        self._velocity_trapezoidal_profile = True
        return self

    def with_trapezoidal_profile_linear_velocity(self, max_accel_ms2: float,
                                                  max_jerk_ms3: float) -> "SmartMotorControllerConfig":
        """Velocity-mode trapezoidal profile (linear)."""
        self._exponential_profile = None
        self._trapezoid_profile = TrapezoidProfile.Constraints(max_accel_ms2, max_jerk_ms3)
        self._velocity_trapezoidal_profile = True
        self._linear_closed_loop_controller = True
        return self

    def with_exponential_profile_constraints(self, constraints) -> "SmartMotorControllerConfig":
        DriverStation.reportWarning(
            "Exponential profile will be given rotations/s and rotations/s^2 for rotational "
            "closed loop controllers."
        )
        DriverStation.reportWarning(
            "Exponential profile will be given meters/s and meters/s^2 for linear closed loop "
            "controllers."
        )
        self._exponential_profile = constraints
        self._trapezoid_profile = None
        return self

    def with_exponential_profile_arm(self, max_volts: float, motor: object,
                                      moi_kg_m2: float) -> "SmartMotorControllerConfig":
        """
        Build ExponentialProfile.Constraints from arm system identification.
        max_volts: volts; moi_kg_m2: kg·m²
        Requires gearing to be set first.
        """
        self._moi = moi_kg_m2
        # Approximate system matrices for a single-jointed arm
        # (Replicates LinearSystemId.createSingleJointedArmSystem logic)
        # A and B come from state-space: x_dot = A*x + B*u
        # For a flywheel/arm: A = -Kv/Ka, B = 1/Ka (in rad/s units)
        # We use rotations/s here to match the profile units.
        # Without the real WPILib LinearSystemId, use approximate kV/kA from motor params.
        import math as _math
        # Convert radians to rotations
        rads_to_rots = 1.0 / (2 * _math.pi)
        # Stub: derive kV and kA from motor if it exposes them, else use 1.0 defaults
        kV_rots = getattr(motor, 'KvRadPerSecPerVolt', 1.0) * rads_to_rots
        kA_rots = 1.0  # stub — replace with real system ID
        self._trapezoid_profile = None
        self._exponential_profile = ExponentialProfile.Constraints.from_characteristics(
            max_volts, kV_rots, kA_rots)
        return self

    def with_exponential_profile_elevator(self, max_volts: float, motor: object,
                                           mass_kg: float,
                                           drum_radius_m: float) -> "SmartMotorControllerConfig":
        """
        Build ExponentialProfile.Constraints for an elevator.
        Requires gearing.
        """
        # Stub approximation — replace with real LinearSystemId calculations
        kV_ms = 1.0
        kA_ms = 1.0
        self._trapezoid_profile = None
        self._exponential_profile = ExponentialProfile.Constraints.from_characteristics(
            max_volts, kV_ms, kA_ms)
        self._linear_closed_loop_controller = True
        return self

    def with_exponential_profile_angular(self, max_volts: float, max_vel_rps: float,
                                          max_accel_rps2: float) -> "SmartMotorControllerConfig":
        self._trapezoid_profile = None
        self._exponential_profile = ExponentialProfile.Constraints.from_state_space(
            max_volts,
            max_volts / max_vel_rps,
            max_volts / max_accel_rps2
        )
        return self

    def get_exponential_profile(self):
        """Returns ExponentialProfile.Constraints or None."""
        self._basic_options.discard(_BasicOptions.ExponentialProfile)
        if IS_SIMULATION and self._sim_exponential_profile is not None:
            return self._sim_exponential_profile
        return self._exponential_profile

    def get_trapezoid_profile(self):
        """Returns TrapezoidProfile.Constraints or None."""
        self._basic_options.discard(_BasicOptions.TrapezoidalProfile)
        if IS_SIMULATION and self._sim_trapezoid_profile is not None:
            return self._sim_trapezoid_profile
        return self._trapezoid_profile

    # Closed loop controllers

    # --- Real controller ---
    def with_closed_loop_controller_pid(self, kP: float, kI: float,
                                         kD: float) -> "SmartMotorControllerConfig":
        self._pid = PIDController(kP, kI, kD)
        # self._lqr = None
        return self

    def with_closed_loop_controller_pid_instance(self, controller: Optional[PIDController]) -> "SmartMotorControllerConfig":
        self._pid = controller
        # self._lqr = None
        return self

    def with_closed_loop_controller_lqr(self, controller: Optional[LQRController]) -> "SmartMotorControllerConfig":
        self._pid = None
        # self._lqr = controller
        return self

    def with_closed_loop_controller_linear(self, kP: float, kI: float, kD: float,
                                            max_vel_ms: float,
                                            max_accel_ms2: float) -> "SmartMotorControllerConfig":
        if self._mechanism_circumference is None:
            raise SmartMotorControllerConfigurationException(
                "Mechanism circumference is undefined",
                "Closed loop controller cannot be created.",
                "with_mechanism_circumference(meters)"
            )
        self._pid = PIDController(kP, kI, kD)
        # self._lqr = None
        return self.with_trapezoidal_profile_linear(max_vel_ms, max_accel_ms2)

    def with_closed_loop_controller_angular(self, kP: float, kI: float, kD: float,
                                             max_vel_rps: float,
                                             max_accel_rps2: float) -> "SmartMotorControllerConfig":
        self._pid = PIDController(kP, kI, kD)
        # self._lqr = None
        return self.with_trapezoidal_profile_angular(max_vel_rps, max_accel_rps2)

    def with_closed_loop_controller_angular_velocity(self, kP: float, kI: float, kD: float,
                                                      max_accel_rps2: float,
                                                      max_jerk_rps3: float) -> "SmartMotorControllerConfig":
        self._pid = PIDController(kP, kI, kD)
        # self._lqr = None
        return self.with_trapezoidal_profile_angular_velocity(max_accel_rps2, max_jerk_rps3)

    def with_closed_loop_controller_linear_velocity(self, kP: float, kI: float, kD: float,
                                                     max_accel_ms2: float,
                                                     max_jerk_ms3: float) -> "SmartMotorControllerConfig":
        self._pid = PIDController(kP, kI, kD)
        # self._lqr = None
        return self.with_trapezoidal_profile_linear_velocity(max_accel_ms2, max_jerk_ms3)

    def get_pid(self) -> Optional[PIDController]:
        self._basic_options.discard(_BasicOptions.PID)
        if IS_SIMULATION and self._sim_pid is not None:
            return self._sim_pid
        return self._pid

    # def get_lqr_closed_loop_controller(self) -> Optional[LQRController]:
    #     self._basic_options.discard(_BasicOptions.PID)
    #     if IS_SIMULATION and self._sim_lqr is not None:
    #         return self._sim_lqr
    #     return self._lqr

    # --- Sim controller ---
    def with_sim_closed_loop_controller_pid(self, kP: float, kI: float,
                                             kD: float) -> "SmartMotorControllerConfig":
        self._sim_pid = PIDController(kP, kI, kD)
        # self._sim_lqr = None
        return self

    def with_sim_closed_loop_controller_pid_instance(self, controller: Optional[PIDController]) -> "SmartMotorControllerConfig":
        self._sim_pid = controller
        # self._sim_lqr = None
        return self

    # def with_sim_closed_loop_controller_lqr(self, controller: Optional[LQRController]) -> "SmartMotorControllerConfig":
    #     self._sim_pid = None
    #     self._sim_lqr = controller
    #     return self

    def with_sim_closed_loop_controller_linear(self, kP: float, kI: float, kD: float,
                                                max_vel_ms: float,
                                                max_accel_ms2: float) -> "SmartMotorControllerConfig":
        self._linear_closed_loop_controller = True
        self._sim_pid = PIDController(kP, kI, kD)
        self._sim_exponential_profile = None
        self._sim_trapezoid_profile = TrapezoidProfile.Constraints(max_vel_ms, max_accel_ms2)
        # self._sim_lqr = None
        return self

    def with_sim_closed_loop_controller_angular(self, kP: float, kI: float, kD: float,
                                                 max_vel_rps: float,
                                                 max_accel_rps2: float) -> "SmartMotorControllerConfig":
        self._sim_pid = PIDController(kP, kI, kD)
        # self._sim_lqr = None
        self._sim_exponential_profile = None
        self._sim_trapezoid_profile = TrapezoidProfile.Constraints(max_vel_rps, max_accel_rps2)
        return self

    def with_sim_closed_loop_controller_angular_velocity(self, kP: float, kI: float, kD: float,
                                                          max_accel_rps2: float,
                                                          max_jerk_rps3: float) -> "SmartMotorControllerConfig":
        self._sim_pid = PIDController(kP, kI, kD)
        # self._sim_lqr = None
        self._sim_exponential_profile = None
        self._sim_trapezoid_profile = TrapezoidProfile.Constraints(max_accel_rps2, max_jerk_rps3)
        self._velocity_trapezoidal_profile = True
        return self

    def with_sim_closed_loop_controller_linear_velocity(self, kP: float, kI: float, kD: float,
                                                         max_accel_ms2: float,
                                                         max_jerk_ms3: float) -> "SmartMotorControllerConfig":
        self._sim_pid = PIDController(kP, kI, kD)
        # self._sim_lqr = None
        self._sim_exponential_profile = None
        self._sim_trapezoid_profile = TrapezoidProfile.Constraints(max_accel_ms2, max_jerk_ms3)
        self._velocity_trapezoidal_profile = True
        self._linear_closed_loop_controller = True
        return self

    # Unit conversion helpers
    # All convert between rotations <-> meters using mechanism circumference.

    def _require_circumference(self, context: str) -> None:
        if self._mechanism_circumference is None:
            raise SmartMotorControllerConfigurationException(
                "Mechanism circumference is undefined",
                context,
                "with_mechanism_circumference(meters)"
            )

    def convert_to_mechanism_from_distance(self, meters: float) -> float:
        """meters -> rotations"""
        self._require_circumference("Cannot convert Distance to Angle.")
        return meters / self._mechanism_circumference

    def convert_from_mechanism_to_distance(self, rotations: float) -> float:
        """rotations -> meters"""
        self._require_circumference("Cannot convert Angle to Distance.")
        return rotations * self._mechanism_circumference

    def convert_to_mechanism_from_linear_velocity(self, m_per_s: float) -> float:
        """m/s -> rot/s"""
        self._require_circumference("Cannot convert LinearVelocity to AngularVelocity.")
        return m_per_s / self._mechanism_circumference

    def convert_from_mechanism_to_linear_velocity(self, rot_per_s: float) -> float:
        """rot/s -> m/s"""
        self._require_circumference("Cannot convert AngularVelocity to LinearVelocity.")
        return rot_per_s * self._mechanism_circumference

    def convert_to_mechanism_from_linear_accel(self, m_per_s2: float) -> float:
        """m/s² -> rot/s²"""
        self._require_circumference("Cannot convert LinearAcceleration to AngularAcceleration.")
        return m_per_s2 / self._mechanism_circumference

    def convert_from_mechanism_to_linear_accel(self, rot_per_s2: float) -> float:
        """rot/s² -> m/s²"""
        self._require_circumference("Cannot convert AngularAcceleration to LinearAcceleration.")
        return rot_per_s2 * self._mechanism_circumference

    def convert_to_mechanism_from_jerk_linear(self, m_per_s3: float) -> float:
        """m/s³ -> rot/s³"""
        self._require_circumference("Cannot convert linear jerk to angular jerk.")
        return m_per_s3 / self._mechanism_circumference

    def convert_from_mechanism_to_jerk_linear(self, rot_per_s3: float) -> float:
        """rot/s³ -> m/s³"""
        self._require_circumference("Cannot convert angular jerk to linear jerk.")
        return rot_per_s3 * self._mechanism_circumference

    # Convenience alias matching the Java naming used in SmartMotorController
    def convert_from_mechanism(self, value: float) -> float:
        """
        Generic conversion: rotations -> meters (or rot/s -> m/s, etc.).
        The Java version is overloaded; Python callers should use the explicit helpers above
        for clarity, but this alias works for position/velocity conversion.
        """
        return self.convert_from_mechanism_to_distance(value)

    # Validation

    def reset_validation_check(self) -> None:
        self._basic_options = set(_BasicOptions)
        self._external_encoder_options = set(_ExternalEncoderOptions)

    def validate_basic_options(self) -> None:
        if self._basic_options:
            import sys
            print("========= Basic Option Validation FAILED ==========", file=sys.stderr)
            for opt in self._basic_options:
                print(f"Missing required option: {opt}", file=sys.stderr)
            raise SmartMotorControllerConfigurationException(
                "Basic options are not applied",
                "Cannot validate basic options.",
                "Ensure all get_*() methods are called in apply_config()"
            )

    def validate_external_encoder_options(self) -> None:
        if self._external_encoder_options:
            import sys
            print("========= External Encoder Option Validation FAILED ==========", file=sys.stderr)
            for opt in self._external_encoder_options:
                print(f"Missing required option: {opt}", file=sys.stderr)
            raise SmartMotorControllerConfigurationException(
                "External encoder options are not applied",
                "Cannot validate external encoder options.",
                "Ensure all get_*() methods are called in apply_config()"
            )

    # Debug

    def __repr__(self) -> str:
        name = self._telemetry_name or "unnamed"
        mode = self._motor_controller_mode.name
        linear = self.get_linear_closed_loop_controller_use()
        return (f"SmartMotorControllerConfig(name={name!r}, mode={mode}, "
                f"linear={linear}, circ={self._mechanism_circumference}m)")