from wpilib import Preferences, SmartDashboard
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpiutil import Sendable, SendableBuilder
from wpimath.controller import (
    PIDController,
    ProfiledPIDController,
    ProfiledPIDControllerRadians,
    SimpleMotorFeedforwardMeters,
    ElevatorFeedforward,
    ArmFeedforward,
    LTVDifferentialDriveController,
    LTVUnicycleController,
)
from wpimath.units import meters, seconds
from wpimath.system import LinearSystem_2_2_2
from .controller import SmartController
from phoenix6.configs import Slot0Configs
from phoenix6 import signals

from .nettables import SmartNT


class SmartProfile(Sendable):
    """Used to store multiple gains and configuration values for several
    different types of controllers. This can optionally interface with
    NetworkTables so that the gains may be dynamically updated without
    needing to redeploy code. This class has several helper methods
    which can be used to create `SmartController` objects already supplied
    with the necessary gains. Please note that gains will NOT be
    dynamically updated in `SmartController` objects: therefore it is
    recommended to create a new `SmartController` object on enable.
    """

    def __init__(self, profile_key: str, gains: dict[str, float], tuning_enabled: bool):
        """Creates a SmartProfile.
        Recommended gain keys (for use with `SmartController`):
        kP: Proportional Gain
        kI: Integral Gain
        kD: Derivative Gain
        kS: Static Gain
        kV: Velocity Gain
        kG: Gravity Gain
        kA: Acceleration Gain
        kMinInput: Minimum expected measurement value (used for continuous input)
        kMaxInput: Maximum expected measurement value (used for continuous input)

        Q1, Q2, Q3, Q4, Q5: State weighting for LTV controllers
        R1, R2: Input weighting for LTV controllers

        :param str profile_key: Prefix for associated NetworkTables keys
        :param dict[str, float] gains: Dictionary containing gain_key: value pairs
        :param bool tuning_enabled: Specify whether or not to send and retrieve
            data from NetworkTables. If true, values from NetworkTables
            are given precedence over values set in code.
        """
        Sendable.__init__(self)
        self.profile_key = profile_key
        self.nt = SmartNT(f"SmartProfile/{profile_key}", True)
        self.tuning_enabled = tuning_enabled
        self.gains = gains
        if tuning_enabled:
            for gain in gains:
                Preferences.initDouble(f"{profile_key}_{gain}", gains[gain])
                self.gains[gain] = Preferences.getDouble(
                    f"{profile_key}_{gain}", gains[gain]
                )
            SmartDashboard.putData(f"SmartProfile/{profile_key}", self)

    def initSendable(self, builder: SendableBuilder):
        builder.setSmartDashboardType("SmartController")
        for gain_key in self.gains:
            builder.addDoubleProperty(
                gain_key,
                # optional arguments used to hackily avoid late binding
                (lambda key=gain_key: self.gains[key]),
                (lambda value, key=gain_key: self._set_gain(key, value)),
            )

    def _set_gain(self, key: str, value: float):
        self.gains[key] = value
        if self.tuning_enabled:
            Preferences.setDouble(f"{self.profile_key}_{key}", value)

    def _requires(requirements: set[str]):
        def inner(func):
            def wrapper(self, key, feedback_enabled=None):
                missing_reqs = requirements - set(self.gains.keys())
                assert (
                    len(missing_reqs) == 0
                ), f"Requires gains: {', '.join(missing_reqs)}"
                return func(self, key, feedback_enabled)

            return wrapper

        return inner

    @_requires({"kP", "kI", "kD"})
    def create_pid_controller(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a PID controller.
        Requires kP, kI, kD, [kMinInput, kMaxInput optional]
        """
        controller = PIDController(self.gains["kP"], self.gains["kI"], self.gains["kD"])
        if "kMinInput" in self.gains.keys() and "kMaxInput" in self.gains.keys():
            controller.enableContinuousInput(
                self.gains["kMinInput"], self.gains["kMaxInput"]
            )
        return SmartController(
            key,
            (lambda y, r: controller.calculate(y, r)),
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )

    def create_wpi_pid_controller(self) -> PIDController:
        """Creates a wpilib PIDController. Use `create_pid_controller()`
        instead if possible.
        Requires kP, kI, kD, [kMinInput, kMaxInput optional]
        """
        controller = PIDController(self.gains["kP"], self.gains["kI"], self.gains["kD"])
        if "kMinInput" in self.gains.keys() and "kMaxInput" in self.gains.keys():
            controller.enableContinuousInput(
                self.gains["kMinInput"], self.gains["kMaxInput"]
            )
        return controller

    def create_ctre_pid_controller(self) -> Slot0Configs:
        """Creates a CTRE PIDController. Use `create_pid_controller()`
        instead if possible.
        """
        controller = Slot0Configs()
        controller.with_k_p(self.gains["kP"])
        controller.with_k_i(self.gains["kI"])
        controller.with_k_d(self.gains["kD"])
        return controller

    def create_ctre_turret_controller(self) -> Slot0Configs:
        """Creates a CTRE PIDController. Use `create_pid_controller()`
        instead if possible.
        Requires kP, kI, kD,kS, kV,kA
        """
        controller = Slot0Configs()
        controller.with_k_p(self.gains["kP"])
        controller.with_k_i(self.gains["kI"])
        controller.with_k_d(self.gains["kD"])
        controller.with_k_s(self.gains["kS"])
        controller.with_k_v(self.gains["kV"])
        controller.with_k_a(self.gains["kA"])
        controller.with_static_feedforward_sign(
            signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
        )
        return controller

    def create_ctre_flywheel_controller(self) -> Slot0Configs:
        """Creates a CTRE PIDController. Use `create_pid_controller()`
        instead if possible.
        Requires kP, kI, kD,kS, kV,kA
        """
        controller = Slot0Configs()
        controller.with_k_p(self.gains["kP"])
        controller.with_k_i(self.gains["kI"])
        controller.with_k_d(self.gains["kD"])
        controller.with_k_s(self.gains["kS"])
        controller.with_k_v(self.gains["kV"])
        controller.with_k_a(self.gains["kA"])
        controller.with_static_feedforward_sign(
            signals.StaticFeedforwardSignValue.USE_VELOCITY_SIGN
        )
        return controller

    @_requires({"kP", "kI", "kD", "kMaxV", "kMaxA"})
    def create_profiled_pid_controller(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a profiled PID controller.
        Requires kP, kI, kD, kMaxV, kMaxA, [kMinInput, kMaxInput optional]
        """
        controller = ProfiledPIDController(
            self.gains["kP"],
            self.gains["kI"],
            self.gains["kD"],
            TrapezoidProfile.Constraints(self.gains["kMaxV"], self.gains["kMaxA"]),
        )
        if "kMinInput" in self.gains.keys() and "kMaxInput" in self.gains.keys():
            controller.enableContinuousInput(
                self.gains["kMinInput"], self.gains["kMaxInput"]
            )
        return SmartController(
            key,
            (lambda y, r: controller.calculate(y, r)),
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )

    def create_wpi_profiled_pid_controller_radians(
        self,
    ) -> ProfiledPIDControllerRadians:
        """Creates a wpilib ProfiledPIDControllerRadians.
        Requires kP, kI, kD, kMaxV, kMaxA, kMinInput, kMaxInput
        """
        controller = ProfiledPIDControllerRadians(
            self.gains["kP"],
            self.gains["kI"],
            self.gains["kD"],
            TrapezoidProfileRadians.Constraints(
                self.gains["kMaxV"], self.gains["kMaxA"]
            ),
        )
        controller.enableContinuousInput(
            self.gains["kMinInput"], self.gains["kMaxInput"]
        )
        return controller

    def create_ltv_unicycle_controller(
        self, plant: LinearSystem_2_2_2, trackwidth: meters, dt: seconds = 0.02
    ) -> LTVUnicycleController:
        """Creates a wpilib LTVUnicyvleController.
        Requires Qelems tuple(5 elements of SupportsFloat),
        Relems tuple(2 elements of SupportsFloat)
        """
        controller = LTVUnicycleController(
            dt,
            self.gains["kMaxV"],
        )
        return controller

    @_requires({"kS", "kV"})
    def create_simple_feedforward(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a simple DC motor feedforward controller.
        Requires kS, kV, [kA optional]
        """
        controller = SimpleMotorFeedforwardMeters(
            self.gains["kS"],
            self.gains["kV"],
            self.gains["kA"] if "kA" in self.gains else 0,
        )
        return SmartController(
            key,
            (lambda y, r: controller.calculate(r)),
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )

    @_requires({"kP", "kI", "kD", "kS", "kV"})
    def create_flywheel_controller(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a PID controller combined with a DC motor feedforward controller.
        Requires kP, kI, kD, kS, kV, [kA optional]
        """
        pid = PIDController(self.gains["kP"], self.gains["kI"], self.gains["kD"])
        if "kMinInput" in self.gains.keys() and "kMaxInput" in self.gains.keys():
            pid.enableContinuousInput(self.gains["kMinInput"], self.gains["kMaxInput"])
        feedforward = SimpleMotorFeedforwardMeters(
            self.gains["kS"],
            self.gains["kV"],
            self.gains["kA"] if "kA" in self.gains else 0,
        )
        return SmartController(
            key,
            (lambda y, r: pid.calculate(y, r) + feedforward.calculate(r)),
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )

    @_requires({"kP", "kI", "kD", "kS", "kV", "kMaxV", "kMaxA"})
    def create_turret_controller(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a profiled PID controller combined with a DC motor feedforward controller.
        Requires kP, kI, kD, kS, kV, [kA, kMinInput, kMaxInput optional]
        """
        pid = ProfiledPIDController(
            self.gains["kP"],
            self.gains["kI"],
            self.gains["kD"],
            TrapezoidProfile.Constraints(self.gains["kMaxV"], self.gains["kMaxA"]),
        )
        if "kMinInput" in self.gains.keys() and "kMaxInput" in self.gains.keys():
            pid.enableContinuousInput(self.gains["kMinInput"], self.gains["kMaxInput"])
        feedforward = SimpleMotorFeedforwardMeters(
            self.gains["kS"],
            self.gains["kV"],
            self.gains["kA"] if "kA" in self.gains else 0,
        )

        def calculate(y, r):
            pid_output = pid.calculate(y, r)
            setpoint = pid.getSetpoint()
            # add acceleration eventually
            feedforward_output = feedforward.calculate(setpoint.velocity)
            return pid_output + feedforward_output

        return SmartController(
            key,
            calculate,
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )

    @_requires({"kP", "kI", "kD", "kS", "kG", "kV", "kMaxV", "kMaxA"})
    def create_elevator_controller(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a profiled PID controller combined with an elevator feedforward controller.
        Requires kP, kI, kD, kS, kV, kG, [kA optional]
        """
        pid = ProfiledPIDController(
            self.gains["kP"],
            self.gains["kI"],
            self.gains["kD"],
            TrapezoidProfile.Constraints(self.gains["kMaxV"], self.gains["kMaxA"]),
        )
        feedforward = ElevatorFeedforward(
            self.gains["kS"],
            self.gains["kG"],
            self.gains["kV"],
            self.gains["kA"] if "kA" in self.gains else 0,
        )

        def calculate(y, r):
            pid_output = pid.calculate(y, r)
            setpoint = pid.getSetpoint()
            # add acceleration eventually
            feedforward_output = feedforward.calculate(setpoint.velocity)
            return pid_output + feedforward_output

        return SmartController(
            key,
            calculate,
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )

    @_requires({"kP", "kI", "kD", "kS", "kG", "kV", "kMaxV", "kMaxA"})
    def create_arm_controller(
        self, key: str, feedback_enabled: bool = None
    ) -> SmartController:
        """Creates a profiled PID controller combined with an arm feedforward controller.
        Requires kP, kI, kD, kS, kV, kG, [kA optional]
        """
        pid = ProfiledPIDController(
            self.gains["kP"],
            self.gains["kI"],
            self.gains["kD"],
            TrapezoidProfile.Constraints(self.gains["kMaxV"], self.gains["kMaxA"]),
        )
        feedforward = ArmFeedforward(
            self.gains["kS"],
            self.gains["kG"],
            self.gains["kV"],
            self.gains["kA"] if "kA" in self.gains else 0,
        )

        def calculate(
            y,
            r,
        ):
            pid_output = pid.calculate(y, r)
            setpoint = pid.getSetpoint()
            # add acceleration eventually
            feedforward_output = feedforward.calculate(
                setpoint.position, setpoint.velocity
            )
            return pid_output + feedforward_output

        return SmartController(
            key,
            calculate,
            self.tuning_enabled if feedback_enabled is None else feedback_enabled,
        )
