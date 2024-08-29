from .eboard import EBoard


class PushModel:
    """
    This class models the push behavior of a rider on an eboard. It computes the push acceleration
    and the initial slowdown acceleration based on the provided force and push duration.
    It is assumed that the two acceleration values are applied continuously for the entire separate durations.

    The class provides a method to step through the
    entire duration of the push (which includes slowdown section) and to compute the
    acceleration and delta velocity at each step.

    Attributes:
        __eboard (EBoard): The eboard object associated with this push model.
        __rider_accel_ms2 (float): The rider acceleration in m/s^2.
        __push_duration_s (float): The duration of the push in seconds.
        __initial_slowdown_duration_s (float): The initial slowdown duration in seconds.
        __rider_slowdown_accel_ms2 (float): The rider slowdown acceleration in m/s^2.
        __elapsed_time_s (float): The elapsed time since the beginning of the push.
        __push_active (bool): A flag indicating whether the push is still active or not.
    """

    SLOWDOWN_DURATION_FACTOR = 0.10

    def __init__(self, eboard: EBoard) -> None:
        self.__eboard = eboard
        self.__push_active = False

    def setup(self, force_rider_N: float, push_duration_ms: int) -> None:
        """
        Computes the rider acceleration, the slowdown acceleration, and the initial
        slowdown duration based on the provided force and push duration.

        Args:
            force_rider_N (float): The force applied by the rider in Newtons.
            push_duration_ms (int): The duration of the push in milliseconds.
        Returns:
            None
        """
        self.__rider_accel_ms2 = force_rider_N / self.__eboard.total_weight_with_rider_kg
        self.__push_duration_s = push_duration_ms / 1000
        self.__initial_slowdown_duration_s = PushModel.SLOWDOWN_DURATION_FACTOR * self.__push_duration_s
        self.__rider_slowdown_accel_ms2 = -(PushModel.SLOWDOWN_DURATION_FACTOR * self.__rider_accel_ms2)
        self.__elapsed_time_s = 0
        self.__push_active = True

    @property
    def push_active(self) -> bool:
        """
        Returns:
            bool: True if the push is still active, False otherwise
        """
        return self.__push_active

    @property
    def slowdown_duration_ms(self) -> int:
        """
        Returns:
            int: The duration of the slowdown phase in milliseconds
        """
        return int(self.__initial_slowdown_duration_s * 1000)

    def step(self, time_step_ms: int) -> tuple[float, float]:
        """
        To emulate reality, the rider's acceleration is not all applied at once for the
        entire push. Instead, the acceleration is increased over time linearly such that it
        would be equivalent to the rider's acceleration being constant throughout the push duration.

        For the initial slowdown phase, the acceleration is decreased linearly following the same
        approach as above.

        Args:
            time_step_ms (int): The time step in milliseconds
        Returns:
            tuple: A tuple containing the acceleration in m/s^2 and the delta velocity in m/s
        """
        t_sec = time_step_ms / 1000.0
        acceleration_ms2, delta_velocity_mps = None, None
        if self.__elapsed_time_s <= self.__initial_slowdown_duration_s:
            acceleration_ms2 = 2 * (
                self.__rider_slowdown_accel_ms2 * (1.0 - (self.__elapsed_time_s / self.__initial_slowdown_duration_s))
            )
            delta_velocity_mps = acceleration_ms2 * t_sec
        else:
            fraction = (self.__elapsed_time_s - self.__initial_slowdown_duration_s) / self.__push_duration_s
            acceleration_ms2 = 2 * (self.__rider_accel_ms2 * fraction)
            delta_velocity_mps = acceleration_ms2 * t_sec
        self.__elapsed_time_s += t_sec
        if self.__elapsed_time_s > (self.__push_duration_s + self.__initial_slowdown_duration_s):
            self.__push_active = False
        return acceleration_ms2, delta_velocity_mps
