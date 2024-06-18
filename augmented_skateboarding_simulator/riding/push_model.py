from .eboard import EBoard


class PushModel:
    """TBD"""

    SLOWDOWN_DURATION_FACTOR = 0.10

    def __init__(self, eboard: EBoard, time_step_ms: int) -> None:
        self.__eboard = eboard
        self.__time_step_s = time_step_ms / 1000

    def setup(self, force_rider_N: float, push_duration_ms: int) -> None:
        """TBD"""
        self.__rider_accel_ms2 = (
            force_rider_N / self.__eboard.total_weight_with_rider_kg
        )
        self.__push_duration_s = push_duration_ms / 1000
        self.__initial_slowdown_duration_s = (
            PushModel.SLOWDOWN_DURATION_FACTOR * self.__push_duration_s
        )
        self.__rider_slowdown_accel_ms2 = -(
            PushModel.SLOWDOWN_DURATION_FACTOR * self.__rider_accel_ms2
        )
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

    def step(self) -> tuple[float, float]:
        """TBD"""
        acceleration_ms2, delta_velocity_mps = None, None
        if self.__elapsed_time_s <= self.__initial_slowdown_duration_s:
            acceleration_ms2 = 2 * (
                self.__rider_slowdown_accel_ms2
                * (1.0 - (self.__elapsed_time_s / self.__initial_slowdown_duration_s))
            )
            delta_velocity_mps = acceleration_ms2 * self.__time_step_s
        else:
            fraction = (
                self.__elapsed_time_s - self.__initial_slowdown_duration_s
            ) / self.__push_duration_s
            acceleration_ms2 = 2 * (self.__rider_accel_ms2 * fraction)
            delta_velocity_mps = acceleration_ms2 * self.__time_step_s
        self.__elapsed_time_s += self.__time_step_s
        if self.__elapsed_time_s > (
            self.__push_duration_s + self.__initial_slowdown_duration_s
        ):
            self.__push_active = False
        return acceleration_ms2, delta_velocity_mps
