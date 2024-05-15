from .eboard import EBoard


class FrictionalDecelerationModel:
    """
    Frictional deceleration model

    Provides the calculation to determine the velocity reduction due to friction and drag to subtract
    from current velocity of the skateboard over a fixed time step.
    """

    GRAVITY = 9.81  # m/s^2
    AIR_DENSITY = 1.225  # kg/m3

    def __init__(
        self,
        mu_rolling: float,
        c_drag: float,
        eboard: EBoard,
    ) -> None:
        """
        Args:
            mu_rolling: coefficient of rolling friction
            c_drag: coefficient of drag
        Return:
            None
        """
        self.__mu_rolling = mu_rolling
        self.__c_drag = c_drag
        self.eboard = eboard
        self.__force_friction = (
            self.__mu_rolling * self.eboard.total_weight_with_rider_kg * self.GRAVITY
        )

    def decelerate(self, current_velocity_m_per_s: float, time_step_ms: float) -> float:
        """
        Args:
            current_velocity_m_per_s: current velocity of skateboarder in m/s
            time_step_ms: time step in milliseconds
        Return:
            velocity of skateboarder that is reduced by the friction and drag over the time_step_ms
        """
        force_drag = (
            self.__c_drag
            * self.AIR_DENSITY
            * (current_velocity_m_per_s**2)
            * self.eboard.frontal_area_of_rider_m2
        )
        velocity_reduction_m_per_s = (
            (self.__force_friction + force_drag)
            / self.eboard.total_weight_with_rider_kg
            * time_step_ms
            / 1000
        )
        return max(0, current_velocity_m_per_s - velocity_reduction_m_per_s)
