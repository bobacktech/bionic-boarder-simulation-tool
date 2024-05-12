from .eboard import EBoard
import numpy


class FrictionalDecelerationModel:
    """
    Frictional deceleration model
    """

    GRAVITY = 9.81  # m/s^2
    AIR_DENSITY = 1.225  # kg/m3

    def __init__(
        self,
        mu_rolling: float,
        c_drag: float,
        frontal_area_m2: float,
        eboard: EBoard,
    ) -> None:
        """
        Args:
            mu_rolling: coefficient of rolling friction
            c_drag: coefficient of drag
            frontal_area_m2: area of skateboarder facing the wind in square meters
            air_density: air density at sea level in kg/m3
        Return:
            None
        """
        self.mu_rolling = mu_rolling
        self.c_drag = c_drag
        self.frontal_area_m2 = frontal_area_m2
        self.eboard = eboard

    def decelerate(self, current_velocity_m_per_s: float, time_step_ms: float) -> float:
        """
        Args:
            current_velocity_m_per_s: current velocity of skateboarder in m/s
            time_step_ms: time step in milliseconds
        Return:
            velocity of skateboarder that is reduced by the friction and drag over the time_step_ms
        """
        force_friction = (
            self.mu_rolling * self.eboard.total_weight_with_rider_kg * self.GRAVITY
        )
        force_drag = (
            self.c_drag
            * self.AIR_DENSITY
            * (current_velocity_m_per_s**2)
            * self.frontal_area_m2
        )
        net_deceleration_m_per_s2 = (
            force_friction + force_drag
        ) / self.eboard.total_weight_with_rider_kg
        velocity_reduction_m_per_s = net_deceleration_m_per_s2 * (time_step_ms / 1000)
        return max(0, current_velocity_m_per_s - velocity_reduction_m_per_s)
