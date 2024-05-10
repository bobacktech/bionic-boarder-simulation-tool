from .eboard import EBoard
import numpy


class FrictionalDecelerationModel:
    """
    Frictional deceleration model
    """

    GRAVITY = 9.81  # m/s^2

    def __init__(
        self,
        mu_r: float,
        Cd: float,
        frontal_area_m2: float,
        air_density_kg_per_m3: float,
        eboard: EBoard,
    ) -> None:
        """
        Args:
            mu_r: coefficient of rolling friction
            Cd: coefficient of drag
            frontal_area_m2: area of skateboarder facing the wind in square meters
            air_density: air density at sea level in kg/m3
        Return:
            None
        """
        self.mu_r = mu_r
        self.Cd = Cd
        self.frontal_area_m2 = frontal_area_m2
        self.air_density_kg_per_m3 = air_density_kg_per_m3
        self.eboard = eboard

    def decelerate(self, current_velocity_m_per_s: float, time_step_ms: float) -> float:
        """
        Args:
            current_velocity_m_per_s: current velocity of skateboarder in m/s
            time_step_ms: time step in milliseconds
        Return:
            velocity of skateboarder in m/s
        """
