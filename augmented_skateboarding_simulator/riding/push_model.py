from .eboard import EBoard
import math


class PushModel:
    """
    This class is responsible for simulating the acceleration of the skateboard when the user
    pushes the skateboard with either his foot or a stick paddle. The acceleration achieved is a
    function of the force imparted by the user's foot or the stick paddle and the quickness of the
    pushing motion. The simulated acceleration is along the x-axis of the skateboard, which is the
    long axis of the skateboard. The assumption is that there is no significant acceleration in either
    the y-axis or z-axis.

    The force applied by the user's foot or the stick paddle is not constant over the entire push duration
    and is not always in the positive x-axis direction. The force right when the foot/paddle makes contact
    with the ground is in the negative x-axis direction which cause a slight slowing down of the skateboard's
    velocity. Then within a very short time, the force turns positive and the skateboard's velocity is increasing.

    Use this breakdown for the acceleration applied:
    1. First 10% of the push duration, the force is in the negative x-axis direction.
    2. The remaining 90% of the push duration, the force is in the positive x-axis direction.
    This above breakdown is arbitrary and needs to be further analyzed.
    """

    GRAVITY_MPS2 = 9.81

    PUSH_TIME_S = 0.5
    """
    Typical push time for a skateboard rider to make contact with the ground to accelerate the skateboard
    """
    INITIAL_SLOWDOWN_TIME_S = 0.05
    """
    The time in seconds that the skateboard is slowed down when the force is in the negative x-axis direction.
    """

    def __init__(self, eboard: EBoard):
        self.eboard = eboard
        self.__elapsed_time_s: int = 0
        self.__push_active = False

    def setup(self, velocity_increase_mps: float, slope_angle_deg: float):
        """
        F_total = F_gravity + F_rider

        F_total is the total force applied to the skateboard to increase the skateboard's velocity
        to velocity_increase_kph over the time interval of PUSH_TIME_MS.
        F_gravity is the force due to gravity along the x-axis of the skateboard.
        F_rider is the force due to the rider pushing the skateboard.

        The force right when the foot/paddle makes contact with the ground is in the negative x-axis direction.
        This causes a slight slowing down of the skateboard's velocity. This negative x-axis force is modeled
        simply as a small percentage of the total force.

        Args:
            velocity_increase_mps: the desired velocity increase in kph for this current PUSH
            slope_angle_deg: the angle of the slope in degrees. Ranges between +90 and -90 degrees
        """
        self.__push_active = True
        self.__elapsed_time_s = 0
        F_gravity_x_axis_N = (
            self.eboard.total_weight_with_rider_kg
            * self.GRAVITY_MPS2
            * math.sin(math.radians(slope_angle_deg))
        )
        F_rider_x_axis_N = self.eboard.total_weight_with_rider_kg * (
            velocity_increase_mps / self.PUSH_TIME_S
        )
        self.__F_total_N = F_rider_x_axis_N + F_gravity_x_axis_N
        self.__F_slowdown_N = 0.10 * self.__F_total_N

    def step(self, time_step_s: int) -> float:
        """
        This method returns the velocity change in m/s when the force is applied for
        time_step_s duration.
        """
        if not self.__push_active:
            return None

        velocity_delta_mps = None
        if self.__elapsed_time_s < self.INITIAL_SLOWDOWN_TIME_S:
            neg_acceleration_mps2 = (
                self.__F_slowdown_N / self.eboard.total_weight_with_rider_kg
            )
            velocity_delta_mps = neg_acceleration_mps2 * time_step_s
        else:
            acceleration_mps2 = (
                self.__F_total_N / self.eboard.total_weight_with_rider_kg
            )
            velocity_delta_mps = acceleration_mps2 * time_step_s

        self.__elapsed_time_s += time_step_s
        if self.__elapsed_time_s >= self.PUSH_TIME_S:
            self.__push_active = False
        return velocity_delta_mps
