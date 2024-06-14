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

    PUSH_TIME_S = 0.45
    """
    Typical push time in seconds for a skateboard rider to accelerate the skateboard with his foot/paddle.
    """

    INITIAL_SLOWDOWN_TIME_S = 0.05
    """
    The time in seconds that the skateboard is slowed down when the rider makes initial contact with 
    the ground with his foot/paddle.  
    """

    def __init__(self, eboard: EBoard):
        self.eboard = eboard
        self.__elapsed_time_s: int = 0
        self.__push_active = False
        self.__F_rider_x_axis_N: float = 0
        self.__F_slowdown_x_axis_N: float = 0

    def setup(self, velocity_increase_mps: float, slope_angle_deg: float):
        """
        F_rider = F_gravity + F_net

        F_rider is the total force required by the rider to increase the skateboard's velocity to
        velocity_increase_mps over the time interval of PUSH_TIME_S.
        F_gravity is the force due to gravity along the x-axis of the skateboard.
        F_net is the force required to increase the skateboard's velocity to velocity_increase_mps over
        the time interval of PUSH_TIME_S.

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
        F_net_x_axis_N = self.eboard.total_weight_with_rider_kg * (
            velocity_increase_mps / self.PUSH_TIME_S
        )
        self.__F_rider_x_axis_N = F_net_x_axis_N + F_gravity_x_axis_N
        """
        When the foot/paddle makes contact with the ground initially the board will
        slowdown due to a negative force.  That force is simply a small fraction of the 
        total force required by the rider to move the board to velocity_increase_mps.
        """
        self.__F_slowdown_x_axis_N = -(0.10 * self.__F_rider_x_axis_N)

    @property
    def force_rider_N(self) -> float:
        """
        Returns the the force required by the rider to increase the skateboard's velocity
        to velocity_increase_mps over the time interval of PUSH_TIME_S.
        """
        return self.__F_rider_x_axis_N

    @property
    def force_slowdown_N(self) -> float:
        """
        Returns the force that causes a small slowdown in the skateboard's velocity when
        a push is started by the rider.
        """
        return self.__F_slowdown_x_axis_N

    @property
    def push_active(self) -> bool:
        """
        Returns True if the push is active, False otherwise.
        """
        return self.__push_active

    def step(self, time_step_s: int) -> tuple[float, float]:
        """
        Computes the acceleration and velocity deltas, along the x-axis, for the skateboard over
        the time_step_s.

        During each time step, a proportion of the rider force is imparted such
        that over the entire push duration, the total force applied is equivalent to the rider force
        being applied at every time step over the entire push duration.  This is done to emulate how
        a the skateboard is accelerated when the rider pushes the skateboard with a foot/paddle.
        Similarly, the same logic is used in the slowdown interval.

        Args:
            time_step_s: the time step in seconds used for computing the acceleration and velocity
        """
        acceleration_mps2 = None
        velocity_delta_mps = None
        if self.__elapsed_time_s <= self.INITIAL_SLOWDOWN_TIME_S:
            force_applied_N = 2 * (
                self.__F_slowdown_x_axis_N
                - self.__F_slowdown_x_axis_N
                * (self.__elapsed_time_s / self.INITIAL_SLOWDOWN_TIME_S)
            )
            acceleration_mps2 = force_applied_N / self.eboard.total_weight_with_rider_kg
            velocity_delta_mps = acceleration_mps2 * time_step_s
        else:
            fraction = (
                self.__elapsed_time_s - self.INITIAL_SLOWDOWN_TIME_S
            ) / self.PUSH_TIME_S
            force_applied_N = 2 * (fraction * self.__F_rider_x_axis_N)
            acceleration_mps2 = force_applied_N / self.eboard.total_weight_with_rider_kg
            velocity_delta_mps = acceleration_mps2 * time_step_s
        self.__elapsed_time_s += time_step_s
        if self.__elapsed_time_s > (self.PUSH_TIME_S + self.INITIAL_SLOWDOWN_TIME_S):
            self.__push_active = False
        return acceleration_mps2, velocity_delta_mps
