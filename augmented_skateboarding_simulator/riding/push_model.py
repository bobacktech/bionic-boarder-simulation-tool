from .eboard import EBoard
from .eboard_kinematic_state import EboardKinematicState
from threading import Lock
import random


class PushModel:
    """
    This class is responsible for simulating the acceleration of the skateboard when the user
    pushes the skateboard with either his foot or a stick paddle. The acceleration achieved is a
    function of the force imparted by the user's foot or the stick paddle and the quickness of the
    pushing motion. The simulated acceleration is along the x-axis of the skateboard, which is the
    long axis of the skateboard. The assumption is that there is no significant acceleration in either
    the y-axis or z-axis.

    A typical push time, which is the time the foot/paddle makes contact with the ground, ranges from
    a minimum of 500ms to a maximum of 1500ms. The duration of each push is determined at random.

    The force applied by the user's foot or the stick paddle is not constant over the entire push duration
    and is not always in the positive x-axis direction. The force right when the foot/paddle makes contact
    with the ground is in the negative x-axis direction which cause a slight slowing down of the skateboard's
    velocity. Then within a very short time, the force turns positive and the skateboard's velocity is increasing.

    Use this breakdown for the acceleration applied:
    1. First 10% of the push duration, the force is in the negative x-axis direction.
    2. The remaining 90% of the push duration, the force is in the positive x-axis direction.
    This above breakdown is arbitrary and needs to be further analyzed.
    """

    def __init__(self, eboard: EBoard, eks: EboardKinematicState, eks_lock: Lock):
        self.eboard = eboard
        self.eks = eks
        self.eks_lock = eks_lock
        self.__duration_ms: int = 0
        self.__elapsed_time_ms: int = 0
        self.__push_active = False

    def activate(self, slope_angle_deg: float):
        self.__push_active = True
        self.__duration_ms = random.randint(500, 1500)
        self.__elapsed_time_ms = 0
        """Define the acceleration breakdown for this push"""

    def step(self, time_step_ms: int):
        """
        This increases the velocity of the skateboard by a small amount using the acceleration
        algorithm for accelerating the skateboard.
        """
        if not self.__push_active:
            return
        self.eks_lock.acquire()
        # TBD
        self.eks_lock.release()
        self.__elapsed_time_ms += time_step_ms
        if self.__elapsed_time_ms >= self.__duration_ms:
            self.__push_active = False
