from bionic_boarder_simulation_tool.riding.frictional_deceleration_model import FrictionalDecelerationModel
from .eboard_kinematic_state import EboardKinematicState
from threading import Lock, BoundedSemaphore, Thread, Event
from .eboard import EBoard
import math
import time
from bionic_boarder_simulation_tool.logger import Logger


class MotorController:

    def __init__(self, eb: EBoard, eks: EboardKinematicState, eks_lock: Lock, fdm: FrictionalDecelerationModel) -> None:
        self.__eks = eks
        self.__eks_lock = eks_lock
        self.__eb = eb
        self.__fdm = fdm

        # Specify motor efficiency. This is an estimate to be used for any motor setup.
        self.__motor_efficiency = 0.90
        # Specify controller efficiency. This is an estimate for the VESC controller
        self.__controller_efficiency = 0.97

        # Compute max acceleration the motor can move this particular eboard in ERPM/sec
        wheel_radius = eb.wheel_diameter_m / 2
        torque_at_wheel = eb.motor_max_torque * eb.gear_ratio
        force_at_wheel = torque_at_wheel / wheel_radius
        linear_acceleration = force_at_wheel / eb.total_weight_with_rider_kg
        angular_acceleration_wheel_rad_per_sec2 = linear_acceleration / wheel_radius
        angular_acceleration_wheel_rpm_sec = angular_acceleration_wheel_rad_per_sec2 * (60 / (2 * math.pi))
        motor_acceleration_rpm_sec = angular_acceleration_wheel_rpm_sec * eb.gear_ratio
        self.__erpm_per_sec = motor_acceleration_rpm_sec * eb.motor_pole_pairs


        self.__target_current = 0.0
        self.__current_sem = BoundedSemaphore(1)
        self.__current_thread = Thread(target=self.__current_control)
        self.__current_thread.daemon = True
        self.__stop_event = Event()

        self.__control_time_step_sec = 0
        self.__zero_current_flag = False

    def start(self) -> None:
        # Decrement the semaphore counters by 1
        self.__current_sem.acquire()
        self.__current_thread.start()

    def stop(self) -> None:
        self.__stop_event.set()
        self.__current_sem.release()

    def __current_control(self) -> None:
        """
        At this time, the only purpose of this motor control scheme is to set the current to 0.0
        so that the motor will disengage out of ERPM control and just coast. In the future, this scheme will
        be replaced with a more sophisticated control scheme.
        """
        while not self.__stop_event.is_set():
            self.__current_sem.acquire()
            if self.__target_current == 0.0:
                self.__zero_current_flag = True
                Logger().logger.info("Current control has set motor current to 0")
            else:
                raise ValueError("Target Current must be set to 0.0")

    @property
    def control_time_step_ms(self) -> int:
        return int(self.__control_time_step_sec * 1000.0)

    @control_time_step_ms.setter
    def control_time_step_ms(self, value: int) -> None:
        self.__control_time_step_sec = value / 1000.0

    @property
    def erpm_per_sec(self) -> float:
        return self.__erpm_per_sec

    @property
    def target_current(self) -> float:
        return self.__target_current

    @target_current.setter
    def target_current(self, value: float) -> None:
        if value != 0.0:
            raise ValueError("Target Current must be set to 0.0")
        self.__target_current = value

    @property
    def current_sem(self) -> BoundedSemaphore:
        return self.__current_sem
