from .eboard_kinematic_state import EboardKinematicState
from threading import Lock, Semaphore, Thread, Event
from .eboard import EBoard
import math
import time


class MotorController:

    def __init__(self, eb: EBoard, eks: EboardKinematicState, eks_lock: Lock) -> None:
        self.__eks = eks
        self.__eks_lock = eks_lock

        # Compute max acceleration the motor can move this particular eboard in ERPM/sec
        torque_at_wheel = eb.motor_max_torque * eb.gear_ratio
        wheel_radius = eb.wheel_diameter_m / 2
        force_at_wheel = torque_at_wheel / wheel_radius
        linear_acceleration = force_at_wheel / eb.total_weight_with_rider_kg
        angular_acceleration_wheel_rad_per_sec2 = linear_acceleration / wheel_radius
        angular_acceleration_motor_rad_per_sec2 = angular_acceleration_wheel_rad_per_sec2 / eb.gear_ratio
        self.__erpm_per_sec = angular_acceleration_motor_rad_per_sec2 * (60 / (2 * math.pi)) * eb.motor_pole_pairs
        self.__erpm = 0
        self.__erpm_sem = Semaphore(0)
        self.__erpm_thread = Thread(target=self.__erpm_control)

        self.__current = 0.0
        self.__current_sem = Semaphore(0)
        self.__current_thread = Thread(target=self.__current_control)
        self.__stop_event = Event()

    def start(self) -> None:
        self.__erpm_thread.start()
        self.__current_thread.start()

    def stop(self) -> None:
        self.__stop_event.set()
        self.__erpm_sem.release()
        self.__current_sem.release()

    def __erpm_control(self) -> None:
        while not self.__stop_event.is_set():
            self.__erpm_sem.acquire()
            self.__eks_lock.acquire()
            self.__eks.erpm = self.__erpm
            delta_erpm = self.__erpm - self.__eks.erpm
            wait_time_sec = abs(delta_erpm) / self.__erpm_per_sec
            et = time.perf_counter() + wait_time_sec
            while time.perf_counter() < et:
                pass
            self.__eks.erpm = self.__erpm
            self.__eks_lock.release()

    def __current_control(self) -> None:
        while not self.__stop_event.is_set():
            self.__current_sem.acquire()
            self.__eks_lock.acquire()
            self.__eks.input_current = 0.0
            self.__eks_lock.release()

    @property
    def erpm_per_sec(self) -> float:
        return self.__erpm_per_sec

    @property
    def erpm(self) -> int:
        return self.__erpm

    @erpm.setter
    def erpm(self, value: int) -> None:
        self.__erpm = value

    @property
    def current(self) -> float:
        return self.__current

    @current.setter
    def current(self, value: float) -> None:
        self.__current = value

    @property
    def erpm_sem(self) -> Semaphore:
        return self.__erpm_sem

    @property
    def duty_sem(self) -> Semaphore:
        return self.__duty_sem

    @property
    def current_sem(self) -> Semaphore:
        return self.__current_sem
