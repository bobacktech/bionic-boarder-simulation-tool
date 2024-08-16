from .eboard_kinematic_state import EboardKinematicState
from threading import Lock, Semaphore, Thread, Event
from .eboard import EBoard
import math
import time


class MotorController:

    def __init__(self, eb: EBoard, eks: EboardKinematicState, eks_lock: Lock) -> None:
        self.__eks = eks
        self.__eks_lock = eks_lock
        self.__eb = eb

        # Compute max acceleration the motor can move this particular eboard in ERPM/sec
        wheel_radius = eb.wheel_diameter_m / 2
        torque_at_wheel = eb.motor_max_torque * eb.gear_ratio
        force_at_wheel = torque_at_wheel / wheel_radius
        linear_acceleration = force_at_wheel / eb.total_weight_with_rider_kg
        angular_acceleration_wheel_rad_per_sec2 = linear_acceleration / wheel_radius
        angular_acceleration_wheel_rpm_sec = angular_acceleration_wheel_rad_per_sec2 * (60 / (2 * math.pi))
        motor_acceleration_rpm_sec = angular_acceleration_wheel_rpm_sec * eb.gear_ratio
        self.__erpm_per_sec = motor_acceleration_rpm_sec * eb.motor_pole_pairs
        self.__target_erpm = 0
        self.__erpm_sem = Semaphore(0)
        self.__erpm_thread = Thread(target=self.__erpm_control)
        self.__erpm_thread.daemon = True

        self.__target_current = 0.0
        self.__current_sem = Semaphore(0)
        self.__current_thread = Thread(target=self.__current_control)
        self.__current_thread.daemon = True
        self.__stop_event = Event()

        self.__control_time_step_sec = 0
        self.__zero_current_flag = False

    def start(self) -> None:
        self.__erpm_thread.start()
        self.__current_thread.start()

    def stop(self) -> None:
        self.__stop_event.set()
        self.__erpm_sem.release()
        self.__current_sem.release()

    def __erpm_control(self) -> None:
        while not self.__stop_event.is_set():
            self.__zero_current_flag = False
            self.__erpm_sem.acquire()
            target_erpm = self.__target_erpm
            with self.__eks_lock:
                starting_erpm = self.__eks.erpm
            if starting_erpm == self.__target_erpm:
                continue
            erpm_step = self.__erpm_per_sec * self.__control_time_step_sec
            erpm_step = erpm_step if starting_erpm < target_erpm else -erpm_step
            last_erpm_value = starting_erpm
            while (erpm_step > 0) == (last_erpm_value < target_erpm) and not self.__stop_event.is_set():
                st = time.perf_counter()
                while (time.perf_counter() - st) < self.__control_time_step_sec:
                    pass
                with self.__eks_lock:
                    self.__eks.erpm += erpm_step
                    self.__eks.velocity = ((self.__eks.erpm / self.__eb.motor_pole_pairs) / self.__eb.gear_ratio) * (
                        (math.pi * self.__eb.wheel_diameter_m) / 60
                    )
                    mechanical_rpm = self.__eks.erpm / self.__eb.motor_pole_pairs
                    mechanical_rad_per_sec = (mechanical_rpm * 2 * math.pi) / 60
                    wheel_radius_m = self.__eb.wheel_diameter_m / 2
                    gravitational_force_N = self.__eb.total_weight_with_rider_kg * 9.81
                    torque_required_Nm = (gravitational_force_N * wheel_radius_m) / self.__eb.gear_ratio
                    power_required_watts = torque_required_Nm * mechanical_rad_per_sec
                    self.__eks.input_current = power_required_watts / self.__eb.battery_max_voltage
                    # Calculate linear acceleration in m/s^2
                    force_required_N = torque_required_Nm / wheel_radius_m
                    self.__eks.acceleration_x = force_required_N / self.__eb.total_weight_with_rider_kg
                last_erpm_value += erpm_step
                if self.__target_erpm != target_erpm:
                    target_erpm = self.__target_erpm
                    erpm_step = erpm_step if last_erpm_value < target_erpm else -erpm_step
                if self.__zero_current_flag:
                    with self.__eks_lock:
                        self.__eks.input_current = 0.0
                    break

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
            else:
                raise ValueError("Target Current must be set to 0.0")
            with self.__eks_lock:
                self.__eks.input_current = self.__target_current

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
    def target_erpm(self) -> int:
        return self.__target_erpm

    @target_erpm.setter
    def target_erpm(self, value: int) -> None:
        self.__target_erpm = value

    @property
    def target_current(self) -> float:
        return self.__target_current

    @target_current.setter
    def target_current(self, value: float) -> None:
        if value != 0.0:
            raise ValueError("Target Current must be set to 0.0")
        self.__target_current = value

    @property
    def erpm_sem(self) -> Semaphore:
        return self.__erpm_sem

    @property
    def current_sem(self) -> Semaphore:
        return self.__current_sem
