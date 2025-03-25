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
        self.__target_erpm = 0
        self.__erpm_sem = BoundedSemaphore(1)
        self.__erpm_thread = Thread(target=self.__erpm_control)
        self.__erpm_thread.daemon = True

        self.__target_current = 0.0
        self.__current_sem = BoundedSemaphore(1)
        self.__current_thread = Thread(target=self.__current_control)
        self.__current_thread.daemon = True
        self.__stop_event = Event()

        self.__control_time_step_sec = 0
        self.__zero_current_flag = False

    def start(self) -> None:
        # Decrement the semaphore counters by 1
        self.__erpm_sem.acquire()
        self.__current_sem.acquire()
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
                previous_velocity_m_per_s = self.__eks.velocity
            if starting_erpm == self.__target_erpm:
                continue
            Logger().logger.info(
                "Control loop activated to change motor's speed to target ERPM",
                starting_erpm=starting_erpm,
                target_erpm=self.__target_erpm,
            )
            erpm_step = round(self.__erpm_per_sec * self.__control_time_step_sec)
            if erpm_step == 0:
                erpm_step = 1
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
                    motor_angular_velocity_rad_per_sec = (mechanical_rpm * 2 * math.pi) / 60
                    wheel_radius_m = self.__eb.wheel_diameter_m / 2
                    wheel_speed_m_per_sec = (motor_angular_velocity_rad_per_sec / self.__eb.gear_ratio) * wheel_radius_m
                    frictional_acceleration_m_per_s2 = self.__fdm.decelerate(
                        wheel_speed_m_per_sec, self.__control_time_step_sec * 1000.0
                    )[0]
                    total_resistive_force_N = frictional_acceleration_m_per_s2 * self.__eb.total_weight_with_rider_kg
                    wheel_torque_Nm = total_resistive_force_N * wheel_radius_m
                    motor_torque_Nm = wheel_torque_Nm / self.__eb.gear_ratio
                    motor_kt = 60 / (2 * math.pi * self.__eb.motor_kv)
                    self.__eks.motor_current = motor_torque_Nm / motor_kt
                    mechanical_power = motor_torque_Nm * motor_angular_velocity_rad_per_sec
                    self.__eks.input_current = mechanical_power / (
                        self.__eb.battery_max_voltage * self.__motor_efficiency * self.__controller_efficiency
                    )
                    motor_acceleration_m_per_s2 = (
                        self.__eks.velocity - previous_velocity_m_per_s
                    ) / self.__control_time_step_sec
                    self.__eks.acceleration_x = motor_acceleration_m_per_s2 - frictional_acceleration_m_per_s2
                    previous_velocity_m_per_s = self.__eks.velocity
                last_erpm_value += erpm_step
                if self.__target_erpm != target_erpm:
                    target_erpm = self.__target_erpm
                    erpm_step = erpm_step if last_erpm_value < target_erpm else -erpm_step
                if self.__zero_current_flag:
                    with self.__eks_lock:
                        self.__eks.motor_current = 0.0
                        self.__eks.input_current = 0.0
                    break
            Logger().logger.info(
                "ERPM control loop deactivated", target_erpm=self.__target_erpm, last_computed_erpm=last_erpm_value
            )

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
            with self.__eks_lock:
                self.__eks.motor_current = self.__target_current

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
    def erpm_sem(self) -> BoundedSemaphore:
        return self.__erpm_sem

    @property
    def current_sem(self) -> BoundedSemaphore:
        return self.__current_sem
