import math
from .eboard import EBoard
from .eboard_kinematic_state import EboardKinematicState
from .frictional_deceleration_model import FrictionalDecelerationModel
from .push_model import PushModel
import time
from threading import Lock
import random


class KinematicLoop:
    """
    This class implements the main loop for the kinematic model of the electric skateboard. It
    moves the skateboard through time via the provided push model and frictional deceleration model.
    """

    def __init__(
        self,
        eb: EBoard,
        eks: EboardKinematicState,
        eks_lock: Lock,
        fdm: FrictionalDecelerationModel,
        pm: PushModel,
    ) -> None:
        self.__fixed_time_step_ms = 0
        self.__eb = eb
        self.__eks = eks
        self.__eks_lock = eks_lock
        self.__fdm = fdm
        self.__pm = pm
        self.__push_period_sec = -1.0
        self.__theta_slope_period_sec = -1.0
        self.__initial_theta_slope_deg = 0.0
        self.__current_theta_slope_deg = 0.0
        self.__loop_active = False
        self.__slope_range_bound_deg = None

    @property
    def slope_range_bound_deg(self) -> float:
        return self.__slope_range_bound_deg

    @slope_range_bound_deg.setter
    def slope_range_bound_deg(self, value: float) -> None:
        self.__slope_range_bound_deg = value

    @property
    def fixed_time_step_ms(self) -> int:
        return self.__fixed_time_step_ms

    @fixed_time_step_ms.setter
    def fixed_time_step_ms(self, value: int) -> None:
        self.__fixed_time_step_ms = value

    @property
    def theta_slope_period_sec(self) -> float:
        return self.__theta_slope_period_sec

    @theta_slope_period_sec.setter
    def theta_slope_period_sec(self, value: float) -> None:
        self.__theta_slope_period_sec = value

    @property
    def push_period_sec(self) -> int:
        return self.__push_period_sec

    @push_period_sec.setter
    def push_period_sec(self, value: int) -> None:
        self.__push_period_sec = value

    @property
    def initial_theta_slope_deg(self) -> float:
        return self.__initial_theta_slope_deg

    @initial_theta_slope_deg.setter
    def initial_theta_slope_deg(self, value: float) -> None:
        self.__initial_theta_slope_deg = value

    @property
    def current_theta_slope_deg(self) -> float:
        return self.__current_theta_slope_deg

    def loop(self) -> None:
        self.__loop_active = True
        self.__current_theta_slope_deg = self.__initial_theta_slope_deg
        theta_slope_time_step_sec = 0
        push_period_time_step_sec = 0

        while True:
            if self.__eks.input_current > 0:
                """
                This means that the electric motor is controlling the skateboard because a current is 
                being injected into the motor. In this case, the skateboard's kinematics will not be
                adjusted due to frictional forces, gravity, and/or a user's push. Instead, just skip to
                next iteration of the loop after the fixed time step.
                """
                time.sleep(self.__fixed_time_step_ms / 1000.0)
                continue
            start_time = time.perf_counter()
            if theta_slope_time_step_sec >= self.__theta_slope_period_sec:
                if self.__current_theta_slope_deg == 0.0:
                    self.__current_theta_slope_deg = random.uniform(
                        -self.__slope_range_bound_deg,
                        self.__slope_range_bound_deg,
                    )
                else:
                    self.__current_theta_slope_deg = 0.0
                theta_slope_time_step_sec = 0
                with self.__eks_lock:
                    self.__eks.pitch = self.__current_theta_slope_deg
            theta_slope_time_step_sec += self.__fixed_time_step_ms / 1000.0
            if push_period_time_step_sec >= self.__push_period_sec:
                force_1g_N = self.__eb.total_weight_with_rider_kg * 9.81
                force_push_x_N = random.uniform(force_1g_N, 2 * force_1g_N)
                push_duration_ms = random.randint(400, 600)
                self.__pm.setup(force_push_x_N, push_duration_ms)
                push_period_time_step_sec = 0
            push_period_time_step_sec += self.__fixed_time_step_ms / 1000.0
            self.__eks_lock.acquire()
            accel_friction_ms2, delta_velocity_friction_m_per_s = self.__fdm.decelerate(
                self.__eks.velocity, self.fixed_time_step_ms
            )
            if self.__eks.velocity < 0.0:
                self.__eks.velocity = min(0, self.__eks.velocity + delta_velocity_friction_m_per_s)
                self.__eks.acceleration_x = accel_friction_ms2
            else:
                self.__eks.velocity = max(0, self.__eks.velocity - delta_velocity_friction_m_per_s)
                self.__eks.acceleration_x = -accel_friction_ms2
            accel_gravity_x_m_per_s2 = 9.81 * math.sin(math.radians(abs(self.__current_theta_slope_deg)))
            delta_velocity_gravity_x_m_per_s = accel_gravity_x_m_per_s2 * self.__fixed_time_step_ms / 1000.0
            if self.__current_theta_slope_deg >= 0.0:
                self.__eks.velocity -= delta_velocity_gravity_x_m_per_s
                self.__eks.acceleration_x -= accel_gravity_x_m_per_s2
            else:
                self.__eks.velocity += delta_velocity_gravity_x_m_per_s
                self.__eks.acceleration_x += accel_gravity_x_m_per_s2
            if self.__pm.push_active:
                accel_x_m_per_s2, delta_velocity_push_m_per_s = self.__pm.step(self.__fixed_time_step_ms)
                self.__eks.acceleration_x += accel_x_m_per_s2
                self.__eks.velocity += delta_velocity_push_m_per_s
            wheel_rpm = (self.__eks.velocity / (self.__eb.wheel_diameter_m * math.pi)) * 60
            motor_rpm = wheel_rpm * self.__eb.gear_ratio
            self.__eks.erpm = int(self.__eb.motor_pole_pairs * motor_rpm)
            self.__eks_lock.release()
            if not self.__loop_active:
                break
            elapsed_time = time.perf_counter() - start_time
            sleep_time = max(0, self.__fixed_time_step_ms / 1000.0 - elapsed_time)
            time.sleep(sleep_time)

    def stop(self) -> None:
        self.__loop_active = False
