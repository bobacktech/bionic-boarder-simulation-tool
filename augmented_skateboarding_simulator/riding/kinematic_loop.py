import math
from .eboard import EBoard
from .eboard_kinematic_state import EboardKinematicState
from .frictional_deceleration_model import FrictionalDecelerationModel
from .push_model import PushModel
import time
from threading import Lock
import random


class KinematicLoop:

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

    def loop(self) -> None:
        self.__loop_active = True
        theta_slope_deg = 0.0
        theta_slope_time_step_sec = 0
        push_period_time_step_sec = 0

        while True:
            start_time = time.perf_counter()
            if theta_slope_time_step_sec >= self.__theta_slope_period_sec:
                if theta_slope_deg == 0.0:
                    theta_slope_deg = random.uniform(
                        -self.__slope_range_bound_deg,
                        self.__slope_range_bound_deg,
                    )
                else:
                    theta_slope_deg = 0.0
                theta_slope_time_step_sec = 0
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
            self.__eks.velocity -= delta_velocity_friction_m_per_s
            self.__eks.acceleration_x = -accel_friction_ms2
            accel_gravity_x_m_per_s2 = 9.81 * math.sin(math.radians(abs(theta_slope_deg)))
            delta_velocity_gravity_x_m_per_s = accel_gravity_x_m_per_s2 * self.__fixed_time_step_ms / 1000.0
            if theta_slope_deg >= 0.0:
                self.__eks.velocity -= delta_velocity_gravity_x_m_per_s
                self.__eks.acceleration_x -= accel_gravity_x_m_per_s2
            else:
                self.__eks.velocity += delta_velocity_gravity_x_m_per_s
                self.__eks.acceleration_x += accel_gravity_x_m_per_s2
            if self.__eks.velocity < 0.0:
                self.__eks.velocity = 0.0
            if self.__pm.push_active:
                accel_x_m_per_s2, delta_velocity_push_m_per_s = self.__pm.step(self.__fixed_time_step_ms)
                self.__eks.acceleration_x += accel_x_m_per_s2
                self.__eks.velocity += delta_velocity_push_m_per_s
            self.__eks_lock.release()
            if not self.__loop_active:
                break
            elapsed_time = time.perf_counter() - start_time
            sleep_time = max(0, self.__fixed_time_step_ms / 1000.0 - elapsed_time)
            time.sleep(sleep_time)

    def stop(self) -> None:
        self.__loop_active = False
