import pytest
from augmented_skateboarding_simulator.riding.motor_controller import MotorController
from augmented_skateboarding_simulator.riding.eboard import EBoard
from augmented_skateboarding_simulator.riding.eboard_kinematic_state import EboardKinematicState
from threading import Lock
import time


@pytest.fixture
def eks():
    return EboardKinematicState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


class TestMotorController:
    def test_max_erpm_per_sec_calculation(self, eks: EboardKinematicState):
        eboard = EBoard(
            total_weight_with_rider_kg=80.0,
            frontal_area_of_rider_m2=0.5,
            wheel_diameter_m=0.1,
            battery_max_capacity_Ah=10.0,
            battery_max_voltage=36.0,
            gear_ratio=2.0,
            motor_kv=190,
            motor_max_torque=6.0,
            motor_max_amps=50.0,
            motor_max_power_watts=500.0,
            motor_pole_pairs=7,
        )
        mc = MotorController(eboard, eks, Lock())
        erpm_per_sec_1 = mc.erpm_per_sec
        assert mc.erpm_per_sec > 0
        eboard = EBoard(
            total_weight_with_rider_kg=80.0,
            frontal_area_of_rider_m2=0.5,
            wheel_diameter_m=0.1,
            battery_max_capacity_Ah=10.0,
            battery_max_voltage=36.0,
            gear_ratio=2.0,
            motor_kv=170,
            motor_max_torque=7.0,
            motor_max_amps=50.0,
            motor_max_power_watts=500.0,
            motor_pole_pairs=7,
        )
        mc = MotorController(eboard, eks, Lock())
        erpm_per_sec_2 = mc.erpm_per_sec
        assert erpm_per_sec_1 < erpm_per_sec_2

    def test_erpm_and_current_control_threads_started_properly(self, eks: EboardKinematicState):
        eboard = EBoard(
            total_weight_with_rider_kg=80.0,
            frontal_area_of_rider_m2=0.5,
            wheel_diameter_m=0.1,
            battery_max_capacity_Ah=10.0,
            battery_max_voltage=36.0,
            gear_ratio=2.0,
            motor_kv=190,
            motor_max_torque=6.0,
            motor_max_amps=50.0,
            motor_max_power_watts=500.0,
            motor_pole_pairs=7,
        )
        mc = MotorController(eboard, eks, Lock())
        assert mc.current_sem._value == 0
        assert mc.erpm_sem._value == 0
        assert mc._MotorController__current_thread.is_alive() == False
        assert mc._MotorController__erpm_thread.is_alive() == False
        mc.start()
        time.sleep(0.1)
        assert mc._MotorController__current_thread.is_alive() == True
        assert mc._MotorController__erpm_thread.is_alive() == True

    def test_stop(self, eks: EboardKinematicState):
        eboard = EBoard(
            total_weight_with_rider_kg=80.0,
            frontal_area_of_rider_m2=0.5,
            wheel_diameter_m=0.1,
            battery_max_capacity_Ah=10.0,
            battery_max_voltage=36.0,
            gear_ratio=2.0,
            motor_kv=190,
            motor_max_torque=6.0,
            motor_max_amps=50.0,
            motor_max_power_watts=500.0,
            motor_pole_pairs=7,
        )
        mc = MotorController(eboard, eks, Lock())
        mc.control_time_step_ms = 20
        mc.start()
        time.sleep(0.1)
        assert mc._MotorController__current_thread.is_alive() == True
        assert mc._MotorController__erpm_thread.is_alive() == True
        mc.stop()
        time.sleep(0.1)
        assert mc._MotorController__current_thread.is_alive() == False
        assert mc._MotorController__erpm_thread.is_alive() == False

    def test_increase_motor_erpm(self, eks: EboardKinematicState):
        eboard = EBoard(
            total_weight_with_rider_kg=80.0,
            frontal_area_of_rider_m2=0.5,
            wheel_diameter_m=0.1,
            battery_max_capacity_Ah=10.0,
            battery_max_voltage=36.0,
            gear_ratio=2.0,
            motor_kv=190,
            motor_max_torque=6.0,
            motor_max_amps=50.0,
            motor_max_power_watts=500.0,
            motor_pole_pairs=7,
        )
        eks_lock = Lock()
        mc = MotorController(eboard, eks, eks_lock)
        mc.control_time_step_ms = 20
        mc.start()
        mc.target_erpm = 2000
        assert eks.erpm == 0
        mc.erpm_sem.release()
        while eks.erpm < mc.target_erpm:
            pass
        assert eks.erpm >= mc.target_erpm
        assert eks.velocity > 0
        mc.stop()

    def test_decrease_motor_erpm(self, eks: EboardKinematicState):
        eboard = EBoard(
            total_weight_with_rider_kg=80.0,
            frontal_area_of_rider_m2=0.5,
            wheel_diameter_m=0.1,
            battery_max_capacity_Ah=10.0,
            battery_max_voltage=36.0,
            gear_ratio=2.0,
            motor_kv=190,
            motor_max_torque=6.0,
            motor_max_amps=50.0,
            motor_max_power_watts=500.0,
            motor_pole_pairs=7,
        )
        eks_lock = Lock()
        mc = MotorController(eboard, eks, eks_lock)
        mc.control_time_step_ms = 20
        mc.start()
        mc.target_erpm = 2000
        assert eks.erpm == 0
        mc.erpm_sem.release()
        while eks.erpm < mc.target_erpm:
            pass
        assert eks.erpm >= mc.target_erpm
        velocity_before = eks.velocity
        time.sleep(0.1)
        assert eks.velocity == velocity_before
        mc.target_erpm = 1000
        mc.erpm_sem.release()
        while eks.erpm > mc.target_erpm:
            pass
        assert eks.erpm <= mc.target_erpm
        assert eks.velocity < velocity_before
        mc.stop()

    def test_current_control(self, eks: EboardKinematicState):
        eboard = EBoard(
            total_weight_with_rider_kg=80.0,
            frontal_area_of_rider_m2=0.5,
            wheel_diameter_m=0.1,
            battery_max_capacity_Ah=10.0,
            battery_max_voltage=36.0,
            gear_ratio=2.0,
            motor_kv=190,
            motor_max_torque=6.0,
            motor_max_amps=50.0,
            motor_max_power_watts=500.0,
            motor_pole_pairs=7,
        )
        eks_lock = Lock()
        mc = MotorController(eboard, eks, eks_lock)
        eks.input_current = 10.0
        mc.target_current = 0.0
        mc.start()
        mc.current_sem.release()
        time.sleep(0.1)
        assert eks.input_current == mc.target_current
        mc.stop()
