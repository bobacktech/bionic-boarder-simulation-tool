import pytest
from augmented_skateboarding_simulator.riding.kinematic_loop import KinematicLoop
from augmented_skateboarding_simulator.riding.eboard import EBoard
from augmented_skateboarding_simulator.riding.motor_state import MotorState
from unittest.mock import MagicMock
from augmented_skateboarding_simulator.riding.frictional_deceleration_model import FrictionalDecelerationModel
from augmented_skateboarding_simulator.riding.push_model import PushModel
from augmented_skateboarding_simulator.riding.eboard_kinematic_state import EboardKinematicState
from threading import Lock
import threading
import time


@pytest.fixture
def eboard():
    return EBoard(80, 0, 0.0508, 0, 0, 2, 0, 0, 0, 0, 7)


@pytest.fixture
def eks():
    return EboardKinematicState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


@pytest.fixture
def fdm_mock():
    fdm_mock = MagicMock(spec=FrictionalDecelerationModel)
    fdm_mock.decelerate.side_effect = lambda x, y: (0.1, 0.02)
    return fdm_mock


@pytest.fixture
def pm_mock():
    pm_mock = MagicMock(spec=PushModel)
    pm_mock.setup.side_effect = lambda x, y: None
    pm_mock.push_active = False
    return pm_mock


@pytest.fixture
def kloop(eboard: EBoard, eks: EboardKinematicState, fdm_mock: MagicMock, pm_mock: MagicMock) -> KinematicLoop:
    return KinematicLoop(eboard, eks, Lock(), fdm_mock, pm_mock)


class TestKinematicLoop:
    def test_kinematic_loop_instance(self, kloop: KinematicLoop):
        assert isinstance(kloop, KinematicLoop)

    def test_initial_velocity_zero_no_push(self, kloop: KinematicLoop, eks: EboardKinematicState):
        kloop.fixed_time_step_ms = 20
        kloop.slope_range_bound_deg = 10
        kloop.push_period_sec = 0.6
        kloop.theta_slope_period_sec = 0.6
        t = threading.Thread(target=kloop.loop)
        t.start()
        time.sleep(0.3)
        kloop.stop()
        assert eks.velocity == 0.0
        assert eks.erpm == 0

    def test_initial_velocity_nonzero_no_push(self, kloop: KinematicLoop, eks: EboardKinematicState):
        kloop.fixed_time_step_ms = 20
        kloop.slope_range_bound_deg = 10
        kloop.push_period_sec = 0.6
        kloop.theta_slope_period_sec = 0.6
        eks.velocity = 10
        eks.acceleration_x = 0
        t = threading.Thread(target=kloop.loop)
        t.start()
        time.sleep(0.3)
        kloop.stop()
        assert eks.velocity < 10
        assert eks.acceleration_x == -0.1
        assert eks.erpm > 0

    def test_initial_velocity_zero_push(self, kloop: KinematicLoop, eks: EboardKinematicState, pm_mock: MagicMock):
        kloop.fixed_time_step_ms = 20
        kloop.slope_range_bound_deg = 10
        kloop.push_period_sec = 0.8
        kloop.theta_slope_period_sec = 0.8
        pm_mock.push_active = True
        pm_mock.step.side_effect = lambda x: (0.1, 0.2)
        assert eks.erpm == 0
        t = threading.Thread(target=lambda: (time.sleep(0.5), kloop.stop()))
        t.start()
        kloop.loop()
        assert eks.velocity < (25 * 0.2)
        assert eks.velocity >= ((25 * 0.2) - (25 * 0.02))
        assert eks.erpm > 0

    def test_initial_velocity_nonzero_push(self, kloop: KinematicLoop, eks: EboardKinematicState, pm_mock: MagicMock):
        kloop.fixed_time_step_ms = 20
        kloop.slope_range_bound_deg = 10
        kloop.push_period_sec = 0.8
        kloop.theta_slope_period_sec = 0.8
        eks.velocity = 10
        pm_mock.push_active = True
        pm_mock.step.side_effect = lambda x: (0.1, 0.2)
        temp_erpm = 0

        def run():
            nonlocal temp_erpm
            time.sleep(0.1)
            temp_erpm = eks.erpm
            time.sleep(0.4)
            kloop.stop()
            return None

        t = threading.Thread(target=run)
        t.start()
        kloop.loop()
        assert eks.velocity < 10 + (25 * 0.2)
        assert eks.velocity >= 10 + ((25 * 0.2) - (25 * 0.02))
        assert eks.erpm > temp_erpm

    def test_initial_velocity_zero_no_push_negative_velocity(self, kloop: KinematicLoop, eks: EboardKinematicState):
        kloop.fixed_time_step_ms = 20
        kloop.slope_range_bound_deg = 10
        kloop.push_period_sec = 1.0
        kloop.theta_slope_period_sec = 1.0
        kloop.initial_theta_slope_deg = 5.0
        eks.velocity = 0
        eks.erpm = 0
        t = threading.Thread(target=kloop.loop)
        t.start()
        time.sleep(0.2)
        kloop.stop()
        assert eks.velocity < 0
        assert eks.erpm < 0
