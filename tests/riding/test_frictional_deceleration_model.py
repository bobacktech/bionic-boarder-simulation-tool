import pytest
from augmented_skateboarding_simulator.riding.frictional_deceleration_model import (
    FrictionalDecelerationModel,
)
from augmented_skateboarding_simulator.riding.eboard import EBoard


class TestFrictionalDecelerationModel:

    def test_decelerate(self):
        eboard = EBoard(80, 1.8288 * 0.6096, 0, 0, 0, 0, 0, 0, 0, 0)
        fdm = FrictionalDecelerationModel(mu_rolling=0.01, c_drag=0.90, eboard=eboard)
        current_velocity_m_per_s = 1000 / 3600
        time_step_ms = (20, 40, 60, 80, 100)
        temp_cv_m_per_s = current_velocity_m_per_s
        for t in time_step_ms:
            accel_ms2, delta_v_m_per_s = fdm.decelerate(current_velocity_m_per_s, t)
            assert accel_ms2 > 0
            new_velocity_m_per_s = temp_cv_m_per_s - delta_v_m_per_s
            assert new_velocity_m_per_s < temp_cv_m_per_s
            temp_cv_m_per_s = new_velocity_m_per_s

        initial_velocity_m_per_s = 1000 / 3600
        current_velocity_m_per_s = initial_velocity_m_per_s
        accel_ms2, delta_v_m_per_s = fdm.decelerate(current_velocity_m_per_s, 20)
        current_velocity_m_per_s -= delta_v_m_per_s
        for i in range(10):
            a, v = fdm.decelerate(current_velocity_m_per_s, 20)
            assert a > 0
            current_velocity_m_per_s -= v
            assert v < delta_v_m_per_s
            delta_v_m_per_s = v
        assert current_velocity_m_per_s < initial_velocity_m_per_s
