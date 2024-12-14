from bionic_boarder_simulation_tool.riding.frictional_deceleration_model import (
    FrictionalDecelerationModel,
)
from bionic_boarder_simulation_tool.riding.eboard import EBoard


class TestFrictionalDecelerationModel:

    def test_decelerate(self):
        """
        Test the decelerate method of the FrictionalDecelerationModel class.

        This test creates an instance of the FrictionalDecelerationModel class with specific
        values for mu_rolling, c_drag, and eboard. It then tests the decelerate method by
        calling it with different time steps and asserting that the returned acceleration
        and delta velocity are positive and that the new velocity is less than the previous
        velocity.

        The test also checks that the decelerate method returns a non-zero acceleration when
        the velocity is greater than 0. It then calls the decelerate method repeatedly
        with a time step of 20 ms and asserts that the returned acceleration is positive
        and that the returned delta velocity is less than the previous delta velocity.
        This test ensures that the decelerate method behaves as expected for different
        velocity values.
        """
        eboard = EBoard(80, 1.8288 * 0.6096, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        fdm = FrictionalDecelerationModel(mu_rolling=0.01, c_drag=0.90, eboard=eboard)
        current_velocity_m_per_s = 1000 / 3600
        time_step_ms = (20, 40, 60, 80, 100)
        for t in time_step_ms:
            accel_ms2, delta_v_m_per_s = fdm.decelerate(current_velocity_m_per_s, t)
            assert accel_ms2 > 0
            assert delta_v_m_per_s < current_velocity_m_per_s

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
