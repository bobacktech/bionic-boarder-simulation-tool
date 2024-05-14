import pytest
from augmented_skateboarding_simulator.riding.frictional_deceleration_model import (
    FrictionalDecelerationModel,
)
from augmented_skateboarding_simulator.riding.eboard import EBoard


class TestFrictionalDecelerationModel:
    """
    A few tests of FrictionalDecelerationModel class that vary the coefficient of rolling friction and the coefficient of drag.
    """

    @pytest.fixture
    def eboard(self):
        return EBoard(80, 1.8288 * 0.6096, 0, 0, 0, 0, 0, 0, 0, 0)

    def test_1(self, eboard: EBoard):
        v = EBoard(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        fdm = FrictionalDecelerationModel(
            rolling_friction_coefficient=0.1, drag_coefficient=0.1, eboard=eboard
        )
        current_velocity_m_per_s = 1000
        time_step_ms = 20
