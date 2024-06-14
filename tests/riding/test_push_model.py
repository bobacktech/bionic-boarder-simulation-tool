import pytest
from augmented_skateboarding_simulator.riding.push_model import PushModel
from augmented_skateboarding_simulator.riding.eboard import EBoard


@pytest.fixture
def push_model():
    eboard = EBoard(80, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    return PushModel(eboard)


class TestPushModel:

    def test_setup(self, push_model: PushModel):
        assert push_model.force_slowdown_N == 0
        assert push_model.force_rider_N == 0
        velocity_increase_mps = 2.78  # approx equal to 10 km/hr
        slope_angle_deg = 30
        push_model.setup(velocity_increase_mps, slope_angle_deg)
        assert push_model.force_rider_N > 0
        assert push_model.force_slowdown_N < 0
        force_rider_temp = push_model.force_rider_N
        push_model.setup(velocity_increase_mps, slope_angle_deg - 10)
        assert push_model.force_rider_N < force_rider_temp
