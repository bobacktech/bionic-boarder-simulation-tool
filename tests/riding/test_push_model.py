import pytest
from augmented_skateboarding_simulator.riding.push_model import PushModel
from augmented_skateboarding_simulator.riding.eboard import EBoard


@pytest.fixture
def pm():
    eboard = EBoard(80, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    pm = PushModel(eboard)
    return pm


class TestPushModel:

    def test_slowdown(self, pm: PushModel):
        pm.setup(200, 500)
        assert pm.push_active == True

        prev_accel, prev_delta_v = -float("inf"), -float("inf")
        elapsed_time_ms = 0
        while elapsed_time_ms <= pm.slowdown_duration_ms:
            accel, delta_v = pm.step(10)
            assert accel > prev_accel
            assert delta_v > prev_delta_v
            prev_accel, prev_delta_v = accel, delta_v
            elapsed_time_ms += 10
        assert elapsed_time_ms > pm.slowdown_duration_ms
        assert pm.push_active == True
        assert prev_accel == 0
        assert prev_delta_v == 0

    def test_accelerate(self, pm: PushModel):
        pm.setup(200, 500)
        assert pm.push_active == True
        elapsed_time_ms = 0
        while elapsed_time_ms <= pm.slowdown_duration_ms:
            pm.step(10)
            elapsed_time_ms += 10
        prev_accel, prev_delta_v = 0, 0
        while pm.push_active:
            accel, delta_v = pm.step(10)
            assert accel > prev_accel
            assert delta_v > prev_delta_v
            prev_accel, prev_delta_v = accel, delta_v
        twice_accel = 2 * (200 / 80)
        assert prev_accel == pytest.approx(twice_accel, abs=0.1)
