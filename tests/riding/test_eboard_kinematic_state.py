import pytest
from augmented_skateboarding_simulator.riding.eboard_kinematic_state import (
    EboardKinematicState,
)


@pytest.fixture
def test_state():
    return EboardKinematicState(
        velocity=1.0,
        acceleration_x=2.0,
        acceleration_y=3.0,
        acceleration_z=4.0,
        pitch=5.0,
        roll=6.0,
        yaw=7.0,
        erpm=8,
        input_current=9.0,
    )


def test_velocity(test_state):
    assert test_state.velocity == 1.0


def test_acceleration_x(test_state):
    assert test_state.acceleration_x == 2.0


def test_acceleration_y(test_state):
    assert test_state.acceleration_y == 3.0


def test_acceleration_z(test_state):
    assert test_state.acceleration_z == 4.0


def test_pitch(test_state):
    assert test_state.pitch == 5.0


def test_roll(test_state):
    assert test_state.roll == 6.0


def test_yaw(test_state):
    assert test_state.yaw == 7.0


def test_erpm(test_state):
    assert test_state.erpm == 8


def test_input_current(test_state):
    assert test_state.input_current == 9.0
