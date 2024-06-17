import pytest
from augmented_skateboarding_simulator.riding.push_model import PushModel
from augmented_skateboarding_simulator.riding.eboard import EBoard


class TestPushModel:

    eboard = EBoard(80, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    def test_1(self):
        pm = PushModel(0, 0, TestPushModel.eboard)
