from dataclasses import dataclass


@dataclass
class MotorState:
    duty_cycle: float
    erpm: int
    input_current: float
