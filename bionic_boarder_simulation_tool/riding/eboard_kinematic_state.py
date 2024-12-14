from dataclasses import dataclass


"""
This class specifies the kinematic state of the electric land paddle board. The coordinate 
reference frame for the data values is the land paddle board itself, i.e. the body coordinate frame. 

x-axis is along the length of the land paddle board.
y-axis is along the width of the land paddle board.
z-axis is along the height of the land paddle board.   

The velocity is the speed along the long axis, i.e. x-axis, of the electric land paddle board.
"""


@dataclass
class EboardKinematicState:
    """Velocity is the speed along the long axis of the eboard. Units m/s"""

    velocity: float = 0.0

    """3-Dimensional linear acceleration vector. Units m/s^2"""
    acceleration_x: float = 0.0
    acceleration_y: float = 0.0
    acceleration_z: float = 0.0

    """Attitude - pitch, roll, yaw - in degrees"""
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0

    """Electric Motor State"""
    erpm: int = 0
    input_current: float = 0.0
