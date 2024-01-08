import time


class SimTimeManager:
    """
    A class to manage simulation time.

    This class provides functionalities to set and manage time steps for simulations,
    track the start and elapsed time, and reset the simulation time settings.

    Attributes:
        __time_step_ms (int): The time step for the simulation in milliseconds.
        __sim_start_time_ms (int): The start time of the simulation in milliseconds.
    """

    __time_step_ms = 0
    __sim_start_time_ms = 0

    def __init__(self):
        """Initializes the SimTimeManager class."""
        pass

    def set_sim_time_step(self, step_ms):
        """
        Sets the time step for the simulation.

        The time step can only be set once and is immutable thereafter.

        Args:
            step_ms (int): The time step in milliseconds.
        """
        if SimTimeManager.__time_step_ms == 0:
            SimTimeManager.__time_step_ms = step_ms

    def sim_time_step(self) -> int:
        """Returns the current simulation time step in milliseconds."""
        return SimTimeManager.__time_step_ms

    def start_sim(self):
        """
        Starts the simulation timer.

        Sets the start time of the simulation in milliseconds,
        but only if it has not already been set.
        """
        if SimTimeManager.__sim_start_time_ms == 0:
            SimTimeManager.__sim_start_time_ms = int(time.time_ns() // 1000000)

    def sim_elapsed_time(self) -> int:
        """
        Calculates the elapsed time since the start of the simulation.

        Returns:
            int: The elapsed time in milliseconds.
        """
        return int(time.time_ns() // 1000000) - SimTimeManager.__sim_start_time_ms

    def reset_sim(self):
        """
        Resets the simulation time settings.

        This method resets both the simulation start time and the time step to zero.
        """
        SimTimeManager.__sim_start_time_ms = 0
        SimTimeManager.__time_step_ms = 0
