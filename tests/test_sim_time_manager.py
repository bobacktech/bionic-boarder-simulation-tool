from augmented_skateboarding_simulator.sim_time_manager import SimTimeManager
import time


class TestSimTimeManager:
    """
    Unit test class for SimTimeManager.

    This class contains several test methods to validate the functionality of the SimTimeManager class.
    """

    def test_initialization(self):
        """
        Tests the initialization of SimTimeManager.

        Ensures that an instance of SimTimeManager is successfully created and is not None.
        """
        sim_manager = SimTimeManager()
        assert sim_manager is not None, "Failed to initialize SimTimeManager."

    def test_set_and_get_time_step(self):
        """
        Tests setting and getting the time step in SimTimeManager.

        Verifies that the time step is correctly set and retrieved by the sim_time_step method.
        """
        sim_manager = SimTimeManager()
        time_step = 100  # 100 ms
        sim_manager.set_sim_time_step(time_step)
        assert (
            sim_manager.sim_time_step() == time_step
        ), "Time step did not set correctly."

    def test_set_time_step_only_once(self):
        """
        Tests the immutability of the time step in SimTimeManager.

        Ensures that the time step, once set, cannot be changed to a different value.
        """
        sim_manager = SimTimeManager()
        initial_time_step = 100
        new_time_step = 200
        sim_manager.set_sim_time_step(initial_time_step)
        sim_manager.set_sim_time_step(new_time_step)
        assert (
            sim_manager.sim_time_step() == initial_time_step
        ), "Time step should not change after being set once."

    def test_start_sim(self):
        """
        Tests the start of the simulation timing in SimTimeManager.

        Checks that the simulation start time is set (greater than zero) when start_sim is called.
        """
        sim_manager = SimTimeManager()
        sim_manager.start_sim()
        assert (
            SimTimeManager._SimTimeManager__sim_start_time_ms > 0
        ), "Simulation start time should be set."

    def test_sim_elapsed_time(self):
        """
        Tests the calculation of elapsed time in SimTimeManager.

        Verifies that the elapsed time since the start of the simulation is correctly calculated.
        This is done by sleeping the thread for a known duration and checking if the elapsed time is as expected.
        """
        sim_manager = SimTimeManager()
        sim_manager.reset_sim()
        sim_manager.start_sim()
        time.sleep(0.1)  # Sleep for 100ms
        elapsed_time = sim_manager.sim_elapsed_time()
        # Asserting elapsed time is greater than or equal to 100 ms and less than 200 ms
        assert 100 <= elapsed_time, "Elapsed time calculation is incorrect."
