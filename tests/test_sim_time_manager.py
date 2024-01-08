from augmented_skateboarding_simulator.sim_time_manager import SimTimeManager
import time


class TestSimTimeManager:
    def test_initialization(self):
        sim_manager = SimTimeManager()
        assert sim_manager is not None, "Failed to initialize SimTimeManager."

    def test_set_and_get_time_step(self):
        sim_manager = SimTimeManager()
        time_step = 100  # 100 ms
        sim_manager.set_sim_time_step(time_step)
        assert (
            sim_manager.sim_time_step() == time_step
        ), "Time step did not set correctly."

    def test_set_time_step_only_once(self):
        sim_manager = SimTimeManager()
        initial_time_step = 100
        new_time_step = 200
        sim_manager.set_sim_time_step(initial_time_step)
        sim_manager.set_sim_time_step(new_time_step)
        assert (
            sim_manager.sim_time_step() == initial_time_step
        ), "Time step should not change after being set once."

    def test_start_sim(self):
        sim_manager = SimTimeManager()
        sim_manager.start_sim()
        assert (
            SimTimeManager._SimTimeManager__sim_start_time_ms > 0
        ), "Simulation start time should be set."

    def test_sim_elapsed_time(self):
        sim_manager = SimTimeManager()
        sim_manager.reset_sim()
        x = SimTimeManager._SimTimeManager__sim_start_time_ms

        sim_manager.start_sim()
        time.sleep(0.1)  # Sleep for 100ms
        elapsed_time = sim_manager.sim_elapsed_time()
        # Asserting elapsed time is greater than or equal to 100 ms and less than 200 ms
        assert 100 <= elapsed_time, "Elapsed time calculation is incorrect."
