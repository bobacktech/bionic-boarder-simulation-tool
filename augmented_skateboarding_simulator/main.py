from sim_time_manager import SimTimeManager
import argparse
import re


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    """
        Parse command line arguments to configure execution of the simulation.
    """

    """
        Instantiate published VESC messages, specifically the System state message and the IMU state message.
        These objects will hold the last updated values create by the simulation.
    """

    """
        Instantiate execution flow semaphore that allows to main thread to sleep while simulation is running.
    """

    """
        Launch VESC command processor on a separate thread.
    """
    stm = SimTimeManager()
    stm.set_sim_time_step(10)
    stm.start_sim()
    """
        Launch skateboard accelerator on a separate thread.
    """

    """
        Use the semaphore above to block main thread.  This semaphore will wake up or release in another thread to allow 
        the main thread to finish.
    """
