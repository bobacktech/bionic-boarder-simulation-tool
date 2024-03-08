from sim_time_manager import SimTimeManager
import argparse
import re
import threading


def firmwareRegex(argValue, pattern=re.compile(r"^\d*[.]\d*$")):
    if not pattern.match(argValue):
        raise argparse.ArgumentTypeError(
            "VESC firmware version specified as $MajorVersion.$MinorVersion, e.g 2.18"
        )
    return argValue


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--vescFW",
        help="Specifies the VESC firmware version to be used in the simulation.",
        type=firmwareRegex,
    )
    parser.add_argument(
        "--comPort", help="The com port for the attached USB FTDI module."
    )
    args = parser.parse_args()
    com_port = args.comPort
    vesc_fw = args.vescFW

    """
        Instantiate published VESC messages, specifically the System state message and the IMU state message.
        These objects will hold the last updated values create by the simulation.
    """

    """
        Instantiate execution flow semaphore that allows to main thread to sleep while simulation is running.
    """
    flowSem = threading.Semaphore(0)

    """
        Instantiate electric skateboard description object.
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
    flowSem.acquire()
