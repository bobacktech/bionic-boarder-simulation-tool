# Bionic Boarder Simulation Tool

This simulation is created for the purpose of supporting the development of the Bionic Boarder android application. See [Bionic Boarder](https://github.com/bobacktech/bionic-boarder).

The tool simulates a person riding a land paddling board where the rider accelerates the board with a stick paddle.  In the simulation, the land paddle board is equipped with 
an electric motor that is controlled by a [VESC speed controller](https://github.com/vedderb/bldc). The VESC also specifically has an integrated IMU on the controller that provides 
orientation and acceleration data in real time. This application essentially is a 2DOF simulation that computes the acceleration along the long axis of the board and the pitch of the board as the person is paddling the board with a stick on a surface where the slope changes over time. Communication with the simulated VESC is done over bluetooth. The PC that the simulation executes
on requires an HC-06 UART to classic Bluetooth module connected to the PC using an FTDI USB adapter.  The simulation processes these VESC messages: set current, set rpm, heartbeat, get firmware, get state, and get IMU state.


## Software requirements

This project is managed by the Poetry dependency management and packaging system.  All the dependencies for the application are in the pyproject.toml file.
The minimum python version to use is 3.12.3.

## Running the simulation

*  **No logging:** <p> poetry run python main.py <path-to-app_input_arguments.json>

*  **With logging:** <p> poetry run python main.py <path-to-app_input_arguments.json> --enable-logging

*  **With data recording:** <p> poetry run python main.py <path-to-app_input_arguments.json> --enable-data-recording


## Documentation

* [HC-06 Classic Bluetooth to UART module Setup](https://docs.google.com/presentation/d/1iqZNpbXgkZIJNUv7u3ZKozi3m8eBVSuuvXjyms1xasU/edit?usp=sharing)
