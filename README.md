# Bionic Boarder Simulation Tool

This simulation is created for the purpose of supporting the development of the [Bionic Boarder](https://github.com/bobacktech/bionic-boarder) android application.

The tool simulates a person riding a land paddle board where the rider accelerates the board with a stick paddle along a non flat terrain.  When the simulation starts, the rider is not moving. The slope of the terrain changes randomly and at a fixed frequency. The rider pushes the board with the stick paddle on a periodic basis.  Gravity and/or the rider is responsible
for moving the board. The primary parameters that are computed over time from the rider and/or gravity moving the board are velocity along nose/long axis of the board, acceleration along the nose/long axis of the board, and the pitch relative to the long axis of the board.


Additionally, in the simulation, the land paddle board is equipped with an electric motor that is controlled by a [VESC speed controller](https://github.com/vedderb/bldc). The VESC specifically has an integrated IMU on the controller that provides orientation and acceleration data in real time. The IMU is situated on the board such that the x-axis is along the nose/long axis of the board, the y-axis is along the width of the board, and the z-axis is up through the board itself.  The emulation of the VESC controller has a simplified motor controller that provides ERPM control to change the speed of the motor. The purpose for the VESC in the simulation is to allow for an external application to control the board's movement as the rider paddles the board. 

The following VESC commands are processed by the simulation:

*  **COMM_FW_VERSION**
*  **COMM_GET_VALUES**
*  **COMM_GET_IMU_DATA**
*  **COMM_SET_CURRENT**
*  **COMM_SET_RPM**
*  **COMM_ALIVE**

See VESC BLDC [datatypes.h](https://github.com/vedderb/bldc/blob/release_6_00/datatypes.h) for more details about the above commands.

Communication with the simulated VESC is done over classic Bluetooth. The PC that the simulation executes on requires an HC-06 (or equivalent) UART to classic Bluetooth module connected to the PC using an FTDI USB adapter.  

### Supported VESC BLDC firmware versions 
* 6.00

## Software requirements

This project is managed by the Poetry dependency management and packaging system.  All the dependencies for the application are in the pyproject.toml file.
The minimum python version to use is 3.12.3.

## Running the simulation

*  **No logging:** <p> poetry run python main.py <path-to-app_input_arguments.json>

*  **With logging:** <p> poetry run python main.py <path-to-app_input_arguments.json> --enable-logging

*  **With data recording:** <p> poetry run python main.py <path-to-app_input_arguments.json> --enable-data-recording

*  **With data recording and logging:** <p> poetry run python main.py <path-to-app_input_arguments.json> --enable-data-recording --enable-logging

## Format for the required inputs to the simulation

* [App Inputs JSON Schema](https://github.com/bobacktech/bionic-boarder-simulation-tool/blob/master/bionic_boarder_simulation_tool/app_input_arguments.schema.json)

## Documentation

* [HC-06 Classic Bluetooth to UART module Setup](https://docs.google.com/presentation/d/1iqZNpbXgkZIJNUv7u3ZKozi3m8eBVSuuvXjyms1xasU/edit?usp=sharing)