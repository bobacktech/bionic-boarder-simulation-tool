# Bionic Boarder Simulation Tool

This simulation tool's purpose is to support the ongoing development of the [Bionic Boarder](https://github.com/bobacktech/bionic-boarder) Android application.

The simulation models a rider operating a land paddle board, where propulsion is achieved using a stick paddle across a dynamically varying terrain.  At the start of the simulation, the rider is stationary. The terrain slope changes randomly at periodic intervals, and the rider applies paddle strokes at regular intervals as well. Movement of the board is driven by a combination of gravitational force and rider input.

Key parameters computed over time include the board’s velocity and acceleration along its longitudinal (nose-to-tail) axis, as well as the pitch angle relative to that axis.

The simulated board is also equipped with an electric motor controlled by a [VESC speed controller](https://github.com/vedderb/bldc). The VESC includes an integrated IMU (Inertial Measurement Unit) that provides real-time orientation and acceleration data. In the simulation, the IMU is oriented such that the x-axis aligns with the board’s longitudinal axis, the y-axis spans the width of the board, and the z-axis points vertically through the board.

The motor controller is emulated with a simplified ERPM (Electrical RPM) control scheme, allowing external applications to control the board’s speed in response to rider input. This enables testing and development of control strategies for the Bionic Boarder system.

The simulation supports the following VESC communication commands:

*  **COMM_FW_VERSION**
*  **COMM_GET_VALUES**
*  **COMM_SET_CURRENT**
*  **COMM_SET_RPM**
*  **COMM_ALIVE**
*  **COMM_BIONIC_BOARDER** - Custom Command

See VESC BLDC [datatypes.h](https://github.com/vedderb/bldc/blob/release_6_00/datatypes.h) for more details about the above commands.

Communication with the simulated VESC is done over classic Bluetooth. The PC that the simulation executes on requires an HC-06 (or equivalent) UART to classic Bluetooth module connected to the PC using an FTDI USB adapter.  

### Supported VESC BLDC firmware versions 
* [6.00](https://github.com/vedderb/bldc/tree/release_6_00)
* [6.02](https://github.com/vedderb/bldc/tree/release_6_02)
* [6.05](https://github.com/vedderb/bldc/tree/release_6_05)

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