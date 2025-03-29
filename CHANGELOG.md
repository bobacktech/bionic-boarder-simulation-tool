## [1.0.0] - 03/29/2025

### Added
- **Initial release** of the Bionic Boarder Simulation Tool.
- **Core functionality** includes:
  - **Supported VESC BLDC commands**:
    - `COMM_FW_VERSION`
    - `COMM_GET_VALUES`
    - `COMM_GET_IMU_DATA`
    - `COMM_SET_RPM`
    - `COMM_SET_CURRENT`
    - `COMM_ALIVE`
  - **Supported VESC BLDC firmware versions**:
    - Firmware version `6.00`
  - **Motor control features**:
    - RPM-based motor control.
    - Motor current control, currently limited to setting motor current to zero for relinquishing the motor control from the board's wheel.
  - **Terrain simulation**:
    - A simple terrain model with a constant slope angle that periodically changes from downhill, flat, and to uphill and vice versa.
  - **Push simulation**:
    - A human-like push model to simulate rider acceleration using a stick paddle.
  - **Environmental effects**:
    - Basic air resistance and friction model, treating the board and rider as a single rigid body.
  - **Data logging**:
    - Kinematic data logging for detailed analysis.
  - **Analysis scripts**:
    - Plotting kinematic data from binary data log files.
    - Detection of long axis acceleration spikes that arise from the rider pushing the board from the acceleration data.