## [1.0.0] - 03/26/2025

### Added
- **Initial release** of the Bionic Boarder Simulation Tool.
- **Core functionality** includes:
  - **Supported VESC BLDC messages**:
    - `GET FIRMWARE`
    - `GET STATE`
    - `GET IMU STATE`
    - `SET RPM`
    - `SET CURRENT`
    - `HEARTBEAT`
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