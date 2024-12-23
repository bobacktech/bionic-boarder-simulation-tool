## [1.1.1] - MM-DD-YYYY
### Fixed
- Minor bug fix: [Description of the bug fix].
- Addressed edge case in [specific functionality].

---

## [1.1.0] - MM-DD-YYYY
### Added
- New feature: [Description of the new feature].
- Support for [specific enhancement].

### Changed
- Improved performance of [specific functionality].
- Updated documentation to reflect new changes.

### Fixed
- Resolved issue with [specific bug or issue].
- Fixed compatibility with [specific environment or dependency].

---

## [1.0.0] - 12-22-2024
### Added
- Initial release of the simulation.
- Core functionality implemented:
  - Support for VESC FW 6.00
  - Handling and processing of these VESC messages: 
  -<p> GET FIRMWARE, GET STATE, GET IMU STATE, SET RPM, SET CURRENT, HEARTBEAT
  
  - RPM motor control
  - Current control is used only to stop current from flowing into the motor.
  - Simple terrain model where the slope angle is constant for a short period of time and changes from downhill to flat to uphill and vice versa
  - Models only acceleration of the land paddle board by the rider.
  - Simple air resistance and friction model that treats the board and rider as a single rigid body
  - Kinematic data logging
  - Utility scripts to plot data from log files and detect when the rider accelerates the board from the acceleration data