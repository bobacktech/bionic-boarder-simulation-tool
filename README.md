# Description

This application simulates a person that is normally skateboarding or land paddling on a skateboard that is equipped with an electric motor, a VESC speed controller, and an IMU that is connected to the VESC. The VESC injects a very small amount of current to the motor to allow the skateboard to flow freely like a regular skateboard.  The simulated VESC communicates the motor state and the IMU state over Bluetooth (via a UART to Bluetooth adapter) to another application running on an Android device.  The VESC also receives commands from the other application through the Bluetooth channel to accelerate or decelerate the motor.  When the VESC is commanded to accelerate or decelerate the motor from the Android application, the simulator will not allow the "rider" to accelerate or decelerate the skateboard.

## Documentation

* [HC-06 Classic Bluetooth to UART module Setup](https://docs.google.com/presentation/d/1iqZNpbXgkZIJNUv7u3ZKozi3m8eBVSuuvXjyms1xasU/edit?usp=sharing)
