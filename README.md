# Description

This application simulates a person that is normally skateboarding on an electric skateboard that is equipped with a 
VESC speed controller which has an IMU connected to the VESC.  Specifically what that means is the rider accelerates and decelerates the skateboard as if it was a regular skateboard with no electric motor.  As the person is riding, the simulated VESC is broadcasting the motor state and IMU state over Bluetooth to another application running on an Android device.  The 
Android application uses this state data to determine how to increase the rider's velocity as the skateboard accelerates and then sends commands over Bluetooth back to the simulated VESC to achieve the boost in velocity.  The same thing happens for 
when the rider decelerates the skateboard.