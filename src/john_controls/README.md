# john_controls

This package is responsible for allocating thruster inputs, based on the computed forces in *navigation*. 


Planning:

Subscribes to nodes defined in navigation that gives us force input

Publishes to IMU driver/controls the drivers/imu/out (sensor_msgs/Imu) : IMU data: linear acceleration and angular velocity
