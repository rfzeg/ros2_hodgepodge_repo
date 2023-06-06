# The encoder_ticks ROS2 package

- Author: Roberto Zegers

## Description

This package contains a ros2 node written in C++ that subscribes to the topic `/joint_states` of type `sensor_msgs/msg/JointState` and reads in the field `velocity`. It assumes that this field is a list of four numbers:  
- front_right_wheel_velocity
- rear_left_wheel_velocity
- rear_right_wheel_velocity
- front_left_wheel_velocity

The node calculates the encoder ticks for each wheel based on their velocities using the formula:  
`Encoder ticks = (wheel_velocity / (2 * pi * wheel_radius)) * ticks_per_rotation`
This program assumes 360 encoder ticks for one wheel rotation and a wheel_radius of 0.05 but these values are parameters that can be modified. 
The code accumulates the total ticks for each wheel since the program started and prints them out to the console.   


## Instructions

Build with:

```
cd ~/ros2_ws; colcon build --packages-select encoder_ticks; source install/setup.bash
```

Run it with:  

```
ros2 run encoder_ticks encoder_ticks
```


## Dependencies
- ROS2 Humble