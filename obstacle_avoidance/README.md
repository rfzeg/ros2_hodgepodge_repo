# The obstacle_avoidance ROS2 package

- Author: Roberto Zegers

## Description

- Minimal laser-based obstacle avoidance node (c++)


## Instructions

Build with:

`colcon build`   

After compiling, source the workspace otherwise you will get the "Package 'obstacle_avoidance' not found" error message:

`source install/setup.bash`  

Launch using ROS2 XML launch file:  

`ros2 launch obstacle_avoidance launch_project.launch.xml`  


## Dependencies
- ROS2 Galactic  