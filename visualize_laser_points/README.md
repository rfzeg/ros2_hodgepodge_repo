# The visualize_laser_points ROS2 package

- Author: Roberto Zegers

## Description

- Visualize laser points from obstacle as Rviz marker points (python)


## Instructions

Build with:

`colcon build`  

After compiling, source the workspace otherwise you will get the "Package 'visualize_laser_points' not found" error message:

`source install/setup.bash`  

Launch using ROS2 XML launch file:  

`ros2 launch visualize_laser_points launch_project.launch.xml`  


## Dependencies
- ROS2 Galactic  