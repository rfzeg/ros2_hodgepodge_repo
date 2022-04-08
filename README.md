# The my_package ROS2 package

- Author: Roberto Zegers

## Description

- Minimal Rviz marker publisher node (c++)
- Minimal laser-based obstacle avoidance node (c++)


## Instructions

Build with:

`colcon build --symlink-install`  

After compiling, source the workspace otherwise you will get the "Package 'my_package' not found" error message:

`source install/setup.bash`  

Launch using ROS2 XML launch file:  

`ros2 launch my_package launch_project.launch.xml`  


## Dependencies
- ROS2 Galactic  