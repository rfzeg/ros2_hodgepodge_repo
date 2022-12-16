# The go_to_pose_controller ROS2 package

- Author: Roberto Zegers
- Date: December 2022
- License: BSD-3-Clause

## Description

- Minimal go to pose controller node (Python)
- Subscribes to `/goal_pose` topic (published for instance using Rviz2)
- Publishes to `/cmd_vel` topic
- Robot will turn in place until heading towards the goal, then move forward correcting its heading if neccesary
- Goal can be updated any time


## Instructions

Inside your ROS2 workspace root build with:  

```
colcon build --packages-select go_to_pose_controller
```

After compiling, source the workspace otherwise you will get the "Package 'go_to_pose_controller' not found" error message:  
```
source install/setup.bash 
```

Run with:  
```
ros2 launch go_to_pose_controller go_to_pose_controller.launch.py
```

To remap and display debug log messages use:  
```
ros2 run go_to_pose_controller go_to_pose_controller --ros-args -r odom:=mp_500/odom -r cmd_vel:=mp_500/cmd_vel --log-level DEBUG --log-level rcl:=INFO
```

## Dependencies
- ROS2 Galactic  