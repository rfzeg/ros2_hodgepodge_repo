#!/usr/bin/env python3
"""
Minimal go to pose controller node, ROS2
Implements alternating turn and forward movement sequences
Author: Roberto Zegers
Date: December 2022
License: BSD-3-Clause
"""

import traceback as tb
from math import pow, atan2, sqrt, pi

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class GoToPoseController(Node):

    def __init__(self):
        """
        class constructor, initializes the Node object
        """
        super().__init__('go_to_pose_controller')
        self.received_goal = False
        self.at_goal_position = False
        self.distance_tolerance = 0.2  # meters
        self.angular_tolerance = 0.2  # radians
        self.max_linear_vel = 0.4  # meters/sec

        # create the publisher object
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # define the timer period for 0.5 seconds
        timer_period = 0.5
        # create a timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cmd_vel_msg = Twist()

        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        # prevent unused variable warning
        self.odom_subscription

        # attribute to store odometry pose (x,y,yaw) in odometry frame
        self.robot_pose_in_xy_plane = None

        self.goal_subscription = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        # attribute to store goal pose (x,y,yaw) in goal pose frame
        self.goal_pose_in_xy_plane = None

    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((self.goal_pose_in_xy_plane[0] - self.robot_pose_in_xy_plane[0]), 2) +
                    pow((self.goal_pose_in_xy_plane[1] - self.robot_pose_in_xy_plane[1]), 2))

    def linear_vel(self, constant=1):
        """
        Linear velocity proportional controller
        limits maximal linear velocity
        """
        linear_velocity = constant * self.euclidean_distance()
        if linear_velocity >= self.max_linear_vel:
            return self.max_linear_vel
        return linear_velocity

    def steering_angle(self):
        """
        Find angle in radians from one pose to another in 2D space
        angle in the frame of reference of current_pose and pose to aim at
        atan2 returns angle in rad in the range of [-pi, pi)
        This is the angle the robot will travel.
        """
        delta_y = self.goal_pose_in_xy_plane[1] - \
            self.robot_pose_in_xy_plane[1]
        delta_x = self.goal_pose_in_xy_plane[0] - \
            self.robot_pose_in_xy_plane[0]

        desired_heading_angle = atan2(delta_y, delta_x)

        return desired_heading_angle

    def angular_vel(self, constant=0.3):
        """
        Angular velocity proportional controller
        """
        return constant * (self.steering_angle() - self.robot_pose_in_xy_plane[2])

    def timer_callback(self):
        """
        Guides the robot to the goal
        """
        # check if odometry has been received
        if self.robot_pose_in_xy_plane is None:
            return

        # check if goal pose has been received
        if self.received_goal:

            if not self.at_goal_position:
                # turn to goal
                if abs(self.steering_angle() - self.robot_pose_in_xy_plane[2]) > self.angular_tolerance:
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_msg.angular.z = self.angular_vel()
                # move forward controlling angular and linear velocity simultaneously
                else:
                    self.cmd_vel_msg.angular.z = self.angular_vel(0.5)
                    if self.euclidean_distance() >= self.distance_tolerance:
                        # use linear velocity proportional controller that also limits velocity to max_linear_vel
                        self.cmd_vel_msg.linear.x = self.linear_vel()
                    else:
                        # at goal position
                        self.cmd_vel_msg.linear.x = 0.0
                        self.at_goal_position = True

            if self.at_goal_position:
                # to-do: final alignment to desired heading angle
                self.get_logger().info('Robot arrived at goal pose.')
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.linear.y = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                # allow receiving new goal inputs once the robot arrived at the goal pose
                self.received_goal = False
                self.at_goal_position = False

            # publish message to move the robot
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)

            self.get_logger().debug('Output produced by the proportional controller:')
            self.get_logger().debug(
                f"Robot pose: \t X: {self.robot_pose_in_xy_plane[0]:.3f}, Y: {self.robot_pose_in_xy_plane[1]:.3f}, Theta: {self.robot_pose_in_xy_plane[2]:.3f}")
            self.get_logger().debug(
                f"Goal pose: \t X: {self.goal_pose_in_xy_plane[0]:.3f}, Y: {self.goal_pose_in_xy_plane[1]:.3f}, Theta: {self.goal_pose_in_xy_plane[2]:.3f}")
            self.get_logger().debug(
                f"cmd_vel_msg: \t .linear.x: {self.cmd_vel_msg.linear.x:.3f}, .linear.y: {self.cmd_vel_msg.linear.y:.3f}, .angular.z: {self.cmd_vel_msg.angular.z:.3f}")

    def odom_callback(self, msg):
        """
        Odometry pose orientation in rad range of (-pi,pi)
        """
        self.robot_pose_in_xy_plane = self.pose_object_to_pose_2d(msg.pose)

        self.get_logger().debug(
            f"Robot pose in '{msg.header.frame_id}' frame (x: {self.robot_pose_in_xy_plane[0]:.2f}, y: {self.robot_pose_in_xy_plane[1]:.2f}, theta: {self.robot_pose_in_xy_plane[2]:.2f})",
            throttle_duration_sec=0.5  # Throttle logging frequency to max 2Hz
        )

    def goal_callback(self, pose_msg):
        """
        Goal pose orientation in rad in range of (-pi,pi)
        """
        self.goal_pose_in_xy_plane = self.pose_object_to_pose_2d(pose_msg)
        self.received_goal = True
        self.get_logger().info(
            f"Received goal pose. Frame: '{pose_msg.header.frame_id}', x: {self.goal_pose_in_xy_plane[0]:.2f}, y: {self.goal_pose_in_xy_plane[1]:.2f}, theta: {self.goal_pose_in_xy_plane[2]:.2f}"
        )

    @staticmethod
    def pose_object_to_pose_2d(ros_pose_obj):
        quaternion = (
            ros_pose_obj.pose.orientation.x,
            ros_pose_obj.pose.orientation.y,
            ros_pose_obj.pose.orientation.z,
            ros_pose_obj.pose.orientation.w
        )

        # convert quaternion to euler angles
        _, _, yaw = euler_from_quaternion(quaternion)

        pose_in_plane = (
            # x position in meters (pose obj. frame)
            ros_pose_obj.pose.position.x,
            # x position in meters (pose obj. frame)
            ros_pose_obj.pose.position.y,
            # orientation in radians (pose obj. frame)
            yaw
        )

        return pose_in_plane

    def stop(self):
        cmd_vel = Twist()
        self.cmd_vel_publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    go_to_pose_controller = GoToPoseController()
    try:
        rclpy.spin(go_to_pose_controller)
    except KeyboardInterrupt:
        print("User requested shutdown.")
    except BaseException as e:
        print(f"Some error had occured: {e}")
        tb.print_exc()
    # ensure robot stops before exiting
    print("Stopping robot.")
    go_to_pose_controller.stop()
    go_to_pose_controller.destroy_node()


if __name__ == '__main__':
    main()
