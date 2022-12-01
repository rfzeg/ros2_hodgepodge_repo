#!/usr/bin/env python

"""
Rviz LaserScan filter and visualization markers
Author: Roberto Zegers R.
Copyright: Copyright (c) 2022, Roberto Zegers R.
License: BSD-3-Clause
Date: December 2022
"""
import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan


class LaserPointVisualizer(Node):

    def __init__(self):
        super().__init__('laser_point_visualizer')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.callback_scan,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # prevent unused variable warning
        self.subscription
        self.filtered_ranges = []
        self.filtered_intensities = []
        # initialize LaserScan message
        self.filtered_scan_msg = LaserScan()
        self.filtered_scan_publisher_ = self.create_publisher(
            LaserScan, '/filtered_scan', qos_profile_sensor_data)

        self.rviz_marker_publisher_ = self.create_publisher(
            Marker, '/scan_rviz_markers', qos_profile_sensor_data)
        # initialize Marker message
        self.marker_header = Header()
        self.marker_header.frame_id = ''
        self.riz_marker_msg = Marker(header=self.marker_header,
                                     ns="laser_scan_markers",
                                     id=0,
                                     type=Marker.POINTS,
                                     action=Marker.ADD,
                                     scale=Vector3(x=0.2, y=0.2, z=0.2),
                                     color=ColorRGBA(r=0.5, g=0., b=0.5, a=1.))

    def callback_scan(self, data):
        self.get_logger().debug('Number of laser ray range values: "%s"' % len(data.ranges))
        self.get_logger().debug('Number of laser ray intensity values: "%s"' %
                                len(data.intensities))

        self.filtered_ranges = copy.copy(data.ranges)
        self.filtered_intensities = copy.copy(data.intensities)

        # convert to list
        self.filtered_ranges = list(self.filtered_ranges)
        self.filtered_intensities = list(self.filtered_intensities)

        # filter out the laser rays based off fixed index values
        # use range(0, 0) to bypass this filter
        for x in range(0, 0):
            self.filtered_ranges[x] = 0.0
            self.filtered_intensities[x] = 0.0

        # read laser points from laser scan
        for index, distance in enumerate(self.filtered_ranges):
            angle_rad = index * (data.angle_max - data.angle_min) / \
                len(data.ranges) + data.angle_min
            # polar to cartesian coordinate values [x,y] w.r.t. laser sensor frame
            x_coordinate = distance * math.cos(angle_rad)
            y_coordinate = distance * math.sin(angle_rad)

            self.riz_marker_msg.points.append(
                Point(x=x_coordinate, y=y_coordinate))

        # fill header, then publish rviz marker
        self.riz_marker_msg.header.stamp = self.get_clock().now().to_msg()
        self.riz_marker_msg.header.frame_id = data.header.frame_id
        self.rviz_marker_publisher_.publish(self.riz_marker_msg)
        # clear points
        self.riz_marker_msg.points.clear()

        # fill message fields
        self.filtered_scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.filtered_scan_msg.header.frame_id = data.header.frame_id
        self.filtered_scan_msg.angle_min = data.angle_min
        self.filtered_scan_msg.angle_max = data.angle_max
        self.filtered_scan_msg.angle_increment = data.angle_increment
        self.filtered_scan_msg.range_min = data.range_min
        self.filtered_scan_msg.range_max = data.range_max
        self.filtered_scan_msg.ranges = self.filtered_ranges
        self.filtered_scan_msg.intensities = self.filtered_intensities
        # publish
        self.filtered_scan_publisher_.publish(self.filtered_scan_msg)

        self.filtered_ranges.clear()
        self.filtered_intensities.clear()


def main(args=None):
    rclpy.init(args=args)

    laser_point_visualizer = LaserPointVisualizer()
    rclpy.spin(laser_point_visualizer)

    # Destroy the node explicitly (optional)
    laser_point_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
