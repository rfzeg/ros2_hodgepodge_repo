import rclpy
from rclpy.node import Node
import copy
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile, qos_profile_sensor_data


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
        # initialize message
        self.filtered_scan_msg = LaserScan()
        self.filtered_scan_publisher_ = self.create_publisher(
            LaserScan, '/filtered_scan', qos_profile_sensor_data)

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
        for x in range(30, 120):
            self.filtered_ranges[x] = 0.0
            self.filtered_intensities[x] = 0.0

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

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laser_point_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
