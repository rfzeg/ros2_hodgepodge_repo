import rclpy
from rclpy.node import Node
import copy
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile


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
        self.ranges_filter = []
        self.intensities_filter = []
        self.filterScan = LaserScan()

    def callback_scan(self, data):
        self.get_logger().debug('Number of laser ray range values: "%s"' % len(data.ranges))
        self.get_logger().debug('Number of laser ray intensity values: "%s"' %
                                len(data.intensities))


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
