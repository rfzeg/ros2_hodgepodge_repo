import traceback as tb
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class GoToPoseController(Node):

    def __init__(self):
        """
        class constructor, initializes the Node object
        """
        super().__init__('go_to_pose_controller')
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

    def timer_callback(self):
        """
        timer callback method
        """
        # define a linear x-axis velocity
        self.cmd_vel_msg.linear.x = 0.2
        # define an angular z-axis velocity
        self.cmd_vel_msg.angular.z = 0.2
        # publish the message to the topic
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)

    def odom_callback(self, msg):
        """
        Odometry pose orientation in rad range of (-pi,pi)
        """
        self.robot_pose_in_xy_plane = self.pose_object_to_pose_2d(msg.pose)

        self.get_logger().info(
            f"Robot pose in '{msg.header.frame_id}' frame (x: {self.robot_pose_in_xy_plane[0]:.2f}, y: {self.robot_pose_in_xy_plane[1]:.2f}, theta: {self.robot_pose_in_xy_plane[2]:.2f})",
            throttle_duration_sec=0.5  # Throttle logging frequency to max 2Hz
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
