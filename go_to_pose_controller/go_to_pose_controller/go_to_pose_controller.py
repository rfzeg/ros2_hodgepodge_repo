import traceback as tb
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


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
