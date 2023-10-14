## Shijie Zhao, Fukang Liu
import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import LaserScan
import math


class LaserSubscriber(Node):

    def __init__(self):
        super().__init__('laser_subscriber')
        self.subscription = self.create_subscription(
           LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)


    def listener_callback(self, msg):
        self.get_logger().info('I heard: {}'.format(len(msg.ranges)))




        # self.get_logger().info('I heard: "%s"' % msg.data)

    # def timer_callback(self):
    #     twist = Twist()
    #     twist.angular.z = 0.0
    #     self._vel_publish.publish(twist)
    #     self.get_logger().info('Publishing: "%s"' % twist.angular.z)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = LaserSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()