## Shijie Zhao, Fukang Liu
import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import Twist
import math


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            'center_coordination',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 5)
        self.image_height = 240
        self.image_width = 320
        self.angular_vel = 0.0
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)


    def listener_callback(self, msg):
        self.get_logger().info('I heard: {},{}'.format(msg.data[0],msg.data[1]))
        if abs(self.image_width//2 - msg.data[0]) <10:
            self.angular_vel = 0.0
        elif self.image_width//2 > msg.data[0]:
            self.angular_vel = 0.1
        else:
            self.angular_vel = -0.1
        twist = Twist()
        twist.angular.z = self.angular_vel
        self._vel_publish.publish(twist)



        # self.get_logger().info('I heard: "%s"' % msg.data)

    # def timer_callback(self):
    #     twist = Twist()
    #     twist.angular.z = 0.0
    #     self._vel_publish.publish(twist)
    #     self.get_logger().info('Publishing: "%s"' % twist.angular.z)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()