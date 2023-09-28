import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16MultiArray

import cv2 
from perception import Detection

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(UInt16MultiArray, 'topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.cap = cv2.VideoCapture(0)

    def timer_callback(self):
        ret, frame = self.cap.read()
        detect = Detection()
        img = detect.process_frame(frame)
        cv2.imshow("camera", img)
        cv2.waitKey(1)

        msg = UInt16MultiArray()
        msg.data = [self.i,self.i+1]
        self.publisher_.publish(msg)
        self.get_logger().info('publishing: {},{}'.format(msg.data[0],msg.data[1]))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()