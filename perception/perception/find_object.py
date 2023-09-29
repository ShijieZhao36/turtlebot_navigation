# import rclpy
# from rclpy.node import Node
#
# from std_msgs.msg import UInt16MultiArray
#
# import cv2
# from perception import Detection
#
# class MinimalPublisher(Node):
#
#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher_ = self.create_publisher(UInt16MultiArray, 'topic', 10)
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0
#         self.cap = cv2.VideoCapture(0)
#
#     def timer_callback(self):
#         ret, frame = self.cap.read()
#         detect = Detection()
#         img = detect.process_frame(frame)
#         cv2.imshow("camera", img)
#         cv2.waitKey(1)
#
#         msg = UInt16MultiArray()
#         msg.data = [self.i,self.i+1]
#         self.publisher_.publish(msg)
#         self.get_logger().info('publishing: {},{}'.format(msg.data[0],msg.data[1]))
#         self.i += 1
#
#
# def main(args=None):
#     rclpy.init(args=args)
#
#     minimal_publisher = MinimalPublisher()
#
#     rclpy.spin(minimal_publisher)
#
#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()

#####################MyCode#############

# Bare Bones Code to View the Image Published from the Turtlebot3 on a Remote Computer
# Intro to Robotics Research 7785
# Georgia Institute of Technology
# Sean Wilson, 2022

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

from perception import Detection

class MinimalVideoSubscriber(Node):

    def __init__(self):
        # Creates the node.
        super().__init__('minimal_video_subscriber')

        # Set Parameters
        self.declare_parameter('show_image_bool', True)
        self.declare_parameter('window_name', "Raw Image")

        # Determine Window Showing Based on Input
        self._display_image = bool(self.get_parameter('show_image_bool').value)

        # Declare some variables
        self._titleOriginal = self.get_parameter('window_name').value  # Image Window Title
        if (self._display_image):
            # Set Up Image Viewing
            cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE)  # Viewing Window
            cv2.moveWindow(self._titleOriginal, 50, 50)  # Viewing Window Original Location


        # Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(depth=5)
        image_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        image_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        image_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
        self._video_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self._image_callback,
            image_qos_profile)
        self._video_subscriber  # Prevents unused variable warning.

    def _image_callback(self, CompressedImage):
        # The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        if (self._display_image):
            # Display the image in a window
            self.show_image(self._imgBGR)

    def get_image(self):
        return self._imgBGR

    def get_user_input(self):
        return self._user_input

    def show_image(self, img):
        detect = Detection()
        self.processed_img = detect.process_frame(img)
        cv2.imshow(self._titleOriginal, self.processed_img)
        # Cause a slight delay so image is displayed
        self._user_input = cv2.waitKey(50)  # Use OpenCV keystroke grabber for delay.Done
        if self.get_user_input() == ord('q'):
            cv2.destroyAllWindows()
            raise SystemExit
    def get_processed_frame(self):
        return self.processed_img


class MinimalPublisher(Node):
    def __init__(self,img):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.img = img

    def timer_callback(self):
        msg = CompressedImage()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main():
    rclpy.init()  # init routine needed for ROS2.
    video_subscriber = MinimalVideoSubscriber()  # Create class object to be used.

    try:
        rclpy.spin(video_subscriber)  # Trigger callback processing.
    except SystemExit:
        rclpy.logging.get_logger("Camera Viewer Node Info...").info("Shutting Donw")
    # Clean up and shutdown.
    video_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
