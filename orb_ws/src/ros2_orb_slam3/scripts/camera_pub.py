#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import time


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_pub')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.timer = self.create_timer(1.0 / 30.0, self.publish_image)  # Publish at 30 FPS (0.0333s)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            exit(1)
        
        self.get_logger().info("Publishing camera images")

    def publish_image(self):
        # Capture a frame from the camera
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return
        
        # Convert the frame to a ROS Image message
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Create a header for the image message and set the frame_id
        ros_image.header = Header()
        ros_image.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
        ros_image.header.frame_id = "camera"  # Set the frame_id
        
        self.publisher_.publish(ros_image)


    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_pub = CameraPublisher()
    rclpy.spin(camera_pub)
    camera_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
