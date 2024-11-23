import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageExtractorNode(Node):
    def __init__(self, image_topic, output_dir):
        super().__init__('image_extractor_node')
        self.image_topic = image_topic
        self.output_dir = output_dir
        self.bridge = CvBridge()

        # Create the output directory if it doesn't exist
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Create a timestamp-based filename
            timestamp = f"{msg.header.stamp.sec}{msg.header.stamp.nanosec:09d}"
            filename = os.path.join(self.output_dir, f"{timestamp}.jpg")

            # Save the image
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved image: {filename}")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Define the topic and output directory
    image_topic = '/camera/left/image'
    output_dir = 'images/mav0/cam0/data'

    # Start the node
    node = ImageExtractorNode(image_topic, output_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

