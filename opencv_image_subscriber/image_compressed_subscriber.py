import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageCompressedSubscriberNode(Node):
    def __init__(self):
        super().__init__('image_compressed_subscriber')

        self.frame_id = "camera_optical"

        # Create a subscriber for CompressedImage messages
        self.subscription = self.create_subscription(
            CompressedImage,
            f'{self.frame_id}/image_raw/compressed',
            self.image_callback,
            10
        )

        # Initialize CvBridge for converting ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        self.get_logger().info('image_compressed subscriber node has started')

    def image_callback(self, msg):
        try:
            # self.get_logger().info('Receiving compressed image frame')

            # Decode the compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Display the raw image
            cv2.imshow("Raw Image", image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to decode and display image: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressedSubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Compressed Image Subscriber Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()