import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageRawSubscriberNode(Node):
    def __init__(self):
        super().__init__('image_raw_subscriber')

        self.frame_id = "camera_optical"

        # Create a subscriber to the Image topic
        self.subscription = self.create_subscription(
            Image,
            f'{self.frame_id}/image_raw',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Initialize CvBridge for converting ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        self.get_logger().info('image_raw subscriber node has started')

    def image_callback(self, msg):
        # self.get_logger().info('Receiving image frame')
        
        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display the image frame using OpenCV
        cv2.imshow('Camera Frame', frame)

        # Close the window when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Shutting down image subscriber')
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageRawSubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Image Subscriber Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
