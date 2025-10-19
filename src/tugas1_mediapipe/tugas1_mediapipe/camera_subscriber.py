import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Mediapipe(Node):
    def __init__(self):
        super().__init__('mediapipe')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  
            self.listener_callback,
            10)
        self.subscription  
        self.br = CvBridge()
        self.get_logger().info('sub to camera')

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    mediapipe_subscriber = Mediapipe()
    rclpy.spin(mediapipe_subscriber)
    mediapipe_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
