import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tugas1_msgs.msg import HandResult

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class Turtlesim(Node):
    def __init__(self):
        super().__init__("tugas1_turtlesim")
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        # hasil deteksi tangan
        self.subscription_ = self.create_subscription(
            HandResult, "/hand_direction", self.hand_callback, 10
        )

        # subscribe hasil kamera
        self.image_subscription_ = self.create_subscription(
            Image, "/mediapipe/image", self.image_callback, 10
        )
        
        self.br = CvBridge()
        self.get_logger().info('Turtlesim node initialized')

    def hand_callback(self, msg):
        pose = Twist()

        if msg.arahtangan == "down":
            pose.linear.y = -1.
        elif msg.arahtangan == "up":
            pose.linear.y = 1.
        elif msg.arahtangan == "left":
            pose.linear.x = -1.
        elif msg.arahtangan == "right":
            pose.linear.x = 1.
        else:
            pose.linear.x = 0.0
            pose.linear.y = 0.0

        self.publisher_.publish(pose)
        # self.get_logger().info(f'Hand direction: {msg.arahtangan}')


def main(args=None):
    rclpy.init(args=args)
    mediapipe_node = Turtlesim()
    rclpy.spin(mediapipe_node)
    
    cv2.destroyAllWindows()
    mediapipe_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()