import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tugas1_msgs.msg import HandResult


class Turtlesim(Node):
    def __init__(self):
        super().__init__("tugas1_turtlesim")
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.subscription_ = self.create_subscription(
            HandResult, "/hand_direction", self.hand_callback, 10
        )

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

        self.publisher_.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    mediapipe_node = Turtlesim()
    rclpy.spin(mediapipe_node)
    mediapipe_node.destroy_node()
    rclpy.shutdown()
