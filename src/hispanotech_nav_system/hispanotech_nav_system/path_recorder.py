import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from collections import deque

class PathRecorder(Node):
    def __init__(self, max_length=1000):
        super().__init__('path_recorder')
        
        self.path_points = deque(maxlen=max_length)  # Almacenar puntos
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.pose_callback,
            10
        )
        self.subscription  # Evitar advertencia de variable no utilizada

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.path_points.append((x, y))
        self.get_logger().info(f"Guardado punto: ({x}, {y})")

    def get_path(self):
        return list(self.path_points)

    def clear_path(self):
        self.path_points.clear()
        self.get_logger().info("Historial de puntos borrado.")
