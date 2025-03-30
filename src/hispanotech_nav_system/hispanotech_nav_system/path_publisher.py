import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.path_pub = self.create_publisher(Path, "/path_traj", 10)
    
    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for x, y in path_points:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Trayectoria publicada en /path_traj")

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)  # Mantiene el nodo ejecutándose
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
