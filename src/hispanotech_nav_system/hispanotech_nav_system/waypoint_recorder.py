import rclpy
import csv
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_srvs.srv import Trigger
from nav_msgs.msg import Path

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        # Suscribirse a la posición del robot en /amcl_pose
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # Servicio para guardar manualmente un waypoint
        self.srv = self.create_service(Trigger, 'save_waypoint', self.save_waypoint_callback)

        # Publicador para visualizar los waypoints en RViz
        self.path_pub = self.create_publisher(Path, '/waypoints_path', 10)

        # Timer para guardar automáticamente cada 20 segundos
        self.timer = self.create_timer(20.0, self.auto_save_waypoint)

        # Lista de waypoints
        self.waypoints = []
        self.last_position = None
        self.epsilon = 0.2  # Margen para detectar movimiento

        # Directorio para guardar el CSV
        self.save_dir = os.path.expanduser("~/HispanoTech-UGC-ROS2/src/hispanotech_nav_system/save_paths")
        os.makedirs(self.save_dir, exist_ok=True)
        self.csv_path = os.path.join(self.save_dir, "waypoints.csv")

        self.get_logger().info("Waypoint Recorder iniciado")

    def pose_callback(self, msg):
        """Recibe la posición actual del robot y guarda un waypoint si ha cambiado de posición."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_position is None or self._has_moved(x, y):
            self.last_position = (x, y)
            self.waypoints.append((x, y))
            self.get_logger().info(f"Nuevo waypoint guardado: X={x:.2f}, Y={y:.2f}")
            self.publish_waypoints()

    def _has_moved(self, x, y):
        """Verifica si el robot se ha movido significativamente."""
        if self.last_position is None:
            return True
        last_x, last_y = self.last_position
        return abs(last_x - x) > self.epsilon or abs(last_y - y) > self.epsilon

    def auto_save_waypoint(self):
        """Guarda automáticamente un waypoint cada 20 segundos."""
        if self.last_position:
            self.waypoints.append(self.last_position)
            self.get_logger().info(f"Waypoint guardado automáticamente: {self.last_position}")
            self.save_to_csv()
            self.publish_waypoints()

    def save_waypoint_callback(self, request, response):
        """Guarda manualmente un waypoint."""
        if not self.waypoints:
            response.success = False
            response.message = "No hay waypoints registrados aún."
            return response

        self.save_to_csv()
        response.success = True
        response.message = f"Waypoints guardados en {self.csv_path}"
        self.get_logger().info(response.message)
        return response

    def save_to_csv(self):
        """Guarda la lista de waypoints en un archivo CSV."""
        with open(self.csv_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["X", "Y"])
            writer.writerows(self.waypoints)
        self.get_logger().info(f"Waypoints guardados en '{self.csv_path}'.")

    def publish_waypoints(self):
        """Publica los waypoints en un tópico para RViz."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Waypoints publicados en /waypoints_path")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
