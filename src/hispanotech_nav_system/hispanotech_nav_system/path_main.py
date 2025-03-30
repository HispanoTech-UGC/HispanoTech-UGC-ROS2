import rclpy
from rclpy.node import Node
from hispanotech_nav_system.path_recorder import PathRecorder
from hispanotech_nav_system.file_manager import FileManager
from hispanotech_nav_system.path_publisher import PathPublisher

class PathSystem(Node):
    def __init__(self):
        super().__init__('path_system')

        self.recorder = PathRecorder()
        self.file_manager = FileManager()
        self.publisher = PathPublisher()
        self.counter = 0

        # Crear un timer que se ejecuta a 1 Hz (1 vez por segundo)
        self.timer = self.create_timer(1.0, self.update_path)

    def update_path(self):
        """Captura la trayectoria y la publica cada segundo."""
        path = self.recorder.get_path()
        self.publisher.publish_path(path)

        # Guardar cada 10 puntos
        self.counter += 1
        if self.counter % 10 == 0:
            self.file_manager.save_path(path)
            self.get_logger().info("Ruta guardada exitosamente.")

def main(args=None):
    rclpy.init(args=args)
    node = PathSystem()
    rclpy.spin(node)  # Mantener el nodo activo
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
