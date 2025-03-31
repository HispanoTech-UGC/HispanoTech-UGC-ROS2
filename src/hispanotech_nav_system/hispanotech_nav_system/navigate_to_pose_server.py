import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose

class NavigateToPoseServer(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_server')
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        """Proceso principal para alcanzar el objetivo."""
        self.get_logger().info(f'📍 Objetivo recibido: X={goal_handle.request.pose.pose.position.x}, '
                               f'Y={goal_handle.request.pose.pose.position.y}, '
                               f'W={goal_handle.request.pose.pose.orientation.w}')
        
        # Aquí puedes implementar la lógica para mover el robot al objetivo
        # Simulamos un retraso en la navegación
        self.get_logger().info('🚗 Navegando hacia el objetivo...')
        rclpy.sleep(5)  # Simulación de tiempo (reemplázalo con la lógica real)

        self.get_logger().info('🎯 Objetivo alcanzado.')
        result = NavigateToPose.Result()
        result.result = True
        goal_handle.succeed()

        return result

def main(args=None):
    rclpy.init(args=args)
    server = NavigateToPoseServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info("🛑 Servidor interrumpido.")
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
