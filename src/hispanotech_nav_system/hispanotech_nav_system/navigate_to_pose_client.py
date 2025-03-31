import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import sys  # Para leer argumentos desde la terminal
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionServer


class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self, x, y, w):
        """Envía un objetivo (goal) de navegación."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = w  # Orientación (1.0 para mantener dirección original)

        self.get_logger().info(f'📍 Enviando objetivo: X={x}, Y={y}, W={w}')
        
        # Esperar por el servidor de acción con timeout
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Servidor de navegación no disponible. Asegúrate de que nav2 está corriendo.")
            return

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Verifica si el objetivo fue aceptado."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('⚠️ Objetivo rechazado por el servidor.')
            return

        self.get_logger().info('✅ Objetivo aceptado, esperando resultado...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Se ejecuta cuando el robot llega al objetivo o falla."""
        result = future.result()
        if result:
            self.get_logger().info('🎯 Objetivo alcanzado con éxito.')
        else:
            self.get_logger().error('❌ No se pudo completar el objetivo.')

        # Una vez que el resultado ha sido recibido, cerramos el nodo
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Leer posición desde la terminal (X, Y, W)
    if len(sys.argv) < 4:
        print("⚠️ Uso: ros2 run hispanotech_nav_system navigate_to_pose_client X Y W")
        print("Ejemplo para volver al origen: ros2 run hispanotech_nav_system navigate_to_pose_client 0.0 0.0 1.0")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    w = float(sys.argv[3])

    client = NavigateToPoseClient()
    client.send_goal(x, y, w)

    # Mantener el nodo activo hasta que termine la acción
    executor = SingleThreadedExecutor()
    executor.add_node(client)
    try:
       rclpy.spin(client)
    except KeyboardInterrupt:
       client.get_logger().info("🛑 Cliente interrumpido.")
    finally:
       client.destroy_node()
       rclpy.shutdown()



if __name__ == '__main__':
    main()
