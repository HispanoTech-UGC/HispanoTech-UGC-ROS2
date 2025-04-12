#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
import os

## @class WaypointFollowerClient
#  @brief Nodo ROS2 que carga una ruta de waypoints desde un archivo y la envía a un servidor de acción para que el robot la siga.
class WaypointFollowerClient(Node):

    ## @brief Constructor de la clase WaypointFollowerClient.
    #  Inicializa el cliente de acción, carga los waypoints y los envía al servidor.
    def __init__(self):
        super().__init__('waypoint_follower_client')

        # Declarar parámetro para path del archivo
        self.declare_parameter('ruta_waypoints', '/home/javier/HispanoTech-UGC-ROS2/src/hispanotech_nav_system/save_paths/ruta_guardada.txt')
        self.ruta_archivo = self.get_parameter('ruta_waypoints').get_parameter_value().string_value

        ## Cliente de acción para enviar la secuencia de waypoints
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Esperar a que el servidor esté disponible
        self._action_client.wait_for_server()
        self.get_logger().info('Servidor de acción follow_waypoints disponible.')

        # Ejecutar el envío de puntos
        self.send_waypoints()

    ## @brief Carga los waypoints desde un archivo de texto.
    #  @return Lista de objetos PoseStamped con las coordenadas leídas del archivo.
    def define_waypoints(self):
        """
        Carga waypoints desde un archivo de texto con coordenadas X,Y por línea.
        """
        waypoints = []

        if not os.path.exists(self.ruta_archivo):
            self.get_logger().error(f"Archivo no encontrado: {self.ruta_archivo}")
            return waypoints

        try:
            with open(self.ruta_archivo, 'r') as f:
                for line in f:
                    x_str, y_str = line.strip().split(',')
                    x = float(x_str)
                    y = float(y_str)

                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = self.get_clock().now().to_msg()

                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0  # orientación sin rotación

                    waypoints.append(pose)

            self.get_logger().info(f'{len(waypoints)} waypoints cargados desde archivo.')
        except Exception as e:
            self.get_logger().error(f"Error leyendo waypoints: {e}")

        return waypoints

    ## @brief Envía la lista de waypoints al servidor de acción.
    def send_waypoints(self):
        """
        Envía la lista de waypoints al servidor de acción follow_waypoints.
        """
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.define_waypoints()

        if not goal_msg.poses:
            self.get_logger().warn('No se enviaron waypoints porque la lista está vacía.')
            return

        self.get_logger().info(f'Enviando {len(goal_msg.poses)} waypoints...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    ## @brief Callback que maneja la respuesta del servidor al goal enviado.
    #  @param future Objeto future que contiene el goal_handle.
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('¡El goal fue rechazado!')
            return

        self.get_logger().info('Goal aceptado. Esperando resultado...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    ## @brief Callback que maneja la retroalimentación (feedback) del servidor de acción.
    #  @param feedback_msg Mensaje de feedback recibido durante la ejecución del goal.
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Waypoint actual: {feedback.current_waypoint}')

    ## @brief Callback que se ejecuta al finalizar la ejecución del goal.
    #  @param future Objeto future que contiene el resultado final.
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Waypoints completados: {result.missed_waypoints}')
        rclpy.shutdown()


## @brief Función principal. Inicializa el nodo y mantiene su ejecución.
#  @param args Argumentos opcionales de línea de comandos.
def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerClient()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
