#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import os

## @class RouteRecorder
#  @brief Nodo ROS2 que graba coordenadas del robot mientras se mueve, utilizando el tópico /amcl_pose.
class RouteRecorder(Node):
    ## @brief Constructor de la clase RouteRecorder.
    #  Declara parámetros, inicializa suscripciones y prepara el archivo de salida para guardar rutas.
    def __init__(self):
        super().__init__('route_recorder')

        # Parámetro para path del archivo
        self.declare_parameter('ruta_salida', './src/hispano_nav_system/save_paths/ruta_guardada.txt')
        self.output_path = self.get_parameter('ruta_salida').get_parameter_value().string_value

        # Limpiar archivo si existe (sobrescribir)
        try:
            with open(self.output_path, 'w') as f:
                f.write('')  # vacía el archivo
            self.get_logger().info(f'Archivo sobrescrito: {self.output_path}')
        except Exception as e:
            self.get_logger().error(f'Error al limpiar el archivo: {e}')

        ## Suscripción al tópico /amcl_pose para recibir la pose estimada del robot
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        ## Última pose registrada
        self.last_pose = None
        ## Umbral mínimo de distancia para registrar una nueva pose
        self.threshold = 0.2
        ## Lista de coordenadas guardadas
        self.saved_coords = []

        self.get_logger().info(f'Grabando coordenadas en: {self.output_path}')
        self.get_logger().info('Mueve el robot con teleop y se irán guardando automáticamente.')

    ## @brief Callback que se ejecuta al recibir un nuevo mensaje de pose.
    #  @param msg Mensaje recibido del tipo PoseWithCovarianceStamped.
    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_pose is None:
            self.save_pose(x, y)
            return

        last_x, last_y = self.last_pose
        dist = ((x - last_x)**2 + (y - last_y)**2)**0.5

        if dist > self.threshold:
            self.save_pose(x, y)

    ## @brief Guarda una nueva posición (x, y) en la lista y en el archivo de salida.
    #  @param x Coordenada X de la posición.
    #  @param y Coordenada Y de la posición.
    def save_pose(self, x, y):
        self.saved_coords.append((x, y))
        self.last_pose = (x, y)

        with open(self.output_path, 'a') as f:
            f.write(f'{x},{y}\n')

        self.get_logger().info(f'Guardado: x={x:.2f}, y={y:.2f}')


## @brief Función principal. Inicializa el nodo y mantiene la ejecución hasta interrupción del usuario.
#  @param args Argumentos opcionales de línea de comandos.
def main(args=None):
    rclpy.init(args=args)
    node = RouteRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Grabación interrumpida por el usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
