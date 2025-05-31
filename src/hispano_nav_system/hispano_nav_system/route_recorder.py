#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import os
from tf_transformations import euler_from_quaternion


class RouteRecorder(Node):
    def __init__(self):
        super().__init__('route_recorder')

        self.declare_parameter('ruta_salida', './src/hispano_nav_system/save_paths/ruta_guardada.txt')
        self.output_path = self.get_parameter('ruta_salida').get_parameter_value().string_value

        # Limpiar archivo
        try:
            with open(self.output_path, 'w') as f:
                f.write('')
            self.get_logger().info(f'Archivo sobrescrito: {self.output_path}')
        except Exception as e:
            self.get_logger().error(f'Error al limpiar el archivo: {e}')

        # Publicador en /initialpose
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Suscriptor temporal a /odom para inicializar pose automáticamente
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Suscriptor real para grabar ruta
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.first_pose_saved = False
        self.last_pose = None
        self.threshold = 0.2
        self.saved_coords = []
        self.initialized = False  # Marcador para no reusar la posición inicial más de una vez

        self.get_logger().info(f'Grabando coordenadas en: {self.output_path}')
        self.get_logger().info('Esperando posición inicial para AMCL...')

    def odom_callback(self, msg):
        if self.initialized:
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # AMCL espera "map" como frame de referencia

        pose_msg.pose.pose = msg.pose.pose  # Copia la pose actual desde odom

        # Opcional: ajustar covarianza para AMCL si es necesario
        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]


        self.initialpose_pub.publish(pose_msg)
        self.initialized = True
        self.get_logger().info('Posición inicial enviada automáticamente a AMCL.')
        self.destroy_subscription(self.odom_subscriber)

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(quat)

        if not self.first_pose_saved:
            self.save_pose(x, y, yaw)
            self.first_pose_saved = True
            return

        last_x, last_y, _ = self.last_pose
        dist = ((x - last_x) ** 2 + (y - last_y) ** 2) ** 0.5

        if dist > self.threshold:
            self.save_pose(x, y, yaw)

    def save_pose(self, x, y, yaw):
        self.saved_coords.append((x, y, yaw))
        self.last_pose = (x, y, yaw)

        with open(self.output_path, 'a') as f:
            f.write(f'{x},{y},{yaw}\n')

        self.get_logger().info(f'Guardado: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')


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
