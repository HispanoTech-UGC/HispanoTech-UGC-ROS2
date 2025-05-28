#!/usr/bin/env python3

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# import tf_transformations
import os


class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('nav_to_pose_action_client')

        # Espera al servidor de accion
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._action_client.wait_for_server(timeout_sec=10.0)

        # Ruta al archivo de coordenadas
        self.declare_parameter('ruta_waypoints', './src/my_nav2_system/config/ruta_guardada.txt')
        ruta_archivo = self.get_parameter('ruta_waypoints').get_parameter_value().string_value

        self.poses = self.load_goals_from_file(ruta_archivo)
        self.goal_index = 0

        if self.poses:
            self.get_logger().info(f'{len(self.poses)} metas cargadas.')
            self.send_next_goal()
        else:
            self.get_logger().error('No se cargaron metas. Verifica el archivo.')

    def load_goals_from_file(self, filepath):
        poses = []
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    if line.strip():
                        x, y, yaw = map(float, line.strip().split(','))
                        poses.append((x, y, yaw))
        except Exception as e:
            self.get_logger().error(f'Error al leer archivo de ruta: {e}')
        return poses

    def send_next_goal(self):
        if self.goal_index >= len(self.poses):
            self.get_logger().info('🚩 Todas las metas fueron completadas.')
            return

        x, y, yaw = self.poses[self.goal_index]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = yaw

        # Convertir yaw a cuaterniones
        # q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'➡️ Enviando meta #{self.goal_index + 1}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'❌ Meta #{self.goal_index + 1} rechazada.')
            return

        self.get_logger().info(f'✅ Meta #{self.goal_index + 1} aceptada.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'🎯 Meta #{self.goal_index + 1} completada.')
        else:
            self.get_logger().warn(f'⚠️ Meta #{self.goal_index + 1} falló con estado: {status}')

        self.goal_index += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        x = feedback.current_pose.pose.position.x
        y = feedback.current_pose.pose.position.y
        self.get_logger().info(f'📍 Progreso: x={x:.2f}, y={y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    action_client = NavToPoseActionClient()
    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
