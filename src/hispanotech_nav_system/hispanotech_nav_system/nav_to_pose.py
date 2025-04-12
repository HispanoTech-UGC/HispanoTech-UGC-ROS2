from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('nav_to_pose_action_client')

        # Crear el cliente de acción para NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._action_client.wait_for_server(timeout_sec=10.0)

    def send_goal(self):
        # Crear el goal con la pose objetivo (ejemplo: sala de reuniones)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Coordenadas de ejemplo — ajústalas según la imagen
        goal_msg.pose.pose.position.x = 2.00792
        goal_msg.pose.pose.position.y = 7.5931
        goal_msg.pose.pose.position.z = 0.0

        # Orientación (en cuaterniones) — frente al frente, sin rotación
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Goal creado :)')

        self._action_client.wait_for_server()
        self.get_logger().info('Acción activa :)')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('Goal lanzado :)')

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded! 🥳')
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progreso: {feedback.current_pose.pose.position.x:.2f}, {feedback.current_pose.pose.position.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    action_client = NavToPoseActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
