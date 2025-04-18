from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyVelocityPublisher(Node):

    def __init__(self):
        super().__init__('joy_velocity_publisher')
        qos_profile = QoSProfile(depth=10)

        self._linear_scale = 0.4
        self._angular_scale = 1.5

        self.joy_status = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile)
        self.robot_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        self.get_logger().info(f"{self.get_name()} started")

    def joy_callback(self, msg):
        # Eje R2 analógico — comúnmente en axes[5] (ajusta si es otro)
        r2_axis_raw = msg.axes[5]  # 1.0 (no pulsado) → -1.0 (pulsado)
        r2_pressure = max(0.0, (1.0 - r2_axis_raw) / 2.0)  # Normaliza a 0.0 → 1.0 y asegura no valores negativos

        if r2_pressure <= 0.05:
            # No se mueve si no se pulsa
            self.robot_vel.publish(Twist())
            return

        # Movimiento con stick izquierdo
        LSTICK_x_axis = msg.axes[0]  # izquierda/derecha
        LSTICK_y_axis = msg.axes[1]  # adelante/atrás

        # Escalar velocidades proporcionalmente a la presión de R2
        cmd_vel = Twist()
        cmd_vel.linear.x = LSTICK_y_axis * self._linear_scale * r2_pressure
        cmd_vel.angular.z = LSTICK_x_axis * self._angular_scale * r2_pressure

        self.robot_vel.publish(cmd_vel)

def main(args=None):
    try:
        rclpy.init(args=args)
        joy_velocity_publisher = JoyVelocityPublisher()
        rclpy.spin(joy_velocity_publisher)
    except KeyboardInterrupt: 
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
