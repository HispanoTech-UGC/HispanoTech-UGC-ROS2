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
        self.get_logger().info("🎮 JoyDebugger activo — mueve sticks o pulsa botones para ver el mapping")

    def joy_callback(self, msg):

        #ejes y botones
        x_btn = msg.buttons[0]      # boton x
        o_btn = msg.buttons[1]      # boton o
        tr_btn = msg.buttons[2]     # boton triángulo
        sq_btn = msg.buttons[3]     # boton cuadrado

        l1_btn = msg.buttons[4]     # boton L1
        r1_btn = msg.buttons[5]     # boton R1
        l2_btn = msg.buttons[6]     # boton L2 
        r2_btn = msg.buttons[7]     # boton R2

        start_btn = msg.buttons[9]  # boton start

        ji_btn = msg.buttons[11]    # joystick izquiqedo pulsar
        jd_btn = msg.buttons[12]    # joystick derecho pulsar

        fa_btn = msg.buttons[13]    # flecha arriba
        fb_btn = msg.buttons[14]    # flecha abajo
        fi_btn = msg.buttons[15]    # flecha izquierda
        fd_btn = msg.buttons[16]    # flecha derecha
       
        eje_izq_a_b = msg.axes[1]   # eje move izquierdo arriba abajo entre 1 y -1
        eje_izq_i_d = msg.axes[0]   # eje move izquierdo izquierda derecha 1 y -1 
        eje_der_a_b = msg.axes[4]   # eje move derecha   arriba abajo entre 1 y -1 
        eje_der_i_d = msg.axes[3]   # eje move derecha   izquierda derecha 1 y -1 

        '''
        # Mostrar ejes
        for i, axis in enumerate(msg.axes):
            if abs(axis) > 0.1:  # Ignora valores pequeños por ruido
                self.get_logger().info(f"🌀 Eje [{i}] → valor: {axis:.2f}")

        # Mostrar botones
        for i, btn in enumerate(msg.buttons):
            if btn:
                self.get_logger().info(f"🔘 Botón [{i}] pulsado")
        '''
        
        r2_pressure = max(0.0, (1.0 - r2_btn) / 2.0)  # Normaliza a 0.0 → 1.0 y asegura no valores negativos

        if r2_pressure <= 0.05:
            # No se mueve si no se pulsa
            self.robot_vel.publish(Twist())
            return

        # Escalar velocidades proporcionalmente a la presión de R2
        cmd_vel = Twist()
        cmd_vel.linear.x = eje_izq_i_d * self._linear_scale * r2_pressure
        cmd_vel.angular.z = eje_izq_a_b * self._angular_scale * r2_pressure

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
