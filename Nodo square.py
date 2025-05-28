##### Codigo Nodo square_path:
 ```python
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class SquarePath(Node):

    def __init__(self):
        super().__init__('square_path')

        # Publicador del nodo
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Variables de velocidad
        self.twist_v = Twist()

        # Spin en un hilo separado
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

        self.rate_freq = 10.0
        self.rate = self.create_rate(self.rate_freq)

        # Espera la conexión del suscriptor
        while self.vel_pub.get_subscription_count() == 0:
            self.get_logger().info('Esperando conexión...')
            time.sleep(0.5)

        self.get_logger().info('Ejecutando camino cuadrado...')

        # Ejecuta el camino cuadrado
        try:
            for _ in range(4):
                # Avanzar 2 metros (a 1 m/s durante 2 segundos)
                self.twist_v.linear.x = 1.0
                self.twist_v.angular.z = 0.0
                self.vel_pub.publish(self.twist_v)
                time.sleep(4.5)       #Deberia ser 2, pero se coloca este valor por demoras en la simulacion

                # Detener momentáneamente
                self.twist_v.linear.x = 0.0
                self.vel_pub.publish(self.twist_v)
                time.sleep(0.2)

                # Girar 90° (pi/2 rad) a 0.5 rad/s -> demora pi/2 / 0.5 ≈ 3.14 s
                self.twist_v.linear.x = 0.0
                self.twist_v.angular.z = 0.5
                self.vel_pub.publish(self.twist_v)
                time.sleep(6.30)      #Deberia ser 3.14, pero se coloca este valor por demoras en la simulacion

                # Detener
                self.twist_v.angular.z = 0.0
                self.vel_pub.publish(self.twist_v)
                time.sleep(0.2)

            self.get_logger().info('Camino cuadrado completado.')

        except KeyboardInterrupt:
            self.get_logger().info('Interrumpido por el usuario.')
            self.twist_v.linear.x = 0.0
            self.twist_v.angular.z = 0.0
            self.vel_pub.publish(self.twist_v)

def main(args=None):
    rclpy.init(args=args)
    square_path = SquarePath()
    rclpy.spin(square_path)

if __name__ == '__main__':
    main()
 ```