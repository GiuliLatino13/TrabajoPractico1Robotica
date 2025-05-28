import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.time import Duration
import time  # Importamos time para usar sleep

class SquarePath(Node):
    def __init__(self):
        super().__init__('square_path')
        
        # Publicador de velocidad
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parámetros ajustados para simulador
        self.side_length = 2.0  # metros
        self.linear_speed = 0.15  # REDUCIDO para simulador
        self.angular_speed = 0.3  # REDUCIDO para simulador
        self.duration_tolerance = 1.8  # Factor de compensación
        
        # Estado inicial
        self.current_action = 'movimiento_recto'
        self.action_start_time = self.get_clock().now()
        self.sides_completed = 0
        
        # Timer de control (10Hz)
        self.timer = self.create_timer(0.1, self.execute_path)
        
        self.get_logger().info('Nodo de cuadrado iniciado - Ajustado para simulador')

    def execute_path(self):
        twist = Twist()
        now = self.get_clock().now()
        elapsed = (now - self.action_start_time).nanoseconds / 1e9
        
        if self.current_action == 'movimiento_recto':
            twist.linear.x = self.linear_speed
            if elapsed >= (self.side_length / self.linear_speed) * self.duration_tolerance:
                self.switch_action('giro', now)
        
        elif self.current_action == 'giro':
            twist.angular.z = self.angular_speed
            if elapsed >= (1.5708 / self.angular_speed) * self.duration_tolerance:  # 90 grados
                self.sides_completed += 1
                if self.sides_completed < 4:
                    self.switch_action('movimiento_recto', now)
                else:
                    self.complete_square()
        
        self.publisher.publish(twist)

    def switch_action(self, new_action, time_now):
        # Detener brevemente entre acciones
        stop_twist = Twist()
        self.publisher.publish(stop_twist)
        time.sleep(0.5)  # Pausa de 0.5 segundos
        
        # Actualizar acción
        self.current_action = new_action
        self.action_start_time = self.get_clock().now()
        
        # Mensajes en español
        if new_action == 'movimiento_recto':
            self.get_logger().info('Iniciando movimiento recto')
        else:
            self.get_logger().info('Iniciando giro de 90 grados')

    def complete_square(self):
        # Detener el robot completamente
        stop_twist = Twist()
        self.publisher.publish(stop_twist)
        self.timer.cancel()
        self.get_logger().info('¡Cuadrado completado con éxito!')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SquarePath()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Detener el robot si hay interrupción
        stop_twist = Twist()
        node.publisher.publish(stop_twist)
        node.get_logger().info('Interrupción manual - Robot detenido')
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()