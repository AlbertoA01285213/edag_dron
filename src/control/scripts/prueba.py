#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DroneTester(Node):
    def __init__(self):
        super().__init__('drone_tester')
        # Publicamos en el tópico que configuramos en el bridge
        self.publisher_ = self.create_publisher(Twist, '/dron/cmd_vel', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.up = True
        self.get_logger().info('Nodo DroneTester iniciado. ¡Despegamos!')

    def timer_callback(self):
        msg = Twist()
        
        if self.up:
            msg.linear.z = 1.0  # Sube a 1 m/s
            self.get_logger().info('Subiendo...')
        else:
            msg.linear.z = -1.0 # Baja a 1 m/s
            self.get_logger().info('Bajando...')
            
        self.publisher_.publish(msg)
        self.up = not self.up # Cambia de dirección cada 2 segundos

def main(args=None):
    rclpy.init(args=args)
    node = DroneTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Al apagar, mandamos velocidad 0 para que no siga volando
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.get_logger().info('Deteniendo dron...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# ros2 topic pub /dron/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}"