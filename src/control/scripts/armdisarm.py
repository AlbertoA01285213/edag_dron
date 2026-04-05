#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand

class DronTester(Node):

    def __init__(self):
        super().__init__('dron_tester_node')
        
        # Publicador para enviarle comandos a PX4
        self.command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            10
        )
        
        # Variable para saber el estado actual
        self.dron_armado = False
        
        # Timer que se ejecuta cada 10 segundos
        self.timer = self.create_timer(10.0, self.timer_callback)
        self.get_logger().info('=== Nodo de prueba iniciado. Esperando 10 segundos... ===')

    def timer_callback(self):
        if not self.dron_armado:
            self.enviar_comando(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info('>>> Enviando comando de ARMAR dron...')
            self.dron_armado = True
        else:
            self.enviar_comando(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            self.get_logger().info('<<< Enviando comando de DESARMAR dron...')
            self.dron_armado = False

    def enviar_comando(self, command, param1=0.0):
        """Metodo auxiliar para llenar el mensaje de comando de PX4"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1  # 1.0 = Armar, 0.0 = Desarmar
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        # PX4 requiere un timestamp en microsegundos
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DronTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
