#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class UdpToRos2(Node):
    def __init__(self):
        super().__init__('receptor_udp_node')
        
        # Configurar servidor UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 5600))
        self.sock.settimeout(1.0) # No quedarse trabado si no hay datos
        
        # Publicador de ROS 2 y herramienta de conversión
        self.publisher_ = self.create_publisher(Image, 'camara_dron/image_raw', 10)
        self.bridge = CvBridge()
        
        # Timer para revisar el puerto UDP a 30 FPS
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("📥 Escuchando puerto 5600 y publicando en camara_dron/image_raw")

    def timer_callback(self):
        try:
            # Recibir la foto JPEG del puerto
            data, addr = self.sock.recvfrom(65507)
            
            # Decodificar el JPEG a una imagen normal de OpenCV
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                # Convertir la imagen de OpenCV a mensaje de ROS 2 y publicarla
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(msg)
                
        except socket.timeout:
            pass # No llegó nada en este ciclo, no pasa nada
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UdpToRos2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()