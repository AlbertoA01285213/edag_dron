#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode

class GzToRosBridge(Node):
    def __init__(self):
        super().__init__('gz_to_ros_bridge')
        
        # Publicador de ROS 2
        self.publisher_ = self.create_publisher(Image, 'camara_dron/image_raw', 10)
        self.bridge = CvBridge()

        # Suscriptor de Gazebo (Usando el transporte nativo de Gazebo)
        self.gz_node = GzNode()
        self.gz_node.subscribe(GzImage, "/camera_main", self.gz_callback)
        
        self.get_logger().info("🚀 Puente Gazebo -> ROS 2 iniciado (Full HD Soportado)")

    def gz_callback(self, msg: GzImage):
        try:
            # 1. Extraer imagen original de Gazebo (SIN REDIMENSIONAR)
            # El mensaje de Gazebo ya trae el width y height original del sensor
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            
            # Gazebo entrega RGB, ROS 2 suele preferir BGR para OpenCV
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # 2. Convertir directamente a mensaje de ROS 2
            ros_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            ros_msg.header.stamp = self.get_clock().now().to_msg()
            ros_msg.header.frame_id = "camera_link"

            # 3. Publicar
            self.publisher_.publish(ros_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error en el puente: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GzToRosBridge()
    
    # IMPORTANTE: Como Gazebo tiene su propio loop, 
    # necesitamos que ROS 2 se quede escuchando.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()