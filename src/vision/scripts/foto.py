#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import os
from cv_bridge import CvBridge
from std_msgs.msg import Int16
from sensor_msgs.msg import Image

class Picture(Node):
    def __init__(self):
        super().__init__('Picture_node')
        self.bridge = CvBridge()
        self.count = 0
        self.latest_frame = None  # Aquí guardaremos el objeto de imagen de ROS

        # Suscripciones
        self.create_subscription(Int16, 'take_picture', self.trigger_callback, 10)
        self.create_subscription(Image, 'camara_dron/image_raw', self.camara_feed_callback, 10)

        # Publicador para el analizador
        self.img_dron_pub = self.create_publisher(Image, 'image_dron', 10)
        self.img_aruco_pub = self.create_publisher(Image, 'image_aruco', 10) 

        # Asegurar que la carpeta de destino exista
        self.save_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'Fotos_dron', 'foto_original')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"Carpeta creada en: {self.save_dir}")

        self.get_logger().info("Nodo de Captura listo y esperando señal...")

    def camara_feed_callback(self, msg):
        # Guardamos el mensaje completo de la imagen
        self.latest_frame = msg

    def trigger_callback(self, msg):
        # Si recibimos un 1 y tenemos una imagen guardada en el buffer
        if msg.data == 1:
            if self.latest_frame is not None:
                # 1. Publicar la imagen al analizador inmediatamente
                self.img_dron_pub.publish(self.latest_frame)
                self.get_logger().info("Foto enviada al analizador")

                try:
                    # 2. Convertir a formato OpenCV para guardar en disco
                    frame_cv2 = self.bridge.imgmsg_to_cv2(self.latest_frame, desired_encoding='bgr8')
                    
                    filename = f"captura_dron_{self.count}.jpg"
                    save_path = os.path.join(self.save_dir, filename)

                    # 3. Guardar imagen
                    cv2.imwrite(save_path, frame_cv2)
                    # self.get_logger().info(f"✅ Imagen {self.count} guardada en: {save_path}")
                    
                    self.count += 1
                except Exception as e:
                    self.get_logger().error(f"Error al procesar/guardar imagen: {e}")
            else:
                self.get_logger().warning("Se recibió señal de disparo pero no hay feed de cámara aún.")

        elif msg.data == 2:
            if self.latest_frame is not None:
                # 1. Publicar la imagen al analizador inmediatamente
                self.img_aruco_pub.publish(self.latest_frame)
                self.get_logger().info("Foto enviada al analizador de aruco")

            else:
                self.get_logger().warning("Se recibió señal de disparo pero no hay feed de cámara aún.")


def main(args=None):
    rclpy.init(args=args)
    node = Picture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()