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

        self.frame_skip_count = 0
        self.skip_n_frames = 2

        # Suscripciones
        self.create_subscription(Int16, 'take_picture', self.trigger_callback, 10)
        self.create_subscription(Image, 'camara_dron/image_raw', self.camara_feed_callback, 10)

        # Publicador para el analizador
        self.img_dron_pub = self.create_publisher(Image, 'image_dron', 10)
        self.img_aruco_pub = self.create_publisher(Image, 'image_aruco', 10) 
        self.img_qr_pub = self.create_publisher(Image, 'image_qr', 10) 
        self.low_res_feed_pub = self.create_publisher(Image, 'low_res_feed', 10)

        # Asegurar que la carpeta de destino exista
        self.save_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'Fotos_dron', 'foto_original')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"Carpeta creada en: {self.save_dir}")

        self.get_logger().info("Nodo de Captura listo y esperando señal...")

    def camara_feed_callback(self, msg):
        # Guardamos el mensaje completo de la imagen
        self.latest_frame = msg

        self.frame_skip_count += 1
        if self.frame_skip_count >= self.skip_n_frames:
            try:
                # 1. Convertir a OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
                # 2. Redimensionar
                cv_image_resized = cv2.resize(cv_image, (640, 360))

                # 3. CONVERSIÓN CORRECTA A MENSAJE ROS
                # Es vital usar bridge.cv2_to_imgmsg para reconstruir el header
                low_res_msg = self.bridge.cv2_to_imgmsg(cv_image_resized, encoding="bgr8")
                low_res_msg.header = msg.header # Mantener el timestamp original
                
                self.low_res_feed_pub.publish(low_res_msg)
                self.frame_skip_count = 0 # Reiniciar contador
            except Exception as e:
                self.get_logger().error(f"Error en low_res: {e}")

    def trigger_callback(self, msg):
        try:
            frame_cv2 = self.bridge.imgmsg_to_cv2(self.latest_frame, desired_encoding='bgr8')
        
            filename = f"captura_dron_{self.count}.jpg"
            save_path = os.path.join(self.save_dir, filename)
            cv2.imwrite(save_path, frame_cv2)
            # self.get_logger().info(f"✅ Imagen {self.count} guardada en: {save_path}")
            
            self.count += 1

        except Exception as e:
            self.get_logger().error(f"Error al procesar/guardar imagen: {e}")

        if msg.data == 1:
            # Mandar la foto al qr_detector
            if self.latest_frame is not None:
                self.img_qr_pub.publish(self.latest_frame)
                self.get_logger().info("Foto enviada al analizador de qr")

            else:
                self.get_logger().warning("Se recibió señal de disparo pero no hay feed de cámara aún.")


        elif msg.data == 2:
            # Mandar la foto al aruco detector
            if self.latest_frame is not None:
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