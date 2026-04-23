#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose # Cambiamos a Pose para tener más campos
from cv_bridge import CvBridge

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, 'image_aruco', self.image_callback, 10)
        self.error_pub = self.create_publisher(Pose, 'aruco_error', 10) # Ahora publicamos Pose

        # Configuración ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 4  # Más pequeño = más intentos de umbral (más lento pero más robusto)
        self.aruco_params.minMarkerPerimeterRate = 0.02 # Para detectar ArUcos pequeños/lejanos
        self.aruco_params.errorCorrectionRate = 0.6     # Más tolerancia a bits erróneos
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.foto_buffer = [] 
        self.max_intentos = 2

        # Parámetro de diseño: ¿Cuánto debe medir el lado del ArUco en pixeles?
        self.target_pixel_size = 300.0 
        self.img_counter = 0

        self.save_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'Fotos_dron', 'foto_aruco')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"Carpeta creada en: {self.save_dir}")

        self.get_logger().info("Detector 4D (X, Y, Z, Yaw) listo.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            h, w, _ = cv_image.shape
            center_img = (w // 2, h // 2)

            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            enhanced_gray = clahe.apply(gray)
            proc_image = cv2.cvtColor(enhanced_gray, cv2.COLOR_GRAY2BGR)

            corners, ids, _ = self.detector.detectMarkers(proc_image)
            error_msg = Pose()

            debug_img = cv_image.copy()

            if ids is not None:
                self.process_and_publish(cv_image, corners, ids)
                self.foto_buffer = [] # Limpiamos buffer

            else:
                # self.foto_buffer.append(False)
                self.get_logger().warn(f"Intento {len(self.foto_buffer)} fallido.")

                error_msg = Pose()
                error_msg.position.x = 999.0
                error_msg.position.y = 999.0
                error_msg.position.z = 999.0
                self.error_pub.publish(error_msg)

                # Solo si fallamos N veces, mandamos el 999
                # if len(self.foto_buffer) >= self.max_intentos:
                #     error_msg = Pose()
                #     error_msg.position.x = 999.0
                #     self.error_pub.publish(error_msg)
                #     self.get_logger().error("ArUco no detectado tras múltiples intentos.")
                #     self.foto_buffer = []

                file_path = os.path.join(self.save_dir, f'error_aruco_debug_{self.img_counter}.jpg')
                cv2.imwrite(file_path, debug_img)

                self.img_counter += 1

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def process_and_publish(self, img, corners, ids):
        debug_img = img.copy()

        h, w, _ = img.shape
        center_img = (w // 2, h // 2)
        c = corners[0][0]

        error_msg = Pose()
  
        # 1. Centro del ArUco (Errores X, Y)
        aruco_center_x = np.mean(c[:, 0])
        aruco_center_y = np.mean(c[:, 1])
        error_msg.position.x = float((aruco_center_x - center_img[0]) / (w / 2))
        error_msg.position.y = float((center_img[1] - aruco_center_y) / (h / 2))

        current_size = (np.linalg.norm(c[0]-c[1]) + np.linalg.norm(c[1]-c[2])) / 2
        error_msg.position.z = float((self.target_pixel_size - current_size) / self.target_pixel_size)

        self.error_pub.publish(error_msg)


        cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)
        cv2.drawMarker(debug_img, center_img, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        textos = [
            f"ID: {ids[0][0]}",
            f"Err X: {error_msg.position.x:.2f}",
            f"Err Y: {error_msg.position.y:.2f}",
            f"Size: {current_size:.1f}px",
            f"Err Z: {error_msg.position.z:.2f}"
        ]
        for i, txt in enumerate(textos):
            cv2.putText(debug_img, txt, (10, 30 + (i * 25)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Guardar la imagen con la máscara
        file_path = os.path.join(self.save_dir, f'aruco_debug_{self.img_counter}.jpg')
        cv2.imwrite(file_path, debug_img)
        self.img_counter += 1
        

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArucoDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()