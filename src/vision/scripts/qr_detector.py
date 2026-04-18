#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import os
import sqlite3
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector_node')

        self.bridge = CvBridge()
        self.create_subscription(Pose, 'pose_dron', self.pose_callback, 10)
        self.create_subscription(Image, 'image_qr', self.camara_feed_callback, 10)

        self.pose_actual = [0.0]*2

        self.cajon = 1
        self.img_counter = 0

        db_dir = os.path.expanduser('~/Documents/edag_dron/src/master/config')
        if not os.path.exists(db_dir):
            os.makedirs(db_dir)

        self.db_path = os.path.join(db_dir, 'edag_db.db')


        self. save_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'Fotos_dron', 'foto_qr')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"Carpeta creada en: {self.save_dir}")


        self.conn = sqlite3.connect(self.db_path)
        self.cursor = self.conn.cursor()
        self.setup_db()

        self.get_logger().info("Detector de QR listo")



    def camara_feed_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detector = cv2.QRCodeDetector()

            h, w, _ = cv_image.shape

            data, points, _ = detector.detectAndDecode(cv_image)

            if not data:
                self.get_logger().warn(f"⚠️ No se detectó QR en el intento {self.img_counter}")
                self.cajon += 1
                return
            
            qr_center_x = np.mean(points[0][:, 0])
            
            # Definimos el margen central (ejemplo: solo aceptar en el 40% central)
            # Esto ignora el 30% de la izquierda y el 30% de la derecha
            limite_izquierdo = 0.25
            limite_derecho = 0.75

            if qr_center_x < w*limite_izquierdo or qr_center_x > w*limite_derecho:
                self.get_logger().info(f"⏭️ Ignorando QR lateral en X={qr_center_x:.1f} (fuera de {limite_izquierdo:.1f}-{limite_derecho:.1f})")
                return
            
            query = "INSERT INTO carros (cajon, x, y, lat, long, info) VALUES (?, ?, ?, ?, ?, ?)"
            valores = (self.cajon, round(self.pose_actual[0], 3), round(self.pose_actual[1], 3), 0.0, 0.0, data)
            
            self.cursor.execute(query, valores)
            self.conn.commit() # Guardar cambios en disco

            self.get_logger().info(f"✅ QR Detectado: {data} | Guardado en DB")

            self.process_img(cv_image, points, data, limite_izquierdo)

            self.cajon += 1

        except Exception as e:
            self.get_logger().error(f"Error en camara_feed_callback: {e}")

    def setup_db(self):
        """Crea la tabla si no existe"""
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS carros (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                cajon INTEGER,
                x REAL,
                y REAL,
                lat REAL,
                long REAL,
                info TEXT
            )
        """)

        self.cursor.execute("DELETE FROM carros")
        self.cursor.execute("DELETE FROM sqlite_sequence WHERE name='carros'")
        
        self.conn.commit()
        # self.get_logger().info("🧹 Base de datos limpiada para nueva misión.")


    def pose_callback(self, msg):
        try:
            self.pose_actual[0] = msg.position.x
            self.pose_actual[1] = msg.position.y

        except Exception as e:
            self.get_logger().error(f"Error en pose_callback: {e}")

    def process_img(self, img, points, content, limit_ratio):
        debug_img = img.copy()

        h, w, _ = img.shape
        center_img = (w // 2, h // 2)
        c = points[0][0]


        if points is not None:
            pts = points.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(debug_img, [pts], isClosed=True, color=(0, 255, 0), thickness=4)
        
        textos = [
            f"Cajon: {self.cajon}",
            f"Info: {content}",
            f"X: {self.pose_actual[0]:.2f}", 
            f"Y: {self.pose_actual[1]:.2f}"
        ]

        x_izq = int(w * limit_ratio)
        x_der = int(w * (1 - limit_ratio))

        cv2.line(debug_img, (x_izq, 0), (x_izq, h), (0, 0, 255), thickness=2)
        cv2.line(debug_img, (x_der, 0), (x_der, h), (0, 0, 255), thickness=2)

        cv2.drawMarker(debug_img, center_img, (0, 255, 0), cv2.MARKER_CROSS, 20,2)
        for i, txt in enumerate(textos):
            cv2.putText(debug_img, txt, (10, 30 + (i * 25)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


        file_path = os.path.join(self.save_dir, f'qr_debug_{self.img_counter}.jpg')
        cv2.imwrite(file_path, debug_img)
        self.img_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()