#!/usr/bin/env python3
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix  # o el tipo que uses para odometría
from PySide6.QtGui import QPixmap, QImage
from PySide6.QtCore import Signal, QThread, Qt, Slot
from PySide6.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                             QVBoxLayout, QHBoxLayout, QWidget, QStackedWidget, 
                             QLabel, QFrame, QGridLayout)
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

class RosWorker(QThread):
    # Señales para enviar datos a la GUI
    image_signal = Signal(QPixmap)          # Imagen lista para mostrar
    odom_signal = Signal(str)               # Texto con posición (o coordenadas separadas)
    status_signal = Signal(str)             # Para logs/estado

    def __init__(self):
        super().__init__()
        self.node = None
        self.bridge = CvBridge()
        self.executor = None

    def run(self):
        # Inicializar ROS 2 en este hilo
        rclpy.init(args=None)
        self.node = rclpy.create_node('dashboard_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Suscripciones
        self.node.create_subscription(
            Image,
            'camara_dron/image_raw',          # Ajusta al nombre real de tu tópico
            self.camera_callback,
            10
        )
        self.node.create_subscription(
            Pose,                 # O el mensaje que uses (Odometry, PoseStamped...)
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )
        
        self.status_signal.emit("Nodo ROS iniciado. Esperando datos...")
        
        # Crear un executor y empezar a procesar callbacks
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        try:
            self.executor.spin()
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

    def camera_callback(self, msg):
        """Convierte Image de ROS a QPixmap y emite la señal."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convertir BGR (OpenCV) a RGB para Qt
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_signal.emit(pixmap)
        except Exception as e:
            self.status_signal.emit(f"Error en cámara: {e}")

    def odom_callback(self, msg):
        """Formatea la posición GPS y la emite como texto."""
        x, y, z = msg.position[0], msg.position[1], msg.position[2]
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        text = f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}m | Yaw: {yaw:.2f}"
        self.odom_signal.emit(text)

class DroneDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        # ... (tu código existente)
        self.ros_worker = RosWorker()
        self.setup_ros_connections()
        self.ros_worker.start()   # Arrancar el hilo ROS

    def setup_ros_connections(self):
        # Conectar señales del worker a slots de la GUI
        self.ros_worker.image_signal.connect(self.update_camera_feed)
        self.ros_worker.odom_signal.connect(self.update_odometry)
        self.ros_worker.status_signal.connect(self.update_status)

    def init_pages(self):
        # ... (páginas de config y resultados igual)

        # Página 2: Vuelo mejorada
        self.page_flight = QWidget()
        f_layout = QVBoxLayout()

        # Sección de odometría
        self.odom_label = QLabel("Esperando odometría...")
        self.odom_label.setStyleSheet("font-size: 16px; color: blue;")
        f_layout.addWidget(self.odom_label)

        # Feed de cámara
        self.camera_label = QLabel()
        self.camera_label.setMinimumSize(640, 360)
        self.camera_label.setStyleSheet("border: 2px solid gray;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        f_layout.addWidget(self.camera_label)

        # Estado
        self.status_label = QLabel("● ROS no conectado")
        self.status_label.setStyleSheet("color: orange;")
        f_layout.addWidget(self.status_label)

        # Botón de emergencia
        self.btn_kill = QPushButton("EMERGENCY STOP")
        self.btn_kill.setStyleSheet("background-color: red; font-weight: bold; height: 50px;")
        self.btn_kill.clicked.connect(self.emergency_stop)
        f_layout.addWidget(self.btn_kill)

        self.page_flight.setLayout(f_layout)
        self.pages.addWidget(self.page_flight)

    # Slots
    def update_camera_feed(self, pixmap):
        """Actualiza la imagen de la cámara."""
        scaled_pixmap = pixmap.scaled(
            self.camera_label.width(),
            self.camera_label.height(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.camera_label.setPixmap(scaled_pixmap)

    def update_odometry(self, text):
        self.odom_label.setText(text)

    def update_status(self, text):
        self.status_label.setText(f"● {text}")
        self.status_label.setStyleSheet("color: green;")

    def emergency_stop(self):
        # Aquí podrías publicar un mensaje de apagado o llamar a un servicio
        print("EMERGENCY STOP presionado")
        self.status_label.setText("● EMERGENCIA ACTIVADA")
        self.status_label.setStyleSheet("color: red;")
        # Ejemplo: publicar un mensaje vacío en un tópico de control
        # (necesitarías un publisher en el worker)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Creamos la instancia de la ventana
    dashboard = DroneDashboard()
    dashboard.show()
    
    # Iniciamos el bucle de eventos de Qt
    try:
        sys.exit(app.exec())
    except SystemExit:
        print("Cerrando aplicación...")