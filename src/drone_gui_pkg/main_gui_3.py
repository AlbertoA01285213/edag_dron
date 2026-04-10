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
            'pose_dron',
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
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        text = f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}m | Yaw: {yaw:.2f}"
        self.odom_signal.emit(text)

# --- INTERFAZ PRINCIPAL ---
class DroneDashboard(QMainWindow):
    def __init__(self):
        super().__init__()

        # Iniciar hilo paralelo con ros2
        self.ros_worker = RosWorker()
        self.setup_ros_connections()
        self.ros_worker.start()
        
        self.setWindowTitle("Drone Mission Control v1.0")
        self.resize(900, 600)

        self.camera_label = None
        self.odom_label = None
        self.status_label = None

        # Layout Principal
        self.main_layout = QHBoxLayout()
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.central_widget.setLayout(self.main_layout)

        # 1. Barra Lateral (Menu)
        self.sidebar = QVBoxLayout()
        self.btn_config = QPushButton("⚙️ Configuración")
        self.btn_vuelo = QPushButton("🚀 Vuelo")
        self.btn_results = QPushButton("📊 Resultados")
        
        self.sidebar.addWidget(self.btn_config)
        self.sidebar.addWidget(self.btn_vuelo)
        self.sidebar.addWidget(self.btn_results)
        self.sidebar.addStretch() # Empuja los botones hacia arriba
        self.main_layout.addLayout(self.sidebar, 1)

        # 2. Contenedor de Páginas (Stacked Widget)
        self.pages = QStackedWidget()
        self.main_layout.addWidget(self.pages, 4)

        self.init_pages()
        
        # Conectar botones
        self.btn_config.clicked.connect(lambda: self.pages.setCurrentIndex(0))
        self.btn_vuelo.clicked.connect(lambda: self.pages.setCurrentIndex(1))
        self.btn_results.clicked.connect(lambda: self.pages.setCurrentIndex(2))

    def init_pages(self):
        # Página 1: Configuración
        self.page_config = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("<h2>Ajustes de Misión</h2>"))
        layout.addWidget(QLabel("Punto de inicio (X, Y):"))
        # Aquí pondrías tus QSpinBoxes...
        self.page_config.setLayout(layout)
        self.pages.addWidget(self.page_config)

        # Página 2: Vuelo (La más importante)
        self.page_flight = QWidget()
        f_layout = QVBoxLayout()
        self.odom_label = QLabel("Esperando telemetría...")
        self.odom_label.setStyleSheet("font-size: 20px; color: green;")
        f_layout.addWidget(self.odom_label)

        self.camera_label = QLabel()
        self.camera_label.setMinimumSize(640, 360)
        self.camera_label.setStyleSheet("border: 2px solid gray;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        f_layout.addWidget(self.camera_label)

        self.status_label = QLabel("● ROS no conectado")
        self.status_label.setStyleSheet("color: orange;")
        f_layout.addWidget(self.status_label)
        
        self.btn_kill = QPushButton("EMERGENCY STOP")
        self.btn_kill.setStyleSheet("background-color: red; font-weight: bold; height: 50px;")
        self.btn_kill.clicked.connect(self.emergency_stop)
        f_layout.addWidget(self.btn_kill)

        self.page_flight.setLayout(f_layout)
        self.pages.addWidget(self.page_flight)

        # Página 3: Resultados
        self.page_res = QWidget()
        self.page_res.setLayout(QVBoxLayout())
        self.page_res.layout().addWidget(QLabel("Historial de ArUcos detectados"))
        self.pages.addWidget(self.page_res)

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
        """Maneja el botón de parada de emergencia."""
        print("🚨 EMERGENCY STOP ACTIVADO")
        if self.status_label:
            self.status_label.setText("● EMERGENCIA ACTIVADA")
            self.status_label.setStyleSheet("color: red; font-weight: bold; padding: 5px;")
        
        # Aquí puedes agregar lógica para publicar un mensaje de stop
        # Por ejemplo, si tienes un publisher en el worker:
        # self.ros_worker.publish_emergency_stop()

    def closeEvent(self, event):
        """Maneja el cierre limpio de la aplicación."""
        print("Cerrando aplicación...")
        if self.ros_worker and self.ros_worker.isRunning():
            if self.ros_worker.executor:
                self.ros_worker.executor.shutdown()
            self.ros_worker.quit()
            self.ros_worker.wait(5000)  # Esperar máximo 5 segundos
        event.accept()

    def setup_ros_connections(self):
        # Conectar señales del worker a slots de la GUI
        self.ros_worker.image_signal.connect(self.update_camera_feed)
        self.ros_worker.odom_signal.connect(self.update_odometry)
        self.ros_worker.status_signal.connect(self.update_status)

if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = DroneDashboard()
    window.show()

    try:
        sys.exit(app.exec())
    except SystemExit:
        print("Cerrando aplicación...")