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
                             QLabel, QFrame, QGridLayout, QSpinBox, QDoubleSpinBox)
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from example_interfaces.srv import SetBool

try:
    from master.srv import ConfigurarVuelo
except ImportError:
    print("No se pudo importar ConfigurarVuelo")
    ConfigurarVuelo = None


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
        self.config_client = None

    def run(self):
        # Inicializar ROS 2 en este hilo
        rclpy.init(args=None)
        self.node = rclpy.create_node('dashboard_node')

        # qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        if ConfigurarVuelo:
            self.config_client = self.node.create_client(
                ConfigurarVuelo,
                'configurar_vuelo'
            )
            self.status_signal.emit("Cliente de configuracion creado")
        else:
            self.status_signal.emit("Servicio personalizado no disponible")


        # self.client = self.node.create_client(SetBool, 'iniciar_mision')

        self.node.create_subscription(Image, 'low_res_feed', self.camera_callback, 10)
        self.node.create_subscription(Pose, 'pose_dron', self.odom_callback, 10)
        
        self.status_signal.emit("Nodo ROS iniciado. Esperando datos...")
        
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        try:
            self.executor.spin()
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_signal.emit(pixmap)
        except Exception as e:
            self.status_signal.emit(f"Error en cámara: {e}")

    def odom_callback(self, msg):
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        text = f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}m | Yaw: {yaw:.2f}"
        self.odom_signal.emit(text)

    def llamar_servicio_vuelo(self, estado, altura, ancho, largo):
        """Método que será llamado desde la GUI"""
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.status_signal.emit("Servicio no disponible")
            return

        req = SetBool.Request()
        req.data = estado
        # Aquí podrías usar un servicio personalizado que acepte altura, ancho, etc.
        # Por ahora enviamos el log de lo que se mandaría
        self.status_signal.emit(f"Enviando Config: Altura={altura}m, Cajón={ancho}x{largo}m")
        self.client.call_async(req)

    def configurar_vuelo(self, iniciar, altura, velocidad, largo, ancho):
        if not self.config_client:
            self.status_signal.emit("Servicio no disponible")
            return False
        if not self.config_client.wait_for_service(timeout_sec=1.0):
            self.status_signal.emit("Servicio de configuracion no encontrado")
            return False
        
        req = ConfigurarVuelo.Request()
        req.iniciar = iniciar
        req.altura_vuelo = altura
        req.velocidad_max = velocidad
        req.largo_cajon = largo
        req.ancho_cajon = ancho
        
        self.status_signal.emit("Enviando config")

        future = self.config_client.call_async(req)
        return True

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
        # Página 1: Configuración =================================================
        self.page_config = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("<h2>Ajustes de Misión</h2>"))

        layout.addWidget(QLabel("Ajustes de dron"))
        layout.addWidget(QLabel("<b>Altura de despegue (m):</b>"))
        self.despegue_altura = QDoubleSpinBox()
        self.despegue_altura.setRange(0.5, 5.0) # Altura segura
        self.despegue_altura.setValue(1.5)
        layout.addWidget(self.despegue_altura)

        layout.addWidget(QLabel("<b>Velocidad maxima (m/s):</b>"))
        self.vel_max = QDoubleSpinBox()
        self.vel_max.setRange(0.5, 4.0) # Altura segura
        self.vel_max.setValue(2.0)
        layout.addWidget(self.vel_max)

        layout.addWidget(QLabel("<b>Largo del cajón (m):</b>"))
        self.cajon_largo = QDoubleSpinBox()
        self.cajon_largo.setRange(1.0, 10.0)
        self.cajon_largo.setValue(5.0)
        layout.addWidget(self.cajon_largo)

        layout.addWidget(QLabel("<b>Ancho del cajón (m):</b>"))
        self.cajon_ancho = QDoubleSpinBox()
        self.cajon_ancho.setRange(1.0, 10.0)
        self.cajon_ancho.setValue(2.6)
        layout.addWidget(self.cajon_ancho)

        self.iniciar_vuelo_btn = QPushButton("Iniciar vuelo")
        self.iniciar_vuelo_btn.setStyleSheet("background-color: green; font-weight: bold; height: 50px;")
        self.iniciar_vuelo_btn.clicked.connect(self.iniciar_vuelo)
        layout.addWidget(self.iniciar_vuelo_btn)

        layout.addStretch() # Empuja todo hacia arriba
        self.page_config.setLayout(layout) # <--- CRÍTICO: Asignar el layout
        self.pages.addWidget(self.page_config)


        # Página 2: Vuelo (La más importante) =====================================
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

        # Página 3: Resultados ====================================================
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

    def iniciar_vuelo(self):
        altura = self.despegue_altura.value()
        velocidad = self.vel_max.value()
        largo = self.cajon_largo.value()
        ancho = self.cajon_ancho.value()

        success = self.ros_worker.configurar_vuelo(True, altura, velocidad, largo, ancho)

        if success:
            self.pages.setCurrentIndex(1)
            print(f"✅ Vuelo configurado: Alt={altura}m, Vel={velocidad}m/s, Área={largo}x{ancho}m")
        else:
            print("❌ Error al configurar el vuelo")


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
    except KeyboardInterrupt:
        print("\nInterrupción por teclado")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Programa finalizado")