#!/usr/bin/env python3
import os
import sys
import cv2
import sqlite3
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix  # o el tipo que uses para odometría
from PySide6.QtGui import QPixmap, QImage, QPainter, QColor, QPen, QCursor
from PySide6.QtCore import Signal, QThread, Qt, Slot, QPoint
from PySide6.QtWidgets import (QApplication, QMainWindow, QPushButton, QToolTip,
                             QVBoxLayout, QHBoxLayout, QWidget, QStackedWidget, QMessageBox,
                             QLabel, QFrame, QGridLayout, QSpinBox, QDoubleSpinBox, QWidget)
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from example_interfaces.srv import SetBool

try:
    from master.srv import ConfigurarVuelo, CondicionesVuelo
except ImportError:
    print("No se pudo importar ConfigurarVuelo")
    ConfigurarVuelo = None
    CondicionesVuelo = None


class RosWorker(QThread):
    # Señales para enviar datos a la GUI
    image_signal = Signal(QPixmap)          # Imagen lista para mostrar
    odom_signal = Signal(str)               # Texto con posición (o coordenadas separadas)
    wp_signal = Signal(str)
    accion_signal = Signal(str)
    status_signal = Signal(str)             # Para logs/estado

    def __init__(self):
        super().__init__()
        self.node = None
        self.bridge = CvBridge()
        self.executor = None
        self.config_client_ConfigurarVuelo = None
        self.config_client_CondicionesVuelo = None

    def run(self):
        # Inicializar ROS 2 en este hilo
        rclpy.init(args=None)
        self.node = rclpy.create_node('dashboard_node')

        # qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        if ConfigurarVuelo:
            self.config_client_ConfigurarVuelo = self.node.create_client(
                ConfigurarVuelo,
                'configurar_vuelo'
            )
            self.status_signal.emit("Cliente de configuracion creado")
        else:
            self.status_signal.emit("Servicio personalizado no disponible")

        if CondicionesVuelo:
            self.config_client_CondicionesVuelo = self.node.create_client(
                CondicionesVuelo,
                'condiciones_vuelo'
            )
            self.status_signal.emit("Cliente de configuracion creado")
        else:
            self.status_signal.emit("Servicio personalizado no disponible")


        # self.client = self.node.create_client(SetBool, 'iniciar_mision')

        self.node.create_subscription(Image, 'low_res_feed', self.camera_callback, 10)
        self.node.create_subscription(Pose, 'pose_dron', self.odom_callback, 10)
        self.node.create_subscription(String, 'accion_odom', self.accion_callback, 10)
        self.node.create_subscription(Pose, 'waypoint_odom', self.wp_callback, 10)
        
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

    def wp_callback(self, msg):
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        q = msg.orientation.z
        # siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        # cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        # yaw = np.arctan2(siny_cosp, cosy_cosp)

        text = f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}m | Yaw: {q:.2f}"
        self.wp_signal.emit(text)

    def accion_callback(self, msg):
        accion = msg.data
        self.accion_signal.emit(accion)


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

    def configurar_vuelo(self, iniciar, altura, velocidad, largo, ancho, wp_x, wp_y, wp_z, wp_yaw):
        if not self.config_client_ConfigurarVuelo:
            self.status_signal.emit("Servicio no disponible")
            return False
        if not self.config_client_ConfigurarVuelo.wait_for_service(timeout_sec=1.0):
            self.status_signal.emit("Servicio de configuracion no encontrado")
            return False
        
        req = ConfigurarVuelo.Request()
        req.iniciar = iniciar
        req.altura_vuelo = altura
        req.velocidad_max = velocidad
        req.largo_cajon = largo
        req.ancho_cajon = ancho
        req.primer_wp_x = wp_x
        req.primer_wp_y = wp_y
        req.primer_wp_z = wp_z
        req.primer_wp_yaw = wp_yaw
        
        self.status_signal.emit("Enviando config")

        future = self.config_client_ConfigurarVuelo.call_async(req)
        return True
    
    def condiciones_vuelo(self, manual, emergencia, rtb):
        if not self.config_client_CondicionesVuelo:
            self.status_signal.emit("Servicio no disponible")
            return False
        if not self.config_client_CondicionesVuelo.wait_for_service(timeout_sec=1.0):
            self.status_signal.emit("Servicio de configuracion no encontrado")
            return False
        
        req = CondicionesVuelo.Request()
        req.manual = manual
        req.emergencia = emergencia
        req.rtb = rtb
        
        self.status_signal.emit("Enviando config")

        future = self.config_client_CondicionesVuelo.call_async(req)
        return True

class ParkingMapWidget(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.detecciones = []
        # Configuración del mapa (AJUSTA ESTO SEGÚN TU MUNDO DE GAZEBO)
        self.world_bounds = {
            'x_min': -1.0, 'x_max': 20.0,
            'y_min': -42.0, 'y_max': 14.0
        }
        self.setMouseTracking(True)

    def mouseMoveEvent(self, event):
        """Detecta qué carro hay bajo el mouse y muestra un ToolTip"""
        mouse_pos = event.pos()
        found = False

        for det in self.detecciones:
            px, py = self.world_to_pixel(det[1], det[2])
            target_pos = QPoint(px, py)
            
            # Calculamos la distancia entre el mouse y el punto del carro
            dist = (target_pos - mouse_pos).manhattanLength()
            
            if dist < 12:  # Radio de proximidad (ajustable)
                # Formateamos el texto con HTML para que se vea elegante
                info_text = (
                    f"<b>🚗 Cajón: {det[0]}</b><br>"
                    f"📍 Posición: ({det[1]:.2f}, {det[2]:.2f})<br>"
                    f"📝 Info: {det[3]}"
                )
                
                # Mostramos el ToolTip en la posición global del cursor
                QToolTip.showText(QCursor.pos(), info_text, self)
                found = True
                # Cambiamos el cursor a una mano para indicar interactividad
                self.setCursor(Qt.PointingHandCursor)
                break
        
        if not found:
            # Si no hay nada cerca, ocultamos el ToolTip y regresamos el cursor
            QToolTip.hideText()
            self.setCursor(Qt.ArrowCursor)

    def load_data(self):
        """Lee la base de datos y guarda las coordenadas"""
        db_path = os.path.expanduser('~/Documents/edag_dron/src/master/config/edag_db.db')
        if not os.path.exists(db_path): return

        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT cajon, x, y, info FROM carros")
        self.detecciones = cursor.fetchall()
        conn.close()
        self.update() # Llama a paintEvent

    def world_to_pixel(self, x_world, y_world):
        """Convierte metros a píxeles basándose en el tamaño actual del widget"""
        w, h = self.width(), self.height()
        
        # Proporción
        u = (y_world - self.world_bounds['y_min']) / (self.world_bounds['y_max'] - self.world_bounds['y_min'])
        v = (self.world_bounds['x_max'] - x_world) / (self.world_bounds['x_max'] - self.world_bounds['x_min'])
        
        return int(u * w), int(v * h)

    def paintEvent(self, event):
        super().paintEvent(event) # Dibuja la imagen de fondo (el mapa)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        pen = QPen(QColor(255, 0, 0), 10)
        painter.setPen(pen)

        for det in self.detecciones:
            px, py = self.world_to_pixel(det[1], det[2])
            painter.drawPoint(px, py)

    def mousePressEvent(self, event):
        """Detecta clics cerca de los puntos registrados"""
        click_pos = event.pos()
        for det in self.detecciones:
            px, py = self.world_to_pixel(det[1], det[2])
            dist = (QPoint(px, py) - click_pos).manhattanLength()
            
            if dist < 15: # Radio de clic de 15 píxeles
                QMessageBox.information(self, f"Info Cajón {det[0]}", 
                                      f"Coordenadas: ({det[1]:.2f}, {det[2]:.2f})\nDatos QR: {det[3]}")
                break


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
        layout.addWidget(QLabel("<b>Primer waypoint (m):</b>"))
        self.primer_wp_x = QDoubleSpinBox()
        self.primer_wp_x.setRange(-100.0, 100.0)
        self.primer_wp_x.setValue(2.0)
        layout.addWidget(self.primer_wp_x)

        self.primer_wp_y = QDoubleSpinBox()
        self.primer_wp_y.setRange(-100.0, 100.0)
        self.primer_wp_y.setValue(-5.3)
        layout.addWidget(self.primer_wp_y)

        self.primer_wp_z = QDoubleSpinBox()
        self.primer_wp_z.setRange(-100.0, 100.0)
        self.primer_wp_z.setValue(2.0)
        layout.addWidget(self.primer_wp_z)
  
        self.primer_wp_yaw = QDoubleSpinBox()
        self.primer_wp_yaw.setRange(-100.0, 100.0)
        self.primer_wp_yaw.setValue(0.0)
        layout.addWidget(self.primer_wp_yaw)



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
    
        self.camera_label = QLabel()
        self.camera_label.setMinimumSize(640, 360)
        self.camera_label.setStyleSheet("border: 2px solid gray;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        f_layout.addWidget(self.camera_label)

        self.odom_title = QLabel("Posicion:")
        self.odom_title.setStyleSheet("font-size: 20px; color: green;")
        f_layout.addWidget(self.odom_title)
        self.odom_label = QLabel("Esperando telemetría...")
        self.odom_label.setStyleSheet("font-size: 20px; color: green;")
        f_layout.addWidget(self.odom_label)

        self.wp_title = QLabel("Waypoint: ")
        self.wp_title.setStyleSheet("font-size: 20px; color: green;")
        f_layout.addWidget(self.wp_title)
        self.wp_label = QLabel("Esperando telemetría...")
        self.wp_label.setStyleSheet("font-size: 20px; color: green;")
        f_layout.addWidget(self.wp_label)

        self.accion_title = QLabel("Accion: ")
        self.accion_title.setStyleSheet("font-size: 20px; color: green;")
        f_layout.addWidget(self.accion_title)
        self.accion_label = QLabel("Esperando telemetría...")
        self.accion_label.setStyleSheet("font-size: 20px; color: green;")
        f_layout.addWidget(self.accion_label)

        self.status_label = QLabel("● ROS no conectado")
        self.status_label.setStyleSheet("color: orange;")
        f_layout.addWidget(self.status_label)
        
        self.btn_kill = QPushButton("EMERGENCY STOP")
        self.btn_kill.setStyleSheet("background-color: red; font-weight: bold; height: 50px;")
        self.btn_kill.clicked.connect(self.emergency_stop)
        f_layout.addWidget(self.btn_kill)

        self.btn_rtb = QPushButton("RTB")
        self.btn_rtb.setStyleSheet("background-color: red; font-weight: bold; height: 50px;")
        self.btn_rtb.clicked.connect(self.rtb)
        f_layout.addWidget(self.btn_rtb)

        self.page_flight.setLayout(f_layout)
        self.pages.addWidget(self.page_flight)

        # Página 3: Resultados ====================================================
        self.page_res = QWidget()
        res_layout = QVBoxLayout()
        self.btn_refresh = QPushButton("Actualizar Resultados")
        self.btn_refresh.clicked.connect(self.refresh_results)
        res_layout.addWidget(self.btn_refresh)

        self.parking_map = ParkingMapWidget()
        # Aquí pon una captura de pantalla de Gazebo desde arriba
        map_img_path = os.path.expanduser('~/Documents/edag_dron/src/master/config/mapa_parking.png')
        self.parking_map.setPixmap(QPixmap(map_img_path))
        self.parking_map.setScaledContents(True)
        
        res_layout.addWidget(self.parking_map)
        self.page_res.setLayout(res_layout)
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

    def update_wp(self, text):
        self.wp_label.setText(text)

    def update_accion(self, text):
        self.accion_label.setText(text)

    def update_status(self, text):
        self.status_label.setText(f"● {text}")
        self.status_label.setStyleSheet("color: green;")

    def refresh_results(self):
        self.parking_map.load_data()

    def emergency_stop(self):
        """Maneja el botón de parada de emergencia."""
        print("🚨 EMERGENCY STOP ACTIVADO")
        if self.status_label:
            self.status_label.setText("● EMERGENCIA ACTIVADA")
            self.status_label.setStyleSheet("color: red; font-weight: bold; padding: 5px;")

            manual = 0
            emergencia = 1
            rtb = 0

            condiciones_vuelo = self.ros_worker.condiciones_vuelo(manual, emergencia, rtb)

            if condiciones_vuelo:
                self.pages.setCurrentIndex(1)
                print(f"Emergencia activado")
            else:
                print("Fallo para emergencia")

    def rtb(self):
        print("RTB ACTIVADO")
        if self.status_label:
            self.status_label.setText("RTB ACTIVADA")
            self.status_label.setStyleSheet("color: red; font-weight: bold; padding: 5px;")

            manual = 0
            emergencia = 0
            rtb = 1

            condiciones_vuelo = self.ros_worker.condiciones_vuelo(manual, emergencia, rtb)

        if condiciones_vuelo:
            self.pages.setCurrentIndex(1)
            print(f"Regreso a base activado")
        else:
            print("Fallo para regresar a base")


    def iniciar_vuelo(self):
        altura = self.despegue_altura.value()
        velocidad = self.vel_max.value()
        largo = self.cajon_largo.value()
        ancho = self.cajon_ancho.value()

        wp_x = self.primer_wp_x.value()
        wp_y = self.primer_wp_y.value()
        wp_z = self.primer_wp_z.value()
        wp_yaw = self.primer_wp_yaw.value()

        success = self.ros_worker.configurar_vuelo(True, altura, velocidad, largo, ancho, wp_x, wp_y, wp_z, wp_yaw)

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
        self.ros_worker.wp_signal.connect(self.update_wp)
        self.ros_worker.accion_signal.connect(self.update_accion)
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