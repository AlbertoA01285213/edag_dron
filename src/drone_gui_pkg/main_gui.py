#!/usr/bin/env python3
import sys
from PySide6.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                             QVBoxLayout, QHBoxLayout, QWidget, QStackedWidget, 
                             QLabel, QFrame)
from PySide6.QtCore import QThread, Signal, Qt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # O tus tipos de mensaje de pose

# --- HILO DE ROS 2 ---
class RosWorker(QThread):
    pose_signal = Signal(str) # Señal para enviar datos a la GUI

    def run(self):
        # Aquí inicializas un nodo simple que escuche al dron
        # y emita señales para actualizar la interfaz
        pass

# --- INTERFAZ PRINCIPAL ---
class DroneDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Mission Control v1.0")
        self.resize(900, 600)

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
        self.telemetry_label = QLabel("Esperando telemetría...")
        self.telemetry_label.setStyleSheet("font-size: 20px; color: green;")
        f_layout.addWidget(self.telemetry_label)
        
        self.btn_kill = QPushButton("EMERGENCY STOP")
        self.btn_kill.setStyleSheet("background-color: red; font-weight: bold; height: 50px;")
        f_layout.addWidget(self.btn_kill)
        self.page_flight.setLayout(f_layout)
        self.pages.addWidget(self.page_flight)

        # Página 3: Resultados
        self.page_res = QWidget()
        self.page_res.setLayout(QVBoxLayout())
        self.page_res.layout().addWidget(QLabel("Historial de ArUcos detectados"))
        self.pages.addWidget(self.page_res)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DroneDashboard()
    window.show()
    sys.exit(app.exec())