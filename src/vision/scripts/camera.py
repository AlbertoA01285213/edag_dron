#!/usr/bin/env python3
import sys
import socket
import cv2
import time
import numpy as np
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode

# Configuración de Red UDP
UDP_IP = "127.0.0.1"
UDP_PORT = 5600
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def camera_callback(msg: GzImage):
    # 1. Convertir el mensaje de Gazebo a matriz de OpenCV
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    # 2. Redimensionar un poco para asegurar que quepa en un paquete UDP (< 65KB)
    frame_resized = cv2.resize(frame_bgr, (640, 480))
    
    # 3. Comprimir a JPEG (Calidad 50 para que sea súper rápido y ligero)
    success, encoded_image = cv2.imencode('.jpg', frame_resized, [cv2.IMWRITE_JPEG_QUALITY, 50])
    
    if success:
        # 4. Enviar la foto comprimida por el puerto UDP
        data = encoded_image.tobytes()
        if len(data) < 65507: # Límite máximo de un paquete UDP
            sock.sendto(data, (UDP_IP, UDP_PORT))
            print("➤ Foto JPEG enviada al puerto 5600", end="\r")
        else:
            print("⚠️ La imagen comprimida es demasiado grande para UDP!")

# Conectarse a Gazebo
node = GzNode()
node.subscribe(GzImage, "/camera_main", camera_callback)

print(f"📡 Plan B Activo. Transmitiendo JPEGs por el puerto {UDP_PORT}...")

while True:
    time.sleep(1)
