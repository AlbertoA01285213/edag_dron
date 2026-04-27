#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from std_msgs.msg import Bool, String, Float32, Int16
from master.srv import ConfigurarVuelo, CondicionesVuelo
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import os
import numpy as np
import math
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import VehicleCommand, VehicleOdometry, VehicleCommandAck, TrajectorySetpoint, OffboardControlMode, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MissionHandler(Node):
    def __init__(self):
        super().__init__('mission_handler')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)

        try:
            default_path = os.path.join(get_package_share_directory('master'), 'misiones', 'escaneo.yaml')
        except:
            default_path = ""

        self.declare_parameter('mission_file', default_path)
        mission_path = self.get_parameter('mission_file').value

        self.get_logger().info(f"Cargando misión desde: {mission_path}")

        # 3. Validar si el archivo existe antes de abrirlo
        if not os.path.exists(mission_path):
            self.get_logger().error(f"¡ARCHIVO DE MISIÓN NO ENCONTRADO!: {mission_path}")
            return

        with open(mission_path, 'r') as f:
            self.mission = yaml.safe_load(f)

        # Variables de serivcios
        self.mision_iniciada = False
        self.altura_vuelo = -1.5  # Negativo para PX4 (NED)
        self.velocidad_max = 2.0
        self.parking_largo = 5.0
        self.parking_ancho = 2.6

        self.primer_waypoint_x = 1.0
        self.primer_waypoint_y = 1.0
        self.primer_waypoint_z = 1.0
        self.primer_waypoint_yaw = 1.0

        self.srv_configurar = self.create_service(ConfigurarVuelo, 'configurar_vuelo', self.configurar_vuelo_callback)

        self.manual = 0
        self.emergencia = 0
        self.rtb = 0

        self.srv_condiciones = self.create_service(CondicionesVuelo, 'condiciones_vuelo', self.condiciones_vuelo_callback)
        
        self.libreria_de_rutinas = {
            1: [ # ID 1: Barrido Derecha
                {"type": "scan_2", "offset": 1.0, "cajones": 6, "width": 2.5, "side": 1},
                {"type": "land"}
            ],
            2: [ # ID 2: Barrido Izquierda
                {"type": "scan_2", "offset": 1.0, "cajones": 6, "width": 2.5, "side": -1},
                {"type": "land"}
            ]
        }

        self.actions = []
        self.fase_busqueda = True
        self.despegue_exitoso = False
        self.search_aruco = True

        self.mensaje_impreso = 1   
        
        # Variables de nodo
        self.actions = self.mission["actions"]
        self.idx = 0
        self.inicio_idx = 0
        self.barrido_sub_idx = 0
        self.rtb_sub_idx = 0
        self.last_idx = -1
        self.time_stamp = 0

        self.pose = [0.0]*6
        self.pose_inicial = [0.0]*6
        self.pose_actual = [0.0]*6
        self.pose_esperada = [0.0]*6

        self.nuevo_yaw = 0.0


        self.velocidad_actual = [0.0]*6

        self.comando_enviado = 0

        self.current_aruco_error = None

        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.pose_callback, qos_profile)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.ack_callback, qos_profile)
        self.create_subscription(Pose, 'aruco_error', self.aruco_error_callback, 10)
        self.create_subscription(Int16, 'aruco_id', self.aruco_id_callback, 10)

        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

        self.waypoint_odom_pub = self.create_publisher(Pose, 'waypoint_odom', 10)

        self.tomar_foto_pub = self.create_publisher(Int16, 'take_picture', 10)

        self.pose_inicial = self.pose_actual

        self.timer = self.create_timer(0.1, self.run)

    def pose_callback(self, msg: VehicleOdometry):
        try:
            self.pose_actual[0] = msg.position[0]
            self.pose_actual[1] = msg.position[1]
            self.pose_actual[2] = msg.position[2]
            quat = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
            roll, pitch, yaw = euler_from_quaternion(quat)
            self.pose_actual[3] = self.from_drone_yaw(roll)
            self.pose_actual[4] = pitch
            self.pose_actual[5] = yaw

            # self.get_logger().info(f"X_local: {self.pose_actual[0]:.2f}, Y_local: {self.pose_actual[1]:.2f}, Z_local: {self.pose_actual[2]:.2f}")
            # self.get_logger().info(f"RX_local: {self.pose_actual[3]:.2f}, RY_local: {self.pose_actual[4]:.2f}, RZ_local: {self.pose_actual[5]:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error en pose_callback: {e}")

    def ack_callback(self, msg: VehicleCommandAck):
        self.ack_command = msg.command
        self.ack_result = msg.result

    def aruco_error_callback(self, msg):
        # El error esta en xyz y el id del aruco se manda por orientation w (esta dividido por 10)
        self.current_aruco_error = msg

    def aruco_id_callback(self, msg):
        self.aruco_id = msg.data

    def configurar_vuelo_callback(self, request, response):
        """Callback del servicio de configuración"""
        self.mision_iniciada = request.iniciar
        self.altura_vuelo = -abs(request.altura_vuelo)  # Negativo para NED
        self.velocidad_max = request.velocidad_max
        self.parking_largo = request.largo_cajon
        self.parking_ancho = request.ancho_cajon

        if self.mision_iniciada:
            self.get_logger().info(f"""
            🚀 MISIÓN CONFIGURADA:
            - Altura: {abs(self.altura_vuelo)}m
            - Velocidad máxima: {self.velocidad_max}m/s
            - Área de estacionamiento: {self.parking_largo}x{self.parking_ancho}m
            """)
            response.success = True
            response.message = "Misión configurada correctamente"
        else:
            self.get_logger().info("⏹️ Misión detenida")
            response.success = True
            response.message = "Misión detenida"
        
        return response
    
    def condiciones_vuelo_callback(self, request, response):
        """Callback del servicio de condiciones"""

        self.manual = request.manual
        self.emergencia = request.emergencia
        self.rtb = request.rtb
        
        return response

    def to_drone_yaw(self, mission_yaw):
        """Convierte el ángulo de tu lógica al que entiende PX4."""
        # Según tus pruebas: Input 0 (Sur) -> Output 180 (Sur)
        # La relación es una rotación de pi (180 grados)
        return self.normalize_angle(mission_yaw + np.pi)

    def from_drone_yaw(self, drone_yaw):
        """Convierte lo que dice la odometría a tu lógica de misión."""
        return self.normalize_angle(drone_yaw - np.pi)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def send_setpoint(self, x, y, z, yaw_rad):
        # drone_yaw = self.to_drone_yaw(yaw_rad)
        # drone_yaw = yaw_rad
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw_rad)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

        self.target_vuelo = [float(x), float(y), float(z), float(yaw_rad)]

        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.orientation.z = float(yaw_rad)
        self.waypoint_odom_pub.publish(msg)

    def get_global_coordinates(self, forward, right, down):
        # Convierte distancias relativas al cuerpo del dron (Body Frame) a coordenadas globales del mundo (NED) basadas en el Yaw actual.
        yaw = self.pose_actual[3]
        
        delta_norte = (forward * np.cos(yaw)) - (right * np.sin(yaw))
        delta_este  = (forward * np.sin(yaw)) + (right * np.cos(yaw))
        
        target_x = float(self.pose_actual[0] + delta_norte)
        target_y = float(self.pose_actual[1] + delta_este)
        target_z = float(self.pose_actual[2] + down)
        target_yaw = float(yaw)
        
        return target_x, target_y, target_z, target_yaw
    
    def armar_dron(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # 1 = Arm
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_publisher.publish(msg)
        self.get_logger().info("Armando drone...")

    def run(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_publisher.publish(offboard_msg)

        if self.rtb == 1:
            if not hasattr(self, 'rtb_iniciado'):
                self.rtb_iniciado = True
                self.rtb_sub_idx = 1
                
                self.start_x = self.pose_actual[0]
                self.start_y = self.pose_actual[1]
                self.start_z = self.pose_actual[2]
                self.start_yaw = self.pose_actual[3]                
                return
            
            target_x, target_y, target_z, target_yaw = 0.0, 0.0, 0.0, 0.0 
       
            if self.rtb_sub_idx == 1:
                target_x = self.start_x
                target_y = self.start_y
                target_z = self.start_z - 2.0
                target_yaw = self.start_yaw
                label = "Subiendo por seguridad"

                self.send_setpoint(target_x, target_y, target_z, target_yaw)

                dx = target_x - self.pose_actual[0]
                dy = target_y - self.pose_actual[1]
                dz = target_z - self.pose_actual[2]
                distancia = np.sqrt(dx**2 + dy**2 + dz**2)

                if distancia < 0.3:
                    self.get_logger().info(f"✅ Paso {self.rtb_sub_idx} completado: {label}")
                    self.rtb_sub_idx += 1

            elif self.rtb_sub_idx == 2:
                target_x = 0
                target_y = 0
                target_z = self.start_z - 2.0
                target_yaw = self.start_yaw
                label = "Regresar al origen"

                self.send_setpoint(target_x, target_y, target_z, target_yaw)

                dx = target_x - self.pose_actual[0]
                dy = target_y - self.pose_actual[1]
                dz = target_z - self.pose_actual[2]
                distancia = np.sqrt(dx**2 + dy**2 + dz**2)

                if distancia < 0.3:
                    self.get_logger().info(f"✅ Paso {self.rtb_sub_idx} completado: {label}")
                    self.rtb_sub_idx += 1

            elif self.rtb_sub_idx == 3:
                if not hasattr(self, 'land_sent'):
                    msg = VehicleCommand()
                    msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
                    msg.target_system = 1
                    msg.target_component = 1
                    msg.source_system = 1
                    msg.source_component = 1
                    msg.from_external = False
                    msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                    self.command_publisher.publish(msg)

                    self.land_sent = True
                    self.ha_empezado_a_bajar = False 
                    return

                if not self.ha_empezado_a_bajar:
                    if self.velocidad_actual[2] > 0.2:
                        self.ha_empezado_a_bajar = True
                        self.get_logger().info("Descenso iniciado...")
                    return 

                if self.ha_empezado_a_bajar and abs(self.velocidad_actual[2]) <= 0.05:
                    self.get_logger().info("Aterrizado con éxito.")
                    del self.land_sent
                    del self.ha_empezado_a_bajar
                    del self.rtb_iniciado
                    label = "Aterrizado"

            elif self.rtb_sub_idx == 4:
                msg = VehicleCommand()
                msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                msg.param1 = 0.0

                msg.target_system = 1
                msg.target_component = 1
                msg.source_system = 1
                msg.source_component = 1
                msg.from_external = True

                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

                self.command_publisher.publish(msg)
                label = "Aterrizado"

        if self.emergencia == 1:
            if not hasattr(self, 'land_sent'):
                msg = VehicleCommand()
                msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
                msg.target_system = 1
                msg.target_component = 1
                msg.source_system = 1
                msg.source_component = 1
                msg.from_external = False
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.command_publisher.publish(msg)

                self.land_sent = True
                self.ha_empezado_a_bajar = False 
                return

            if not self.ha_empezado_a_bajar:
                if self.velocidad_actual[2] > 0.2:
                    self.ha_empezado_a_bajar = True
                    self.get_logger().info("Descenso iniciado...")
                return 

            if self.ha_empezado_a_bajar and abs(self.velocidad_actual[2]) <= 0.05:
                self.get_logger().info("Aterrizado con éxito.")
                del self.land_sent
                del self.ha_empezado_a_bajar
                
                msg = VehicleCommand()
                msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                msg.param1 = 0.0

                msg.target_system = 1
                msg.target_component = 1
                msg.source_system = 1
                msg.source_component = 1
                msg.from_external = True

                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

                self.command_publisher.publish(msg)
                label = "Aterrizado"


        if not self.mision_iniciada:
            self.send_setpoint(0.0, 0.0, 0.0, 0.0)
            return
        
        elif self.mision_iniciada == True:
            # Despegar e ir a wp inicial

            if self.idx == 0:
                if self.inicio_idx == 0:

                    self.armar_dron()

                    height = 2.0
                    self.send_setpoint(0.0, 0.0, -height, 0.0)

                    msg = VehicleCommand()
                    msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                    msg.param1 = 1.0
                    msg.param2 = 6.0
                    msg.target_system = 1
                    msg.target_component = 1
                    msg.source_system = 1
                    msg.source_component = 1
                    msg.from_external = True
                    msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                    self.command_publisher.publish(msg)

                    self.inicio_idx = 1


                elif self.inicio_idx == 1:
                    height = 2.0
                    self.send_setpoint(0.0, 0.0, -height, 0.0)

                    if abs(self.pose_actual[2]) >= (height - 0.2):
                        if self.mensaje_impreso == 0:
                            self.get_logger().info("🚀 ¡Altura de despegue alcanzada!")
                            self.get_logger().info("Iniciando hold de 2 segundos")  
                            self.mensaje_impreso = 1            

                        
                        duration = 2.0
                        if not hasattr(self, "hold_start"):
                            self.hold_start = time.perf_counter()

                        if time.perf_counter() - self.hold_start >= duration:
                            self.inicio_idx = 2
                            del self.hold_start
                            return


                elif self.inicio_idx == 2:
                    x_obj = 2.0
                    y_obj = -5.3
                    z_obj = -2.0
                    yaw_obj = 0.0

                    setpoint_msg = TrajectorySetpoint()
                    setpoint_msg.position = [x_obj, y_obj, z_obj]
                    setpoint_msg.yaw = np.deg2rad(yaw_obj)
                    setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                    self.trajectory_pub.publish(setpoint_msg)

                    dx = x_obj - self.pose_actual[0]
                    dy = y_obj - self.pose_actual[1]
                    dz = z_obj - self.pose_actual[2]
                    distancia = np.sqrt(dx**2 + dy**2 + dz**2)

                    if distancia < 0.3:
                        if self.mensaje_impreso == 1:
                            self.get_logger().info("Primer waypoint alcanzado")
                            self.mensaje_impreso = 2

                        duration = 2.0
                        if not hasattr(self, "hold_start"):
                            self.hold_start = time.perf_counter()

                        if time.perf_counter() - self.hold_start >= duration:
                            self.idx = 1
                            self.inicio_idx = 0
                            del self.hold_start
                            return
                        
            elif self.idx == 1:
                if self.search_aruco == True:
                    if not hasattr(self, "alineado"): 
                        self.alineado = False
                    if not hasattr(self, "posicionado"):
                        self.posicionado = True

                    if self.posicionado and not hasattr(self, "esperando_foto"):
                        self.get_logger().info("Tomando captura para análisis...")
                        self.tomar_foto_pub.publish(Int16(data=2))
                        self.esperando_foto = True
                        self.current_aruco_error = None
                        self.tomar_foto_pub.publish(Int16(data=0))
                        return    

                    if hasattr(self, "esperando_foto") and self.current_aruco_error is not None:
                        self.tomar_foto_pub.publish(Int16(data=0))
                        del self.esperando_foto
                        
                        err = self.current_aruco_error

                        if err.position.x == 999.0 and err.position.y == 999.0 and err.position.z == 999.0:
                            duration = 2.0
                            if not hasattr(self, "hold_start"):
                                self.hold_start = time.perf_counter()

                                if time.perf_counter() - self.hold_start >= duration:
                                    del self.hold_start

                                    self.posicionado = False
                                    return      

                        elif abs(err.position.x) < 0.07 and abs(err.position.y) < 0.05 and abs(err.position.z) < 0.1:
                            self.get_logger().error("🎯 Dron alineado, continuando")
                            # LIMPIEZA SEGURA
                            if hasattr(self, "alineado"): del self.alineado
                            if hasattr(self, "posicionado"): del self.posicionado

                            self.search_aruco = False 
                            return

                        else:
                            self.get_logger().info("ArUco encontrado, alineando")

                            fwd = err.position.z * 0.7
                            rgt = err.position.x * 0.7
                            dwn = -err.position.y * 0.8

                            self.get_logger().info(f"Error Lateral: {err.position.x}, Altura: {err.position.y}, Distancia: {err.position.z}, id {self.aruco_id}")

                            tx, ty, tz, tyaw = self.get_global_coordinates(fwd, rgt, dwn)

                            target_yaw = self.normalize_angle(tyaw)

                            self.get_logger().info(f"Target X: {tx}, Y: {ty}, Z: {tz}, TY {target_yaw}")

                            self.send_setpoint(tx, ty, tz, tyaw)

                            self.posicionado = False
                            # self.hold_start = time.perf_counter()      

                    if not self.posicionado:
                        distancia = 999.0
                        dyaw = 999.0
                        if hasattr(self, "target_vuelo"):
                            dx = self.target_vuelo[0] - self.pose_actual[0]
                            dy = self.target_vuelo[1] - self.pose_actual[1]
                            dz = self.target_vuelo[2] - self.pose_actual[2]
                            dyaw = abs(self.target_vuelo[3] - self.pose_actual[3])
                            distancia = np.sqrt(dx**2 + dy**2 + dz**2)
                            # self.get_logger().info(f"Dist: {distancia:.2f}m, Error Yaw: {np.rad2deg(dyaw):.1f}°")

                        if distancia < 0.1:
                            self.get_logger().info("Posición alcanzada. Estabilizando para foto...")
                            self.posicionado = True  
                             

                elif self.search_aruco == False:
                    if self.aruco_id == 1:
                        ''' Scan barrido izquierda '''

                        offset = 1.10
                        cajones = 6
                        width = 2.6

                        if self.barrido_sub_idx == 0:
                            if not hasattr(self, 'scan_iniciado'):
                                self.scan_iniciado = True
                                self.current_cajon = 0
                                self.scan_origin_x = self.pose_actual[0]
                                self.scan_origin_y = self.pose_actual[1]
                                self.scan_origin_z = self.pose_actual[2]
                                self.scan_origin_yaw = self.pose_actual[3]
                            
                                self.theta = self.pose_actual[3]


                                fwd = 0
                                rgt = -1 * (cajones * width + offset)
                                dwn = 0
                                
                                self.tx, self.ty, self.tz, self.tyaw = self.get_global_coordinates(fwd, rgt, dwn)

                                yaw_obj = self.scan_origin_yaw

                                self.send_setpoint(self.tx, self.ty, self.tz, yaw_obj)

                                self.get_logger().info(f"🚀 Iniciando barrido para {cajones} cajones.")
                                return 
                            

                            distancia_recorrida = abs(self.pose_actual[1] - self.scan_origin_y)
                            punto_proxima_foto = offset + (self.current_cajon * width)

                            if distancia_recorrida >= punto_proxima_foto and self.current_cajon < cajones:
                                self.get_logger().info(f"📸 Tomando foto cajón {self.current_cajon + 1} a {distancia_recorrida:.2f}m")

                                self.tomar_foto_pub.publish(Int16(data=1))
                                self.tomar_foto_pub.publish(Int16(data=0))

                                self.current_cajon += 1
                            
                            dx = self.tx - self.pose_actual[0]
                            dy = self.ty - self.pose_actual[1]
                            dz = self.tz - self.pose_actual[2]
                                
                            distancia = np.sqrt(dx**2 + dy**2 + dz**2)

                            if distancia < 0.3:
                                self.get_logger().info("¡Punto de destino alcanzado!")
                                self.barrido_sub_idx = 1

                        elif self.barrido_sub_idx == 1:
                            if not hasattr(self, 'retroceso'):
                                self.retroceso = True
                                self.current_cajon = 0

                                fwd = -1.0
                                rgt = 0
                                dwn = 0
                                tx, ty, tz, tyaw = self.get_global_coordinates(fwd, rgt, dwn)

                                self.scan_origin_x = float(tx)
                                self.scan_origin_y = float(ty)
                                self.scan_origin_z = float(tz)
                                self.scan_origin_yaw = float(tyaw)

                                self.get_logger().info(f"Iniciando retroceso")
                                return 
                                            
                            dx = float(self.scan_origin_x - self.pose_actual[0])
                            dy = float(self.scan_origin_y - self.pose_actual[1])
                            dz = float(self.scan_origin_z - self.pose_actual[2])
                            distancia = np.sqrt(dx**2 + dy**2 + dz**2)

                            self.send_setpoint(self.scan_origin_x, self.scan_origin_y, self.scan_origin_z, self.scan_origin_yaw)

                            # setpoint_msg = TrajectorySetpoint()
                            # setpoint_msg.position = [self.scan_origin_x, self.scan_origin_y, self.scan_origin_z]
                            # setpoint_msg.yaw = self.scan_origin_yaw
                            # setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
                            # self.trajectory_pub.publish(setpoint_msg)

                            if distancia < 0.3:
                                self.get_logger().info("Barrido completado")

                                if hasattr(self, 'scan_iniciado'): del self.scan_iniciado
                                if hasattr(self, 'retroceso'): del self.retroceso
                                
                                self.barrido_sub_idx = 0
                                self.search_aruco = True
                                del self.aruco_id
                                return


                    elif self.aruco_id == 2:
                        ''' Scan barrido derecha '''
                        offset = 1.10
                        cajones = 6
                        width = 2.6
                        side = 1

                        if self.barrido_sub_idx == 0:
                            if not hasattr(self, 'scan_iniciado'):
                                self.scan_iniciado = True
                                self.current_cajon = 0
                                self.scan_origin_x = self.pose_actual[0]
                                self.scan_origin_y = self.pose_actual[1]
                                self.scan_origin_z = self.pose_actual[2]
                                self.scan_origin_yaw = self.pose_actual[3]
                            
                                self.theta = self.pose_actual[3]

                                self.get_logger().info(f"🚀 Iniciando barrido para {cajones} cajones.")
                                return 
                            
                            y_obj = self.scan_origin_y + cajones * width - offset

                            x_obj = self.scan_origin_x
                            z_obj = self.scan_origin_z
                            yaw_obj = self.scan_origin_yaw

                            setpoint_msg = TrajectorySetpoint()
                            setpoint_msg.position = [float(x_obj), float(y_obj), float(z_obj)]
                            setpoint_msg.yaw = yaw_obj
                            setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                            self.trajectory_pub.publish(setpoint_msg)

                            distancia_recorrida = abs(self.pose_actual[1] - self.scan_origin_y)
                            punto_proxima_foto = offset + (self.current_cajon * width)

                            if distancia_recorrida >= punto_proxima_foto and self.current_cajon < cajones:
                                self.get_logger().info(f"📸 Tomando foto cajón {self.current_cajon + 1} a {distancia_recorrida:.2f}m")

                                self.tomar_foto_pub.publish(Int16(data=1))
                                self.tomar_foto_pub.publish(Int16(data=0))
                                # Nota: El nodo de la cámara debería apagar el flag solo tras guardar
                                
                                self.current_cajon += 1

                            
                            dx = x_obj - self.pose_actual[0]
                            dy = y_obj - self.pose_actual[1]
                            dz = z_obj - self.pose_actual[2]
                                
                            distancia = np.sqrt(dx**2 + dy**2 + dz**2)

                            if distancia < 0.3:
                                self.get_logger().info("¡Punto de destino alcanzado!")
                                self.barrido_sub_idx = 1

                        elif self.barrido_sub_idx == 1:
                            if not hasattr(self, 'retroceso'):
                                self.retroceso = True
                                self.current_cajon = 0

                                fwd = -1.0
                                rgt = 0
                                dwn = 0
                                tx, ty, tz, tyaw = self.get_global_coordinates(fwd, rgt, dwn)

                                self.scan_origin_x = float(tx)
                                self.scan_origin_y = float(ty)
                                self.scan_origin_z = float(tz)
                                self.scan_origin_yaw = float(tyaw)

                                self.get_logger().info(f"Iniciando retroceso")
                                return 
                                            
                            dx = float(self.scan_origin_x - self.pose_actual[0])
                            dy = float(self.scan_origin_y - self.pose_actual[1])
                            dz = float(self.scan_origin_z - self.pose_actual[2])
                            distancia = np.sqrt(dx**2 + dy**2 + dz**2)

                            setpoint_msg = TrajectorySetpoint()
                            setpoint_msg.position = [self.scan_origin_x, self.scan_origin_y, self.scan_origin_z]
                            setpoint_msg.yaw = self.scan_origin_yaw
                            setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
                            self.trajectory_pub.publish(setpoint_msg)

                            if distancia < 0.3:
                                self.get_logger().info("Barrido completado")

                                if hasattr(self, 'scan_iniciado'): del self.scan_iniciado
                                if hasattr(self, 'retroceso'): del self.retroceso
                                
                                self.barrido_sub_idx = 0
                                self.search_aruco = True
                                del self.aruco_id
                                return
                            
                    elif self.aruco_id == 3:
                        ''' Cambio de fila girando 180 '''
                        offset = 2.0
                        length = 13.0
                        
                        if not hasattr(self, 'change_line_iniciado'):
                            self.get_logger().info(f"Iniciando cambio de linea con variante giros")
                            self.change_line_iniciado = True
                            self.sub_step = 1
                            
                            self.start_x = self.pose_actual[0]
                            self.start_y = self.pose_actual[1]
                            self.start_z = self.pose_actual[2]
                            self.start_yaw = self.pose_actual[3]                
                            return
                        
                        target_x, target_y, target_z, target_yaw = 0.0, 0.0, 0.0, 0.0 
         
                        if self.sub_step == 1:
                            target_x = self.start_x
                            target_y = self.start_y
                            target_z = self.start_z - 2.0
                            target_yaw = self.start_yaw
                            label = "Subiendo por seguridad"

                        elif self.sub_step == 2:
                            target_x = self.start_x + length + offset
                            target_y = self.start_y
                            target_z = self.start_z - 2.0
                            target_yaw = self.start_yaw
                            label = "Moviéndose a la siguiente fila"

                        elif self.sub_step == 3:
                            target_x = self.start_x + length + offset
                            target_y = self.start_y
                            target_z = self.start_z
                            target_yaw = self.start_yaw + 3.14
                            label = "Bajando y rotando a nueva fila"

                        self.send_setpoint(target_x, target_y, target_z, target_yaw)

                        dx = target_x - self.pose_actual[0]
                        dy = target_y - self.pose_actual[1]
                        dz = target_z - self.pose_actual[2]
                        distancia = np.sqrt(dx**2 + dy**2 + dz**2)

                        if distancia < 0.3:
                            self.get_logger().info(f"✅ Paso {self.sub_step} completado")
                            self.sub_step += 1
                            
                            # Si terminamos los 3 pasos, cerramos la acción
                            if self.sub_step > 3:
                                if not hasattr(self, "hold_start"):
                                    self.hold_start = time.perf_counter()

                                if time.perf_counter() - self.hold_start >= 2:
                                    del self.hold_start
                                    self.get_logger().info("Cambio de fila con rotacion terminado")
                                    self.search_aruco = True
                                    del self.aruco_id

                                    del self.change_line_iniciado
                                    del self.sub_step
                                    return

                    elif self.aruco_id == 4:
                        ''' Cambio de fila avance '''
                        offset = 2.0
                        length = 13.0
                        height_offset = 2.0
                        
                        if not hasattr(self, 'cambio_linea_1'):
                            self.get_logger().info(f"Iniciando cambio de linea con salto y giro")
                            self.cambio_linea_1 = True
                            self.sub_step = 1
                            
                            self.start_x = self.pose_actual[0]
                            self.start_y = self.pose_actual[1]
                            self.start_z = self.pose_actual[2]
                            self.start_yaw = self.pose_actual[3]                
                            return
                        
                        # target_x, target_y, target_z, target_yaw = 0.0, 0.0, 0.0, 0.0 
         
                        if self.sub_step == 1:
                            target_x = self.start_x
                            target_y = self.start_y
                            target_z = self.start_z - height_offset
                            target_yaw = self.start_yaw

                        elif self.sub_step == 2:
                            target_x = self.start_x + length + offset
                            target_y = self.start_y
                            target_z = self.start_z - height_offset
                            target_yaw = self.start_yaw

                        elif self.sub_step == 3:
                            target_x = self.start_x + length + offset
                            target_y = self.start_y
                            target_z = self.start_z 
                            target_yaw = self.start_yaw + 3.14

                        elif self.sub_step == 4:
                            if not hasattr(self, "hold_start"):
                                self.get_logger().info("Iniciando hold")
                                self.hold_start = time.perf_counter()

                            if time.perf_counter() - self.hold_start >= 4.0:
                                self.get_logger().info("Hold terminado")
                                del self.hold_start

                                self.get_logger().info("Cambio de fila con rotacion terminado")
                                self.search_aruco = True
                                del self.aruco_id
                                del self.cambio_linea_1
                                del self.sub_step
                                return

                        if self.sub_step <= 3:
                            self.send_setpoint(target_x, target_y, target_z, target_yaw)

                            dx = target_x - self.pose_actual[0]
                            dy = target_y - self.pose_actual[1]
                            dz = target_z - self.pose_actual[2]
                            distancia = np.sqrt(dx**2 + dy**2 + dz**2)

                            if distancia < 0.3:
                                self.get_logger().info(f"✅ Paso {self.sub_step} completado")
                                self.sub_step += 1

                    elif self.aruco_id == 5:
                        ''' Final '''
                        offset = 1.0
                        length = 1.0
                        
                        if not hasattr(self, 'change_line_iniciado'):
                            self.get_logger().info(f"Finalizado")
                            self.change_line_iniciado = True
                            self.sub_step = 1
                            
                            self.start_x = self.pose_actual[0]
                            self.start_y = self.pose_actual[1]
                            self.start_z = self.pose_actual[2]
                            self.start_yaw = self.pose_actual[3]                
                            return
                        
                        target_x, target_y, target_z, target_yaw = 0.0, 0.0, 0.0, 0.0 
         
                        if self.sub_step == 1:
                            target_x = self.start_x
                            target_y = self.start_y
                            target_z = self.start_z - 2.0
                            target_yaw = self.start_yaw
                            label = "Subiendo por seguridad"

                        elif self.sub_step == 2:
                            target_x = 0.0
                            target_y = 0.0
                            target_z = self.start_z - 2.0
                            target_yaw = self.start_yaw
                            label = "Moviéndose a la siguiente fila"

                        elif self.sub_step == 3:
                            target_x = 0.0
                            target_y = 0.0
                            target_z = 0.0
                            target_yaw = self.start_yaw
                            label = "Bajando y rotando a nueva fila"

                        self.send_setpoint(target_x, target_y, target_z, target_yaw)

                        dx = target_x - self.pose_actual[0]
                        dy = target_y - self.pose_actual[1]
                        dz = target_z - self.pose_actual[2]
                        distancia = np.sqrt(dx**2 + dy**2 + dz**2)

                        if distancia < 0.3:
                            self.get_logger().info(f"✅ Paso {self.sub_step} completado")
                            self.sub_step += 1
                            
                            # Si terminamos los 3 pasos, cerramos la acción
                            if self.sub_step > 3:
                                if not hasattr(self, "hold_start"):
                                    self.hold_start = time.perf_counter()

                                if time.perf_counter() - self.hold_start >= 2:
                                    del self.hold_start
                                    self.get_logger().info("Cambio de fila con rotacion terminado")
                                    self.search_aruco = True
                                    del self.aruco_id

                                    del self.change_line_iniciado
                                    del self.sub_step
                                    return


def main():
    rclpy.init()
    rclpy.spin(MissionHandler())
    rclpy.shutdown()

if __name__ == "__main__":
    main()