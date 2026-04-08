#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from std_msgs.msg import Bool, String, Float32, Int16
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import VehicleCommand, VehicleOdometry, VehicleCommandAck, TrajectorySetpoint, OffboardControlMode, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MissionHandler(Node):
    def __init__(self):
        super().__init__('mission_handler')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)

        try:
            default_path = os.path.join(
                get_package_share_directory('master'), 
                'misiones', 'escaneo.yaml'
            )
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

        self.actions = self.mission["actions"]
        self.idx = 0
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

        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

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
            self.pose_actual[3] = roll
            self.pose_actual[4] = pitch
            self.pose_actual[5] = yaw

            self.velocidad_actual[0] = msg.velocity[0]
            self.velocidad_actual[1] = msg.velocity[1]
            self.velocidad_actual[2] = msg.velocity[2]
            # self.velocidad_actual[3] = msg.velocity[3]
            # self.velocidad_actual[4] = msg.velocity[4]
            # self.velocidad_actual[5] = msg.velocity[5]

            # self.get_logger().info(f"X_local: {self.pose_actual[0]:.2f}, Y_local: {self.pose_actual[1]:.2f}, Z_local: {self.pose_actual[2]:.2f}")
            # self.get_logger().info(f"RX_local: {self.pose_actual[3]:.2f}, RY_local: {self.pose_actual[4]:.2f}, RZ_local: {self.pose_actual[5]:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error en pose_callback: {e}")


    def ack_callback(self, msg: VehicleCommandAck):
        self.ack_command = msg.command
        self.ack_result = msg.result

    def aruco_error_callback(self, msg):
        self.current_aruco_error = msg

    def run(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_publisher.publish(offboard_msg)

        if self.idx >= len(self.actions):
        # Es para declarar que la mision ha terminadl
            if not hasattr(self, 'mission_finished_flag'):
                self.get_logger().info("¡MISIÓN COMPLETADA!")
                status_msg = Int16()
                status_msg.data = 1
                self.mission_finished_flag = True
            return

        action = self.actions[self.idx]

        if not hasattr(self, 'current_idx_logged') or self.current_idx_logged != self.idx:
            self.get_logger().info(f"Ejecutando Acción {self.idx}: {action['type']}")
            self.current_idx_logged = self.idx
            self.checkpoint = 0
            # IMPORTANTE: Si es un movimiento nuevo, ignoramos checkpoints viejos
            if action["type"] in ["goto", "rotate"]:
                self.checkpoint = 0

        elif action["type"] == "arm":
            msg = VehicleCommand()
            msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            msg.param1 = 1.0

            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True

            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

            self.command_publisher.publish(msg)

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

            self.idx += 1


        elif action["type"] == "disarm":
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
            self.idx += 1

        elif action["type"] == "takeoff":
            height = float(action["height"])
            if not hasattr(self, 'takeoff_sent'):
                self.pose_despegue_z = self.pose_actual[2]

                msg = VehicleCommand()
                msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
                msg.param7 = float(height)
                msg.target_system = 1
                msg.target_component = 1
                msg.source_system = 1
                msg.source_component = 1
                msg.from_external = True
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.command_publisher.publish(msg)

                self.takeoff_sent = True
                self.get_logger().info(f"Comando de despegue enviado. Altura inicial: {self.pose_despegue_z:.2f}")
                return

            z_objetivo = self.pose_despegue_z - height
            # error_altura = abs(self.pose_actual[2] - z_objetivo)

            error_altura = height - abs(self.pose_actual[2])

            self.get_logger().info(f"Z actual: {self.pose_actual[2]:.2f} | Z objetivo: {z_objetivo:.2f} | Error: {error_altura:.2f} | Pose despegue z: {self.pose_despegue_z:.2f}")
  
            if error_altura < 0.05:
                self.get_logger().info("Altura alcanzada")
                self.idx += 1
                del self.takeoff_sent

        elif action["type"] == "land":
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
                self.idx += 1
                del self.land_sent
                del self.ha_empezado_a_bajar


        elif action["type"] == "goto":
            x_obj = float(action["x"])
            y_obj = float(action["y"])
            z_obj = float(action["z"])
            yaw_obj = float(action["yaw"])
            vel = float(action["vel"])

            if not hasattr(self, 'offboard_switched') or self.offboard_switched != self.idx:
                msg = VehicleCommand()
                msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                msg.param1 = 1.0  # Custom mode
                msg.param2 = 6.0  # Offboard
                msg.target_system = 1
                msg.target_component = 1
                msg.source_system = 1
                msg.source_component = 1
                msg.from_external = True
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.command_publisher.publish(msg)
                
                self.offboard_switched = self.idx
                self.get_logger().info("Solicitando cambio a modo Offboard...")

            # 1. Calcular la distancia y dirección real hacia el objetivo
            dx = x_obj - self.pose_actual[0]
            dy = y_obj - self.pose_actual[1]
            dz = z_obj - self.pose_actual[2]
            distancia = np.sqrt(dx**2 + dy**2 + dz**2)

            setpoint_msg = TrajectorySetpoint()
            setpoint_msg.position = [x_obj, y_obj, z_obj]
            setpoint_msg.yaw = np.deg2rad(yaw_obj)
            
            # 2. Si estamos lejos, calculamos el vector unitario hacia el objetivo
            if distancia > 0.5:
                setpoint_msg.velocity = [
                    (dx / distancia) * vel,
                    (dy / distancia) * vel,
                    (dz / distancia) * vel
                ]

                setpoint_msg.acceleration = [
                    (dx / distancia) * vel,
                    (dy / distancia) * vel,
                    (dz / distancia) * vel
                ]

            else:
                # Cerca del objetivo, dejamos que PX4 use su propio algoritmo para frenar suavemente
                setpoint_msg.velocity = [float('nan'), float('nan'), float('nan')]

            setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            self.trajectory_pub.publish(setpoint_msg)

            self.get_logger().info(f"Yendo a [{x_obj}, {y_obj}, {z_obj}] | Distancia restante: {distancia:.2f}m")

            # 3. Condición de llegada
            if distancia < 0.3:
                self.get_logger().info("¡Punto de destino alcanzado!")
                self.idx += 1

        elif action["type"] == "searchArUco":
            if not hasattr(self, "alineado"): 
                self.alineado = False # Bandera para saber si ya terminamos
            if not hasattr(self, "posicionado"):
                self.posicionado = True # Bandera para saber si el dron está quieto y puede tomar foto

            if self.posicionado and not hasattr(self, "esperando_foto"):
                self.get_logger().info("Tomando captura para análisis...")
                self.tomar_foto_pub.publish(Int16(data=2))
                self.esperando_foto = True
                self.current_aruco_error = None # Limpiamos error viejo
                self.tomar_foto_pub.publish(Int16(data=0))
                return # Esperamos al siguiente ciclo para ver la respuesta
            
                # Empieza

            if hasattr(self, "esperando_foto") and self.current_aruco_error is not None:
                # Apagamos el trigger de la cámara
                self.tomar_foto_pub.publish(Int16(data=0))
                del self.esperando_foto
                
                err = self.current_aruco_error

                if err.position.x == 999.0:
                    # El dron no encontro el Aruco, empezara a rotar hasta encontrarlo
                    self.get_logger().error("Aruco no encontrado. Rotando 45 grados")

                    target_x = self.pose_actual[0]
                    target_y = self.pose_actual[1]
                    target_z = self.pose_actual[2]

                    target_yaw = np.deg2rad(self.pose_actual[3]) - 0.785

                    setpoint_msg = TrajectorySetpoint()
                    setpoint_msg.position = [float(self.pose_actual[0]), float(self.pose_actual[1]), float(self.pose_actual[2])]
                    setpoint_msg.yaw = float(target_yaw) # El dron rota 45 grados
                    setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                    self.trajectory_pub.publish(setpoint_msg)

                    self.target_vuelo = [target_x, target_y, target_z, target_yaw]

                    self.posicionado = False
                    self.hold_start = time.perf_counter()

                elif abs(err.position.x) < 0.1 and abs(err.position.y) < 0.05:
                    self.get_logger().error("🎯 Dron alineado, continuando")
                    # LIMPIEZA SEGURA
                    if hasattr(self, "alineado"): del self.alineado
                    if hasattr(self, "posicionado"): del self.posicionado
                    self.idx += 1
                    return

                else:
                    # En esta condicion encontro el ArUco y se va a posicionar para tener el ArUco en el centro
                    self.get_logger().info("ArUco encontrado, alineando")
                    
                    # Calcula lo que tendra que moverse
                    kp_lateral = 0.9
                    kp_altitud = 0.9
                    kp_yaw = 0.3

                    target_x = self.pose_actual[0] - (err.position.x * kp_lateral)
                    target_y = self.pose_actual[1] + (err.position.z * kp_lateral)
                    target_z = self.pose_actual[2] - (err.position.y * kp_altitud)

                    self.get_logger().info(f"Corrigiendo posición. Error X: {err.position.x:.2f}, Y: {err.position.y:.2f}, Z: {err.position.z:.2f}")
                    
                    # Da la orden de moverse
                    setpoint_msg = TrajectorySetpoint()
                    setpoint_msg.position = [float(target_x), float(target_y), float(target_z)]
                    setpoint_msg.yaw = np.deg2rad(90)
                    setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                    self.trajectory_pub.publish(setpoint_msg)

                    self.target_vuelo = [target_x, target_y, target_z, np.deg2rad(90)]

                    self.posicionado = False
                    self.hold_start = time.perf_counter()

            if not self.posicionado:
                distancia = 999.0
                dyaw = 999.0
                if hasattr(self, "target_vuelo"):
                    dx = self.target_vuelo[0] - self.pose_actual[0]
                    dy = self.target_vuelo[1] - self.pose_actual[1]
                    dz = self.target_vuelo[2] - self.pose_actual[2]
                    dyaw = abs(self.target_vuelo[3] - self.pose_actual[3])
                    distancia = np.sqrt(dx**2 + dy**2 + dz**2)
                    self.get_logger().info(f"Dist: {distancia:.2f}m, Error Yaw: {np.rad2deg(dyaw):.1f}°")

                if distancia < 0.1 and (dyaw < 0.1):
                    self.get_logger().info("Posición alcanzada. Estabilizando para foto...")
                    self.posicionado = True
                    # if hasattr(self, "target_vuelo"): del self.target_vuelo


        elif action["type"] == "scan_2":
            offset = float(action["offset"])
            cajones = int(action["cajones"])
            width = float(action["width"])
            side = int(action["side"])

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
            
            if side == 1:
                y_obj = self.scan_origin_y - cajones * width - offset

            elif side == -1:
                y_obj = self.scan_origin_y + cajones * width + offset

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

                self.tomar_foto_pub.publish(Int16(data=2))
                self.tomar_foto_pub.publish(Int16(data=0))
                # Nota: El nodo de la cámara debería apagar el flag solo tras guardar
                
                self.current_cajon += 1

            
            dx = x_obj - self.pose_actual[0]
            dy = y_obj - self.pose_actual[1]
            dz = z_obj - self.pose_actual[2]
            
            distancia = np.sqrt(dx**2 + dy**2 + dz**2)

            if distancia < 0.3:
                self.get_logger().info("¡Punto de destino alcanzado!")
                del self.scan_iniciado
                self.idx += 1





        elif action["type"] == "scan":
            cajones = int(action["cajones"])
            filas = int(action["filas"])
            cajon_x = float(action["cajon_x"])
            filas_y = float(action["filas_y"])
            altura = float(action["altura"])
            vel = float(action["vel"]) 
            yaw = float(action["yaw"])
            duracion = float(action["duracion"])

            if not hasattr(self, 'scan_iniciado'):
                self.scan_iniciado = True
                self.current_cajon = 0
                self.current_fila = 0
                self.scan_origin_x = self.pose_actual[0]
                self.scan_origin_y = self.pose_actual[1]
            
                self.theta = np.deg2rad(yaw)
                
                self.get_logger().info(f"Iniciando escaneo rotado a {yaw}°...")
                return 

            local_x = self.current_cajon * cajon_x
            local_y = self.current_fila * filas_y

            x_rot = local_x * np.cos(self.theta) - local_y * np.sin(self.theta)
            y_rot = local_x * np.sin(self.theta) + local_y * np.cos(self.theta)

            x_obj = self.scan_origin_x + x_rot
            y_obj = self.scan_origin_y + y_rot
            z_obj = -altura

            dx = x_obj - self.pose_actual[0]
            dy = y_obj - self.pose_actual[1]
            dz = z_obj - self.pose_actual[2]
            distancia = np.sqrt(dx**2 + dy**2 + dz**2)

            setpoint_msg = TrajectorySetpoint()
            setpoint_msg.position = [x_obj, y_obj, z_obj]
            setpoint_msg.yaw = self.theta 
            
            if distancia > 0.5:
                setpoint_msg.velocity = [
                    (dx / distancia) * vel, 
                    (dy / distancia) * vel, 
                    (dz / distancia) * vel
                ]
            else:
                setpoint_msg.velocity = [float('nan'), float('nan'), float('nan')]

            setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_pub.publish(setpoint_msg)
            
            if distancia < 0.3:
                if not hasattr(self, "hold_start"):
                    self.get_logger().info(f"Punto reached. Tomando foto en Cajón [{self.current_cajon}, {self.current_fila}]...")
                    self.hold_start = time.perf_counter()

                
                if time.perf_counter() - self.hold_start >= duracion:
                    self.get_logger().info("Foto tomada. Avanzando...")

                    msg_foto = Int16()
                    msg_foto.data = 1
                    self.tomar_foto_pub.publish(msg_foto)

                    msg_foto.data = 0
                    self.tomar_foto_pub.publish(msg_foto)
                    
                    del self.hold_start
                    
                    self.current_cajon += 1

                    if self.current_cajon >= cajones:
                        self.current_cajon = 0
                        self.current_fila += 1
                        self.get_logger().info(f"Cambiando a la fila {self.current_fila}")

                    if self.current_fila >= filas:
                        self.get_logger().info("¡Escaneo completo de todo el estacionamiento!")
                        del self.scan_iniciado
                        del self.current_cajon
                        del self.current_fila
                        del self.scan_origin_x
                        del self.scan_origin_y
                        del self.theta
                        
                        self.idx += 1


        elif action["type"] == "rtb":
            # El punto de origen en coordenadas locales de PX4 siempre es [0, 0]
            x_obj = 0.0
            y_obj = 0.0
            z_obj = -3.0  # O la altura a la que quieras que regrese
            vel = 1.5     # Velocidad de regreso a casa
            
            # 1. Calculamos distancia al punto de despegue
            dx = x_obj - self.pose_actual[0]
            dy = y_obj - self.pose_actual[1]
            dz = z_obj - self.pose_actual[2]
            distancia = np.sqrt(dx**2 + dy**2 + dz**2)

            setpoint_msg = TrajectorySetpoint()
            setpoint_msg.position = [x_obj, y_obj, z_obj]
            
            # 2. Aplicamos vector de velocidad para el regreso
            if distancia > 0.3:
                setpoint_msg.velocity = [
                    (dx / distancia) * vel, 
                    (dy / distancia) * vel, 
                    (dz / distancia) * vel
                ]
            else:
                setpoint_msg.velocity = [float('nan'), float('nan'), float('nan')]

            setpoint_msg.yaw = 0.0
            setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_pub.publish(setpoint_msg)

            self.get_logger().info(f"Regresando a base... Distancia: {distancia:.2f}m")

            # 3. Al llegar a casa, pasamos a la siguiente acción (que será tu land)
            if distancia < 0.3:
                self.get_logger().info("¡Llegamos a la vertical de base!")
                self.idx += 1

                
        elif action["type"] == "hold":
            duration = action["duration"]
            if not hasattr(self, "hold_start"):
                # self.get_logger().info(f"Holding for {duration} seconds")
                self.hold_start = time.perf_counter()
                # self.get_logger().info()(f"Hold {duration}")

            if time.perf_counter() - self.hold_start >= duration:
                del self.hold_start
                self.idx += 1


def main():
    rclpy.init()
    rclpy.spin(MissionHandler())
    rclpy.shutdown()

if __name__ == "__main__":
    main()