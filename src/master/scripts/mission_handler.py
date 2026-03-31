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
from px4_msgs.msg import VehicleCommand, VehicleOdometry, VehicleCommandAck, TrajectorySetpoint, OffboardControlMode
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MissionHandler(Node):
    def __init__(self):
        super().__init__('mission_handler')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)

        try:
            default_path = os.path.join(
                get_package_share_directory('master'), 
                'misiones', 'prueba.yaml'
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

        self.velocidad_actual = [0.0]*6

        self.comando_enviado = 0

        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.pose_callback, qos_profile)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.ack_callback, qos_profile)

        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

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

        except Exception as e:
            self.get_logger().error(f"Error en pose_callback: {e}")

    def ack_callback(self, msg: VehicleCommandAck):
        self.ack_command = msg.command
        self.ack_result = msg.result

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
            error_altura = abs(self.pose_actual[2] - z_objetivo)

            self.get_logger().info(f"Z actual: {self.pose_actual[2]:.2f} | Z objetivo: {z_objetivo:.2f} | Error: {error_altura:.2f} | Pose despegue z: {self.pose_despegue_z:.2f}")
  
            if error_altura < 0.3:
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
            
            # 2. Si estamos lejos, calculamos el vector unitario hacia el objetivo
            if distancia > 0.5:
                setpoint_msg.velocity = [
                    (dx / distancia) * vel,
                    (dy / distancia) * vel,
                    (dz / distancia) * vel
                ]
            else:
                # Cerca del objetivo, dejamos que PX4 use su propio algoritmo para frenar suavemente
                setpoint_msg.velocity = [float('nan'), float('nan'), float('nan')]

            setpoint_msg.yaw = 0.0
            setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            self.trajectory_pub.publish(setpoint_msg)

            self.get_logger().info(f"Yendo a [{x_obj}, {y_obj}, {z_obj}] | Distancia restante: {distancia:.2f}m")

            # 3. Condición de llegada
            if distancia < 0.3:
                self.get_logger().info("¡Punto de destino alcanzado!")
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