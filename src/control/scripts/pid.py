#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from math import atan2, sin, cos

from std_msgs.msg import Float64MultiArray
# <-- CORRECCIÓN: 'Quaternion' no se usa solo, pero 'Pose' sí es necesario.
from geometry_msgs.msg import Pose
# <-- CORRECCIÓN: Esta librería es necesaria para la conversión de cuaternión a Euler.
from tf_transformations import euler_from_quaternion
# (Asegúrate de haber arreglado el problema de NumPy 2.0 con 'pip install --upgrade transforms3d')


class Init(Node):

    def __init__(self):
        super().__init__('pid')

        # <-- CORRECCIÓN: Ahora 'Pose' está definido.
        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.objetivo_sub = self.create_subscription(Pose, 'pose_objetivo', self.pose_objetivo_callback, 10)

        self.thruster_pub = self.create_publisher(Float64MultiArray, '/forces', 10)
        
        self.kp_ = [0]*6
        self.ki_ = [0]*6
        self.kd_ = [0]*6

        self.pose_actual_ = [0]*6
        self.pose_objetivo_ = [0]*6
        self.pose_output_ = [0]*6

        self.error_anterior_ = [0]*6
        self.integral_ = [0]*6

        self.kp_[0] = 5.2
        self.kp_[1] = 7.2
        self.kp_[2] = 4.569 # 17.47
        self.kp_[3] = 2.2
        self.kp_[4] = 1.0
        self.kp_[5] = 2

        self.ki_[0] = 0.01
        self.ki_[1] = 0.01011
        self.ki_[2] = 0.0301 # 0.1716
        self.ki_[3] = 0.01
        self.ki_[4] = 0.01
        self.ki_[5] = 0.01

        self.kd_[0] = 40
        self.kd_[1] = 45.38
        self.kd_[2] = 32.73 # 16.606
        self.kd_[3] = 8
        self.kd_[4] = 5
        self.kd_[5] = 5.2
#1.57 0.60 1.30
        self.tiempo_anterior_ = self.get_clock().now().nanoseconds
        self.create_timer(0.1, self.calcularpid)

    # <-- CORRECCIÓN: Añadido el tipo de mensaje (buena práctica)
    def pose_callback(self, msg: Pose):
        self.pose_actual_[0] = msg.position.x
        self.pose_actual_[1] = msg.position.y
        self.pose_actual_[2] = msg.position.z

        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # <-- CORRECCIÓN: Ahora 'euler_from_quaternion' está definido.
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.pose_actual_[3] = roll
        self.pose_actual_[4] = pitch
        self.pose_actual_[5] = yaw

    # <-- CORRECCIÓN: Añadido el tipo de mensaje
    def pose_objetivo_callback(self, msg: Pose):
        self.pose_objetivo_[0] = msg.position.x
        self.pose_objetivo_[1] = msg.position.y
        self.pose_objetivo_[2] = msg.position.z

        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # <-- CORRECCIÓN: Ahora 'euler_from_quaternion' está definido.
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.pose_objetivo_[3] = roll
        self.pose_objetivo_[4] = pitch
        self.pose_objetivo_[5] = yaw

    def normalize_angle(self, ang):
        # Tu método es correcto y usa las funciones que importaste.
        ang = atan2(sin(ang), cos(ang))
        return ang


    def calcularpid(self):
        tiempo_actual = self.get_clock().now().nanoseconds
        dt = (tiempo_actual - self.tiempo_anterior_) / 1e9
        
        # Si el dt es 0, no hacer nada para evitar división por cero
        if dt == 0.0:
            return

        for n in range(6):
            error = self.pose_objetivo_[n] - self.pose_actual_[n]

            if n>=3:
                error = self.normalize_angle(error)

            # (Hecho para depuración, lo puedes quitar)
            # if n==5:
            #     print(f"Actual: {self.pose_actual_[n]:.2f}, Objetivo: {self.pose_objetivo_[n]:.2f}, Error: {error:.2f}")

            self.integral_[n] += error * dt
            
            # <-- CORRECCIÓN: Protección contra dt=0 (aunque ya la puse arriba, es doblemente seguro)
            derivada = (error - self.error_anterior_[n]) / dt

            p = self.kp_[n] * error
            i = self.ki_[n] * self.integral_[n]
            d = self.kd_[n] * derivada

            self.pose_output_[n] = p + i + d
            self.error_anterior_[n] = error

        self.tiempo_anterior_ = tiempo_actual


        control = Float64MultiArray()
        control.data = [0.0]*6

        control.data[0] = float(self.pose_output_[0])
        control.data[1] = float(self.pose_output_[1])
        control.data[2] = float(self.pose_output_[2])
        control.data[3] = float(self.pose_output_[3])
        control.data[4] = float(self.pose_output_[4])
        control.data[5] = float(self.pose_output_[5])

        self.thruster_pub.publish(control)


def main(args=None):
    rclpy.init(args=args)
    node = Init()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()